#include "camera_source.h"

#include <algorithm>
#include <chrono>
#include <mutex>
#include <sstream>

#include "HikvisionCamera.h"
#include "rtsp_gpu_stream.h"
#include "utils.h"   // LOG_COMMON: 带时间戳 + 落盘 + 多线程安全

namespace {

    // RFC 3986 userinfo 中的保留字符必须做 percent-encoding,
    // 否则 FFmpeg 解析 rtsp://user:pwd@host 时, 密码里的 @ : / # ? 会被切错.
    // 例如 pwd="Hc@RTG210" 直接拼出 rtsp://admin:Hc@RTG210@host..., FFmpeg
    // 会把第一个 @ 当 userinfo 分隔符, 主机变成 "RTG210@host", 必然连不上.
    // 这里对所有非 unreserved 字符全部编码, 比白名单更稳.
    std::string UrlEncodeUserInfo(const std::string& s) {
        std::ostringstream oss;
        oss << std::uppercase << std::hex;
        for (unsigned char c : s) {
            // unreserved: A-Z a-z 0-9 - _ . ~
            bool unreserved = (c >= 'A' && c <= 'Z')
                || (c >= 'a' && c <= 'z')
                || (c >= '0' && c <= '9')
                || c == '-' || c == '_' || c == '.' || c == '~';
            if (unreserved) {
                oss << static_cast<char>(c);
            }
            else {
                oss << '%';
                if (c < 0x10) oss << '0';
                oss << static_cast<int>(c);
            }
        }
        return oss.str();
    }

    // 海康主码流 RTSP URL: rtsp://user:pwd@ip:554/Streaming/Channels/{channel}01
    // 注意: user / pwd 里可能含 @ : / # ? 等保留字符, 必须 percent-encode.
    std::string BuildHikRtspUrl(const std::string& user, const std::string& pwd,
        const std::string& ip, int channel) {
        std::ostringstream oss;
        oss << "rtsp://" << UrlEncodeUserInfo(user)
            << ":" << UrlEncodeUserInfo(pwd)
            << "@" << ip
            << ":554/Streaming/Channels/" << channel << "01";
        return oss.str();
    }

    // =========================================================================
    // 海康 PTZ 适配器
    // =========================================================================
    class HikvisionPtzAdapter : public IPtzControl {
    public:
        explicit HikvisionPtzAdapter(HikvisionCamera* cam) : cam_(cam) {}

        bool HasPan()  const override { return true; }
        bool HasZoom() const override { return true; }

        bool Move(float pan_deg, float tilt_deg) override {
            return cam_ ? cam_->setRelativeAngle(pan_deg, tilt_deg) : false;
        }
        bool MoveTo(float pan_deg, float tilt_deg) override {
            return cam_ ? cam_->setAbsoluteAngle(pan_deg, tilt_deg) : false;
        }
        bool Zoom(float multiplier) override {
            return cam_ ? cam_->setAbsoluteZoom(multiplier) : false;
        }
        void StopAll() override { if (cam_) cam_->stopAllActions(); }

    private:
        HikvisionCamera* cam_;
    };

    using Clock = std::chrono::steady_clock;

}  // namespace

// =========================================================================
// pImpl
//   - gpu_stream     : GPU 拉流 (主路径)
//   - camera         : 海康 SDK, 负责 PTZ + 软解兜底
//   - ptz            : 把 camera 包成 IPtzControl
//   - sdk_fallback   : 当前是否处于 SDK 软解兜底状态
//   - gpu_open_time  : GPU 流最近一次 Open 的时刻 (用于冷启动宽容判定)
//   - last_recover_attempt: 兜底中最近一次试图恢复 GPU 的时刻
// =========================================================================
struct CameraSource::Impl {
    std::unique_ptr<RtspGpuStream>       gpu_stream;
    std::unique_ptr<HikvisionCamera>     camera;
    std::unique_ptr<HikvisionPtzAdapter> ptz;

    // 三个 time_point 用 epoch (= time_point{}) 而不是 time_point::min().
    // 理由: 后续要做 (now - last_*) 然后 cast 成 int64 毫秒, time_point::min()
    //       本质是 -INT64_MAX 量级, 减法结果不在 int64 范围内 → UB.
    //       实测 elapsed 会变成接近 INT64_MIN 的巨大负数, 导致
    //       elapsed < reconnect_interval_ms 永远成立, 首连分支死循环 sleep.
    Clock::time_point last_connect_attempt{};   // = epoch
    Clock::time_point gpu_open_time{};
    Clock::time_point last_recover_attempt{};
    bool              sdk_fallback = false;

    Impl() {
        gpu_stream = std::make_unique<RtspGpuStream>();
        camera = std::make_unique<HikvisionCamera>();
        ptz = std::make_unique<HikvisionPtzAdapter>(camera.get());
    }
};

// =========================================================================
// CameraSource
// =========================================================================
CameraSource::CameraSource(const CameraSourceConfig& cfg)
    : cfg_(cfg), impl_(std::make_unique<Impl>()) {
    if (cfg_.poll_interval_ms <= 0) cfg_.poll_interval_ms = 33;
    if (cfg_.rtsp_url.empty()) {
        cfg_.rtsp_url = BuildHikRtspUrl(cfg_.user, cfg_.pwd, cfg_.ip, cfg_.channel);
    }
}

CameraSource::~CameraSource() { Stop(); }

bool CameraSource::IsStreamConnected() const {
    return impl_->gpu_stream->IsStreamConnected()
        || impl_->camera->isStreamConnected();
}

IPtzControl* CameraSource::Ptz() { return impl_->ptz.get(); }

// ---- Sink 管理 ----
void CameraSource::AddSink(FrameSink* sink) {
    if (!sink) return;
    std::unique_lock<std::shared_mutex> lock(sinks_mtx_);
    for (auto* s : sinks_) if (s == sink) return;
    sinks_.push_back(sink);
}

void CameraSource::RemoveSink(FrameSink* sink) {
    if (!sink) return;
    std::unique_lock<std::shared_mutex> lock(sinks_mtx_);
    sinks_.erase(std::remove(sinks_.begin(), sinks_.end(), sink), sinks_.end());
}

// =========================================================================
// 主循环里要用的小判定函数 (放进 lambda 不便, 抽出来)
// =========================================================================
namespace {

    // GPU 流是否处于"应该认为它挂了"的状态
    // - 冷启动期 (Open 后还没出过首帧) 内只要不超过 grace_ms 都算正常
    // - 出过首帧后, 超过 stale_threshold_ms 没新帧就算挂
    bool IsGpuStale(const RtspGpuStream& gpu,
        Clock::time_point gpu_open_time,
        int grace_ms,
        int stale_threshold_ms) {
        if (gpu.HasEverProduced()) {
            return gpu.MillisSinceLastFrame() > stale_threshold_ms;
        }
        // 还没出首帧
        auto since_open_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            Clock::now() - gpu_open_time).count();
        return since_open_ms > grace_ms;
    }

}  // namespace

// =========================================================================
// 启停
// =========================================================================
void CameraSource::Start() {
    if (is_running_.exchange(true)) {
        LOG_COMMON("[CameraSource cam" << cfg_.cam_index << "] 已经在运行");
        return;
    }
    if (cfg_.auto_connect) want_connected_.store(true);

    poll_thread_ = std::thread([this] {
        cv::Mat frame;

        while (is_running_.load()) {
            const bool want = want_connected_.load();
            const bool sdk_logged_in = (impl_->camera->isStreamConnected()
                || impl_->camera->isRealPlaying());
            const bool gpu_alive = impl_->gpu_stream->IsStreamConnected();

            // ====== 1) 主动断开 ======
            if (!want && (gpu_alive || sdk_logged_in)) {
                LOG_COMMON("[CameraSource cam" << cfg_.cam_index << "] 主动断开");
                impl_->gpu_stream->Close();
                impl_->camera->disconnect();
                impl_->sdk_fallback = false;
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(cfg_.poll_interval_ms));
                continue;
            }

            // ====== 2) 想连但还没建立任何连接 → 首次连接 ======
            // 判据: 海康 SDK 没登录(isRealPlaying 是 m_realPlayHandle>=0,
            //       但 only_login 模式下我们另判 userID), 用 isStreamConnected+isRealPlaying
            //       不够, 这里就以 gpu_alive==false 且 sdk_logged_in==false 作为"没连"
            if (want && !gpu_alive && !sdk_logged_in) {
                auto now = Clock::now();
                auto elapsed_ms =
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        now - impl_->last_connect_attempt).count();
                if (elapsed_ms < cfg_.reconnect_interval_ms) {
                    std::this_thread::sleep_for(
                        std::chrono::milliseconds(cfg_.poll_interval_ms));
                    continue;
                }
                impl_->last_connect_attempt = now;

                LOG_COMMON("[CameraSource cam" << cfg_.cam_index
                    << "] 尝试连接 GPU=" << cfg_.rtsp_url
                    << " | SDK=" << cfg_.ip << ":" << cfg_.port
                    << " ch=" << cfg_.channel);

                // a) 海康 SDK 仅登录 (PTZ 通道 + 兜底备用), 不开 RealPlay
                bool sdk_ok = impl_->camera->connect(
                    cfg_.ip, cfg_.port, cfg_.user, cfg_.pwd, cfg_.channel,
                    /*only_login=*/true);

                // b) GPU 拉流
                bool gpu_ok = impl_->gpu_stream->Open(cfg_.rtsp_url, cfg_.gpu_device);
                impl_->gpu_open_time = Clock::now();
                impl_->sdk_fallback = false;

                LOG_COMMON("[CameraSource cam" << cfg_.cam_index
                    << "] SDK login=" << (sdk_ok ? "OK" : "FAIL")
                    << " | GPU stream=" << (gpu_ok ? "OK" : "FAIL"));

                std::this_thread::sleep_for(
                    std::chrono::milliseconds(cfg_.poll_interval_ms));
                continue;
            }

            // ====== 3) 取帧 (GPU 优先, SDK 兜底), 状态机切换 ======
            bool got = false;

            if (!impl_->sdk_fallback) {
                // ---- GPU 主路径 ----
                got = impl_->gpu_stream->GetLatestFrame(frame);

                if (IsGpuStale(*impl_->gpu_stream, impl_->gpu_open_time,
                    cfg_.gpu_cold_start_grace_ms,
                    cfg_.gpu_stale_threshold_ms)) {
                    LOG_COMMON("[CameraSource cam" << cfg_.cam_index
                        << "] GPU 流不健康 ("
                        << (impl_->gpu_stream->HasEverProduced()
                            ? std::to_string(impl_->gpu_stream->MillisSinceLastFrame()) + "ms 无新帧"
                            : std::to_string(cfg_.gpu_cold_start_grace_ms) + "ms 内无首帧")
                        << "), 切换到 SDK 软解兜底");

                    impl_->gpu_stream->Close();
                    if (impl_->camera->startRealPlay()) {
                        impl_->sdk_fallback = true;
                        impl_->last_recover_attempt = Clock::now();
                    }
                    else {
                        LOG_COMMON("[CameraSource cam" << cfg_.cam_index
                            << "] SDK 软解启动失败, 下轮再试");
                    }
                }
            }
            else {
                // ---- SDK 软解兜底中 ----
                got = impl_->camera->getLatestFrame(frame);

                // 持续探测 GPU 恢复 (用户要求: 即便回退也要尝试恢复)
                auto now = Clock::now();
                auto since_last_probe = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - impl_->last_recover_attempt).count();
                if (since_last_probe >= cfg_.gpu_recover_probe_ms) {
                    impl_->last_recover_attempt = now;
                    LOG_COMMON("[CameraSource cam" << cfg_.cam_index << "] 探测 GPU 解码恢复...");

                    impl_->gpu_stream->Open(cfg_.rtsp_url, cfg_.gpu_device);
                    impl_->gpu_open_time = Clock::now();

                    // 等首帧到达 (不阻塞过长, 否则 sink 拿不到 SDK 帧)
                    auto wait_deadline = Clock::now() +
                        std::chrono::milliseconds(cfg_.gpu_recover_wait_ms);
                    while (Clock::now() < wait_deadline &&
                        is_running_.load() &&
                        !impl_->gpu_stream->HasEverProduced()) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    }

                    if (impl_->gpu_stream->HasEverProduced()) {
                        LOG_COMMON("[CameraSource cam" << cfg_.cam_index
                            << "] GPU 已恢复, 关闭 SDK 软解");
                        impl_->camera->stopRealPlay();
                        impl_->sdk_fallback = false;
                    }
                    else {
                        LOG_COMMON("[CameraSource cam" << cfg_.cam_index
                            << "] GPU 恢复失败, 继续 SDK 软解");
                        impl_->gpu_stream->Close();
                    }
                }
            }

            // ---- 广播帧 ----
            if (got && !frame.empty()) {
                std::shared_lock<std::shared_mutex> lock(sinks_mtx_);
                for (auto* sink : sinks_) {
                    sink->OnFrame(cfg_.cam_index, frame);
                }
            }

            std::this_thread::sleep_for(
                std::chrono::milliseconds(cfg_.poll_interval_ms));
        }
        });

    LOG_COMMON("[CameraSource cam" << cfg_.cam_index << "] 已启动");
}

void CameraSource::Stop() {
    if (!is_running_.exchange(false)) return;

    want_connected_.store(false);
    if (poll_thread_.joinable()) poll_thread_.join();

    impl_->gpu_stream->Close();
    impl_->camera->disconnect();
    LOG_COMMON("[CameraSource cam" << cfg_.cam_index << "] 已停止");
}

void CameraSource::RequestConnect() { want_connected_.store(true); }
void CameraSource::RequestDisconnect() { want_connected_.store(false); }