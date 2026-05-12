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
    // -------------------------------------------------------------------------
    // 与底层 HikvisionCamera 的区别:
    //   HikvisionCamera 本身永远"理论上能转能变焦"; 但物理相机不一定支持(枪机
    //   就只能变焦不能转, 半球机两者都不支持). 因此这一层加了 has_pan_/has_zoom_
    //   两个开关, 由 CameraSource 在构造时按 cfg 透传进来:
    //     - HasPan()/HasZoom() 直接返回开关值
    //     - Move/MoveTo/Zoom 在对应开关为 false 时直接返回 false,
    //       不会向 SDK 下发指令(避免在不支持的相机上发命令导致 SDK 报错)
    // =========================================================================
    class HikvisionPtzAdapter : public IPtzControl {
    public:
        HikvisionPtzAdapter(HikvisionCamera* cam, bool has_pan, bool has_zoom)
            : cam_(cam), has_pan_(has_pan), has_zoom_(has_zoom) {
        }

        bool HasPan()  const override { return has_pan_; }
        bool HasZoom() const override { return has_zoom_; }

        bool Move(float pan_deg, float tilt_deg) override {
            if (!has_pan_ || !cam_) return false;
            return cam_->setRelativeAngle(pan_deg, tilt_deg);
        }
        bool MoveTo(float pan_deg, float tilt_deg) override {
            if (!has_pan_ || !cam_) return false;
            return cam_->setAbsoluteAngle(pan_deg, tilt_deg);
        }
        bool Zoom(float multiplier) override {
            if (!has_zoom_ || !cam_) return false;
            return cam_->setAbsoluteZoom(multiplier);
        }
        void StopAll() override { if (cam_) cam_->stopAllActions(); }

    private:
        HikvisionCamera* cam_;
        bool             has_pan_;
        bool             has_zoom_;
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
        // ptz 在 CameraSource 构造时按 cfg 建; 这里不建, 因为这里看不到 cfg.
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
    // 只要支持 pan_tilt 或 zoom 中的任一项就给上层一个 IPtzControl;
    // 全不支持就保持 ptz==nullptr, Ptz() 自然返回 nullptr.
    if (cfg_.support_pan_tilt || cfg_.support_zoom) {
        impl_->ptz = std::make_unique<HikvisionPtzAdapter>(
            impl_->camera.get(),
            cfg_.support_pan_tilt,
            cfg_.support_zoom);
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

    // GPU 流是否处于"运行中变 stale"的状态.
    // 仅对"已出过首帧、但最近 stale_threshold_ms 没新帧"返回 true.
    // 冷启动期(还没出首帧)一律返回 false —— RtspGpuStream 内部的 worker
    // 自己会重试 createVideoReader, 外层不要插手, 否则会陷入
    //   "Open → 等几秒 → 误判 stale → Close 重开 → 又从头握手"
    // 的死循环.
    bool IsGpuRunningStale(const RtspGpuStream& gpu, int stale_threshold_ms) {
        if (!gpu.HasEverProduced()) return false;
        return gpu.MillisSinceLastFrame() > stale_threshold_ms;
    }

    // 冷启动超长判据(仅 SDK 兜底场景下有意义):
    //   已经 Open 超过 grace_ms 还没出过首帧 → 视为 GPU 起不来,
    //   值得切到 SDK 软解兜底.
    // 对未启用兜底的相机不应使用此判据.
    bool IsGpuColdStartTimedOut(const RtspGpuStream& gpu,
        Clock::time_point gpu_open_time,
        int grace_ms) {
        if (gpu.HasEverProduced()) return false;
        if (gpu_open_time == Clock::time_point{}) return false;
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

    // 是否需要登录海康 SDK: 任一 PTZ 能力 或 软解兜底 开启 → 必须登录;
    // 三者全关时彻底不碰 SDK, 走纯 GPU 通用 RTSP 模式.
    const bool needs_sdk_login = cfg_.support_pan_tilt
        || cfg_.support_zoom
        || cfg_.enable_sdk_fallback;

    poll_thread_ = std::thread([this, needs_sdk_login] {
        cv::Mat frame;

        while (is_running_.load()) {
            const bool want = want_connected_.load();
            const bool sdk_logged_in = impl_->camera->isLoggedIn();
            const bool gpu_alive = impl_->gpu_stream->IsStreamConnected();

            // ====== 1) 主动断开 ======
            // 这里也不能只看 gpu_alive: 冷启动期 worker 已经在跑但还没出首帧,
            // 此时 gpu_alive==false 但 RtspGpuStream 内部 is_running_ 已经是 true.
            // 用 gpu_open_time 判断: 只要发起过 Open 就需要 Close.
            const bool gpu_in_session =
                impl_->gpu_open_time != Clock::time_point{};
            if (!want && (gpu_in_session || sdk_logged_in)) {
                LOG_COMMON("[CameraSource cam" << cfg_.cam_index << "] 主动断开");
                impl_->gpu_stream->Close();
                if (sdk_logged_in) impl_->camera->disconnect();
                impl_->gpu_open_time = Clock::time_point{};  // 重置, 下次首连判据成立
                impl_->sdk_fallback = false;
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(cfg_.poll_interval_ms));
                continue;
            }

            // ====== 2) 想连但还没建立任何连接 → 首次连接 ======
            // 判据用"是否已经 Open 过 GPU 流", 而不是"GPU 是否在出帧":
            //   - RtspGpuStream::Open() 是非阻塞的, 只起 worker 线程;
            //     真正握手 + 创建 cudacodec 解码器在 worker 里, 通常 5-15s
            //     才出首帧. IsStreamConnected() 在出首帧前一直是 false.
            //   - 如果用 !gpu_alive 当判据, 冷启动这十几秒里每 reconnect_interval_ms
            //     就会再触发一次 Open(), 被 RtspGpuStream 内部幂等拦下来,
            //     徒增日志噪声, 而且这十几秒里取帧分支根本进不去.
            //   - 真正的"GPU 流挂了"情形由取帧分支的 IsGpuRunningStale 处理,
            //     那里会主动 Close() 并把 gpu_open_time 留待下次首连重置.
            //
            // gpu_open_time == epoch (默认值) ? "从未 Open 或已显式 Close",
            // 是判断"该不该走首连"的可靠信号.
            // (这里复用前面在"主动断开"判据里算过的 gpu_in_session.)
            const bool not_connected_yet =
                needs_sdk_login ? (!gpu_in_session && !sdk_logged_in)
                : (!gpu_in_session);

            if (want && not_connected_yet) {
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
                    << (needs_sdk_login
                        ? (std::string(" | SDK=") + cfg_.ip + ":"
                            + std::to_string(cfg_.port)
                            + " ch=" + std::to_string(cfg_.channel))
                        : std::string(" | SDK=disabled"))
                    << " (pan_tilt=" << (cfg_.support_pan_tilt ? 1 : 0)
                    << " zoom=" << (cfg_.support_zoom ? 1 : 0)
                    << " fallback=" << (cfg_.enable_sdk_fallback ? 1 : 0) << ")");

                // a) 海康 SDK 登录(仅当 PTZ 或 软解兜底 启用时)
                bool sdk_ok = true;   // 不需要时记为 true, 不影响后续日志判断
                if (needs_sdk_login) {
                    sdk_ok = impl_->camera->connect(
                        cfg_.ip, cfg_.port, cfg_.user, cfg_.pwd, cfg_.channel,
                        /*only_login=*/true);
                }

                // b) GPU 拉流(无论 SDK 是否登录都拉)
                bool gpu_ok = impl_->gpu_stream->Open(cfg_.rtsp_url, cfg_.gpu_device);
                impl_->gpu_open_time = Clock::now();
                impl_->sdk_fallback = false;

                if (needs_sdk_login) {
                    LOG_COMMON("[CameraSource cam" << cfg_.cam_index
                        << "] SDK login=" << (sdk_ok ? "OK" : "FAIL")
                        << " | GPU stream=" << (gpu_ok ? "OK" : "FAIL"));
                }
                else {
                    LOG_COMMON("[CameraSource cam" << cfg_.cam_index
                        << "] GPU stream=" << (gpu_ok ? "OK" : "FAIL")
                        << " (SDK 跳过)");
                }

                std::this_thread::sleep_for(
                    std::chrono::milliseconds(cfg_.poll_interval_ms));
                continue;
            }

            // ====== 3) 取帧 (GPU 优先, SDK 兜底), 状态机切换 ======
            bool got = false;

            if (!impl_->sdk_fallback) {
                // ---- GPU 主路径 ----
                got = impl_->gpu_stream->GetLatestFrame(frame);

                // 健康判定分两种, 处理路径完全不同:
                //
                //   (a) "运行中变 stale": 已经出过首帧, 之后超过 stale_threshold_ms
                //       没新帧 → 流真的断了, 必须处理.
                //         · 启用兜底 → 切 SDK 软解
                //         · 未启用兜底 → Close+重置 gpu_open_time, 让首连分支重开
                //
                //   (b) "冷启动超时": 还没出过首帧, 但 Open 已经超过 grace_ms.
                //       这个判据的本意是"GPU 起不来, 早点切 SDK", 因此只在
                //       启用 SDK 兜底时才有意义. 未启用兜底的相机不应触发这条 —
                //       那种场景下"冷启动慢"和"GPU 起不来"无法区分, 重开只会
                //       让本来快要握手成功的 worker 从头来过, 适得其反.
                //       RtspGpuStream 内部 worker 会自己 2s 重试 createVideoReader,
                //       外层只需要静静等待.
                const bool stale_running =
                    IsGpuRunningStale(*impl_->gpu_stream,
                        cfg_.gpu_stale_threshold_ms);
                const bool cold_start_timeout =
                    cfg_.enable_sdk_fallback
                    && IsGpuColdStartTimedOut(*impl_->gpu_stream,
                        impl_->gpu_open_time,
                        cfg_.gpu_cold_start_grace_ms);

                if (stale_running || cold_start_timeout) {
                    const std::string reason =
                        stale_running
                        ? std::to_string(impl_->gpu_stream->MillisSinceLastFrame()) + "ms 无新帧"
                        : std::to_string(cfg_.gpu_cold_start_grace_ms) + "ms 内无首帧";

                    if (cfg_.enable_sdk_fallback) {
                        // 走 SDK 软解兜底(stale_running 和 cold_start_timeout 都到这)
                        LOG_COMMON("[CameraSource cam" << cfg_.cam_index
                            << "] GPU 流不健康 (" << reason
                            << "), 切换到 SDK 软解兜底");

                        impl_->gpu_stream->Close();
                        impl_->gpu_open_time = Clock::time_point{};
                        if (impl_->camera->startRealPlay()) {
                            impl_->sdk_fallback = true;
                            impl_->last_recover_attempt = Clock::now();
                        }
                        else {
                            LOG_COMMON("[CameraSource cam" << cfg_.cam_index
                                << "] SDK 软解启动失败, 下轮再试");
                        }
                    }
                    else {
                        // 未启用兜底: 只有 stale_running 能到这分支
                        // (cold_start_timeout 在上面已被 enable_sdk_fallback 短路)
                        LOG_COMMON("[CameraSource cam" << cfg_.cam_index
                            << "] GPU 流不健康 (" << reason
                            << "), 未启用 SDK 兜底, 重开 GPU 解码器");
                        impl_->gpu_stream->Close();
                        impl_->gpu_open_time = Clock::time_point{};
                        // reconnect_interval_ms 节流防止疯狂重试.
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
                        impl_->gpu_open_time = Clock::time_point{};  // 探测失败, GPU 又脱手
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
    if (impl_->camera->isLoggedIn()) impl_->camera->disconnect();
    LOG_COMMON("[CameraSource cam" << cfg_.cam_index << "] 已停止");
}

void CameraSource::RequestConnect() { want_connected_.store(true); }
void CameraSource::RequestDisconnect() { want_connected_.store(false); }