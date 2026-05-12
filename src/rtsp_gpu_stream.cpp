#include "rtsp_gpu_stream.h"

#include <climits>

#include <opencv2/cudacodec.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/core/cuda.hpp>

#include "utils.h"   // LOG_COMMON

namespace {
    constexpr int kReopenIntervalMs = 2000;   // 解码器掉了重建间隔
    constexpr int kIdleSleepMs = 5;           // 取不到帧时短暂让出 CPU

    inline int64_t NowNs() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }
}

RtspGpuStream::RtspGpuStream() = default;

RtspGpuStream::~RtspGpuStream() {
    Close();
}

bool RtspGpuStream::Open(const std::string& rtsp_url, int gpu_device) {
    if (is_running_.exchange(true)) {
        LOG_COMMON("[RtspGpuStream] 已经在运行, 忽略重复 Open");
        return true;
    }
    rtsp_url_ = rtsp_url;
    gpu_device_ = gpu_device;

    // 状态归零: 新一轮 Open 视作冷启动
    is_connected_.store(false, std::memory_order_release);
    has_ever_produced_.store(false, std::memory_order_release);
    last_frame_ns_.store(0, std::memory_order_release);

    worker_ = std::thread(&RtspGpuStream::WorkerLoop, this);
    return true;
}

void RtspGpuStream::Close() {
    if (!is_running_.exchange(false)) return;
    if (worker_.joinable()) worker_.join();

    {
        std::lock_guard<std::mutex> lock(frame_mtx_);
        latest_frame_.release();
    }
    is_connected_.store(false, std::memory_order_release);
    has_ever_produced_.store(false, std::memory_order_release);
    last_frame_ns_.store(0, std::memory_order_release);
}

int64_t RtspGpuStream::MillisSinceLastFrame() const {
    int64_t last = last_frame_ns_.load(std::memory_order_acquire);
    if (last == 0) return INT64_MAX;
    return (NowNs() - last) / 1'000'000;
}

bool RtspGpuStream::GetLatestFrame(cv::Mat& outFrame) {
    std::lock_guard<std::mutex> lock(frame_mtx_);
    if (latest_frame_.empty()) return false;
    latest_frame_.copyTo(outFrame);
    return true;
}

// =========================================================================
// 内部: 拉流线程主循环
// =========================================================================
void RtspGpuStream::WorkerLoop() {
    try {
        cv::cuda::setDevice(gpu_device_);
    }
    catch (const cv::Exception& e) {
        LOG_COMMON("[RtspGpuStream] setDevice 失败: " << e.what());
        is_running_.store(false, std::memory_order_release);
        return;
    }

    cv::Ptr<cv::cudacodec::VideoReader> reader;
    cv::cuda::GpuMat gpu_frame;
    cv::cuda::GpuMat gpu_bgr;
    cv::Mat          cpu_bgr;

    // 用 epoch (= time_point{}) 而不是 time_point::min().
    // 理由: 后续做 (now - last_open_attempt) 然后 cast 成 int64 毫秒, 而
    //       time_point::min() 量级是 -INT64_MAX, 减法结果不在 int64 范围 → UB.
    //       实测会得到接近 INT64_MIN 的负数, 导致 elapsed < kReopenIntervalMs
    //       恒成立, 死循环 sleep 而不调 createVideoReader.
    auto last_open_attempt = std::chrono::steady_clock::time_point{};

    while (is_running_.load(std::memory_order_acquire)) {

        // ---- 没有 reader 就尝试建 ----
        if (!reader) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - last_open_attempt).count();
            if (elapsed < kReopenIntervalMs) {
                std::this_thread::sleep_for(std::chrono::milliseconds(kIdleSleepMs * 5));
                continue;
            }
            last_open_attempt = now;

            try {
                reader = cv::cudacodec::createVideoReader(rtsp_url_);
                if (!reader) {
                    LOG_COMMON("[RtspGpuStream] createVideoReader 返回 null: " << rtsp_url_);
                    continue;
                }
#if (CV_VERSION_MAJOR > 4) || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 5)
                try { reader->set(cv::cudacodec::ColorFormat::BGR); }
                catch (...) { /* 老版本忽略 */ }
#endif
                LOG_COMMON("[RtspGpuStream] 已打开 GPU 解码器: " << rtsp_url_);
            }
            catch (const cv::Exception& e) {
                LOG_COMMON("[RtspGpuStream] 打开失败: " << e.what());
                reader.release();
                is_connected_.store(false, std::memory_order_release);
                continue;
            }
        }

        // ---- 拉一帧 ----
        bool ok = false;
        try {
            ok = reader->nextFrame(gpu_frame);
        }
        catch (const cv::Exception& e) {
            LOG_COMMON("[RtspGpuStream] nextFrame 异常: " << e.what());
            ok = false;
        }

        if (!ok || gpu_frame.empty()) {
            LOG_COMMON("[RtspGpuStream] 拉帧失败, 准备重连");
            reader.release();
            is_connected_.store(false, std::memory_order_release);
            continue;
        }

        // ---- 颜色转 BGR ----
        if (gpu_frame.channels() == 4) {
            cv::cuda::cvtColor(gpu_frame, gpu_bgr, cv::COLOR_BGRA2BGR);
        }
        else {
            gpu_bgr = gpu_frame;
        }

        // ---- 下载并发布 ----
        gpu_bgr.download(cpu_bgr);
        {
            std::lock_guard<std::mutex> lock(frame_mtx_);
            cpu_bgr.copyTo(latest_frame_);
        }

        // 发布成功: 更新时间戳和"是否出过首帧"标志
        last_frame_ns_.store(NowNs(), std::memory_order_release);
        if (!has_ever_produced_.load(std::memory_order_acquire)) {
            has_ever_produced_.store(true, std::memory_order_release);
        }
        if (!is_connected_.load(std::memory_order_acquire)) {
            is_connected_.store(true, std::memory_order_release);
        }
    }

    reader.release();
    is_connected_.store(false, std::memory_order_release);
}