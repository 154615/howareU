#pragma once

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>

#include <opencv2/opencv.hpp>

// =========================================================================
// rtsp_gpu_stream.h —— 基于 OpenCV cudacodec 的 GPU 硬解码 RTSP 拉流模块
// -------------------------------------------------------------------------
// 模块定位:
//   驱动层(第一层)的子模块. 仅负责"从 RTSP 拉视频流并 GPU 硬解, 持续
//   产出 BGR 帧". 不做 PTZ, 不做转码, 不做录像, 不做业务判定.
//
// 设计要点:
//   - 内部一条独立 worker 线程: createVideoReader -> nextFrame(GpuMat)
//     -> cuda::cvtColor(BGRA->BGR) -> download 到 CPU -> 发布到缓存槽
//   - 业务侧只持有最新一帧(覆盖式), 不做帧队列(防撞场景实时性优先)
//   - 流断了内部自己 reopen, 不抛异常; 上层用 IsStreamConnected() 观测
//   - 不持有任何海康 SDK 类型; 切换到大华 / 通用 RTSP 源不用改本文件
//
// 依赖:
//   - OpenCV 4.x with CUDA + cudacodec (依赖 NVIDIA Video Codec SDK / nvcuvid)
//   - 流必须是 H.264 / H.265 / MPEG4, cudacodec 才能硬解
//
// 典型用法:
//     RtspGpuStream s;
//     s.Open("rtsp://admin:pwd@192.168.1.64:554/Streaming/Channels/101", 0);
//     cv::Mat frame;
//     while (running) {
//         if (s.GetLatestFrame(frame)) {
//             // ... 用 frame ...
//         }
//         std::this_thread::sleep_for(std::chrono::milliseconds(33));
//     }
//     s.Close();   // 析构也会自动 Close
//
// 健康度查询(供上层做"超时回退"判断, 见 CameraSource):
//     - HasEverProduced()      自最近一次 Open() 起是否吐过至少一帧
//     - MillisSinceLastFrame() 距离最近一次发布帧的毫秒数
// =========================================================================

class RtspGpuStream {
public:
    // ---------------------------------------------------------------------
    // 构造 / 析构
    // ---------------------------------------------------------------------
    // 构造: 仅做零初始化, 不连接也不起线程. 不抛异常.
    RtspGpuStream();

    // 析构: 自动调用 Close(), 等 worker 线程 join 完后再返回.
    ~RtspGpuStream();

    // 不允许拷贝 / 赋值. worker 线程和帧缓存都不可复制语义.
    RtspGpuStream(const RtspGpuStream&) = delete;
    RtspGpuStream& operator=(const RtspGpuStream&) = delete;

    // ---------------------------------------------------------------------
    // Open() —— 启动拉流
    // ---------------------------------------------------------------------
    // 功能:
    //     起 worker 线程, 异步建立解码器并开始拉帧.
    // 入参:
    //     rtsp_url   完整 URL, 形如:
    //                  "rtsp://user:pwd@ip:554/Streaming/Channels/101"
    //     gpu_device CUDA 设备号(多卡机器才需要 != 0)
    // 返回:
    //     true  线程已起(注意首帧可能要 100~2000ms 才到达, 不代表已连上)
    //     false 重复调用 Open(), 已经在运行
    // 阻塞性: 不阻塞, 立即返回.
    // 线程安全: 不要并发调用; 同一对象上同时只能有一次连接生命周期.
    // 副作用: 把内部 has_ever_produced_ / last_frame_ns_ / is_connected_
    //         全部归零, 等价于"新一轮冷启动".
    // 失败处理:
    //     URL 错误 / 网络不通 / 编码不支持等运行期错误不在这里报, 而是
    //     在 worker 内部循环重试 + IsStreamConnected() 持续返回 false.
    bool Open(const std::string& rtsp_url, int gpu_device = 0);

    // ---------------------------------------------------------------------
    // Close() —— 停止拉流
    // ---------------------------------------------------------------------
    // 功能: 通知 worker 退出 -> join -> 释放 reader -> 清空帧缓存.
    // 阻塞性: 阻塞, 直到 worker 线程退出. 上限 ~ kReopenIntervalMs (2s).
    // 幂等性: 重复调用安全, 第二次起立即返回.
    // 调用时机: 析构会自动调; 也可在主循环里显式调用以重启拉流.
    void Close();

    // ---------------------------------------------------------------------
    // 状态查询(全部线程安全, 可任意线程读)
    // ---------------------------------------------------------------------

    // 当前是否处于"健康在线"状态. 真正能拉到帧才算 true; 流断了 / reader
    // 没建好都返回 false.
    bool IsStreamConnected() const {
        return is_connected_.load(std::memory_order_acquire);
    }

    // 自最近一次 Open() 起, 是否成功发布过至少一帧.
    // 用途: 上层区分"冷启动还没出帧" vs "出过帧之后掉了".
    // 注: Close() / 重新 Open() 后会被重置为 false.
    bool HasEverProduced() const {
        return has_ever_produced_.load(std::memory_order_acquire);
    }

    // 距离最近一次"成功发布一帧"的毫秒数.
    // 返回:
    //     INT64_MAX  worker 还没起 / 还没出过首帧
    //     >=0        正常运行的距离
    // 用途: 上层用 (now - last_frame) > 阈值 判断流是否卡住.
    int64_t MillisSinceLastFrame() const;

    // ---------------------------------------------------------------------
    // GetLatestFrame() —— 取一帧(覆盖式, 非队列)
    // ---------------------------------------------------------------------
    // 功能: 把内部缓存的最新一帧 deep-copy 到 outFrame.
    // 入参:
    //     outFrame   输出帧, BGR 三通道, CV_8UC3.
    // 返回:
    //     true  拿到了有效帧, outFrame 已被填充
    //     false 暂无可用帧(冷启动 / 已断流)
    // 阻塞性: 不阻塞, 拿到就返回, 拿不到也立刻返回.
    // 调用频率: 想多快就多快, 但取到的可能是同一帧(取决于流的 fps);
    //           典型用法是上层 30Hz 轮询.
    // 拷贝代价: 一次 BGR 全帧 deep copy(1080p 约 6MB), 调用方拿到的
    //           是独立内存, 可以随便改.
    bool GetLatestFrame(cv::Mat& outFrame);

private:
    // worker 线程主循环 —— 在 .cpp 里实现, 负责整个解码 + reopen 状态机.
    void WorkerLoop();

    // ---- 配置(Open 时填入, worker 退出前不变) ----
    std::string             rtsp_url_;     // 拉流 URL(含账号密码)
    int                     gpu_device_ = 0;  // CUDA 设备号

    // ---- 线程与状态 ----
    std::thread             worker_;            // 解码 worker
    std::atomic<bool>       is_running_{ false };  // worker 是否应继续运行
    std::atomic<bool>       is_connected_{ false };// 当前是否能拉到帧
    std::atomic<bool>       has_ever_produced_{ false }; // 自 Open 起是否出过首帧

    // 最近一次发布帧时间戳, 用 atomic int64 存 steady_clock 的纳秒计数;
    // 0 表示尚未发布过任何帧.
    std::atomic<int64_t>    last_frame_ns_{ 0 };

    // ---- 帧缓存槽(单帧覆盖式) ----
    std::mutex              frame_mtx_;     // 保护 latest_frame_
    cv::Mat                 latest_frame_;  // 最新帧, BGR / CV_8UC3
};