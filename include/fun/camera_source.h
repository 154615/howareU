#pragma once

#include <atomic>
#include <memory>
#include <shared_mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>

#include "frame_sink.h"
#include "i_ptz_control.h"

// =========================================================================
// camera_source.h —— 单路相机生产端驱动(第一层)
// -------------------------------------------------------------------------
// 模块定位:
//   "驱动层"对外门面. 把"拉流 + 解码 + PTZ + 自动重连 + 多消费者分发"
//   全部封进一个对象, 上层只看到 FrameSink 和 IPtzControl 两个抽象.
//
// 内部实现(对上层不可见, 通过 pImpl 隐藏):
//   - 取流: OpenCV cudacodec GPU 硬解 RTSP        (主路径, 见 RtspGpuStream)
//   - PTZ : 海康 SDK                              (HikvisionCamera, only_login)
//   - 兜底: GPU 解码失败时自动切到 SDK 软解, 同时持续探测 GPU 恢复
//
// 装配示例:
//     CameraSourceConfig cfg;
//     cfg.ip = "192.168.1.64"; cfg.user = "admin"; cfg.pwd = "xxx";
//     cfg.channel   = 1;
//     cfg.cam_index = 0;
//
//     CameraSource src(cfg);
//     src.AddSink(&anti_collision);   // 第三层算法消费者
//     src.AddSink(&recorder);         // 录像
//     src.Start();
//     ...
//     if (auto* ptz = src.Ptz()) ptz->Move(5.0f, 0.0f);
//     ...
//     src.Stop();
//
// 三层接口契约:
//   - 上层只通过 FrameSink::OnFrame 接收帧, 通过 IPtzControl 控制云台.
//   - CameraSource 不知道帧最终被谁消费, 也不知道云台命令是哪个业务发的.
// =========================================================================

// -------------------------------------------------------------------------
// CameraSourceConfig —— 单路相机的全部启动参数
//
// 一切运行期可调的开关都集中在这里, Start() 之后不应再改.
// -------------------------------------------------------------------------
struct CameraSourceConfig {
    // ===== 海康 SDK 连接信息(PTZ 必需; 软解兜底也用这套) =====
    std::string ip;                         // 设备 IP, 如 "192.168.1.64"
    int         port = 8000;             // SDK 登录端口(海康默认 8000, 与 RTSP 的 554 不同)
    std::string user = "admin";          // 登录账号
    std::string pwd;                        // 登录密码(必填)
    int         channel = 1;                // 通道号(球机一般为 1)

    // ===== 该路相机在订阅消费者眼中的逻辑索引 =====
    // sink->OnFrame(cam_index, frame) 会带上这个值, 算法层用它选区域、选缓冲槽
    int         cam_index = 0;

    // ===== GPU 取流参数(可选, 不填走默认) =====
    // rtsp_url 留空时按海康主码流自动拼:
    //     rtsp://user:pwd@ip:554/Streaming/Channels/{channel}01
    // 大华 / 通用 IPC 请显式填这个字段
    std::string rtsp_url;
    int         gpu_device = 0;             // CUDA 设备号(多卡时用)

    // ===== 海康 SDK 登录用途(每路相机独立配置) =====
    // 控制是否调用 HikvisionCamera::connect, 三者只要任一为 true 就登录 SDK,
    // 全部为 false 时彻底不调海康 SDK(适用于无 PTZ 能力或非海康的通用 RTSP 源).
    //
    //   support_pan_tilt     该路相机是否支持云台旋转
    //   support_zoom         该路相机是否支持光学变焦
    //   enable_sdk_fallback  GPU 解码失败时, 是否切到 SDK 软解兜底
    //
    // 行为说明:
    //   - support_pan_tilt 决定 IPtzControl::HasPan() 的返回值;
    //     置 false 时 Move/MoveTo 返回 false, 不会向 SDK 下发转动指令.
    //   - support_zoom 决定 IPtzControl::HasZoom() 的返回值;
    //     置 false 时 Zoom() 返回 false.
    //   - 三者都为 false 时, Ptz() 返回 nullptr, 上层 if(auto* p = src->Ptz())
    //     自然跳过云台调用. 同时也不会做 SDK 软解兜底, GPU 挂了就只能等
    //     RtspGpuStream 自己重连恢复.
    //   - 只要任意一个为 true, 就会 connect(only_login=true) 登录 SDK
    //     (PTZ 命令和软解兜底共用同一份登录会话).
    bool        support_pan_tilt = false;
    bool        support_zoom = false;
    bool        enable_sdk_fallback = false;

    // ===== 轮询与重连 =====
    int         poll_interval_ms = 33;    // 主循环节拍, 33ms ≈ 30Hz
    int         reconnect_interval_ms = 3000;  // 整体重连最小间隔
    bool        auto_connect = true;  // Start() 时是否自动建立连接

    // ===== GPU 解码健康度判据(基于时间, 不是次数) =====
    //
    // 注意: 这两个判据的处理路径完全不同, 别混了:
    //   - gpu_stale_threshold_ms: "已出过首帧, 但最近这么久没新帧" → 流真的断了,
    //     无论是否启用兜底都要处理(启用兜底则切 SDK, 否则 Close+重开).
    //   - gpu_cold_start_grace_ms: "Open 后这么久仍没出过首帧" → 仅供
    //     SDK 兜底场景做"GPU 起不来, 早点切 SDK"的逃逸判定. 未启用兜底
    //     时此值不生效 —— 让 RtspGpuStream 内部 worker 自己慢慢握手即可.
    //
    // 默认值选取依据: 你这套相机 + GPU codec 实测冷启动 5~12 秒;
    // 30 秒留足余量, 又不至于让兜底切换体感太晚.
    int         gpu_cold_start_grace_ms = 30000;
    // 运行期: 已出过首帧后, 超过这个时间没新帧 → 视为流断了
    int         gpu_stale_threshold_ms = 500;
    // GPU 恢复探测间隔(兜底中, 每隔这个时间试一次重开 GPU)
    int         gpu_recover_probe_ms = 5000;
    // GPU 恢复探测时, 等首帧到达的最大等待时间
    int         gpu_recover_wait_ms = 3000;
};

// -------------------------------------------------------------------------
// CameraSource —— 单路相机生产端驱动
//
// 生命周期:
//   构造 --(可选 AddSink)--> Start() --> 运行 --> Stop() --> 析构
//   一个对象对应一路物理相机; 4 路相机就 new 4 个 CameraSource.
//
// 线程模型:
//   Start() 内部起 1 条 poll 线程; 该线程负责取帧并广播给所有 sink.
//   sink->OnFrame() 在 poll 线程里被同步调用 —— 重活要 sink 自己异步化.
//
// 线程安全:
//   - AddSink/RemoveSink/RequestConnect/RequestDisconnect: 任意线程可调
//   - Start/Stop: 只能在管理线程(通常是 main / app 线程)调用
//   - Ptz() 返回的指针线程安全(底层 HikvisionPtzAdapter 内部加锁)
//
// 所有权:
//   - sinks 不持有所有权; 调用方保证 sink 生命周期长于本对象, 或在销毁
//     本对象之前主动 RemoveSink.
//   - PTZ 句柄属于本对象, 调用方不释放.
// -------------------------------------------------------------------------
class CameraSource {
public:
    // 构造: 只复制配置, 不连相机, 不起线程.
    // 如果 cfg.rtsp_url 为空, 这里会按海康格式自动拼出 URL 存到 cfg_.
    explicit CameraSource(const CameraSourceConfig& cfg);

    // 析构: 自动调用 Stop(), 等所有线程退出后再返回.
    ~CameraSource();

    CameraSource(const CameraSource&) = delete;
    CameraSource& operator=(const CameraSource&) = delete;

    // ---------------------------------------------------------------------
    // Sink 管理(运行期可调, 线程安全)
    // ---------------------------------------------------------------------

    // 注册一个帧消费者. sink 在调用方释放前, 必须保证 RemoveSink 或本对象
    // 被销毁. 同一 sink 重复 Add 会被去重.
    // 不取所有权.
    void AddSink(FrameSink* sink);

    // 取消注册. 找不到匹配项是无操作. 不阻塞当前正在进行的 OnFrame() —
    // 极端情况下移除后还可能再被回调一次, sink 实现需保证并发安全.
    void RemoveSink(FrameSink* sink);

    // ---------------------------------------------------------------------
    // 启停
    // ---------------------------------------------------------------------

    // 启动 poll 线程. 若 cfg.auto_connect=true, 同时把"想连"标志置位.
    // 重复调用是无操作.
    // 调用线程: main / app, 不要在 sink 回调里调.
    void Start();

    // 通知 poll 线程退出, 等其 join, 关 GPU 流, 关 SDK 登录, 清状态.
    // 阻塞: 直到 poll 线程退出. 上限 ~ poll_interval_ms + 几百毫秒.
    // 幂等: 重复调用安全.
    void Stop();

    // ---------------------------------------------------------------------
    // 状态查询
    // ---------------------------------------------------------------------

    // poll 线程是否在跑. 不一定有帧.
    bool IsRunning() const { return is_running_.load(); }

    // 相机当前是否能拉到帧. GPU 流 OK 或 SDK 软解 OK 都算 true.
    bool IsStreamConnected() const;

    // 该路相机的逻辑索引(对应 cfg.cam_index)
    int CamIndex() const { return cfg_.cam_index; }

    // ---------------------------------------------------------------------
    // 主动连接控制(异步, 由 poll 线程实际执行)
    // ---------------------------------------------------------------------

    // 把"想连"标志置 true. poll 线程下个周期会去尝试连接.
    // 已连接时是无操作.
    void RequestConnect();

    // 把"想连"标志置 false. poll 线程下个周期会主动断开.
    // 已断开时是无操作. 不影响 PTZ(PTZ 走 SDK, 与流取帧解耦).
    void RequestDisconnect();

    // ---------------------------------------------------------------------
    // PTZ
    // ---------------------------------------------------------------------

    // 返回 PTZ 控制句柄.
    // 返回值:
    //     非空指针 ── 生命周期与本对象绑定; 调用方不负责释放
    //     nullptr  ── 该路不支持 PTZ; 当 cfg.support_pan_tilt 和
    //                 cfg.support_zoom 都为 false 时返回此值.
    //
    // 注:
    //   - 即使取流处于 SDK 软解兜底状态, PTZ 仍然可用 —— 两者共用一份 SDK 登录.
    //   - 即使 Ptz() 非空, 也要进一步用 HasPan()/HasZoom() 区分细分能力.
    //     例如某路只支持变焦不支持转动时, HasPan()==false, Move()/MoveTo() 失败.
    IPtzControl* Ptz();

private:
    // 启动时复制进来的配置. 后续 rtsp_url 可能被自动填充.
    CameraSourceConfig cfg_;

    // pImpl —— 把 RtspGpuStream / HikvisionCamera 等具体类型藏进 .cpp,
    // 头文件不污染海康 / OpenCV CUDA 头.
    struct Impl;
    std::unique_ptr<Impl> impl_;

    // 已注册的消费者列表; 用读写锁, 多 sink 读 vs 偶发 add/remove.
    std::vector<FrameSink*>      sinks_;
    mutable std::shared_mutex    sinks_mtx_;

    // 取帧 + 状态机 + 广播 都在这条线程里跑.
    std::thread                  poll_thread_;

    std::atomic<bool>            is_running_{ false };     // poll 线程是否应继续运行
    std::atomic<bool>            want_connected_{ false }; // 由 RequestConnect/Disconnect 改写
};