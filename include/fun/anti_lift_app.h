#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include "anti_lift_system.h"
#include "camera_source.h"
#include "frame_sink.h"
#include "i_ptz_control.h"
#include "plc_io.h"

// =========================================================================
// anti_lift_app.h —— 防吊起应用耦合层(第二层, 与 AntiCollisionApp 平级)
// -------------------------------------------------------------------------
// 模块定位:
//   把第一层(2 × CameraSource) 和第三层(AntiLiftSystem) 装配起来,
//   通过 PLC 共享缓冲(PlcIoManager 提供)与外部交互.
//
// 与 AntiCollisionApp 的关系:
//   - 两者完全独立, 各自管自己的相机和算法
//   - 共享同一对 PlcReceiveBuffer / PlcSendBuffer (由 main 注入)
//   - 不共享任何线程
//
// 启停流程:
//   构造 → Configure(cfg, rcv_buf, send_buf) → Start() → 跑 → Stop() → 析构
//
// 装配示意(在 main 里):
//   PlcIoManager plc_io(plc_cfg);
//   plc_io.Start();
//
//   AntiLiftApp app;
//   app.Configure(lift_cfg, plc_io.RcvBuffer(), plc_io.SendBuffer());
//   app.Start();
// =========================================================================


// -------------------------------------------------------------------------
// AntiLiftCameraEntry —— 单路相机连接信息(透传给 CameraSourceConfig)
// 与 AntiCollisionApp 里的 CameraEntry 字段语义一致, 但本 App 只用 2 路.
// -------------------------------------------------------------------------
struct AntiLiftCameraEntry {
    std::string ip;                 // 设备 IP
    int         port    = 8000;
    std::string user    = "admin";
    std::string pwd;                // 必填
    int         channel = 1;
    std::string rtsp_url;           // 可选; 留空则按海康主码流自动拼
};


// -------------------------------------------------------------------------
// AntiLiftAppConfig —— 防吊起 App 的全部启动参数
// -------------------------------------------------------------------------
struct AntiLiftAppConfig {
    // ===== 算法层配置 =====
    AntiLiftConfig algo;

    // ===== 2 路相机 =====
    std::array<AntiLiftCameraEntry, 2> cameras;

    // ===== 取流参数 =====
    int  poll_interval_ms      = 33;
    int  reconnect_interval_ms = 3000;
    bool auto_connect          = true;
    int  gpu_device            = 0;

    // ===== PLC 输入轮询周期 =====
    // 多久从 PlcReceiveBuffer 取一次输入喂给算法层. 50ms 足够.
    int plc_input_poll_ms = 50;
};


// =========================================================================
// AntiLiftApp —— 防吊起应用层主体
//
// 线程模型:
//   - 自身: 1 条 PLC 输入轮询线程(plc_input_thread_)
//   - 间接: 2 × CameraSource (各 1 个 poll 线程) + AntiLiftSystem (2 worker + 1 录制)
//
// 线程安全:
//   - Configure / Start / Stop 仅由 main 调用
//   - 查询函数和 PTZ 任意线程可调
// =========================================================================
class AntiLiftApp {
public:
    // 构造: 仅做零初始化, 不读配置, 不起线程, 不连相机.
    AntiLiftApp();

    // 析构: 自动 Stop().
    ~AntiLiftApp();

    AntiLiftApp(const AntiLiftApp&)            = delete;
    AntiLiftApp& operator=(const AntiLiftApp&) = delete;

    // ---------------------------------------------------------------------
    // Configure() —— 装配阶段, Start 之前调用一次
    // ---------------------------------------------------------------------
    // 入参:
    //     cfg          全部配置(内部复制)
    //     rcv_buffer   PLC 接收缓冲(由 PlcIoManager 提供, 由 main 注入).
    //                  寿命必须 > 本 App. 不取所有权.
    //     send_buffer  PLC 发送缓冲. 同上.
    // 返回:
    //     true  配置成功
    //     false 配置非法(model_path 空 / pwd 空 / cameras 缺失) 或重复 Configure
    // 阻塞性: 不阻塞
    bool Configure(const AntiLiftAppConfig& cfg,
                   PlcReceiveBuffer* rcv_buffer,
                   PlcSendBuffer*    send_buffer);

    // ---------------------------------------------------------------------
    // Start() —— 启动所有线程
    // ---------------------------------------------------------------------
    // 启动顺序:
    //     1) ACS(算法层) Start(&state_)    → 加载模型 + 起 worker/录制
    //     2) 2 路 source.Start()            → 起 poll 线程, 开始拉流
    //     3) plc_input_thread_              → 起 PLC 输入轮询线程
    // 必须在 Configure() 之后调用.
    // 返回:
    //     true  启动成功
    //     false 未 Configure 或算法层启动失败
    bool Start();

    // ---------------------------------------------------------------------
    // Stop() —— 关闭所有线程(顺序与启动相反)
    // ---------------------------------------------------------------------
    // 1) plc_input_thread_ 退出
    // 2) 2 路 source.Stop()  (先停生产)
    // 3) 算法层 Stop()        (后停消费)
    // 阻塞: 直到所有线程退出
    // 幂等: 安全
    void Stop();

    // ---------------------------------------------------------------------
    // 状态查询(任意线程可调)
    // ---------------------------------------------------------------------

    // 取某路相机当前对外报警类型(写 PLC 用的). 越界返回 None.
    LiftAlarmType GetAlarm(int cam_index) const;

    // 某路相机当前是否能拉到帧.
    bool IsStreamConnected(int cam_index) const;

    // 直接访问 2 路状态(只读引用, 寿命与本对象一致)
    const AntiLiftState& State() const { return state_; }

    // ---------------------------------------------------------------------
    // PTZ
    // ---------------------------------------------------------------------
    // 越界 / 不支持时返回 nullptr. 返回值不释放.
    IPtzControl* Ptz(int cam_index);

    // ---------------------------------------------------------------------
    // 主动连接控制(异步)
    // ---------------------------------------------------------------------
    void RequestConnect(int cam_index);
    void RequestDisconnect(int cam_index);

    // ---------------------------------------------------------------------
    // 自定义消费者(运行期可挂)
    // ---------------------------------------------------------------------
    // 给某路 source 挂额外 sink (本 App 的 algo_ 已默认挂上, 这里是叠加).
    // 不取所有权.
    void AttachSink(int cam_index, FrameSink* sink);
    void DetachSink(int cam_index, FrameSink* sink);

private:
    // PLC 输入轮询: 周期从 rcv_buffer 取启动条件, 喂给 algo_.UpdatePlcInputs;
    // 同时把当前 alarm 写到 send_buffer 的 PlcSend::LiftAlarm.
    void PlcInputLoop();

private:
    bool                         configured_ = false;
    std::atomic<bool>            running_{false};

    AntiLiftAppConfig            cfg_;

    // ===== 第三层 + 第一层 =====
    std::unique_ptr<AntiLiftSystem>           algo_;
    AntiLiftState                             state_;
    std::array<std::unique_ptr<CameraSource>, 2> sources_;

    // ===== 注入的共享 buffer(不取所有权) =====
    PlcReceiveBuffer* rcv_buffer_  = nullptr;
    PlcSendBuffer*    send_buffer_ = nullptr;

    // ===== PLC 输入轮询线程 =====
    std::thread plc_input_thread_;

    // 上次写到 PLC 的 alarm 值(用于跳变去重日志)
    LiftAlarmType last_published_alarm_ = LiftAlarmType::None;
};
