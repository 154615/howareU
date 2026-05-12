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
// 与 AntiLiftSystem 的分工(2026-05 重构):
//   - 启停判定的全部 PLC 语义在本层. 算法层只暴露 BeginSession/EndSession
//   - 本层每路相机各跑一个独立状态机:
//       Idle → Detecting → (LockedAlarm | LockedPlate) → Idle
//   - 状态机事件源: PLC 输入(轮询) + algo 上报的 alarm/has_plate 标志
//
// 启停判定(每路相机):
//   - 启动: 着箱下降沿 + 起升 < limit_hoist_pos + 小车 < limit_trolley_pos + 闭锁=1
//   - 结束:
//       * algo 报 Lifted/DroveAway → 录像保留, 进 LockedAlarm 等复位
//       * algo 报 has_plate         → 录像删除, 进 LockedPlate 等升出检测区
//       * 起升速度 < spd_hoist_down_threshold (人工介入下降) → 录像删除, 回 Idle
//       * 起升 >= limit_hoist_pos (安全升出) → 录像保留, 回 Idle
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
// -------------------------------------------------------------------------
struct AntiLiftCameraEntry {
    std::string ip;                 // 设备 IP
    int         port = 8000;
    std::string user = "admin";
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
    int  poll_interval_ms = 33;
    int  reconnect_interval_ms = 3000;
    bool auto_connect = true;
    int  gpu_device = 0;

    // ===== PLC 输入轮询周期 =====
    int plc_input_poll_ms = 50;

    // ===== 启停判定阈值(PLC 语义, 与算法无关) =====
    // 起升位置(cm): 大于等于此值视为"已升出检测区"
    //   - 作启动必要条件: 起升 < limit_hoist_pos 才允许进会话
    //   - 作 Detecting 退出条件: 起升 >= limit_hoist_pos 视为安全退出
    //   - 作 LockedPlate 解除条件: 起升 >= limit_hoist_pos 才回 Idle
    int limit_hoist_pos = 600;

    // 小车位置(cm): 小于此值才允许启动会话
    int limit_trolley_pos = 100;

    // 起升速度阈值(mm/s, 含正负): 小于此值(更负) 视为"人工介入下降"
    // 默认 -10 → 起升速度 < -10mm/s 触发"带 10mm/s 死区抗抖的明显下降"
    int spd_hoist_down_threshold = -10;
};


// =========================================================================
// AntiLiftApp —— 防吊起应用层主体
//
// 线程模型:
//   - 自身: 1 条 PLC 输入轮询线程 (PlcInputLoop, 兼任启停状态机)
//   - 间接: 2 × CameraSource (各 1 个 poll 线程) + AntiLiftSystem (2 worker + 1 录制)
//
// 线程安全:
//   - Configure / Start / Stop 仅由 main 调用
//   - 查询函数和 PTZ 任意线程可调
// =========================================================================
class AntiLiftApp {
public:
    AntiLiftApp();
    ~AntiLiftApp();

    AntiLiftApp(const AntiLiftApp&) = delete;
    AntiLiftApp& operator=(const AntiLiftApp&) = delete;

    // ---------------------------------------------------------------------
    // Configure() —— 装配阶段, Start 之前调用一次
    // ---------------------------------------------------------------------
    bool Configure(const AntiLiftAppConfig& cfg,
        PlcReceiveBuffer* rcv_buffer,
        PlcSendBuffer* send_buffer);

    // ---------------------------------------------------------------------
    // Start() / Stop()
    // ---------------------------------------------------------------------
    bool Start();
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
    IPtzControl* Ptz(int cam_index);

    // ---------------------------------------------------------------------
    // 主动连接控制(异步)
    // ---------------------------------------------------------------------
    void RequestConnect(int cam_index);
    void RequestDisconnect(int cam_index);

    // ---------------------------------------------------------------------
    // 自定义消费者(运行期可挂)
    // ---------------------------------------------------------------------
    void AttachSink(int cam_index, FrameSink* sink);
    void DetachSink(int cam_index, FrameSink* sink);

private:
    // PLC 输入轮询 + 启停状态机
    void PlcInputLoop();

    // -------------------- 每路相机的启停状态机 --------------------
    enum class SessionState : uint8_t {
        Idle = 0,   // 等启动
        Detecting = 1,   // 在会话中
        LockedAlarm = 2,   // 算法判过 Lifted/DroveAway, 等复位
        LockedPlate = 3,   // 算法看到 plate, 等升出检测区
    };

    // 输入快照: 由 PlcInputLoop 取自 rcv_buffer_, 透传到下面的 step
    struct PlcSnapshot {
        uint16_t pos_trolley = 0;
        uint16_t hoist_height = 0;
        int16_t  spd_hoist = 0;
        uint16_t lock_status = 0;
        uint16_t box_landed = 0;   // 即"着箱信号"
        uint16_t reset_signal = 0;
    };

    // 每路相机的状态机推进一帧
    void StepSessionState(int cam_index, const PlcSnapshot& snap);

    // 汇总两路 alarm 写 PLC; 维护跳变日志
    void PublishAlarmToPlc();

private:
    bool                         configured_ = false;
    std::atomic<bool>            running_{ false };

    AntiLiftAppConfig            cfg_;

    // ===== 第三层 + 第一层 =====
    std::unique_ptr<AntiLiftSystem>           algo_;
    AntiLiftState                             state_;
    std::array<std::unique_ptr<CameraSource>, 2> sources_;

    // ===== 注入的共享 buffer(不取所有权) =====
    PlcReceiveBuffer* rcv_buffer_ = nullptr;
    PlcSendBuffer* send_buffer_ = nullptr;

    // ===== PLC 输入轮询线程 =====
    std::thread plc_input_thread_;

    // ===== 每路相机状态机的私有数据(只在 PlcInputLoop 线程内访问, 无需锁) =====
    struct PerCamFsm {
        SessionState state = SessionState::Idle;
        uint16_t     prev_zhuoxiang = 0;   // 上一次 box_landed, 用于下降沿检测
        bool         has_prev = false; // 首次循环时不做沿检测
    };
    std::array<PerCamFsm, 2> fsm_;

    // 上次写到 PLC 的 alarm 值(用于跳变去重日志)
    LiftAlarmType last_published_alarm_ = LiftAlarmType::None;
};