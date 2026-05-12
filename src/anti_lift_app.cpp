#include "anti_lift_app.h"

#include <chrono>
#include <sstream>

#include "plc_register_map.h"
#include "utils.h"   // LOG_LIFT

namespace {

    const char* AlarmStr(LiftAlarmType a) {
        switch (a) {
        case LiftAlarmType::Lifted:      return "Lifted";
        case LiftAlarmType::DroveAway:   return "DroveAway";
        case LiftAlarmType::SystemError: return "SystemError";
        default:                         return "None";
        }
    }

}  // namespace

AntiLiftApp::AntiLiftApp() = default;
AntiLiftApp::~AntiLiftApp() { Stop(); }

bool AntiLiftApp::Configure(const AntiLiftAppConfig& cfg,
    PlcReceiveBuffer* rcv_buffer,
    PlcSendBuffer* send_buffer) {
    if (configured_) {
        LOG_LIFT("[AntiLiftApp] 已经 Configure 过, 忽略");
        return false;
    }
    if (cfg.algo.detector.model_path.empty()) {
        LOG_LIFT("[AntiLiftApp] detector.model_path 不能为空");
        return false;
    }
    for (int i = 0; i < 2; ++i) {
        if (cfg.cameras[i].ip.empty()) {
            LOG_LIFT("[AntiLiftApp] cam" << i << " ip 为空");
            return false;
        }
        if (cfg.cameras[i].pwd.empty() && cfg.cameras[i].rtsp_url.empty()) {
            LOG_LIFT("[AntiLiftApp] cam" << i << " 既没有 pwd 也没有 rtsp_url");
            return false;
        }
    }
    if (cfg.limit_hoist_pos <= 0) {
        LOG_LIFT("[AntiLiftApp] limit_hoist_pos 非法: " << cfg.limit_hoist_pos);
        return false;
    }

    cfg_ = cfg;
    rcv_buffer_ = rcv_buffer;
    send_buffer_ = send_buffer;

    // ---- 装配第三层 ----
    algo_ = std::make_unique<AntiLiftSystem>(cfg_.algo);

    // ---- 装配第一层 ----
    for (int i = 0; i < 2; ++i) {
        CameraSourceConfig s_cfg;
        s_cfg.ip = cfg_.cameras[i].ip;
        s_cfg.port = cfg_.cameras[i].port;
        s_cfg.user = cfg_.cameras[i].user;
        s_cfg.pwd = cfg_.cameras[i].pwd;
        s_cfg.channel = cfg_.cameras[i].channel;
        s_cfg.rtsp_url = cfg_.cameras[i].rtsp_url;
        s_cfg.cam_index = i;
        s_cfg.poll_interval_ms = cfg_.poll_interval_ms;
        s_cfg.reconnect_interval_ms = cfg_.reconnect_interval_ms;
        s_cfg.auto_connect = cfg_.auto_connect;
        s_cfg.gpu_device = cfg_.gpu_device;

        sources_[i] = std::make_unique<CameraSource>(s_cfg);
        sources_[i]->AddSink(algo_.get());
    }

    configured_ = true;
    LOG_LIFT("[AntiLiftApp] Configure OK (limit_hoist_pos=" << cfg_.limit_hoist_pos
        << " limit_trolley_pos=" << cfg_.limit_trolley_pos
        << " spd_hoist_down_threshold=" << cfg_.spd_hoist_down_threshold << ")");
    return true;
}

bool AntiLiftApp::Start() {
    if (!configured_) {
        LOG_LIFT("[AntiLiftApp] 未 Configure, 拒绝 Start");
        return false;
    }
    if (running_.exchange(true)) {
        LOG_LIFT("[AntiLiftApp] 已经在运行");
        return false;
    }

    // 启动顺序: 算法 → source → PLC 输入线程
    algo_->Start(&state_);
    for (auto& src : sources_) src->Start();

    // 初始化每路状态机
    for (auto& f : fsm_) {
        f.state = SessionState::Idle;
        f.prev_zhuoxiang = 0;
        f.has_prev = false;
    }

    plc_input_thread_ = std::thread(&AntiLiftApp::PlcInputLoop, this);

    LOG_LIFT("[AntiLiftApp] 已启动");
    return true;
}

void AntiLiftApp::Stop() {
    if (!running_.exchange(false)) return;

    if (plc_input_thread_.joinable()) plc_input_thread_.join();

    // 若退出时仍在会话, 让 algo 把临时录像收掉(保留, 标 abort 字样意义不大, 简单按 keep 处理)
    for (int i = 0; i < 2; ++i) {
        if (state_.cameras[i].in_session.load() && algo_) {
            algo_->EndSession(i, /*keep_recording=*/true);
        }
    }

    // 先停产
    for (auto& src : sources_) {
        if (src) src->Stop();
    }
    // 再停消
    if (algo_) algo_->Stop();

    LOG_LIFT("[AntiLiftApp] 已停止");
}

LiftAlarmType AntiLiftApp::GetAlarm(int cam_index) const {
    if (cam_index < 0 || cam_index >= 2) return LiftAlarmType::None;
    return state_.cameras[cam_index].alarm.load();
}

bool AntiLiftApp::IsStreamConnected(int cam_index) const {
    if (cam_index < 0 || cam_index >= 2) return false;
    return sources_[cam_index] && sources_[cam_index]->IsStreamConnected();
}

IPtzControl* AntiLiftApp::Ptz(int cam_index) {
    if (cam_index < 0 || cam_index >= 2) return nullptr;
    if (!sources_[cam_index]) return nullptr;
    return sources_[cam_index]->Ptz();
}

void AntiLiftApp::RequestConnect(int cam_index) {
    if (cam_index < 0 || cam_index >= 2) return;
    if (sources_[cam_index]) sources_[cam_index]->RequestConnect();
}

void AntiLiftApp::RequestDisconnect(int cam_index) {
    if (cam_index < 0 || cam_index >= 2) return;
    if (sources_[cam_index]) sources_[cam_index]->RequestDisconnect();
}

void AntiLiftApp::AttachSink(int cam_index, FrameSink* sink) {
    if (cam_index < 0 || cam_index >= 2) return;
    if (sources_[cam_index]) sources_[cam_index]->AddSink(sink);
}

void AntiLiftApp::DetachSink(int cam_index, FrameSink* sink) {
    if (cam_index < 0 || cam_index >= 2) return;
    if (sources_[cam_index]) sources_[cam_index]->RemoveSink(sink);
}

// =========================================================================
// PLC 输入轮询: 兼任启停状态机驱动 + alarm 下发
// =========================================================================
void AntiLiftApp::PlcInputLoop() {
    while (running_.load()) {
        if (rcv_buffer_) {
            auto raw = rcv_buffer_->Snapshot();

            PlcSnapshot snap;
            snap.pos_trolley = raw[PlcRcv::PosTrolley];
            snap.hoist_height = raw[PlcRcv::HoistHeight];
            snap.spd_hoist = static_cast<int16_t>(raw[PlcRcv::SpdHoist]);
            snap.lock_status = raw[PlcRcv::LockStatus];
            snap.box_landed = raw[PlcRcv::BoxLanded];
            snap.reset_signal = raw[PlcRcv::ResetSignal];

            // 推进每路相机状态机
            for (int i = 0; i < 2; ++i) {
                StepSessionState(i, snap);
            }
        }

        // 把当前两路 alarm 汇总下发 PLC
        PublishAlarmToPlc();

        std::this_thread::sleep_for(
            std::chrono::milliseconds(cfg_.plc_input_poll_ms));
    }
}

// -------------------------------------------------------------------------
// 单路状态机推进
// -------------------------------------------------------------------------
void AntiLiftApp::StepSessionState(int cam_index, const PlcSnapshot& snap) {
    auto& fsm = fsm_[cam_index];
    auto& cam_state = state_.cameras[cam_index];

    // 着箱信号下降沿: 上一次=1 且 当前=0
    const bool edge_zhuoxiang_falling =
        fsm.has_prev && (fsm.prev_zhuoxiang == 1) && (snap.box_landed == 0);
    // 注意: prev 在状态机走完后再更新, 这里只是读

    const bool in_detect_range =
        (snap.hoist_height < cfg_.limit_hoist_pos) &&
        (snap.pos_trolley < cfg_.limit_trolley_pos);

    const bool out_of_hoist_range =
        (snap.hoist_height >= cfg_.limit_hoist_pos);

    const bool is_descending =
        (snap.spd_hoist < cfg_.spd_hoist_down_threshold);

    switch (fsm.state) {
    case SessionState::Idle: {
        // 启动条件: 着箱下降沿 + 在检测区 + 闭锁
        if (edge_zhuoxiang_falling &&
            in_detect_range &&
            (snap.lock_status == 1)) {
            if (algo_) algo_->BeginSession(cam_index);
            fsm.state = SessionState::Detecting;
        }
        break;
    }

    case SessionState::Detecting: {
        // 优先级 1: algo 已经判出报警 (Lifted / DroveAway)
        const LiftAlarmType cur_alarm = cam_state.alarm.load();
        if (cur_alarm == LiftAlarmType::Lifted ||
            cur_alarm == LiftAlarmType::DroveAway) {
            if (algo_) algo_->EndSession(cam_index, /*keep_recording=*/true);
            fsm.state = SessionState::LockedAlarm;
            LOG_LIFT("[AntiLiftApp] cam" << cam_index
                << " Detecting → LockedAlarm (alarm=" << AlarmStr(cur_alarm) << ")");
            break;
        }

        // 优先级 2: algo 看到 plate (内集卡作业)
        if (cam_state.has_plate.load()) {
            if (algo_) algo_->EndSession(cam_index, /*keep_recording=*/false);
            cam_state.alarm.store(LiftAlarmType::None);
            fsm.state = SessionState::LockedPlate;
            LOG_LIFT("[AntiLiftApp] cam" << cam_index
                << " Detecting → LockedPlate (plate seen, recording deleted)");
            break;
        }

        // 优先级 3: 人工介入下降
        if (is_descending) {
            if (algo_) algo_->EndSession(cam_index, /*keep_recording=*/false);
            cam_state.alarm.store(LiftAlarmType::None);
            fsm.state = SessionState::Idle;
            LOG_LIFT("[AntiLiftApp] cam" << cam_index
                << " Detecting → Idle (human override descent)");
            break;
        }

        // 优先级 4: 安全升出检测区
        if (out_of_hoist_range) {
            if (algo_) algo_->EndSession(cam_index, /*keep_recording=*/true);
            cam_state.alarm.store(LiftAlarmType::None);
            fsm.state = SessionState::Idle;
            LOG_LIFT("[AntiLiftApp] cam" << cam_index
                << " Detecting → Idle (safe exit, hoist >= limit)");
            break;
        }
        // 否则继续 Detecting, 算法层会持续在 OnFrame 里跑判定
        break;
    }

    case SessionState::LockedAlarm: {
        // 只有复位才解锁; 解锁后清 alarm
        if (snap.reset_signal == 1) {
            cam_state.alarm.store(LiftAlarmType::None);
            cam_state.sub_kind.store(LiftSubKind::None);
            fsm.state = SessionState::Idle;
            LOG_LIFT("[AntiLiftApp] cam" << cam_index
                << " LockedAlarm → Idle (reset)");
        }
        break;
    }

    case SessionState::LockedPlate: {
        // 严格按需求: reset 不解锁, 只有升出检测区才解锁
        if (out_of_hoist_range) {
            fsm.state = SessionState::Idle;
            LOG_LIFT("[AntiLiftApp] cam" << cam_index
                << " LockedPlate → Idle (hoist >= limit)");
        }
        break;
    }
    }

    // 更新 prev_zhuoxiang —— 注意必须放最后, 否则同一 tick 内 box_landed=0 立刻
    // 会让下一 tick 的下降沿失效, 反而无所谓; 但要保证下降沿只触发一次
    fsm.prev_zhuoxiang = snap.box_landed;
    fsm.has_prev = true;
}

// -------------------------------------------------------------------------
// 把两路 alarm 汇总写 PLC
// -------------------------------------------------------------------------
void AntiLiftApp::PublishAlarmToPlc() {
    // 严重度: SystemError(3) > DroveAway(2) > Lifted(1) > None(0)
    LiftAlarmType worst = LiftAlarmType::None;
    for (int i = 0; i < 2; ++i) {
        auto a = state_.cameras[i].alarm.load();
        if (static_cast<uint8_t>(a) > static_cast<uint8_t>(worst)) {
            worst = a;
        }
    }

    if (send_buffer_) {
        send_buffer_->Set(PlcSend::LiftAlarm, static_cast<uint16_t>(worst));
    }

    if (worst != last_published_alarm_) {
        LOG_LIFT("[AntiLiftApp] alarm 跳变: "
            << AlarmStr(last_published_alarm_) << " → " << AlarmStr(worst));
        last_published_alarm_ = worst;
    }
}