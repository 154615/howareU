#include "anti_lift_app.h"

#include <chrono>
#include <iostream>
#include <sstream>

#include "plc_register_map.h"

namespace {

    template <typename... Args>
    void Log(Args&&... args) {
        std::ostringstream oss;
        (oss << ... << args);
        std::cout << oss.str() << std::endl;
    }

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
                            PlcSendBuffer*    send_buffer) {
    if (configured_) {
        Log("[AntiLiftApp] 已经 Configure 过, 忽略");
        return false;
    }
    if (cfg.algo.detector.model_path.empty()) {
        Log("[AntiLiftApp] detector.model_path 不能为空");
        return false;
    }
    for (int i = 0; i < 2; ++i) {
        if (cfg.cameras[i].ip.empty()) {
            Log("[AntiLiftApp] cam", i, " ip 为空");
            return false;
        }
        if (cfg.cameras[i].pwd.empty() && cfg.cameras[i].rtsp_url.empty()) {
            Log("[AntiLiftApp] cam", i, " 既没有 pwd 也没有 rtsp_url");
            return false;
        }
    }

    cfg_ = cfg;
    rcv_buffer_  = rcv_buffer;
    send_buffer_ = send_buffer;

    // ---- 装配第三层 ----
    algo_ = std::make_unique<AntiLiftSystem>(cfg_.algo);

    // ---- 装配第一层 ----
    for (int i = 0; i < 2; ++i) {
        CameraSourceConfig s_cfg;
        s_cfg.ip                     = cfg_.cameras[i].ip;
        s_cfg.port                   = cfg_.cameras[i].port;
        s_cfg.user                   = cfg_.cameras[i].user;
        s_cfg.pwd                    = cfg_.cameras[i].pwd;
        s_cfg.channel                = cfg_.cameras[i].channel;
        s_cfg.rtsp_url               = cfg_.cameras[i].rtsp_url;
        s_cfg.cam_index              = i;
        s_cfg.poll_interval_ms       = cfg_.poll_interval_ms;
        s_cfg.reconnect_interval_ms  = cfg_.reconnect_interval_ms;
        s_cfg.auto_connect           = cfg_.auto_connect;
        s_cfg.gpu_device             = cfg_.gpu_device;

        sources_[i] = std::make_unique<CameraSource>(s_cfg);
        sources_[i]->AddSink(algo_.get());
    }

    configured_ = true;
    Log("[AntiLiftApp] Configure OK");
    return true;
}

bool AntiLiftApp::Start() {
    if (!configured_) {
        Log("[AntiLiftApp] 未 Configure, 拒绝 Start");
        return false;
    }
    if (running_.exchange(true)) {
        Log("[AntiLiftApp] 已经在运行");
        return false;
    }

    // 启动顺序: 算法 → source → PLC 输入线程
    algo_->Start(&state_);
    for (auto& src : sources_) src->Start();

    plc_input_thread_ = std::thread(&AntiLiftApp::PlcInputLoop, this);

    Log("[AntiLiftApp] 已启动");
    return true;
}

void AntiLiftApp::Stop() {
    if (!running_.exchange(false)) return;

    if (plc_input_thread_.joinable()) plc_input_thread_.join();

    // 先停产
    for (auto& src : sources_) {
        if (src) src->Stop();
    }
    // 再停消
    if (algo_) algo_->Stop();

    Log("[AntiLiftApp] 已停止");
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
// PLC 输入轮询: 把 PLC 输入透传给算法层 + 把算法报警写到 PLC 发送缓冲
// =========================================================================
void AntiLiftApp::PlcInputLoop() {
    while (running_.load()) {
        if (rcv_buffer_) {
            auto snap = rcv_buffer_->Snapshot();

            AntiLiftPlcInputs in;
            in.pos_trolley   = snap[PlcRcv::PosTrolley];
            in.hoist_height  = snap[PlcRcv::HoistHeight];
            in.spd_hoist     = static_cast<int16_t>(snap[PlcRcv::SpdHoist]);  // 有正负
            in.lock_status   = snap[PlcRcv::LockStatus];
            in.box_landed    = snap[PlcRcv::BoxLanded];
            in.reset_signal  = snap[PlcRcv::ResetSignal];
            in.spreader_size = snap[PlcRcv::SpreaderSize];

            if (algo_) algo_->UpdatePlcInputs(in);
        }

        // 取两路相机当前 alarm; 多路并存时取"严重度更高"的一路下发 PLC
        // 优先级: SystemError(3) > DroveAway(2) > Lifted(1) > None(0)
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
            Log("[AntiLiftApp] alarm 跳变: ", AlarmStr(last_published_alarm_),
                " → ", AlarmStr(worst));
            last_published_alarm_ = worst;
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(cfg_.plc_input_poll_ms));
    }
}
