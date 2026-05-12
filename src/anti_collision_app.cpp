#include "anti_collision_app.h"

#include <chrono>
#include <sstream>

#include "plc_register_map.h"
#include "utils.h"   // LOG_AC / LOG_COMMON / SAFE_LOG: 带时间戳 + 落盘 + 多线程安全

AntiCollisionApp::AntiCollisionApp() = default;
AntiCollisionApp::~AntiCollisionApp() { Stop(); }

// =========================================================================
// Configure
// =========================================================================
bool AntiCollisionApp::Configure(const AntiCollisionAppConfig& cfg,
    PlcReceiveBuffer* rcv_buffer,
    PlcSendBuffer* send_buffer) {
    if (configured_) {
        LOG_AC("[App] Configure 只能调用一次");
        return false;
    }
    if (running_.load()) {
        LOG_AC("[App] 不能在 Start 之后再 Configure");
        return false;
    }

    // ---- 校验 ----
    for (int i = 0; i < 4; ++i) {
        const auto& r = cfg.regions[i];
        if (r.quad.size() != 4 || r.frame_width <= 0 || r.frame_height <= 0) {
            LOG_AC("[App] cam" << (i + 1) << " 区域配置无效");
            return false;
        }
    }
    if (cfg.detector.model_path.empty()) {
        LOG_AC("[App] detector.model_path 不能为空");
        return false;
    }

    cfg_ = cfg;
    rcv_buffer_ = rcv_buffer;
    send_buffer_ = send_buffer;

    // ---- 装配第三层 (消费端) ----
    AntiCollisionConfig acs_cfg;
    acs_cfg.detector = cfg.detector;
    acs_cfg.regions = cfg.regions;
    acs_cfg.split_ratio = cfg.split_ratio;
    acs_cfg.retain_days = cfg.retain_days;
    acs_cfg.enable_debug_show = cfg.enable_debug_show;
    acs_cfg.conf_threshold = cfg.conf_threshold;
    acs_cfg.intrusion_pixel_thresh = cfg.intrusion_pixel_thresh;
    acs_cfg.alarm_hold_ms = cfg.alarm_hold_ms;
    acs_cfg.snapshot_dir = cfg.snapshot_dir;
    acs_cfg.cleanup_dirs = cfg.cleanup_dirs;

    acs_ = std::make_unique<AntiCollisionSystem>(acs_cfg);

    // ---- 装配第一层 (生产端) ----
    for (int i = 0; i < 4; ++i) {
        CameraSourceConfig s_cfg;
        s_cfg.ip = cfg.cameras[i].ip;
        s_cfg.port = cfg.cameras[i].port;
        s_cfg.user = cfg.cameras[i].user;
        s_cfg.pwd = cfg.cameras[i].pwd;
        s_cfg.channel = cfg.cameras[i].channel;
        s_cfg.cam_index = i;
        s_cfg.poll_interval_ms = cfg.poll_interval_ms;
        s_cfg.reconnect_interval_ms = cfg.reconnect_interval_ms;
        s_cfg.auto_connect = cfg.auto_connect;

        sources_[i] = std::make_unique<CameraSource>(s_cfg);
        sources_[i]->AddSink(acs_.get());
    }

    configured_ = true;
    LOG_AC("[App] 配置完成");
    return true;
}

// =========================================================================
// Start
// =========================================================================
bool AntiCollisionApp::Start() {
    if (!configured_) {
        LOG_AC("[App] 必须先 Configure 再 Start");
        return false;
    }
    if (running_.exchange(true)) {
        LOG_AC("[App] 已经在运行");
        return false;
    }

    // ---- 1. 启动第三层 (消费端) ----
    acs_->Start(&state_);

    // ---- 2. 启动第一层 (生产端) ----
    for (auto& src : sources_) src->Start();

    // ---- 3. 启动 PLC 发布线程（防撞结果 → PlcSendBuffer）----
    // 即使 send_buffer_ 为 nullptr 也照样起线程, 内部判空跳过写入 —
    // 这样即便没接 PLC, App 自身的状态查询接口仍然按预期工作.
    plc_publish_thread_ = std::thread(
        &AntiCollisionApp::PlcPublishLoop, this);

    LOG_AC("[App] 已启动");
    return true;
}

// =========================================================================
// Stop
// =========================================================================
void AntiCollisionApp::Stop() {
    if (!running_.exchange(false)) return;

    // 停发布线程
    if (plc_publish_thread_.joinable()) plc_publish_thread_.join();

    // 顺序：先停生产端, 确保不再有新帧灌入消费端; 再停消费端
    for (auto& src : sources_) {
        if (src) src->Stop();
    }
    if (acs_) acs_->Stop();

    // 共享 buffer 由外部 PlcIoManager 管理, 这里不释放也不置空
    // (但 dangling 风险由外部寿命约定保证: PlcIoManager 必须晚于本 App 析构)

    LOG_AC("[App] 已停止");
}

// =========================================================================
// PLC 发布循环
// =========================================================================
void AntiCollisionApp::PlcPublishLoop() {
    while (running_.load()) {
        PublishToPlc();
        std::this_thread::sleep_for(
            std::chrono::milliseconds(cfg_.plc_publish_interval_ms));
    }
}

// =========================================================================
// 方位映射 —— 关键业务参数,已按现场确认配置
//
// 现场约定（已确认）：
//   - 大车坐标系: 左 = 东, 右 = 西
//   - json 里 cam1~cam4 即 海左/海右/陆左/陆右
//
// 因此对应关系：
//   cam0 (海左) → 海东 → 东向相机
//   cam1 (海右) → 海西 → 西向相机
//   cam2 (陆左) → 陆东 → 东向相机
//   cam3 (陆右) → 陆西 → 西向相机
//
// 聚合规则: 同侧任一相机报警 → 该方向触发动作 (OR 逻辑)
//
// ? 如果未来调换某路相机或现场坐标系变化,只改下面三个常量即可
// =========================================================================
namespace {

    // 4 路相机分别对应哪个方位的 PLC 通讯状态接口
    // 0=海东, 1=海西, 2=陆东, 3=陆西 (与 send_*Camera_status 的物理含义对应)
    constexpr int kCamToPlcSlot[4] = {
        /* cam0 海左 */ 0,   // → 海东
        /* cam1 海右 */ 1,   // → 海西
        /* cam2 陆左 */ 2,   // → 陆东
        /* cam3 陆右 */ 3,   // → 陆西
    };

    // 4 路相机分别属于"东向"还是"西向"运动方向
    // 0=东向, 1=西向 (对应 PlcSend::StopDir / SpeedLimitDir 的方向编码)
    constexpr int kCamToDirection[4] = {
        /* cam0 海左 */ 0,   // → 东向
        /* cam1 海右 */ 1,   // → 西向
        /* cam2 陆左 */ 0,   // → 东向
        /* cam3 陆右 */ 1,   // → 西向
    };

    // 聚合规则：东侧两个相机任一报警即触发东向动作 (OR 逻辑)
    // 如果现场需要 AND 逻辑,把下面 || 改成 &&
    constexpr bool kUseOrAggregation = true;

}  // namespace

// =========================================================================
// 单次写入 PLC
// =========================================================================
void AntiCollisionApp::PublishToPlc() {
    // send_buffer_ 为空时(未注入或显式禁用 PLC) 仅做计算不下发, 仍保留日志
    PlcSendBuffer* sb = send_buffer_;

    // ---- 1. 读取 4 路状态（无锁原子读）----
    AlarmLevel l[4];
    bool       conn[4];
    for (int i = 0; i < 4; ++i) {
        l[i] = state_.cameras[i].level.load();
        conn[i] = sources_[i] && sources_[i]->IsStreamConnected();
    }

    // ---- 2. 按方位映射相机通讯状态, 写入 send_buffer ----
    // slot[0]=海东, slot[1]=海西, slot[2]=陆东, slot[3]=陆西
    uint16_t slot_conn[4] = { 0, 0, 0, 0 };
    for (int i = 0; i < 4; ++i) {
        slot_conn[kCamToPlcSlot[i]] = conn[i] ? 1 : 0;
    }
    if (sb) {
        sb->Set(PlcSend::CamSeaEastStatus, slot_conn[0]);
        sb->Set(PlcSend::CamSeaWestStatus, slot_conn[1]);
        sb->Set(PlcSend::CamLandEastStatus, slot_conn[2]);
        sb->Set(PlcSend::CamLandWestStatus, slot_conn[3]);
    }

    // ---- 3. 按方向聚合 stop / decel ----
    bool dir_stop[2] = { false, false };
    bool dir_decel[2] = { false, false };

    bool any_stop_seen[2] = { false, false };
    bool all_stop_seen[2] = { true,  true };
    bool any_decel_seen[2] = { false, false };
    bool all_decel_seen[2] = { true,  true };
    int  cam_count[2] = { 0, 0 };

    for (int i = 0; i < 4; ++i) {
        int dir = kCamToDirection[i];
        cam_count[dir]++;
        bool s = (l[i] == AlarmLevel::Stop);
        bool d = (l[i] == AlarmLevel::Decel) || s;
        any_stop_seen[dir] = any_stop_seen[dir] || s;
        all_stop_seen[dir] = all_stop_seen[dir] && s;
        any_decel_seen[dir] = any_decel_seen[dir] || d;
        all_decel_seen[dir] = all_decel_seen[dir] && d;
    }

    for (int dir = 0; dir < 2; ++dir) {
        if (cam_count[dir] == 0) continue;
        dir_stop[dir] = kUseOrAggregation ? any_stop_seen[dir] : all_stop_seen[dir];
        dir_decel[dir] = kUseOrAggregation ? any_decel_seen[dir] : all_decel_seen[dir];
    }

    // ---- 4. 编码 + 写 send_buffer ----
    // 协议: 0=无, 1=东, 2=西, 3=两侧/双向
    uint16_t stop_code =
        (dir_stop[0] && dir_stop[1]) ? 3
        : dir_stop[0] ? 1
        : dir_stop[1] ? 2
        : 0;

    bool decel_only_e = dir_decel[0] && !dir_stop[0];
    bool decel_only_w = dir_decel[1] && !dir_stop[1];
    uint16_t speed_code =
        (decel_only_e && decel_only_w) ? 3
        : decel_only_e ? 1
        : decel_only_w ? 2
        : 0;

    if (sb) {
        sb->Set(PlcSend::StopDir, stop_code);
        sb->Set(PlcSend::SpeedLimitDir, speed_code);
    }

    // ---- 5. 状态变化时打印日志 ----
    auto code_to_zone = [](uint16_t c) -> const char* {
        return (c == 0) ? "无"
            : (c == 1) ? "东侧"
            : (c == 2) ? "西侧"
            : "双向";
        };
    if (stop_code != last_stop_code_ || speed_code != last_speed_code_) {
        LOG_AC("[PLC 汇总下发] 急停区域: [" << code_to_zone(stop_code)
            << "] | 减速区域: [" << code_to_zone(speed_code) << "]");
        last_stop_code_ = stop_code;
        last_speed_code_ = speed_code;
    }

    // 注：DistEast / DistWest 暂不实现 —— 当前算法只输出报警等级, 不输出物体距离.
}

// =========================================================================
// 状态查询接口
// =========================================================================
AlarmLevel AntiCollisionApp::GetAlarmLevel(int cam_index) const {
    if (cam_index < 0 || cam_index >= 4) return AlarmLevel::Safe;
    return state_.cameras[cam_index].level.load();
}

bool AntiCollisionApp::IsStreamConnected(int cam_index) const {
    if (cam_index < 0 || cam_index >= 4) return false;
    if (!sources_[cam_index]) return false;
    return sources_[cam_index]->IsStreamConnected();
}

IPtzControl* AntiCollisionApp::Ptz(int cam_index) {
    if (cam_index < 0 || cam_index >= 4) return nullptr;
    if (!sources_[cam_index]) return nullptr;
    return sources_[cam_index]->Ptz();
}

void AntiCollisionApp::RequestConnect(int cam_index) {
    if (cam_index < 0 || cam_index >= 4) return;
    if (sources_[cam_index]) sources_[cam_index]->RequestConnect();
}

void AntiCollisionApp::RequestDisconnect(int cam_index) {
    if (cam_index < 0 || cam_index >= 4) return;
    if (sources_[cam_index]) sources_[cam_index]->RequestDisconnect();
}

void AntiCollisionApp::AttachSink(int cam_index, FrameSink* sink) {
    if (cam_index < 0 || cam_index >= 4) return;
    if (sources_[cam_index]) sources_[cam_index]->AddSink(sink);
}

void AntiCollisionApp::DetachSink(int cam_index, FrameSink* sink) {
    if (cam_index < 0 || cam_index >= 4) return;
    if (sources_[cam_index]) sources_[cam_index]->RemoveSink(sink);
}