#include "config_loader.h"

#include <iostream>
#include <sstream>

#include "modbus_cfg.h"   // IP 默认宏
#include "utils.h"        // LoadJsonFile / GetJsonString / GetJsonInt / GetJsonFloat / HasMember / LOG_COMMON

// =========================================================================
// 内部工具
// =========================================================================
namespace {

    // 把 "a,b,c" 切成 {"a","b","c"}, 自动去前后空白; 空串返回空 vector
    std::vector<std::string> SplitCsv(const std::string& s) {
        std::vector<std::string> out;
        std::string item;
        std::stringstream ss(s);
        while (std::getline(ss, item, ',')) {
            auto l = item.find_first_not_of(" \t");
            auto r = item.find_last_not_of(" \t");
            if (l == std::string::npos) continue;
            out.emplace_back(item.substr(l, r - l + 1));
        }
        return out;
    }

    // 读 4 角点 (REGION 子节点)
    std::vector<cv::Point> ReadQuad(const Json::Value& region_node) {
        std::vector<cv::Point> pts;
        pts.reserve(4);
        for (int i = 1; i <= 4; ++i) {
            const std::string pref = "PT" + std::to_string(i);
            int x = GetJsonInt(region_node, pref + "_X", 0);
            int y = GetJsonInt(region_node, pref + "_Y", 0);
            pts.emplace_back(x, y);
        }
        return pts;
    }

    // 共享凭证: COMMON.DEFAULT_CAMERA
    struct DefaultCamCred {
        std::string user = "admin";
        std::string pwd;
        int         port = 8000;
    };

    DefaultCamCred ReadDefaultCred(const Json::Value& common_node) {
        DefaultCamCred d;
        const Json::Value& def = HasMember(common_node, "DEFAULT_CAMERA")
            ? common_node["DEFAULT_CAMERA"]
            : Json::Value(Json::nullValue);
        d.user = GetJsonString(def, "USER", "admin");
        d.pwd = GetJsonString(def, "PWD", "");
        int p = GetJsonInt(def, "PORT", 0);
        d.port = (p > 0) ? p : 8000;
        return d;
    }

    // 通用: 读单路相机连接信息(包括 RTSP URL + 3 个 SDK 用途开关)
    template <typename Entry>
    Entry ReadCameraGeneric(const Json::Value& cam_node,
        const DefaultCamCred& def) {
        Entry e;
        e.ip = GetJsonString(cam_node, "IP", "");

        int port = GetJsonInt(cam_node, "PORT", 0);
        e.port = (port > 0) ? port : def.port;

        std::string user = GetJsonString(cam_node, "USER", "");
        e.user = user.empty() ? def.user : user;

        std::string pwd = GetJsonString(cam_node, "PWD", "");
        e.pwd = pwd.empty() ? def.pwd : pwd;

        int ch = GetJsonInt(cam_node, "CHANNEL", 0);
        e.channel = (ch > 0) ? ch : 1;

        e.rtsp_url = GetJsonString(cam_node, "RTSP_URL", "");

        e.support_pan_tilt = (GetJsonInt(cam_node, "SUPPORT_PAN_TILT", 0) != 0);
        e.support_zoom = (GetJsonInt(cam_node, "SUPPORT_ZOOM", 0) != 0);
        e.enable_sdk_fallback = (GetJsonInt(cam_node, "ENABLE_SDK_FALLBACK", 0) != 0);

        return e;
    }

    // 通用: 读检测器配置(DETECTOR 子节点)
    void ReadDetectorConfig(const Json::Value& det_node,
        Yolov8DetectorConfig& d,
        TaskType default_task) {
        d.model_path = GetJsonString(det_node, "MODEL_PATH", d.model_path);

        std::string task = GetJsonString(det_node, "TASK", "");
        if (task.empty()) {
            d.task = default_task;
        }
        else {
            d.task = (task == "detect" || task == "Detect")
                ? TaskType::Detect : TaskType::Seg;
        }

        int w = GetJsonInt(det_node, "IMG_W", 0);
        int h = GetJsonInt(det_node, "IMG_H", 0);
        if (w > 0) d.img_width = w;
        if (h > 0) d.img_height = h;

        float conf = GetJsonFloat(det_node, "CONF", 0.0f);
        float nms = GetJsonFloat(det_node, "NMS", 0.0f);
        float msk = GetJsonFloat(det_node, "MASK", 0.0f);
        if (conf > 0.0f) d.conf_threshold = conf;
        if (nms > 0.0f) d.nms_threshold = nms;
        if (msk > 0.0f) d.mask_threshold = msk;

        std::string classes = GetJsonString(det_node, "CLASSES", "");
        if (!classes.empty()) d.class_names = SplitCsv(classes);

        // USE_CUDA / CUDA_ID: 不存在时保持默认
        if (HasMember(det_node, "USE_CUDA")) {
            d.use_cuda = (GetJsonInt(det_node, "USE_CUDA", 1) != 0);
        }
        if (HasMember(det_node, "CUDA_ID")) {
            int cuda_id = GetJsonInt(det_node, "CUDA_ID", 0);
            if (cuda_id >= 0) d.cuda_id = cuda_id;
        }
    }

    // -------------------------------------------------------------------------
    // 子加载器: PLC IO  ( COMMON.PLC )
    // -------------------------------------------------------------------------
    void LoadPlcIo(const Json::Value& common_node,
        PlcIoManager::Config& out) {
        std::string default_ip = IP;   // modbus_cfg.h 宏

        const Json::Value& plc = HasMember(common_node, "PLC")
            ? common_node["PLC"]
            : Json::Value(Json::nullValue);

        std::string rcv = GetJsonString(plc, "RCV_IP", "");
        std::string send = GetJsonString(plc, "SEND_IP", "");

        out.rcv_ip = rcv.empty() ? default_ip : rcv;
        out.send_ip = send.empty() ? out.rcv_ip : send;

        int rcv_iv = GetJsonInt(plc, "RCV_INTERVAL_MS", 0);
        if (rcv_iv > 0) out.rcv_interval_ms = rcv_iv;

        int snd_iv = GetJsonInt(plc, "SEND_INTERVAL_MS", 0);
        if (snd_iv > 0) out.send_interval_ms = snd_iv;

        // ENABLE: 0 → 不连 PLC, 走纯本地模式
        if (HasMember(plc, "ENABLE")) {
            out.enable = (GetJsonInt(plc, "ENABLE", 1) != 0);
        }
    }

    // -------------------------------------------------------------------------
    // 子加载器: 防撞 App ( ANTI_COLLISION )
    // -------------------------------------------------------------------------
    bool LoadAntiCollision(const Json::Value& ac_node,
        const DefaultCamCred& def,
        AntiCollisionAppConfig& out) {
        if (!ac_node.isObject()) {
            LOG_COMMON("[ConfigLoader] ANTI_COLLISION 节点缺失或不是 object");
            return false;
        }

        // ---- 检测器 ----
        const Json::Value& det = HasMember(ac_node, "DETECTOR")
            ? ac_node["DETECTOR"]
            : Json::Value(Json::nullValue);
        ReadDetectorConfig(det, out.detector, /*default_task=*/TaskType::Seg);

        // ---- 业务参数 ----
        const Json::Value& biz = HasMember(ac_node, "BUSINESS")
            ? ac_node["BUSINESS"]
            : Json::Value(Json::nullValue);

        float split = GetJsonFloat(biz, "SPLIT_RATIO", 0.0f);
        if (split > 0.0f) out.split_ratio = split;

        out.retain_days = GetJsonInt(biz, "MAX_RETAIN_DAYS", out.retain_days);
        out.enable_debug_show = (GetJsonInt(biz, "DEBUG_SHOW", 0) != 0);

        int interval = GetJsonInt(biz, "INTERVAL", 0);
        if (interval > 0) out.poll_interval_ms = interval;

        int plc_pub = GetJsonInt(biz, "PLC_PUBLISH_INTERVAL_MS", 0);
        if (plc_pub > 0) out.plc_publish_interval_ms = plc_pub;

        // ---- 4 路相机 ----
        int W = static_cast<int>(GetJsonFloat(biz, "RES_X", 0.0f));
        int H = static_cast<int>(GetJsonFloat(biz, "RES_Y", 0.0f));
        if (W <= 0) W = 1920;
        if (H <= 0) H = 1080;

        for (int i = 0; i < 4; ++i) {
            const std::string key = "CAM" + std::to_string(i + 1);
            if (!HasMember(ac_node, key)) {
                LOG_COMMON("[ConfigLoader] ANTI_COLLISION." << key << " 缺失");
                return false;
            }
            const Json::Value& cam = ac_node[key];
            out.cameras[i] = ReadCameraGeneric<CameraEntry>(cam, def);

            // 区域
            const Json::Value& region = HasMember(cam, "REGION")
                ? cam["REGION"]
                : Json::Value(Json::nullValue);
            out.regions[i].quad = ReadQuad(region);
            out.regions[i].frame_width = W;
            out.regions[i].frame_height = H;
        }

        // ---- 校验 ----
        if (out.detector.model_path.empty()) {
            LOG_COMMON("[ConfigLoader] ANTI_COLLISION.DETECTOR.MODEL_PATH 为空");
            return false;
        }
        if (out.detector.class_names.empty()) {
            LOG_COMMON("[ConfigLoader] ANTI_COLLISION.DETECTOR.CLASSES 为空");
            return false;
        }
        for (int i = 0; i < 4; ++i) {
            const auto& cam = out.cameras[i];
            if (cam.ip.empty()) {
                LOG_COMMON("[ConfigLoader] ANTI_COLLISION.CAM" << (i + 1) << ".IP 为空");
                return false;
            }
            // 只有需要登录 SDK 时才校验 pwd
            const bool need_sdk = cam.support_pan_tilt
                || cam.support_zoom
                || cam.enable_sdk_fallback;
            if (need_sdk && cam.pwd.empty()) {
                LOG_COMMON("[ConfigLoader] ANTI_COLLISION.CAM" << (i + 1)
                    << ".PWD 为空, 但 SUPPORT_PAN_TILT/SUPPORT_ZOOM/ENABLE_SDK_FALLBACK"
                    << " 至少一项开启, 必须填密码");
                return false;
            }
            if (out.regions[i].quad.size() != 4) {
                LOG_COMMON("[ConfigLoader] ANTI_COLLISION.CAM" << (i + 1)
                    << ".REGION 角点缺失");
                return false;
            }
        }
        return true;
    }

    // -------------------------------------------------------------------------
    // 子加载器: 防吊起 App ( ANTI_LIFT )
    // -------------------------------------------------------------------------
    bool LoadAntiLift(const Json::Value& lift_node,
        const DefaultCamCred& def,
        AntiLiftAppConfig& out) {
        if (!lift_node.isObject()) {
            LOG_COMMON("[ConfigLoader] ANTI_LIFT 节点缺失或不是 object");
            return false;
        }

        // ---- 检测器(默认 detect) ----
        const Json::Value& det = HasMember(lift_node, "DETECTOR")
            ? lift_node["DETECTOR"]
            : Json::Value(Json::nullValue);
        ReadDetectorConfig(det, out.algo.detector, /*default_task=*/TaskType::Detect);

        // ---- 算法阈值 ----
        const Json::Value& algo_lim = HasMember(lift_node, "ALGO_LIMIT")
            ? lift_node["ALGO_LIMIT"]
            : Json::Value(Json::nullValue);
        int v;
        v = GetJsonInt(algo_lim, "HOLE_Y", 0);
        if (v > 0) out.algo.limit_hole_y = v;
        v = GetJsonInt(algo_lim, "WHEEL_Y", 0);
        if (v > 0) out.algo.limit_wheel_y = v;
        v = GetJsonInt(algo_lim, "HORIZON_LEFT", 0);
        if (v > 0) out.algo.limit_horizon_left = v;
        v = GetJsonInt(algo_lim, "ROTATE_LIFT_PLT", 0);
        if (v > 0) out.algo.limit_rotate_lift_plt = v;
        v = GetJsonInt(algo_lim, "ROTATE_LIFT_WHEEL", 0);
        if (v > 0) out.algo.limit_rotate_lift_wheel = v;

        // ---- PLC 启停阈值 ----
        const Json::Value& plc_lim = HasMember(lift_node, "PLC_LIMIT")
            ? lift_node["PLC_LIMIT"]
            : Json::Value(Json::nullValue);
        out.limit_hoist_pos = GetJsonInt(plc_lim, "HOIST_POSITION", out.limit_hoist_pos);
        out.limit_trolley_pos = GetJsonInt(plc_lim, "TROLLEY_POSITION", out.limit_trolley_pos);

        // ---- 录像 ----
        const Json::Value& rec = HasMember(lift_node, "RECORD")
            ? lift_node["RECORD"]
            : Json::Value(Json::nullValue);
        if (HasMember(rec, "ENABLE")) {
            out.algo.enable_record = (GetJsonInt(rec, "ENABLE", 1) != 0);
        }
        int fps = GetJsonInt(rec, "FPS", 0);
        if (fps > 0) out.algo.record_fps = fps;
        std::string rec_dir = GetJsonString(rec, "DIR", "");
        if (!rec_dir.empty()) out.algo.record_dir = rec_dir;

        // ---- 调试 ----
        out.algo.enable_debug_show =
            (GetJsonInt(lift_node, "DEBUG_SHOW", 0) != 0);

        // ---- 取流节拍 ----
        int interval = GetJsonInt(lift_node, "INTERVAL", 0);
        if (interval > 0) out.poll_interval_ms = interval;

        // ---- 2 路相机 ----
        for (int i = 0; i < 2; ++i) {
            const std::string key = "CAM" + std::to_string(i + 1);
            if (!HasMember(lift_node, key)) {
                LOG_COMMON("[ConfigLoader] ANTI_LIFT." << key << " 缺失");
                return false;
            }
            out.cameras[i] = ReadCameraGeneric<AntiLiftCameraEntry>(
                lift_node[key], def);
        }

        // ---- 校验 ----
        if (out.algo.detector.model_path.empty()) {
            LOG_COMMON("[ConfigLoader] ANTI_LIFT.DETECTOR.MODEL_PATH 为空");
            return false;
        }
        if (out.algo.detector.class_names.empty()) {
            LOG_COMMON("[ConfigLoader] ANTI_LIFT.DETECTOR.CLASSES 为空");
            return false;
        }
        for (int i = 0; i < 2; ++i) {
            const auto& cam = out.cameras[i];
            if (cam.ip.empty()) {
                LOG_COMMON("[ConfigLoader] ANTI_LIFT.CAM" << (i + 1) << ".IP 为空");
                return false;
            }
            const bool need_sdk = cam.support_pan_tilt
                || cam.support_zoom
                || cam.enable_sdk_fallback;
            // 取流必须有路径: rtsp_url 非空 或 (SDK 兜底开启 + pwd 非空)
            if (cam.rtsp_url.empty() && cam.pwd.empty()) {
                LOG_COMMON("[ConfigLoader] ANTI_LIFT.CAM" << (i + 1)
                    << " RTSP_URL 和 PWD 都为空, 无法取流");
                return false;
            }
            if (need_sdk && cam.pwd.empty()) {
                LOG_COMMON("[ConfigLoader] ANTI_LIFT.CAM" << (i + 1)
                    << ".PWD 为空, 但 SUPPORT_PAN_TILT/SUPPORT_ZOOM/ENABLE_SDK_FALLBACK"
                    << " 至少一项开启, 必须填密码");
                return false;
            }
        }
        return true;
    }

}  // namespace


// =========================================================================
// 顶层接口
// =========================================================================
bool LoadConfigFromJson(const std::string& json_path, AppBundleConfig& out) {
    Json::Value root;
    if (!LoadJsonFile(json_path, root)) {
        LOG_COMMON("[ConfigLoader] 无法加载 " << json_path);
        return false;
    }

    // ---- COMMON ----
    const Json::Value& common_node = HasMember(root, "COMMON")
        ? root["COMMON"]
        : Json::Value(Json::nullValue);

    // App 总开关
    if (HasMember(common_node, "ENABLE_ANTI_COLLISION")) {
        out.enable_anti_collision =
            (GetJsonInt(common_node, "ENABLE_ANTI_COLLISION", 1) != 0);
    }
    if (HasMember(common_node, "ENABLE_ANTI_LIFT")) {
        out.enable_anti_lift =
            (GetJsonInt(common_node, "ENABLE_ANTI_LIFT", 1) != 0);
    }

    // PLC IO
    LoadPlcIo(common_node, out.plc_io);

    // 共享凭证
    const DefaultCamCred def_cred = ReadDefaultCred(common_node);

    // ---- 各 App ----
    if (out.enable_anti_collision) {
        const Json::Value& ac = HasMember(root, "ANTI_COLLISION")
            ? root["ANTI_COLLISION"]
            : Json::Value(Json::nullValue);
        if (!LoadAntiCollision(ac, def_cred, out.anti_collision)) {
            LOG_COMMON("[ConfigLoader] 防撞 App 配置失败");
            return false;
        }
    }
    if (out.enable_anti_lift) {
        const Json::Value& lift = HasMember(root, "ANTI_LIFT")
            ? root["ANTI_LIFT"]
            : Json::Value(Json::nullValue);
        if (!LoadAntiLift(lift, def_cred, out.anti_lift)) {
            LOG_COMMON("[ConfigLoader] 防吊起 App 配置失败");
            return false;
        }
    }

    // ---- 概要日志 ----
    LOG_COMMON("[ConfigLoader] 配置加载成功: " << json_path);
    LOG_COMMON("  PLC: rcv=" << out.plc_io.rcv_ip
        << " send=" << out.plc_io.send_ip
        << " enable=" << (out.plc_io.enable ? 1 : 0));
    LOG_COMMON("  AntiCollision: " << (out.enable_anti_collision ? "ON" : "OFF"));
    LOG_COMMON("  AntiLift:      " << (out.enable_anti_lift ? "ON" : "OFF"));

    if (out.enable_anti_collision) {
        for (int i = 0; i < 4; ++i) {
            const auto& c = out.anti_collision.cameras[i];
            LOG_COMMON("    AC cam" << (i + 1)
                << " IP=" << c.ip << ":" << c.port
                << " ch=" << c.channel
                << " pan_tilt=" << (c.support_pan_tilt ? 1 : 0)
                << " zoom=" << (c.support_zoom ? 1 : 0)
                << " fallback=" << (c.enable_sdk_fallback ? 1 : 0));
        }
    }
    if (out.enable_anti_lift) {
        for (int i = 0; i < 2; ++i) {
            const auto& c = out.anti_lift.cameras[i];
            LOG_COMMON("    LIFT cam" << (i + 1)
                << " IP=" << c.ip << ":" << c.port
                << " ch=" << c.channel
                << " pan_tilt=" << (c.support_pan_tilt ? 1 : 0)
                << " zoom=" << (c.support_zoom ? 1 : 0)
                << " fallback=" << (c.enable_sdk_fallback ? 1 : 0));
        }
    }
    return true;
}


// =========================================================================
// 兼容旧 main 接口: 单独加载防撞 App 配置
// =========================================================================
bool LoadConfigFromJson(const std::string& json_path,
    AntiCollisionAppConfig& out_cfg) {
    AppBundleConfig bundle;
    bundle.enable_anti_lift = false;   // 不要求防吊起字段存在
    if (!LoadConfigFromJson(json_path, bundle)) return false;
    out_cfg = bundle.anti_collision;
    return true;
}