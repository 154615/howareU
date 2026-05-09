#include "config_loader.h"

#include <iostream>
#include <sstream>

#include "modbus_cfg.h"   // IP 默认宏
#include "utils.h"        // read_String_Json / read_Int_Json / read_Float_Json

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

    // 读 4 角点 (用于防撞)
    std::vector<cv::Point> ReadQuad(const std::string& json_path, int cam_id_1based) {
        std::vector<cv::Point> pts;
        pts.reserve(4);
        const std::string prefix = "CAM" + std::to_string(cam_id_1based) + "_PT";
        for (int i = 1; i <= 4; ++i) {
            int x = read_Int_Json(json_path, prefix + std::to_string(i) + "_X");
            int y = read_Int_Json(json_path, prefix + std::to_string(i) + "_Y");
            pts.emplace_back(x, y);
        }
        return pts;
    }

    // 通用: 读单路相机连接信息(适用于 CAMx_ 与 LIFT_CAMx_ 两种前缀)
    template <typename Entry>
    Entry ReadCameraGeneric(const std::string& json_path,
                            const std::string& prefix,
                            const std::string& default_user,
                            const std::string& default_pwd,
                            int default_port) {
        Entry e;
        e.ip = read_String_Json(json_path, prefix + "IP");

        int port = read_Int_Json(json_path, prefix + "PORT");
        e.port = (port > 0) ? port : default_port;

        std::string user = read_String_Json(json_path, prefix + "USER");
        e.user = user.empty() ? default_user : user;

        std::string pwd = read_String_Json(json_path, prefix + "PWD");
        e.pwd = pwd.empty() ? default_pwd : pwd;

        int ch = read_Int_Json(json_path, prefix + "CHANNEL");
        e.channel = (ch > 0) ? ch : 1;

        return e;
    }

    // 通用: 读检测器配置(适用于防撞 / 防吊起两种前缀)
    void ReadDetectorConfig(const std::string& json_path,
                            const std::string& prefix,
                            Yolov8DetectorConfig& d) {
        d.model_path = read_String_Json(json_path, prefix + "MODEL_PATH");

        std::string task = read_String_Json(json_path, prefix + "TASK");
        d.task = (task == "detect" || task == "Detect")
                 ? TaskType::Detect : TaskType::Seg;

        int w = read_Int_Json(json_path, prefix + "IMG_W");
        int h = read_Int_Json(json_path, prefix + "IMG_H");
        if (w > 0) d.img_width  = w;
        if (h > 0) d.img_height = h;

        float conf = read_Float_Json(json_path, prefix + "CONF");
        float nms  = read_Float_Json(json_path, prefix + "NMS");
        float msk  = read_Float_Json(json_path, prefix + "MASK");
        if (conf > 0.0f) d.conf_threshold = conf;
        if (nms  > 0.0f) d.nms_threshold  = nms;
        if (msk  > 0.0f) d.mask_threshold = msk;

        std::string classes = read_String_Json(json_path, prefix + "CLASSES");
        if (!classes.empty()) d.class_names = SplitCsv(classes);

        int use_cuda = read_Int_Json(json_path, prefix + "USE_CUDA");
        if (use_cuda == 0)      d.use_cuda = false;
        else if (use_cuda == 1) d.use_cuda = true;

        int cuda_id = read_Int_Json(json_path, prefix + "CUDA_ID");
        if (cuda_id >= 0) d.cuda_id = cuda_id;
    }

    // -------------------------------------------------------------------------
    // 子加载器: PLC IO
    // -------------------------------------------------------------------------
    void LoadPlcIo(const std::string& json_path, PlcIoManager::Config& out) {
        std::string default_ip = IP;   // modbus_cfg.h 宏

        std::string rcv  = read_String_Json(json_path, "PLC_RCV_IP");
        std::string send = read_String_Json(json_path, "PLC_SEND_IP");

        out.rcv_ip  = rcv.empty()  ? default_ip : rcv;
        out.send_ip = send.empty() ? out.rcv_ip : send;

        int rcv_iv = read_Int_Json(json_path, "PLC_RCV_INTERVAL_MS");
        if (rcv_iv > 0) out.rcv_interval_ms = rcv_iv;

        int snd_iv = read_Int_Json(json_path, "PLC_SEND_INTERVAL_MS");
        if (snd_iv > 0) out.send_interval_ms = snd_iv;

        // PLC_ENABLE: 0 → 不连 PLC, 走纯本地模式
        int en = read_Int_Json(json_path, "PLC_ENABLE");
        // 0 显式关; 其他(含 1 / 缺失) 保持默认 true
        if (en == 0) out.enable = false;
    }

    // -------------------------------------------------------------------------
    // 子加载器: 防撞 App
    //   返回 false 表示必填项缺失
    // -------------------------------------------------------------------------
    bool LoadAntiCollision(const std::string& json_path,
                           AntiCollisionAppConfig& out) {
        // ---- 检测器 ----
        ReadDetectorConfig(json_path, "DETECTOR_", out.detector);
        // 兼容旧字段 MODEL_PATH
        if (out.detector.model_path.empty()) {
            out.detector.model_path = read_String_Json(json_path, "MODEL_PATH");
        }

        // ---- 防撞业务参数 ----
        out.split_ratio = read_Float_Json(json_path, "SPLIT_RATIO");
        out.retain_days = read_Int_Json(json_path, "MAX_RETAIN_DAYS");
        out.enable_debug_show = (read_Int_Json(json_path, "DEBUG_SHOW") != 0);

        // ---- 共享凭证 ----
        std::string default_user = read_String_Json(json_path, "DEFAULT_USER");
        std::string default_pwd  = read_String_Json(json_path, "DEFAULT_PWD");
        int         default_port = read_Int_Json(json_path, "DEFAULT_PORT");
        if (default_user.empty()) default_user = "admin";
        if (default_port <= 0)    default_port = 8000;

        // ---- 4 路相机 ----
        for (int i = 0; i < 4; ++i) {
            const std::string prefix = "CAM" + std::to_string(i + 1) + "_";
            out.cameras[i] = ReadCameraGeneric<CameraEntry>(
                json_path, prefix, default_user, default_pwd, default_port);
        }

        // ---- 4 路区域 ----
        int W = static_cast<int>(read_Float_Json(json_path, "RES_X"));
        int H = static_cast<int>(read_Float_Json(json_path, "RES_Y"));
        if (W <= 0) W = 1920;
        if (H <= 0) H = 1080;

        for (int i = 0; i < 4; ++i) {
            out.regions[i].quad = ReadQuad(json_path, i + 1);
            out.regions[i].frame_width  = W;
            out.regions[i].frame_height = H;
        }

        // ---- 取流参数 ----
        int interval = read_Int_Json(json_path, "INTERVAL");
        if (interval > 0) out.poll_interval_ms = interval;

        // ---- PLC 发布周期(写入 send_buffer 的频率, 不是 modbus 下发频率) ----
        int plc_pub_interval = read_Int_Json(json_path, "PLC_PUBLISH_INTERVAL_MS");
        if (plc_pub_interval > 0) out.plc_publish_interval_ms = plc_pub_interval;

        // ---- 校验 ----
        if (out.detector.model_path.empty()) {
            std::cerr << "[ConfigLoader] DETECTOR_MODEL_PATH 为空" << std::endl;
            return false;
        }
        if (out.detector.class_names.empty()) {
            std::cerr << "[ConfigLoader] DETECTOR_CLASSES 为空" << std::endl;
            return false;
        }
        for (int i = 0; i < 4; ++i) {
            if (out.cameras[i].ip.empty()) {
                std::cerr << "[ConfigLoader] CAM" << (i + 1) << "_IP 为空" << std::endl;
                return false;
            }
            if (out.cameras[i].pwd.empty()) {
                std::cerr << "[ConfigLoader] CAM" << (i + 1) << "_PWD 为空" << std::endl;
                return false;
            }
            if (out.regions[i].quad.size() != 4) {
                std::cerr << "[ConfigLoader] cam" << (i + 1) << " 角点缺失" << std::endl;
                return false;
            }
        }
        return true;
    }

    // -------------------------------------------------------------------------
    // 子加载器: 防吊起 App
    // -------------------------------------------------------------------------
    bool LoadAntiLift(const std::string& json_path, AntiLiftAppConfig& out) {
        // ---- 检测器(LIFT_DETECTOR_*) ----
        ReadDetectorConfig(json_path, "LIFT_DETECTOR_", out.algo.detector);
        // 防吊起默认是 Detect 任务
        std::string task = read_String_Json(json_path, "LIFT_DETECTOR_TASK");
        if (task.empty()) {
            out.algo.detector.task = TaskType::Detect;
        }

        // ---- 阈值 ----
        int v;
        v = read_Int_Json(json_path, "LIFT_LIMIT_HOLE_Y");
        if (v > 0) out.algo.limit_hole_y = v;
        v = read_Int_Json(json_path, "LIFT_LIMIT_WHEEL_Y");
        if (v > 0) out.algo.limit_wheel_y = v;
        v = read_Int_Json(json_path, "LIFT_LIMIT_HORIZON_LEFT");
        if (v > 0) out.algo.limit_horizon_left = v;
        v = read_Int_Json(json_path, "LIFT_LIMIT_ROTATE_LIFT_PLT");
        if (v > 0) out.algo.limit_rotate_lift_plt = v;
        v = read_Int_Json(json_path, "LIFT_LIMIT_ROTATE_LIFT_WHEEL");
        if (v > 0) out.algo.limit_rotate_lift_wheel = v;

        // ---- 录像 ----
        int en_rec = read_Int_Json(json_path, "LIFT_ENABLE_RECORD");
        if (en_rec == 0)      out.algo.enable_record = false;
        else if (en_rec == 1) out.algo.enable_record = true;

        int fps = read_Int_Json(json_path, "LIFT_RECORD_FPS");
        if (fps > 0) out.algo.record_fps = fps;

        std::string rec_dir = read_String_Json(json_path, "LIFT_RECORD_DIR");
        if (!rec_dir.empty()) out.algo.record_dir = rec_dir;

        // ---- 调试 ----
        out.algo.enable_debug_show =
            (read_Int_Json(json_path, "LIFT_DEBUG_SHOW") != 0);

        // ---- 共享凭证(复用防撞那一组) ----
        std::string default_user = read_String_Json(json_path, "DEFAULT_USER");
        std::string default_pwd  = read_String_Json(json_path, "DEFAULT_PWD");
        int         default_port = read_Int_Json(json_path, "DEFAULT_PORT");
        if (default_user.empty()) default_user = "admin";
        if (default_port <= 0)    default_port = 8000;

        // ---- 2 路相机(LIFT_CAMx_*) ----
        for (int i = 0; i < 2; ++i) {
            const std::string prefix = "LIFT_CAM" + std::to_string(i + 1) + "_";
            out.cameras[i] = ReadCameraGeneric<AntiLiftCameraEntry>(
                json_path, prefix, default_user, default_pwd, default_port);
            // RTSP URL 字段(可选, 留空则后端按海康主码流自动拼)
            out.cameras[i].rtsp_url =
                read_String_Json(json_path, prefix + "RTSP_URL");
        }

        // ---- 取流节拍 ----
        int interval = read_Int_Json(json_path, "INTERVAL");
        if (interval > 0) out.poll_interval_ms = interval;

        // ---- 校验 ----
        if (out.algo.detector.model_path.empty()) {
            std::cerr << "[ConfigLoader] LIFT_DETECTOR_MODEL_PATH 为空" << std::endl;
            return false;
        }
        if (out.algo.detector.class_names.empty()) {
            std::cerr << "[ConfigLoader] LIFT_DETECTOR_CLASSES 为空" << std::endl;
            return false;
        }
        for (int i = 0; i < 2; ++i) {
            if (out.cameras[i].ip.empty()) {
                std::cerr << "[ConfigLoader] LIFT_CAM" << (i + 1) << "_IP 为空" << std::endl;
                return false;
            }
            // 允许只填 rtsp_url 不填 pwd 的情况(通用 RTSP 源)
            if (out.cameras[i].pwd.empty() && out.cameras[i].rtsp_url.empty()) {
                std::cerr << "[ConfigLoader] LIFT_CAM" << (i + 1)
                          << " pwd 和 rtsp_url 都为空" << std::endl;
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
    // ---- App 总开关 ----
    int en_ac   = read_Int_Json(json_path, "ENABLE_ANTI_COLLISION");
    int en_lift = read_Int_Json(json_path, "ENABLE_ANTI_LIFT");
    if (en_ac == 0)   out.enable_anti_collision = false;
    if (en_lift == 0) out.enable_anti_lift      = false;

    // ---- PLC IO(无论哪个 App 启用都需要) ----
    LoadPlcIo(json_path, out.plc_io);

    // ---- 各 App 单独加载 ----
    if (out.enable_anti_collision) {
        if (!LoadAntiCollision(json_path, out.anti_collision)) {
            std::cerr << "[ConfigLoader] 防撞 App 配置失败" << std::endl;
            return false;
        }
    }
    if (out.enable_anti_lift) {
        if (!LoadAntiLift(json_path, out.anti_lift)) {
            std::cerr << "[ConfigLoader] 防吊起 App 配置失败" << std::endl;
            return false;
        }
    }

    // ---- 概要日志 ----
    std::cout << "[ConfigLoader] 配置加载成功: " << json_path << std::endl
              << "  PLC: rcv=" << out.plc_io.rcv_ip
              << " send="     << out.plc_io.send_ip
              << " enable="   << (out.plc_io.enable ? 1 : 0) << std::endl
              << "  AntiCollision: " << (out.enable_anti_collision ? "ON" : "OFF") << std::endl
              << "  AntiLift:      " << (out.enable_anti_lift      ? "ON" : "OFF") << std::endl;

    if (out.enable_anti_collision) {
        for (int i = 0; i < 4; ++i) {
            std::cout << "    AC cam" << (i + 1)
                      << " IP=" << out.anti_collision.cameras[i].ip
                      << ":"    << out.anti_collision.cameras[i].port
                      << " ch=" << out.anti_collision.cameras[i].channel
                      << std::endl;
        }
    }
    if (out.enable_anti_lift) {
        for (int i = 0; i < 2; ++i) {
            std::cout << "    LIFT cam" << (i + 1)
                      << " IP=" << out.anti_lift.cameras[i].ip
                      << ":"    << out.anti_lift.cameras[i].port
                      << " ch=" << out.anti_lift.cameras[i].channel
                      << std::endl;
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
