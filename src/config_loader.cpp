#include "config_loader.h"

#include <iostream>
#include <sstream>

#include "utils.h"   // read_String_Json / read_Int_Json / read_Float_Json

namespace {

    // 把 "a,b,c" 切成 {"a","b","c"}, 自动去前后空白; 空串返回空 vector
    std::vector<std::string> SplitCsv(const std::string& s) {
        std::vector<std::string> out;
        std::string item;
        std::stringstream ss(s);
        while (std::getline(ss, item, ',')) {
            // trim
            auto l = item.find_first_not_of(" \t");
            auto r = item.find_last_not_of(" \t");
            if (l == std::string::npos) continue;
            out.emplace_back(item.substr(l, r - l + 1));
        }
        return out;
    }

    // 读 4 角点
    std::vector<cv::Point> ReadQuad(const std::string& json_path,
        int cam_id_1based) {
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

    // 读单路相机配置
    //   优先读 CAMx_USER / CAMx_PWD,缺省回落到 DEFAULT_USER / DEFAULT_PWD
    //   支持"4 路相机共享同一组凭证"的常见情况
    CameraEntry ReadCamera(const std::string& json_path, int cam_id_1based,
        const std::string& default_user,
        const std::string& default_pwd,
        int default_port) {
        const std::string prefix = "CAM" + std::to_string(cam_id_1based) + "_";

        CameraEntry e;
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

}  // namespace

bool LoadConfigFromJson(const std::string& json_path,
    AntiCollisionAppConfig& out) {

    // ---- 检测器配置 ----
    //   - DETECTOR_MODEL_PATH (兼容旧 MODEL_PATH)
    //   - DETECTOR_TASK         "detect" | "seg"      默认 seg
    //   - DETECTOR_IMG_W / IMG_H                       默认 640
    //   - DETECTOR_CONF / NMS / MASK                   默认 0.25 / 0.7 / 0.8
    //   - DETECTOR_CLASSES      逗号分隔字符串
    //   - DETECTOR_USE_CUDA / DETECTOR_CUDA_ID         默认 1 / 0
        {
            auto& d = out.detector;

            d.model_path = read_String_Json(json_path, "DETECTOR_MODEL_PATH");
            if (d.model_path.empty()) {
                d.model_path = read_String_Json(json_path, "MODEL_PATH");  // 兼容旧字段
            }

            std::string task = read_String_Json(json_path, "DETECTOR_TASK");
            d.task = (task == "detect" || task == "Detect") ? TaskType::Detect : TaskType::Seg;

            int w = read_Int_Json(json_path, "DETECTOR_IMG_W");
            int h = read_Int_Json(json_path, "DETECTOR_IMG_H");
            if (w > 0) d.img_width = w;
            if (h > 0) d.img_height = h;

            float conf = read_Float_Json(json_path, "DETECTOR_CONF");
            float nms = read_Float_Json(json_path, "DETECTOR_NMS");
            float msk = read_Float_Json(json_path, "DETECTOR_MASK");
            if (conf > 0.0f) d.conf_threshold = conf;
            if (nms > 0.0f) d.nms_threshold = nms;
            if (msk > 0.0f) d.mask_threshold = msk;

            std::string classes = read_String_Json(json_path, "DETECTOR_CLASSES");
            if (!classes.empty()) d.class_names = SplitCsv(classes);

            // CUDA: 用 -1 表示"未配置, 走默认"; 0/1 显式开关
            int use_cuda = read_Int_Json(json_path, "DETECTOR_USE_CUDA");
            if (use_cuda == 0) d.use_cuda = false;
            else if (use_cuda == 1) d.use_cuda = true;
            // 其他值 (含 -1 / 缺失) 保持默认 true

            int cuda_id = read_Int_Json(json_path, "DETECTOR_CUDA_ID");
            if (cuda_id >= 0) d.cuda_id = cuda_id;
        }

        // ---- 防撞业务参数 ----
        out.split_ratio = read_Float_Json(json_path, "SPLIT_RATIO");
        out.retain_days = read_Int_Json(json_path, "MAX_RETAIN_DAYS");
        out.enable_debug_show = (read_Int_Json(json_path, "DEBUG_SHOW") != 0);

        // ---- 共享凭证（默认值,单路可覆盖）----
        std::string default_user = read_String_Json(json_path, "DEFAULT_USER");
        std::string default_pwd = read_String_Json(json_path, "DEFAULT_PWD");
        int default_port = read_Int_Json(json_path, "DEFAULT_PORT");
        if (default_user.empty()) default_user = "admin";
        if (default_port <= 0)    default_port = 8000;

        // ---- 4 路相机连接信息 ----
        for (int i = 0; i < 4; ++i) {
            out.cameras[i] = ReadCamera(json_path, i + 1,
                default_user, default_pwd, default_port);
        }

        // ---- 4 路区域 ----
        int W = static_cast<int>(read_Float_Json(json_path, "RES_X"));
        int H = static_cast<int>(read_Float_Json(json_path, "RES_Y"));
        if (W <= 0) W = 1920;
        if (H <= 0) H = 1080;

        for (int i = 0; i < 4; ++i) {
            out.regions[i].quad = ReadQuad(json_path, i + 1);
            out.regions[i].frame_width = W;
            out.regions[i].frame_height = H;
        }

        // ---- 取流参数 ----
        int interval = read_Int_Json(json_path, "INTERVAL");
        if (interval > 0) out.poll_interval_ms = interval;

        // ---- PLC 配置 ----
        std::string plc_ip = read_String_Json(json_path, "PLC_IP");
        if (!plc_ip.empty()) {
            out.plc_ip = plc_ip;
        }

        int plc_pub_interval = read_Int_Json(json_path, "PLC_PUBLISH_INTERVAL_MS");
        if (plc_pub_interval > 0) out.plc_publish_interval_ms = plc_pub_interval;

        // ---- 校验 ----
        if (out.detector.model_path.empty()) {
            std::cerr << "[ConfigLoader] DETECTOR_MODEL_PATH (或 MODEL_PATH) 为空" << std::endl;
            return false;
        }
        if (out.detector.class_names.empty()) {
            std::cerr << "[ConfigLoader] DETECTOR_CLASSES 为空 (逗号分隔字符串)" << std::endl;
            return false;
        }
        for (int i = 0; i < 4; ++i) {
            if (out.cameras[i].ip.empty()) {
                std::cerr << "[ConfigLoader] CAM" << (i + 1)
                    << "_IP 为空" << std::endl;
                return false;
            }
            if (out.cameras[i].pwd.empty()) {
                std::cerr << "[ConfigLoader] CAM" << (i + 1)
                    << "_PWD 为空 (且 DEFAULT_PWD 也为空)" << std::endl;
                return false;
            }
            if (out.regions[i].quad.size() != 4) {
                std::cerr << "[ConfigLoader] cam" << (i + 1)
                    << " 角点数据缺失" << std::endl;
                return false;
            }
        }

        std::cout << "[ConfigLoader] 配置加载成功: " << json_path
            << " | PLC IP=" << out.plc_ip << std::endl;
        for (int i = 0; i < 4; ++i) {
            std::cout << "  cam" << (i + 1)
                << " IP=" << out.cameras[i].ip
                << ":" << out.cameras[i].port
                << " ch=" << out.cameras[i].channel
                << " user=" << out.cameras[i].user
                << std::endl;
        }
        return true;
}