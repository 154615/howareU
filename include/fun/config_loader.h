#pragma once

#include <string>

#include "anti_collision_app.h"
#include "anti_lift_app.h"
#include "plc_io.h"

// =========================================================================
// config_loader.h —— 配置加载器: 从 json 文件读出全部 App 配置
// -------------------------------------------------------------------------
// 模块定位:
//   纯工具函数; 不持有任何状态. 把 json 字段名/解析细节从 main 里抽出来,
//   未来切换 yaml / 命令行参数 / 远程下发只动这一处.
// -------------------------------------------------------------------------


// -------------------------------------------------------------------------
// AppBundleConfig —— 顶层配置容器, 同时承载防撞 / 防吊起 / PLC IO
// -------------------------------------------------------------------------
struct AppBundleConfig {
    // 是否启用各 App. 测试时可以单独开/关
    bool enable_anti_collision = true;
    bool enable_anti_lift      = true;

    // 各模块配置
    PlcIoManager::Config       plc_io;          // PLC 接收/发送配置
    AntiCollisionAppConfig     anti_collision;  // 防撞 App 配置
    AntiLiftAppConfig          anti_lift;       // 防吊起 App 配置
};


// -------------------------------------------------------------------------
// json 字段约定(全大写 + 下划线)
// -------------------------------------------------------------------------
//
// === PLC 通讯 ===
//   "PLC_RCV_IP"           string  读取目的 IP, 缺省走 modbus_cfg.h 的 IP 宏
//   "PLC_SEND_IP"          string  写入目的 IP, 缺省同 PLC_RCV_IP.
//                                  测试时填 "127.0.0.1" 实现"读真PLC, 写本地观察"
//   "PLC_RCV_INTERVAL_MS"  int     读取周期, 默认 50
//   "PLC_SEND_INTERVAL_MS" int     写入周期, 默认 50
//   "PLC_ENABLE"           int     0/1, 是否启用 PLC IO 线程, 默认 1
//
// === App 总开关 ===
//   "ENABLE_ANTI_COLLISION"   int  0/1, 默认 1
//   "ENABLE_ANTI_LIFT"        int  0/1, 默认 1
//
// === 防撞 - 检测器 ===
//   "DETECTOR_MODEL_PATH"  string  必填(兼容旧字段 "MODEL_PATH")
//   "DETECTOR_TASK"        string  "detect" 或 "seg", 默认 "seg"
//   "DETECTOR_IMG_W"       int     默认 640
//   "DETECTOR_IMG_H"       int     默认 640
//   "DETECTOR_CONF"        float   默认 0.25
//   "DETECTOR_NMS"         float   默认 0.7
//   "DETECTOR_MASK"        float   默认 0.8 (仅 seg)
//   "DETECTOR_CLASSES"     string  逗号分隔, 必填
//   "DETECTOR_USE_CUDA"    int     0/1, 默认 1
//   "DETECTOR_CUDA_ID"     int     默认 0
//
// === 防撞 - 业务 ===
//   "SPLIT_RATIO"          float   默认 0.5
//   "MAX_RETAIN_DAYS"      int     截图保留天数
//   "DEBUG_SHOW"           int     0/1
//   "INTERVAL"             int     poll 节拍 ms, 默认 33
//   "RES_X" / "RES_Y"      float   默认 1920x1080
//
// === 防撞 - 相机连接(共享凭证) ===
//   "DEFAULT_USER" / "DEFAULT_PWD" / "DEFAULT_PORT"
//
// === 防撞 - 4 路相机 ===
//   "CAMx_IP"           string  必填,  x = 1..4
//   "CAMx_PORT"         int     可选, 默认 DEFAULT_PORT
//   "CAMx_USER"         string  可选, 默认 DEFAULT_USER
//   "CAMx_PWD"          string  可选, 默认 DEFAULT_PWD
//   "CAMx_CHANNEL"      int     默认 1
//   "CAMx_PT1_X..PT4_Y" int     必填
//
// === 防吊起 - 检测器(LIFT_ 前缀) ===
//   "LIFT_DETECTOR_MODEL_PATH"   string  必填
//   "LIFT_DETECTOR_TASK"         string  推荐 "detect"
//   "LIFT_DETECTOR_IMG_W/H"      int
//   "LIFT_DETECTOR_CONF/NMS"     float
//   "LIFT_DETECTOR_CLASSES"      string  逗号分隔, 必填(类名含 hole/wheel 用作判定关键词)
//   "LIFT_DETECTOR_USE_CUDA/CUDA_ID"
//
// === 防吊起 - 阈值 ===
//   "LIFT_LIMIT_HOLE_Y"            int  锁孔垂直位移阈值
//   "LIFT_LIMIT_WHEEL_Y"           int  车轮垂直位移阈值
//   "LIFT_LIMIT_HORIZON_LEFT"      int  水平开走阈值
//   "LIFT_LIMIT_ROTATE_LIFT_PLT"   int  车架侧翻阈值
//   "LIFT_LIMIT_ROTATE_LIFT_WHEEL" int  车轮侧翻阈值
//
// === 防吊起 - 录像 ===
//   "LIFT_ENABLE_RECORD"   int     0/1, 默认 1
//   "LIFT_RECORD_FPS"      int     默认 10
//   "LIFT_RECORD_DIR"      string  默认 "./save_log/lift_record/"
//
// === 防吊起 - 2 路相机 ===
//   "LIFT_CAMx_IP/_PORT/_USER/_PWD/_CHANNEL/_RTSP_URL"   (x = 1..2)
// -------------------------------------------------------------------------


// =========================================================================
// LoadConfigFromJson —— 顶层入口, 解析整份 json 进 AppBundleConfig
// -------------------------------------------------------------------------
// 入参:
//     json_path  json 配置文件路径
//     out        [输出] 解析结果
// 返回:
//     true   解析成功
//     false  必填项缺失或文件无法打开; stderr 输出失败原因
// 阻塞: 同步, 仅 IO 时间
//
// 必填项校验:
//     - 防撞 (若启用): DETECTOR_MODEL_PATH, DETECTOR_CLASSES, 4 路 CAM
//     - 防吊起 (若启用): LIFT_DETECTOR_MODEL_PATH, LIFT_DETECTOR_CLASSES, 2 路 LIFT_CAM
//
// 用法:
//     AppBundleConfig cfg;
//     if (!LoadConfigFromJson("./config.json", cfg)) return 1;
// =========================================================================
bool LoadConfigFromJson(const std::string& json_path, AppBundleConfig& out);


// =========================================================================
// (兼容旧 main 接口) 单独加载防撞 App 配置
// 内部调用顶层 LoadConfigFromJson 后取出 anti_collision 部分.
// 旧的 main / 单元测试代码不用改.
// =========================================================================
bool LoadConfigFromJson(const std::string& json_path,
                        AntiCollisionAppConfig& out_cfg);
