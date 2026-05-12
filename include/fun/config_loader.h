#pragma once

#include <string>

#include "anti_collision_app.h"
#include "anti_lift_app.h"
#include "plc_io.h"

// =========================================================================
// config_loader.h —— 配置加载器: 从分级 json 文件读出全部 App 配置
// -------------------------------------------------------------------------
// 模块定位:
//   纯工具函数; 不持有任何状态. 把 json 字段名/解析细节从 main 里抽出来,
//   未来切换 yaml / 命令行参数 / 远程下发只动这一处.
//
// 2026-05 重构: JSON 由扁平结构(CAM1_IP / DETECTOR_MODEL_PATH ...) 改为
// 分级结构(COMMON / ANTI_COLLISION / ANTI_LIFT), 详见下方字段约定.
// -------------------------------------------------------------------------


// -------------------------------------------------------------------------
// AppBundleConfig —— 顶层配置容器, 同时承载防撞 / 防吊起 / PLC IO
// -------------------------------------------------------------------------
struct AppBundleConfig {
    // 是否启用各 App. 测试时可以单独开/关
    bool enable_anti_collision = true;
    bool enable_anti_lift = true;

    // 各模块配置
    PlcIoManager::Config       plc_io;          // PLC 接收/发送配置
    AntiCollisionAppConfig     anti_collision;  // 防撞 App 配置
    AntiLiftAppConfig          anti_lift;       // 防吊起 App 配置
};


// -------------------------------------------------------------------------
// JSON 字段约定(分级)
// -------------------------------------------------------------------------
//
// 整体结构:
//   {
//     "COMMON": { ... },
//     "ANTI_COLLISION": { ... },
//     "ANTI_LIFT": { ... }
//   }
//
// === COMMON ===
//   ENABLE_ANTI_COLLISION    int   0/1, 默认 1
//   ENABLE_ANTI_LIFT         int   0/1, 默认 1
//
//   PLC: {
//     ENABLE                 int   0/1, 默认 1
//     RCV_IP                 string  默认走 modbus_cfg.h 的 IP 宏
//     SEND_IP                string  默认同 RCV_IP
//     RCV_INTERVAL_MS        int   默认 50
//     SEND_INTERVAL_MS       int   默认 50
//   }
//
//   DEFAULT_CAMERA: {
//     USER                   string  默认 "admin"
//     PWD                    string
//     PORT                   int     默认 8000
//   }
//
// === ANTI_COLLISION ===
//   DETECTOR: { MODEL_PATH(必填), CLASSES(必填, 逗号分隔),
//               TASK("detect"|"seg"), IMG_W/IMG_H, CONF, NMS, MASK,
//               USE_CUDA(0/1), CUDA_ID }
//   BUSINESS: { SPLIT_RATIO, MAX_RETAIN_DAYS, DEBUG_SHOW, INTERVAL,
//               RES_X, RES_Y, PLC_PUBLISH_INTERVAL_MS }
//   CAM1..CAM4: {
//     IP(必填), PORT, USER, PWD, CHANNEL, RTSP_URL,
//     SUPPORT_PAN_TILT(0/1, 默认 0),
//     SUPPORT_ZOOM(0/1, 默认 0),
//     ENABLE_SDK_FALLBACK(0/1, 默认 0),
//     REGION: { PT1_X..PT4_Y }
//   }
//
// === ANTI_LIFT ===
//   DETECTOR: 同上(默认 TASK="detect")
//   ALGO_LIMIT: { HOLE_Y, WHEEL_Y, HORIZON_LEFT, ROTATE_LIFT_PLT, ROTATE_LIFT_WHEEL }
//   PLC_LIMIT:  { HOIST_POSITION, TROLLEY_POSITION }
//   RECORD: { ENABLE(0/1), FPS, DIR }
//   DEBUG_SHOW(0/1), INTERVAL
//   CAM1..CAM2: {
//     IP(必填), PORT, USER, PWD, CHANNEL,
//     RTSP_URL(可选, 留空按海康主码流自动拼),
//     SUPPORT_PAN_TILT, SUPPORT_ZOOM, ENABLE_SDK_FALLBACK
//   }
// -------------------------------------------------------------------------


// =========================================================================
// LoadConfigFromJson —— 顶层入口, 解析整份 json 进 AppBundleConfig
// -------------------------------------------------------------------------
// 入参:
//     json_path  json 配置文件路径
//     out        [输出] 解析结果
// 返回:
//     true   解析成功
//     false  必填项缺失或文件无法打开; LOG_COMMON 输出失败原因
// 阻塞: 同步, 仅 IO 时间
// =========================================================================
bool LoadConfigFromJson(const std::string& json_path, AppBundleConfig& out);


// =========================================================================
// (兼容旧 main 接口) 单独加载防撞 App 配置
// 内部调用顶层 LoadConfigFromJson 后取出 anti_collision 部分.
// 旧的 main / 单元测试代码不用改.
// =========================================================================
bool LoadConfigFromJson(const std::string& json_path,
    AntiCollisionAppConfig& out_cfg);