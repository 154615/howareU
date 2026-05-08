#pragma once

#include <string>

#include "anti_collision_app.h"

// =========================================================================
// config_loader.h —— 配置加载器: 从 json 文件读出 AntiCollisionAppConfig
// -------------------------------------------------------------------------
// 模块定位:
//   纯工具函数; 不持有任何状态. 把 json 字段名/解析细节从 main 里抽出来,
//   未来切换 yaml / 命令行参数 / 远程下发只动这一处.
//
// 设计哲学:
//   - main 只关心配置结构体, 不关心 json 字段名
//   - 所有字段都有合理默认值, json 里缺失时使用默认, 不会失败
//   - 仅几个"必填项"(如 model_path / IP / 角点) 缺失时返回 false
// =========================================================================


// -------------------------------------------------------------------------
// json 字段约定(命名风格: 全大写 + 下划线)
// -------------------------------------------------------------------------
//
// === 检测器 ===
//   "DETECTOR_MODEL_PATH"  string  ONNX 模型路径(必填; 兼容旧字段 "MODEL_PATH")
//   "DETECTOR_TASK"        string  "detect" 或 "seg"(默认 "seg")
//   "DETECTOR_IMG_W"       int     网络输入宽(默认 640)
//   "DETECTOR_IMG_H"       int     网络输入高(默认 640)
//   "DETECTOR_CONF"        float   类别置信度阈值, NMS 之前的(默认 0.25)
//   "DETECTOR_NMS"         float   NMS IOU 阈值(默认 0.7)
//   "DETECTOR_MASK"        float   分割 mask 二值化阈值, 仅 seg 用(默认 0.8)
//   "DETECTOR_CLASSES"     string  类别名, 逗号分隔; 必填.
//                                  形如 "driver,container20,pallet40,..."
//   "DETECTOR_USE_CUDA"    int     0=CPU, 1=GPU, 缺省=GPU
//   "DETECTOR_CUDA_ID"     int     CUDA 设备号(默认 0)
//
// === 业务 ===
//   "SPLIT_RATIO"          float   减速/急停区切分比例 0~1
//   "MAX_RETAIN_DAYS"      int     截图保留天数
//   "DEBUG_SHOW"           int     0/1, 是否开 cv::imshow 调试
//
// === 取流 ===
//   "INTERVAL"             int     poll 节拍, 毫秒(默认 33)
//   "RES_X" / "RES_Y"      float   相机出图分辨率(默认 1920x1080)
//
// === 相机凭证(共享默认值, 单路可覆盖) ===
//   "DEFAULT_USER"         string  默认账号(缺省 "admin")
//   "DEFAULT_PWD"          string  默认密码(必填, 除非每路都覆盖)
//   "DEFAULT_PORT"         int     默认端口(缺省 8000)
//
// === 4 路相机连接 + 角点(x = 1..4) ===
//   "CAMx_IP"              string  必填
//   "CAMx_PORT"            int     可选, 默认 DEFAULT_PORT
//   "CAMx_USER"            string  可选, 默认 DEFAULT_USER
//   "CAMx_PWD"             string  可选, 默认 DEFAULT_PWD; 二者都空时报错
//   "CAMx_CHANNEL"         int     通道号, 默认 1
//   "CAMx_PT1_X" .. "PT4_Y" int    4 个角点坐标; 必填
//
// === PLC ===
//   "PLC_IP"               string  PLC IP, 缺省用 modbus_cfg.h 的 IP 宏
//   "PLC_PUBLISH_INTERVAL_MS" int  写 PLC 周期, 默认 100
// -------------------------------------------------------------------------


// =========================================================================
// LoadConfigFromJson —— 把 json 文件解析进配置结构体
// -------------------------------------------------------------------------
// 入参:
//     json_path  json 配置文件路径(相对路径基于 exe 工作目录).
//                文件不存在或格式错误时, 内部 read_*_Json 会返回空 / 0,
//                函数据此判断必填项缺失而失败.
//
//     out_cfg    [输出] 解析后的配置. 函数会就地写入; 调用方传入的内容会
//                被覆盖. 调用方负责对象寿命.
//
// 返回:
//     true   解析成功. out_cfg 已可用作 AntiCollisionApp::Configure 的入参.
//     false  必填项缺失或非法; stderr 有具体原因. out_cfg 处于部分填充
//            状态, 不应使用.
//
// 必填项校验:
//     - DETECTOR_MODEL_PATH (或旧名 MODEL_PATH) 非空
//     - DETECTOR_CLASSES 非空
//     - 4 路相机各自 CAMx_IP 非空、CAMx_PWD 非空(可由 DEFAULT_PWD 提供)
//     - 4 路相机各自 4 个角点齐全
//
// 阻塞性: 同步阻塞, 仅 IO 时间(几毫秒).
// 线程安全: 只读 json 文件 + 写出参, 多线程加载不同文件互不干扰.
// 异常: 不抛. 任何错误以返回 false 体现.
//
// 用法:
//     AntiCollisionAppConfig cfg;
//     if (!LoadConfigFromJson("./anti_collision.json", cfg)) {
//         return 1;
//     }
//     // 此时 cfg 已可用
// =========================================================================
bool LoadConfigFromJson(const std::string& json_path,
    AntiCollisionAppConfig& out_cfg);