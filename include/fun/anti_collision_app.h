#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "anti_collision_system.h"
#include "camera_source.h"
#include "frame_sink.h"
#include "i_ptz_control.h"
#include "modbus_cfg.h"   // Plc_interact, IP

// =========================================================================
// anti_collision_app.h —— 应用耦合层(第二层)
// -------------------------------------------------------------------------
// 模块定位:
//   把第一层(4 × CameraSource) 和第三层(AntiCollisionSystem) 捏合起来,
//   并耦合 PLC 通讯的"装配车间". main.cpp 极简, 所有装配工作都在这.
//
// 核心职责:
//   1. 装配     建 ACS, 建 4 路 source, 把 ACS 挂为每路 source 的 sink
//   2. 启停顺序 启动: PLC → ACS → sources → PLC 发布线程
//                关闭: PLC 发布线程 → sources(先停产) → ACS(后停消) → PLC
//   3. PLC 业务 100ms 一次把 4 路报警结果聚合成 PLC 寄存器值并写下去
//   4. 状态外露 给 main / UI / 其它业务模块查看每路报警等级和连接状态
//
// 三层接口契约(强约束):
//   - 第一层与第三层互不知道彼此的存在
//   - 它们仅通过 FrameSink (生产→消费) 和 IPtzControl (云台命令) 对话
//   - 本类是它们之间的唯一胶水, 也是 PLC 业务进入算法链路的唯一入口
//
// 典型使用(见 main.cpp):
//     AntiCollisionAppConfig cfg;
//     LoadConfigFromJson("anti_collision.json", cfg);
//
//     AntiCollisionApp app;
//     if (!app.Configure(cfg)) return 1;
//     if (!app.Start())        return 1;
//
//     // 主循环或事件循环
//     while (!quit) std::this_thread::sleep_for(std::chrono::seconds(1));
//
//     app.Stop();   // 析构也会自动调
// =========================================================================


// -------------------------------------------------------------------------
// CameraEntry —— 单路相机的连接信息
//
// 仅是个数据容器, 透传给底层 CameraSourceConfig. 4 路相机就 4 份.
// -------------------------------------------------------------------------
struct CameraEntry {
    std::string ip;                 // 设备 IP, 如 "192.168.1.64"
    int         port = 8000;     // SDK 登录端口(海康默认 8000)
    std::string user = "admin";  // 登录账号
    std::string pwd;                // 登录密码(必填)
    int         channel = 1;        // 通道号(球机一般为 1)
};


// -------------------------------------------------------------------------
// AntiCollisionAppConfig —— App 的全部启动参数
//
// 一站式配置: main 只需把 json 解析进这个结构再传给 Configure() 即可.
// -------------------------------------------------------------------------
struct AntiCollisionAppConfig {
    // ===== 检测器配置(模型路径/类别/阈值/imgsz, 见 yolo_detector.h) =====
    // 必填: detector.model_path 和 detector.class_names
    Yolov8DetectorConfig detector;

    // ===== 防撞业务参数 =====
    // 区域上下切分比例, 0~1; 默认 0.5
    float       split_ratio = 0.5f;
    // 截图保留天数, > 0 启动磁盘清理线程
    int         retain_days = 0;
    // 是否在桌面用 cv::imshow 调试 4 路画面
    bool        enable_debug_show = false;

    // ===== 4 路相机的区域配置(角点 + 帧尺寸) =====
    // 顺序对应 cam_index 0~3, 与 cameras[] 一一对应
    std::array<CameraRegionConfig, 4> regions;

    // ===== 4 路相机的连接信息 =====
    std::array<CameraEntry, 4> cameras;

    // ===== 防撞细节阈值(可选, 不填走默认) =====
    // 业务侧二次过滤阈值(NMS 之后再筛一遍)
    float conf_threshold = 0.5f;
    // 入侵像素数阈值(目标 mask ∩ 区域 mask 像素数)
    int   intrusion_pixel_thresh = 500;
    // 报警保持时间, 毫秒, 防 PLC 抖动
    int   alarm_hold_ms = 2000;
    // 截图保存目录
    std::string snapshot_dir = "./save_log/save_result/";
    // 磁盘清理目标目录列表
    std::vector<std::string> cleanup_dirs = {
        "./save_log/save_result/",
        "./save_log/logs/"
    };

    // ===== 相机源参数(透传给 CameraSourceConfig) =====
    int  poll_interval_ms = 33;     // poll 节拍, ~30Hz
    int  reconnect_interval_ms = 3000;   // 重连最小间隔
    bool auto_connect = true;   // Start 时自动建立连接

    // ===== PLC 配置 =====
    bool        enable_plc = true;  // 是否启动 PLC 通讯线程
    std::string plc_ip = IP;    // 默认用 modbus_cfg.h 里的 IP 宏
    int         plc_publish_interval_ms = 100;   // 防撞结果写 PLC 的周期(毫秒)
};


// =========================================================================
// AntiCollisionApp —— 应用耦合层主体
//
// 生命周期: 默认构造 → Configure(cfg) → Start() → 跑 → Stop() → 析构
//
// 线程模型:
//   - 自身只起 1 条 plc_publish_thread_(由 Start 启动)
//   - 间接持有: 4 × CameraSource(各 1 个 poll 线程) + ACS(4 worker + 清理)
//   - PLC 底层 Plc_interact 内部还有几条 detach 线程(心跳/重连/收数)
//   总计典型 ≈ 1 + 4 + 5 + 4 = 14 条线程
//
// 线程安全:
//   - Configure / Start / Stop 必须由同一管理线程调(通常是 main)
//   - GetAlarmLevel / IsStreamConnected / Ptz 等查询函数任意线程可调
//   - AttachSink / DetachSink 任意线程可调
//
// 所有权:
//   - 4 个 CameraSource 和 ACS 都由本类持有(unique_ptr), 析构时一并销毁
//   - state_ 是值成员, 不存在悬空风险
//   - PTZ 句柄属于 source, 调用方拿到的指针不释放
// =========================================================================
class AntiCollisionApp {
public:
    // 构造: 仅做零初始化, 不读配置, 不起线程, 不连相机.
    AntiCollisionApp();

    // 析构: 自动调用 Stop() 等所有线程退出, 释放所有底层对象.
    // 注意: PLC 底层有几条 detach 线程在阻塞调用中, 析构后短时间内可能
    // 还在跑, 这是 Plc_interact 设计局限, 不影响进程退出.
    ~AntiCollisionApp();

    AntiCollisionApp(const AntiCollisionApp&) = delete;
    AntiCollisionApp& operator=(const AntiCollisionApp&) = delete;

    // ---------------------------------------------------------------------
    // Configure() —— 装配阶段, 必须在 Start() 之前调用一次
    // ---------------------------------------------------------------------
    // 行为:
    //     1) 校验 cfg(model_path 非空 / 区域 4 角点齐全 / 帧尺寸合理 等)
    //     2) 建 ACS(此时 ACS 仅构造, 模型未加载, 线程未起)
    //     3) 建 4 个 CameraSource, 每个挂 ACS 为 sink
    //     4) 配置 PLC 实例(尚未启动)
    // 入参:
    //     cfg  全部配置. 函数内部会复制保留.
    // 返回:
    //     true  配置成功, 可以进入 Start()
    //     false 配置非法或重复调用 Configure
    // 阻塞性: 不阻塞.
    // 副作用: 如果失败, 内部部分对象可能已建好, 但本类不会自动清理 —
    //         按当前实现, 失败后正确做法是直接销毁本对象重来.
    bool Configure(const AntiCollisionAppConfig& cfg);

    // ---------------------------------------------------------------------
    // Start() —— 启动所有线程
    // ---------------------------------------------------------------------
    // 启动顺序(对应内部实现):
    //     1) 启动 PLC 通讯线程(若 enable_plc)
    //     2) ACS Start(&state_)        ← 加载模型 + 起 4 worker
    //     3) 4 路 source.Start()        ← 起 poll 线程, 开始拉流
    //     4) 启动 PLC 发布线程
    // 必须在 Configure() 之后调用. 重复调用是无操作.
    // 返回:
    //     true  启动成功
    //     false 未 Configure 或 ACS 启动失败(模型加载失败等)
    // 阻塞性: 阻塞, 至少要等模型加载和各线程起好(几百 ms ~ 几秒).
    bool Start();

    // ---------------------------------------------------------------------
    // Stop() —— 关闭所有线程
    // ---------------------------------------------------------------------
    // 关闭顺序(与启动相反, 关键!):
    //     1) running_=false → PLC 发布线程退出 + join
    //     2) 4 路 source.Stop()    ← 先停生产, 不再有新帧灌入消费端
    //     3) ACS.Stop()             ← 再停消费, 安全 join 4 worker
    //     4) plc_send / plc_rcv reset
    // 阻塞性: 阻塞, 直到所有线程退出.
    // 幂等: 重复调用安全.
    void Stop();

    // ---------------------------------------------------------------------
    // 状态查询(任意线程可调, 内部都是 atomic / 内部锁)
    // ---------------------------------------------------------------------

    // 取某路相机的当前报警等级.
    // 入参 cam_index: 0~3, 越界返回 AlarmLevel::Safe(安全默认).
    AlarmLevel GetAlarmLevel(int cam_index) const;

    // 某路相机当前是否能拉到帧.
    // 入参 cam_index: 0~3, 越界返回 false.
    bool IsStreamConnected(int cam_index) const;

    // 直接访问全部 4 路状态. 引用与本对象同寿; 调用方读取期间本对象不可销毁.
    const AntiCollisionState& State() const { return state_; }

    // ---------------------------------------------------------------------
    // PTZ 控制
    // ---------------------------------------------------------------------
    // 取某路相机的云台控制句柄.
    // 入参 cam_index: 0~3, 越界返回 nullptr.
    // 返回:
    //     非空指针 ── 生命周期与对应 CameraSource 绑定; 调用方不释放
    //     nullptr  ── 该路不存在 / 不支持 PTZ
    // 注: 即使取流处于 SDK 软解兜底状态, PTZ 仍然可用 —— 取流和 PTZ
    //     共用同一份海康 SDK 登录, 互不影响.
    IPtzControl* Ptz(int cam_index);

    // ---------------------------------------------------------------------
    // 主动连接控制(异步, 由 source 内 poll 线程实际执行)
    // ---------------------------------------------------------------------

    // 让某路相机进入"想连"状态; 当前未连时下个周期自动尝试连接.
    void RequestConnect(int cam_index);

    // 让某路相机进入"不想连"状态; 当前已连时下个周期自动断开.
    // 不影响 PTZ. 越界静默忽略.
    void RequestDisconnect(int cam_index);

    // ---------------------------------------------------------------------
    // 自定义消费者(运行期可挂)
    // ---------------------------------------------------------------------

    // 给某路 source 挂一个额外的 sink(例如录像 / UI 预览).
    // ACS 已经默认挂在每路 source 上, 这里是叠加而非替换.
    // 不取所有权; 调用方负责 sink 寿命长于本 App 或在 Stop 前 Detach.
    // 越界静默忽略.
    void AttachSink(int cam_index, FrameSink* sink);

    // 取消注册. 找不到匹配项是无操作.
    void DetachSink(int cam_index, FrameSink* sink);

    // ---------------------------------------------------------------------
    // PLC 句柄访问(供 main 或其它业务模块直接读写 PLC 时使用)
    // ---------------------------------------------------------------------
    // 返回值生命周期与本 App 绑定(对应 unique_ptr 的裸指针), 不释放.
    // 未 enable_plc 或 Start 之前调用会返回 nullptr.
    Plc_interact* GetPlcRcv() { return plc_rcv_.get(); }
    Plc_interact* GetPlcSend() { return plc_send_.get(); }

private:
    // PLC 发布线程主循环: 每 plc_publish_interval_ms 调一次 PublishToPlc().
    void PlcPublishLoop();

    // 单次发布: 读 state_ 4 路报警 → 方位映射 → 聚合 → 编码 → 写 PLC 寄存器.
    // 内部带跳变去重(只有寄存器值真的变了才打日志).
    void PublishToPlc();

private:
    // 是否已 Configure. 控制 Start 的前置条件.
    bool                                       configured_ = false;
    // PLC 发布线程是否应继续跑. Stop 时置 false.
    std::atomic<bool>                          running_{ false };

    // 启动配置, Configure 时复制保留.
    AntiCollisionAppConfig                     cfg_;

    // ===== 第一层 + 第三层 =====
    std::unique_ptr<AntiCollisionSystem>       acs_;          // 第三层主体
    AntiCollisionState                         state_;        // 算法 → App 的共享状态
    std::array<std::unique_ptr<CameraSource>, 4> sources_;    // 第一层 × 4

    // ===== PLC 通讯(应用层耦合) =====
    std::unique_ptr<Plc_interact>              plc_rcv_;            // 收 PLC
    std::unique_ptr<Plc_interact>              plc_send_;           // 发 PLC
    std::thread                                plc_publish_thread_; // 100ms 发布线程

    // ===== PLC 跳变去重缓存 =====
    // 上一次写入 PLC 的急停码 / 限速码; 用于打日志时判跳变, 避免刷屏.
    uint16_t                                   last_stop_code_ = 0;
    uint16_t                                   last_speed_code_ = 0;
};