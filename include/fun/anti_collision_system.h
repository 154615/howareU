#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>

#include "frame_sink.h"
#include "yolo_detector.h"   // Yolov8Detector / Yolov8DetectorConfig / OutputParams

// =========================================================================
// anti_collision_system.h —— 防撞检测算法消费层(第三层)
// -------------------------------------------------------------------------
// 模块定位:
//   作为 FrameSink 挂在 CameraSource 上, 接收每路相机的实时帧, 跑 YOLOv8
//   分割推理, 判定目标是否落入预设的"减速区 / 急停区", 把每路的报警等级
//   写入共享 AntiCollisionState. 上层(App / PLC 发布线程)读这个 state.
//
// 处理链路:
//   CameraSource ─ OnFrame(i, frame) ─▶ FrameSlot[i] ─cv─▶ Worker[i]
//     ─▶ Yolov8Detector::Detect ─▶ ProcessOneFrame
//     ─▶ 区域 mask 像素相交计数 ─▶ 写 state_->cameras[i].level
//
// 线程模型:
//   - 4 路相机各起 1 条 worker 线程(共 4 条)
//   - 1 条磁盘清理线程(可选, retain_days > 0 时启动)
//   - 帧从 OnFrame 落入 4 个 FrameSlot, worker 用 condition_variable 唤醒,
//     单帧覆盖式(只保留最新一帧, 推理跟不上时直接丢旧帧)
//
// 所有权:
//   - 检测器(Yolov8Detector) 由本类独占, 析构时自动释放
//   - 区域配置 / 历史时间戳数组 等都在本对象内
//   - AntiCollisionState 由外部传入, 本类只读写其字段, 不持有所有权
// =========================================================================


// -------------------------------------------------------------------------
// AlarmLevel —— 防撞报警等级
//
// 数值由小到大递增, 大的覆盖小的(比如同时入侵了减速区和急停区, 取急停).
// 数值与 PLC 协议强绑定, 改这里要同步改 anti_collision_app.cpp 里的
// PublishToPlc 编码逻辑.
// -------------------------------------------------------------------------
enum class AlarmLevel : uint8_t {
    Safe = 0,   // 无入侵
    Decel = 1,   // 入侵减速区(警告)
    Stop = 2    // 入侵急停区(立即停车)
};


// -------------------------------------------------------------------------
// CameraAlarmStatus —— 单路相机的运行状态(算法 → 上层 单向流)
//
// 全部用 atomic, 算法 worker 写, 上层 PLC 发布线程读, 不需要外部加锁.
// -------------------------------------------------------------------------
struct CameraAlarmStatus {
    // 当前报警等级. 由 ProcessOneFrame 写入. 上层据此发 PLC 指令.
    std::atomic<AlarmLevel> level{ AlarmLevel::Safe };

    // 是否曾经收到过帧. 用于 UI / 诊断, 区分"没接相机" vs "接了但全 Safe".
    std::atomic<bool>       has_input{ false };

    // 最近一次收到帧的时间戳, 单位毫秒(steady_clock).
    // 上层可用 (now - last) 判断是否掉流.
    std::atomic<uint64_t>   last_frame_tick_ms{ 0 };

    // 累计处理过的帧数, 仅用于诊断, 不参与业务逻辑.
    std::atomic<uint64_t>   frame_count{ 0 };
};


// -------------------------------------------------------------------------
// AntiCollisionState —— 4 路相机状态汇总
//
// 由 App 层持有(不是 ACS); App 在 Start 时把地址传给 ACS, ACS 持有裸指针
// 写入. App 析构时一并销毁, 此时 ACS 必然已 Stop, 不会再写 → 无生命周期风险.
// -------------------------------------------------------------------------
struct AntiCollisionState {
    std::array<CameraAlarmStatus, 4> cameras;
};


// -------------------------------------------------------------------------
// CameraRegionConfig —— 单路相机的区域配置(像素坐标)
//
// 区域用 4 个角点描述, 内部按 split_ratio 把这块四边形上下切成
// "减速区(上)/急停区(下)"两块. 角点必须按顺时针或逆时针顺序, 不能交叉.
// -------------------------------------------------------------------------
struct CameraRegionConfig {
    // 4 个角点(像素坐标, 来自 json 的 CAMx_PT1~PT4).
    std::vector<cv::Point> quad;

    // 帧分辨率, 用于建 mask. 必须与实际相机出图分辨率一致.
    int                    frame_width = 0;
    int                    frame_height = 0;
};


// -------------------------------------------------------------------------
// AntiCollisionConfig —— 算法消费层启动配置
//
// 关于阈值:
//   - detector.conf_threshold  在检测器内部 NMS 之前过滤(召回相关)
//   - conf_threshold (本结构)   在 NMS 之后再过滤一遍(精度相关)
//   两者用途不同, 名字相同. 一般建议前低后高, 例如 0.25 + 0.5.
// -------------------------------------------------------------------------
struct AntiCollisionConfig {
    // ===== 检测模型(必填 model_path 和 class_names) =====
    Yolov8DetectorConfig detector;

    // ===== 4 路区域配置(必填; 4 个角点 + 帧尺寸) =====
    std::array<CameraRegionConfig, 4> regions;

    // 减速区与急停区的上下切分比例, 0~1 之间, 默认 0.5(对半切).
    // 例如 0.6 = 上 60% 减速 / 下 40% 急停.
    float split_ratio = 0.5f;

    // 截图保留天数, > 0 时启动磁盘清理线程, 周期清理过期文件; 0 表示不清理.
    int   retain_days = 0;

    // 是否在每路开 cv::imshow 显示标注画面. 只用于桌面调试, 部署关掉.
    bool  enable_debug_show = false;

    // 业务侧二次过滤阈值(见上方"关于阈值"注释).
    float conf_threshold = 0.5f;

    // 单帧入侵像素数阈值. 目标 mask 与区域 mask 的交集像素数超过这个
    // 数才算"真入侵", 用于过滤掉边缘掠过的小目标.
    int   intrusion_pixel_thresh = 500;

    // 报警保持时间(毫秒). 一次入侵后即便目标消失也维持报警这么久,
    // 防止目标在边界来回跳导致 PLC 指令抖动.
    int   alarm_hold_ms = 2000;

    // 报警截图保存目录. ACS 启动时若不存在会自动创建.
    std::string snapshot_dir = "./save_log/save_result/";

    // 磁盘清理线程要扫的目录列表(retain_days > 0 时使用).
    std::vector<std::string> cleanup_dirs = {
        "./save_log/save_result/",
        "./save_log/logs/"
    };
};


// -------------------------------------------------------------------------
// AlarmRegion —— ACS 内部用的运行期区域结构(由 CameraRegionConfig 烘焙而来)
//
// 由 BuildRegion 在构造时计算; 包含切分后的两块多边形 + 预计算 mask,
// 后续每帧推理只需把目标 mask 与这两个 mask 做位与即可, 不再做几何计算.
// -------------------------------------------------------------------------
struct AlarmRegion {
    std::vector<cv::Point> decel_points;  // 减速区多边形顶点
    std::vector<cv::Point> stop_points;   // 急停区多边形顶点
    cv::Mat                decel_mask;    // 减速区填充 mask, CV_8UC1, 0/255
    cv::Mat                stop_mask;     // 急停区填充 mask, 同上
    bool                   valid = false; // 是否构建成功(角点齐全 + 帧尺寸合理)
};


// =========================================================================
// AntiCollisionSystem —— 算法消费层主体
//
// 生命周期:
//   构造 ─▶ Start(state) ─▶ 跑 ─▶ Stop() ─▶ 析构
//
// 装配方式:
//   AntiCollisionSystem acs(cfg);
//   for (auto& src : sources) src->AddSink(&acs);   // 挂为 sink
//   acs.Start(&state);
//
// 重要时序:
//   - Start 之前不能让 source 推帧(否则 OnFrame 进来但 worker 没起)
//   - Stop 之前要先把所有 source Stop, 否则会有最后一波帧进来无人处理
//     -> 见 anti_collision_app.cpp 的 Stop() 顺序
// =========================================================================
class AntiCollisionSystem : public FrameSink {
public:
    // 构造: 复制 cfg, 烘焙 4 路区域 mask, 创建 detector_(尚未 Load).
    // 不会启动任何线程, 不会读模型文件.
    explicit AntiCollisionSystem(const AntiCollisionConfig& cfg);

    // 析构: 自动调 Stop() 等所有线程退出, 释放 detector_ + 区域 mask.
    ~AntiCollisionSystem() override;

    AntiCollisionSystem(const AntiCollisionSystem&) = delete;
    AntiCollisionSystem& operator=(const AntiCollisionSystem&) = delete;

    // ---------------------------------------------------------------------
    // Start() —— 加载模型 + 起 4 路 worker + 起清理线程
    // ---------------------------------------------------------------------
    // 入参:
    //     out_state  状态写入目的地. 由 App 层创建并保管, 寿命必须长于
    //                本 ACS 对象. 不接受 nullptr.
    // 副作用:
    //     首次调用会触发模型加载(几百 ms ~ 几秒); 失败时直接返回, 不抛.
    //     已经在跑时是无操作.
    // 阻塞性: 阻塞直到模型加载完毕和 worker 全部起好.
    void Start(AntiCollisionState* out_state);

    // ---------------------------------------------------------------------
    // Stop() —— 停止所有线程
    // ---------------------------------------------------------------------
    // 通知 4 路 worker + 清理线程退出, 并 join 全部. 阻塞.
    // 不释放 detector_(留给析构, 让 Start/Stop/Start 重启的成本最低).
    // 幂等: 重复调用安全.
    void Stop();

    // ---------------------------------------------------------------------
    // OnFrame() —— FrameSink 接口实现, 由 CameraSource 的 poll 线程调用
    // ---------------------------------------------------------------------
    // 入参:
    //     cam_index  哪路相机送来的(0~3); 越界会被忽略
    //     frame      当前最新帧, BGR / CV_8UC3
    // 行为:
    //     把帧写入 slots_[cam_index](覆盖式, 老帧丢弃), notify 对应 worker.
    //     不阻塞 source 线程, 推理本身在 worker 线程做.
    // 线程安全: 安全. 4 路 source 的 poll 线程并发调本函数互不干扰.
    void OnFrame(int cam_index, const cv::Mat& frame) override;

private:
    // -------------------- 内部线程入口 --------------------

    // 单路 worker 主循环. 等帧 -> Detect -> ProcessOneFrame -> 循环.
    void WorkerLoop(int cam_index);

    // 单帧处理: 算 mask 交集, 决定 level, 必要时存截图, 写 state_.
    // frame 非 const, 用于在调试模式下叠加可视化标记.
    void ProcessOneFrame(int cam_index,
        cv::Mat& frame,
        const std::vector<OutputParams>& detections);

    // 磁盘清理线程: 周期扫 cfg_.cleanup_dirs, 删过期文件. retain_days<=0 时不启动.
    void DiskCleanupLoop();

    // -------------------- 静态工具 --------------------

    // 把 CameraRegionConfig(角点 + 帧尺寸) 烘焙成 AlarmRegion(填充 mask).
    static AlarmRegion BuildRegion(const CameraRegionConfig& cfg, float split_ratio);

    // 当前时间戳字符串, 形如 "20260508_173015_123", 用于截图文件名.
    static std::string CurrentTimestamp();

private:
    // 启动配置, 构造时复制. Start 后不应再改.
    AntiCollisionConfig cfg_;

    // 烘焙后的 4 路区域(预计算 mask).
    std::array<AlarmRegion, 4> regions_;

    // 检测器: 封装层, 内部自带锁, 多 worker 并发会被串行化.
    std::unique_ptr<Yolov8Detector> detector_;

    // 模型是否已加载. 控制 Start 重复调用时不重复加载.
    bool                            detector_loaded_ = false;

    // 报警保持期跟踪: 每路相机最近一次进入 Stop / Decel 的时刻.
    std::array<std::chrono::steady_clock::time_point, 4> last_stop_time_;
    std::array<std::chrono::steady_clock::time_point, 4> last_decel_time_;

    // 上一帧的报警等级, 用于检测跳变(只有 Safe→非 Safe 时才存截图).
    std::array<AlarmLevel, 4>                            prev_level_;

    // 状态写入目的地. 不持有所有权, 由 App 创建. Stop 后置 nullptr.
    AntiCollisionState* state_ = nullptr;

    // 单路相机的帧缓冲槽. 单帧覆盖式 + condition_variable 唤醒 worker.
    struct FrameSlot {
        cv::Mat                 frame;         // 待处理的最新帧
        bool                    has_frame = false;
        std::mutex              mtx;
        std::condition_variable cv;
    };
    std::array<std::unique_ptr<FrameSlot>, 4> slots_;

    // 4 路 worker + 清理线程.
    std::array<std::thread, 4> worker_threads_;
    std::thread                cleanup_thread_;

    // 主运行标志, Stop 时置 false 以唤醒所有线程退出.
    std::atomic<bool>          is_running_{ false };
};