#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>

#include "frame_sink.h"
#include "yolo_detector.h"   // Yolov8Detector / OutputParams

// =========================================================================
// anti_lift_system.h —— 防吊起算法层(第三层)
// -------------------------------------------------------------------------
// 模块定位:
//   与 AntiCollisionSystem 平级, 是另一种 FrameSink 实现. 挂在
//   CameraSource 上接收实时帧, 但行为完全不同:
//     - AntiCollisionSystem:  24h 常驻检测, 区域 mask 相交判入侵
//     - AntiLiftSystem(本类): 只有 PLC 触发"启动条件"成立时才进入会话,
//                             用 YOLOv8 检测做锚点, 帧间光流追踪锚点
//                             位移, 累积位移超阈值即判定异常
//
// 业务背景:
//   港口 RTG(轮胎吊)在吊起集装箱时, 偶发集装箱锁孔未真正闭锁, 导致
//   底盘车连同箱子被一起吊起 —— 这是严重事故. 本模块在起升过程中
//   持续监视底盘车的车架/车轮锚点的垂直位移, 异常即报警让 PLC 急停.
//
// 输出维度:
//   对外只有 4 种结果(LiftAlarmType): 安全/吊起/开走/异常
//   内部精细异常(锁孔位移/车轮位移/车架侧翻/车轮侧翻) 都映射到"吊起",
//   只在日志和截图文件名里区分.
//
// 启动条件(由 PLC 输入推断):
//   "已闭锁 + 着箱标志 + 起升速度 > 0" 三者同时满足 → 进入会话;
//   会话期间持续追踪;
//   "复位信号 = 1" 或者 "起升速度 = 0 持续若干秒" → 结束会话.
//
// 录制:
//   会话期间内置录制线程, 每路相机录两份(原始 + 算法标注), 10fps.
//   会话结束按结果命名落盘, 形如:
//     ./save_log/lift_record/cam0_20260508_173015_lifted_raw.mp4
//
// 线程模型:
//   2 路 worker(光流推理) + 1 录制线程 = 3 条
//   不再单独起清理线程 (走与 ACS 共用的清理机制 —— 由 App 层负责)
// =========================================================================


// -------------------------------------------------------------------------
// LiftAlarmType —— 防吊起对外报警类型
//
// 数值与 PlcSend::LiftAlarm 寄存器编码一一对应.
// 内部细分(锁孔/车轮/车架侧翻/车轮侧翻) 全部映射到 Lifted (1).
// -------------------------------------------------------------------------
enum class LiftAlarmType : uint8_t {
    None        = 0,   // 安全/无报警
    Lifted      = 1,   // 检测到吊起异常(含侧翻)
    DroveAway   = 2,   // 车辆水平开走(脱离镜头)
    SystemError = 3,   // 程序异常(模型挂了等), 备用
};


// -------------------------------------------------------------------------
// LiftSubKind —— 内部精细分类(仅日志/截图/录像名用, 不发 PLC)
// -------------------------------------------------------------------------
enum class LiftSubKind : uint8_t {
    None             = 0,
    HoleLifted       = 1,   // 锁孔垂直位移超阈值
    WheelLifted      = 2,   // 车轮垂直位移超阈值
    PlatformRollover = 3,   // 车架侧翻
    WheelRollover    = 4,   // 车轮侧翻
    DroveAway        = 5,   // 水平开走
};


// -------------------------------------------------------------------------
// AntiLiftPlcInputs —— App 层从 PlcReceiveBuffer 提取的输入快照
//
// 由 AntiLiftApp 的 PLC 轮询线程定期填充, 调 UpdatePlcInputs() 喂进算法层.
// 算法层据此判定会话开始/结束. 单位与 PLC 一致.
// -------------------------------------------------------------------------
struct AntiLiftPlcInputs {
    uint16_t pos_trolley   = 0;   // 小车位置 cm
    uint16_t hoist_height  = 0;   // 起升高度 cm
    int16_t  spd_hoist     = 0;   // 起升速度 mm/s, 正升负降
    uint16_t lock_status   = 0;   // 0=开锁, 1=闭锁
    uint16_t box_landed    = 0;   // 0=未着箱, 1=着箱
    uint16_t reset_signal  = 0;   // 0=默认, 1=按下复位
    uint16_t spreader_size = 40;  // 20 / 40 / 45
};


// -------------------------------------------------------------------------
// CameraLiftStatus —— 单路相机当前状态(算法 → 上层 单向流, atomic)
// -------------------------------------------------------------------------
struct CameraLiftStatus {
    // 当前对外报警类型(写 PLC 用)
    std::atomic<LiftAlarmType> alarm{LiftAlarmType::None};

    // 是否处于会话期间(在追踪/等检测异常)
    std::atomic<bool>          in_session{false};

    // 累计处理过的帧数(诊断)
    std::atomic<uint64_t>      frame_count{0};

    // 当前会话的精细分类(仅诊断显示用; 不写 PLC)
    std::atomic<LiftSubKind>   sub_kind{LiftSubKind::None};
};

// 2 路汇总状态. App 持有, ACS 持有指针写入.
struct AntiLiftState {
    std::array<CameraLiftStatus, 2> cameras;
};


// -------------------------------------------------------------------------
// AntiLiftConfig —— 算法层启动配置
//
// 字段全部由 App 层透传, 来源 json. 必填项: detector.model_path, class_names.
// -------------------------------------------------------------------------
struct AntiLiftConfig {
    // ===== 检测模型(用于打锚点; Detect 任务) =====
    Yolov8DetectorConfig detector;

    // ===== 异常判定阈值(像素) =====
    // 历史代码里这些是 extern 全局, 现在收口到这里, 由 json 决定
    int limit_hole_y            = 100;   // 锁孔垂直位移上限
    int limit_wheel_y           = 100;   // 车轮垂直位移上限
    int limit_horizon_left      = 200;   // 水平开走判据
    int limit_rotate_lift_plt   = 80;    // 车架侧翻判据
    int limit_rotate_lift_wheel = 60;    // 车轮侧翻判据

    // ===== 报警保持/触发参数 =====
    int  alarm_hold_ms    = 2000;   // 报警状态最短保持时间(防抖)
    int  session_min_ms   = 500;    // 会话最短长度(过滤误触发)
    int  session_idle_ms  = 3000;   // 起升速度=0 持续这么久 → 结束会话

    // ===== 录像 =====
    bool        enable_record   = true;             // 总开关
    int         record_fps      = 10;               // 录像帧率
    std::string record_dir      = "./save_log/lift_record/";

    // ===== 调试 =====
    bool        enable_debug_show = false;          // 桌面 cv::imshow

    // ===== 截图 =====
    std::string snapshot_dir = "./save_log/lift_snapshot/";
};


// -------------------------------------------------------------------------
// FeatureRect —— 单个被追踪锚点(从原 n_FeatureRect 抽来, 命名清理过)
//
// 作为 worker 内部状态, 本结构不对外暴露(仅 .cpp 用), 但放在头里方便
// 阅读完整数据流. 业务方代码不需要构造它.
// -------------------------------------------------------------------------
struct FeatureRect {
    int       class_id = -1;            // YOLO 输出的类别 id
    cv::Mat   prev_view;                // 上一帧 ROI(灰度), 光流参考
    cv::Mat   curr_view;                // 当前帧 ROI(灰度)
    cv::Rect  target_box;               // 在原图坐标系的目标框
    cv::Rect  view_target_box;          // 在 ROI 坐标系的目标框
    cv::Rect  view_box;                 // ROI 在原图中的位置(目标框周围扩一圈)

    double cur_diff_x       = 0.0;      // 单帧水平位移
    double cur_diff_y       = 0.0;      // 单帧垂直位移
    double total_diff_x     = 0.0;      // 累积水平位移
    double total_diff_y     = 0.0;      // 累积垂直位移
};


// =========================================================================
// AntiLiftSystem —— 算法层主体, 与 AntiCollisionSystem 平级
//
// 生命周期: 构造 → Start(state) → 跑 → Stop() → 析构
//
// 装配:
//   AntiLiftSystem als(cfg);
//   for (auto& src : sources) src->AddSink(&als);
//   als.Start(&state);
//
// 调用约束:
//   - UpdatePlcInputs 必须周期性被调(由 App 的 PLC 输入线程调), 否则
//     会话条件无法被算法感知, 永远停在 Idle 状态
// =========================================================================
class AntiLiftSystem : public FrameSink {
public:
    // 构造: 复制 cfg, 创建 detector_(尚未 Load), 不起线程.
    explicit AntiLiftSystem(const AntiLiftConfig& cfg);

    // 析构: 自动 Stop(), 释放 detector + 所有线程.
    ~AntiLiftSystem() override;

    AntiLiftSystem(const AntiLiftSystem&)            = delete;
    AntiLiftSystem& operator=(const AntiLiftSystem&) = delete;

    // ---------------------------------------------------------------------
    // Start() —— 加载模型 + 起 worker(2 条) + 起录制线程(1 条)
    // ---------------------------------------------------------------------
    // 入参:
    //     out_state  状态写入目的地; 由 App 创建并持有, 寿命 > 本对象
    // 副作用: 模型加载几百 ms ~ 几秒
    // 阻塞性: 阻塞直到模型加载和线程起好
    void Start(AntiLiftState* out_state);

    // ---------------------------------------------------------------------
    // Stop() —— 通知所有线程退出, join, 关闭未结束的录像文件
    // ---------------------------------------------------------------------
    // 阻塞性: 阻塞
    // 幂等: 重复调用安全
    void Stop();

    // ---------------------------------------------------------------------
    // OnFrame() —— FrameSink 接口实现, 由 CameraSource 的 poll 线程调用
    // ---------------------------------------------------------------------
    // 入参:
    //     cam_index    相机路号(0~1, 越界忽略)
    //     frame        BGR / CV_8UC3
    // 行为:
    //     1) 把帧写入 slots_[cam_index] (单帧覆盖)
    //     2) 若处于会话, 同时把 raw 副本入 raw_record_queue_[i]
    //     3) notify worker
    // 不阻塞 source 线程.
    void OnFrame(int cam_index, const cv::Mat& frame) override;

    // ---------------------------------------------------------------------
    // UpdatePlcInputs() —— 喂入 PLC 输入(由 App 周期性调)
    // ---------------------------------------------------------------------
    // 入参:
    //     in   一份完整的 PLC 输入快照
    // 行为:
    //     1) 更新内部 latest_plc_inputs_
    //     2) 根据"闭锁 + 着箱 + 起升速度 > 0"启动会话; 收到复位/起升停止
    //        持续时长后结束会话
    // 线程安全: 安全
    // 调用时机: App 的 PLC 输入轮询线程, 与 PLC 接收同节拍(如 50ms)
    void UpdatePlcInputs(const AntiLiftPlcInputs& in);

private:
    // -------------------- 内部线程入口 --------------------
    void WorkerLoop(int cam_index);
    void RecordLoop();

    // -------------------- 算法核心(原 calc_optical/frame_init/frame_update) --------------------
    void HandleFrame(int cam_index, cv::Mat& frame);
    void InitFeatures(int cam_index, const cv::Mat& gray, const std::vector<OutputParams>& dets);
    void UpdateFeature(int cam_index, const cv::Mat& gray, FeatureRect& f);
    LiftSubKind EvaluateAlarm(int cam_index, const std::vector<FeatureRect>& features);

    // -------------------- 会话管理 --------------------
    void TryStartSession(int cam_index);   // 在 worker 里调; 用最新 plc 输入判定
    void EndSession(int cam_index, LiftAlarmType result, LiftSubKind sub);

    // -------------------- 工具 --------------------
    static std::string CurrentTimestamp();
    static std::string AlarmKindString(LiftAlarmType t, LiftSubKind k);

private:
    AntiLiftConfig cfg_;

    // 模型(共享给 2 个 worker; 内部加锁串行化)
    std::unique_ptr<Yolov8Detector> detector_;
    bool                            detector_loaded_ = false;

    // 状态写入目的地, 由 Start 注入
    AntiLiftState* state_ = nullptr;

    // ---- 帧缓冲槽(单帧覆盖, condition_variable 唤醒 worker) ----
    struct FrameSlot {
        cv::Mat                 frame;
        bool                    has_frame = false;
        std::mutex              mtx;
        std::condition_variable cv;
    };
    std::array<std::unique_ptr<FrameSlot>, 2> slots_;

    // ---- 每路 worker 的私有状态 ----
    struct PerCamera {
        std::vector<FeatureRect>             features;     // 当前追踪的锚点列表
        std::chrono::steady_clock::time_point session_start{};
        std::chrono::steady_clock::time_point last_motion_time{};
        std::chrono::steady_clock::time_point alarm_set_time{};
        bool                                 prev_in_session = false;
        LiftAlarmType                        last_alarm = LiftAlarmType::None;
        LiftSubKind                          last_sub_kind = LiftSubKind::None;

        // 录像端: 会话开始时由 worker 创建, 会话结束时关闭并改名
        std::unique_ptr<cv::VideoWriter>     vw_raw;
        std::unique_ptr<cv::VideoWriter>     vw_annot;
        std::string                          raw_temp_path;     // 写入中临时路径
        std::string                          annot_temp_path;
        std::chrono::steady_clock::time_point last_record_tick{};
    };
    std::array<PerCamera, 2> cams_;

    // ---- 录制队列(2 路 × 2 类) ----
    struct RecordItem {
        int     cam_index;
        bool    is_annotated;     // false=raw, true=annotated
        cv::Mat frame;
    };
    std::deque<RecordItem>          rec_queue_;
    std::mutex                      rec_mtx_;
    std::condition_variable         rec_cv_;
    static constexpr size_t         kRecordQueueMax = 256;  // 防止异常情况无限堆积

    // ---- PLC 输入(最新一份, 供 worker 读) ----
    AntiLiftPlcInputs               latest_plc_inputs_{};
    std::mutex                      plc_mtx_;

    // ---- 线程 ----
    std::array<std::thread, 2>      worker_threads_;
    std::thread                     record_thread_;
    std::atomic<bool>               is_running_{false};
};
