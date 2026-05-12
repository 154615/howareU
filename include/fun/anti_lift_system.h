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
//   纯算法层. 与 AntiCollisionSystem 平级, 是另一种 FrameSink 实现.
//   挂在 CameraSource 上接收实时帧, 但行为完全不同:
//     - AntiCollisionSystem:  24h 常驻检测, 区域 mask 相交判入侵
//     - AntiLiftSystem(本类): 只有外部(App 层)显式 BeginSession 才进入
//                             会话, 用 YOLOv8 检测做锚点, 帧间光流追踪
//                             锚点位移, 累积位移超阈值即上报异常
//
// 与 App 层的分工(2026-05 重构):
//   - 启停判定全部由 App 层根据 PLC 输入做主, 本类不再读 PLC
//   - 本类仅暴露 BeginSession() / EndSession() 两个动词
//   - 算法判出 Lifted / DroveAway 时只 写 alarm, 不自己结会话; 由 App
//     侦测到 alarm 再来调 EndSession
//   - 检测到 plate (内集卡车型) 时写 has_plate 标志, App 侧据此结会话
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
// 录制:
//   会话期间内置录制线程, 每路相机录两份(原始 + 算法标注), 10fps.
//   EndSession 时按 keep_recording 决定改名落盘 或 删除临时文件.
//
// 线程模型:
//   2 路 worker(光流推理) + 1 录制线程 = 3 条
// =========================================================================


// -------------------------------------------------------------------------
// LiftAlarmType —— 防吊起对外报警类型
//
// 数值与 PlcSend::LiftAlarm 寄存器编码一一对应.
// 内部细分(锁孔/车轮/车架侧翻/车轮侧翻) 全部映射到 Lifted (1).
// -------------------------------------------------------------------------
enum class LiftAlarmType : uint8_t {
    None = 0,   // 安全/无报警
    Lifted = 1,   // 检测到吊起异常(含侧翻)
    DroveAway = 2,   // 车辆水平开走(脱离镜头)
    SystemError = 3,   // 程序异常(模型挂了等), 备用
};


// -------------------------------------------------------------------------
// LiftSubKind —— 内部精细分类(仅日志/截图/录像名用, 不发 PLC)
// -------------------------------------------------------------------------
enum class LiftSubKind : uint8_t {
    None = 0,
    HoleLifted = 1,   // 锁孔垂直位移超阈值
    WheelLifted = 2,   // 车轮垂直位移超阈值
    PlatformRollover = 3,   // 车架侧翻
    WheelRollover = 4,   // 车轮侧翻
    DroveAway = 5,   // 水平开走
};


// -------------------------------------------------------------------------
// CameraLiftStatus —— 单路相机当前状态(算法 → 上层 单向流, atomic)
// -------------------------------------------------------------------------
struct CameraLiftStatus {
    // 当前对外报警类型(写 PLC 用)
    // - 会话期间被算法写入(EvaluateAlarm 命中阈值)
    // - 会话结束由 App 决定保留或清零
    std::atomic<LiftAlarmType> alarm{ LiftAlarmType::None };

    // 是否处于会话期间(在追踪/等检测异常). 由 BeginSession/EndSession 翻转.
    std::atomic<bool>          in_session{ false };

    // 本次会话是否看到过 plate (内集卡车型标志)
    // 算法发现 plate 时置 true; BeginSession 时清零.
    // App 层据此判定"内集卡作业, 结会话且不报警"
    std::atomic<bool>          has_plate{ false };

    // 累计处理过的帧数(诊断)
    std::atomic<uint64_t>      frame_count{ 0 };

    // 当前会话的精细分类(仅诊断显示用; 不写 PLC)
    std::atomic<LiftSubKind>   sub_kind{ LiftSubKind::None };
};

// 2 路汇总状态. App 持有, ACS 持有指针写入.
struct AntiLiftState {
    std::array<CameraLiftStatus, 2> cameras;
};


// -------------------------------------------------------------------------
// AntiLiftConfig —— 算法层启动配置
//
// 字段全部由 App 层透传, 来源 json. 必填项: detector.model_path, class_names.
// 注意:
//   - 与启停判定相关的阈值(限高/小车位/起升下降阈值) 不在这里, 在
//     AntiLiftAppConfig 里 —— 算法层不知 PLC 语义.
// -------------------------------------------------------------------------
struct AntiLiftConfig {
    // ===== 检测模型(用于打锚点; Detect 任务) =====
    // class_names 期望包含: "wheel", "hole", "plate"
    Yolov8DetectorConfig detector;

    // ===== 异常判定阈值(像素) =====
    int limit_hole_y = 100;   // 锁孔垂直位移上限
    int limit_wheel_y = 100;   // 车轮垂直位移上限
    int limit_horizon_left = 200;   // 水平开走判据
    int limit_rotate_lift_plt = 80;    // 车架侧翻判据
    int limit_rotate_lift_wheel = 60;    // 车轮侧翻判据

    // ===== 录像 =====
    bool        enable_record = true;             // 总开关
    int         record_fps = 10;               // 录像帧率
    std::string record_dir = "./save_log/lift_record/";

    // ===== 调试 =====
    bool        enable_debug_show = false;        // 桌面 cv::imshow

    // ===== 截图 =====
    std::string snapshot_dir = "./save_log/lift_snapshot/";
};


// -------------------------------------------------------------------------
// FeatureRect —— 单个被追踪锚点
// -------------------------------------------------------------------------
struct FeatureRect {
    int       class_id = -1;            // YOLO 输出的类别 id
    cv::Mat   prev_view;                // 上一帧 ROI(灰度), 光流参考
    cv::Mat   curr_view;                // 当前帧 ROI(灰度)
    cv::Rect  target_box;               // 在原图坐标系的目标框
    cv::Rect  view_target_box;          // 在 ROI 坐标系的目标框
    cv::Rect  view_box;                 // ROI 在原图中的位置(目标框周围扩一圈)

    double cur_diff_x = 0.0;          // 单帧水平位移
    double cur_diff_y = 0.0;          // 单帧垂直位移
    double total_diff_x = 0.0;          // 累积水平位移
    double total_diff_y = 0.0;          // 累积垂直位移
};


// =========================================================================
// AntiLiftSystem —— 算法层主体, 与 AntiCollisionSystem 平级
//
// 生命周期: 构造 → Start(state) → [BeginSession / EndSession ...] → Stop() → 析构
//
// 装配:
//   AntiLiftSystem als(cfg);
//   for (auto& src : sources) src->AddSink(&als);
//   als.Start(&state);
//
// 会话控制(由 App 调):
//   - BeginSession(cam) 进入"追踪+判异常"模式, 开录像
//   - EndSession(cam, keep_recording) 退出, keep_recording=false 则删录像临时文件
//
// 会话期间的行为:
//   - 算法发现 plate    → 写 has_plate=true (不自动结会话)
//   - 算法判出 Lifted   → 写 alarm=Lifted   (不自动结会话)
//   - 算法判出 DroveAway→ 写 alarm=DroveAway(不自动结会话)
//   App 侦测到这些标志后自行决定何时调 EndSession.
// =========================================================================
class AntiLiftSystem : public FrameSink {
public:
    explicit AntiLiftSystem(const AntiLiftConfig& cfg);
    ~AntiLiftSystem() override;

    AntiLiftSystem(const AntiLiftSystem&) = delete;
    AntiLiftSystem& operator=(const AntiLiftSystem&) = delete;

    // ---------------------------------------------------------------------
    // Start() —— 加载模型 + 起 worker(2 条) + 起录制线程(1 条)
    // ---------------------------------------------------------------------
    void Start(AntiLiftState* out_state);

    // ---------------------------------------------------------------------
    // Stop() —— 通知所有线程退出, join, 关闭未结束的录像文件
    // ---------------------------------------------------------------------
    void Stop();

    // ---------------------------------------------------------------------
    // OnFrame() —— FrameSink 接口实现, 由 CameraSource 的 poll 线程调用
    // ---------------------------------------------------------------------
    void OnFrame(int cam_index, const cv::Mat& frame) override;

    // ---------------------------------------------------------------------
    // BeginSession() —— 由 App 显式开始一次会话
    // ---------------------------------------------------------------------
    // 入参:
    //     cam_index  相机路号(0~1, 越界忽略)
    // 行为:
    //     1) 清空该路 features / last_alarm / has_plate / alarm
    //     2) state_->cameras[cam].in_session = true
    //     3) 若 enable_record, 预置 INPROGRESS 临时文件名
    // 重入: 若已经在会话中, 仅打日志后忽略.
    // 线程安全: 安全(任意线程可调; 通常由 App 的 PLC 线程调)
    void BeginSession(int cam_index);

    // ---------------------------------------------------------------------
    // EndSession() —— 由 App 显式结束一次会话
    // ---------------------------------------------------------------------
    // 入参:
    //     cam_index        相机路号(0~1, 越界忽略)
    //     keep_recording   true  → 录像文件按结果改名落盘(正常异常/安全退出)
    //                      false → 删除临时录像文件(plate 命中 / 人工介入下降)
    // 行为:
    //     1) state_->cameras[cam].in_session = false
    //     2) 关闭 VideoWriter; 改名 或 删除 临时文件
    //     3) 当前 alarm 字段保留不动 —— 是否清零由 App 决定
    // 重入: 若不在会话中, 仅打日志后忽略.
    void EndSession(int cam_index, bool keep_recording);

private:
    // -------------------- 内部线程入口 --------------------
    void WorkerLoop(int cam_index);
    void RecordLoop();

    // -------------------- 算法核心 --------------------
    void HandleFrame(int cam_index, cv::Mat& frame, cv::Mat* out_annot = nullptr);
    void InitFeatures(int cam_index, const cv::Mat& gray,
        const std::vector<OutputParams>& dets);
    void UpdateFeature(int cam_index, const cv::Mat& gray, FeatureRect& f);

    // 评估异常并返回 sub_kind; 检查 features 是否有 plate 时更新 has_plate
    LiftSubKind EvaluateAlarm(int cam_index,
        const std::vector<FeatureRect>& features);

    // -------------------- 工具 --------------------
    static std::string CurrentTimestamp();
    static std::string AlarmKindString(LiftAlarmType t, LiftSubKind k);

private:
    AntiLiftConfig cfg_;

    std::unique_ptr<Yolov8Detector> detector_;
    bool                            detector_loaded_ = false;

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
        std::vector<FeatureRect>         features;
        LiftAlarmType                    last_alarm = LiftAlarmType::None;
        LiftSubKind                      last_sub_kind = LiftSubKind::None;

        // 录像: BeginSession 预置 temp_path, HandleFrame 里 lazy init writer
        std::unique_ptr<cv::VideoWriter> vw_raw;
        std::unique_ptr<cv::VideoWriter> vw_annot;
        std::string                      raw_temp_path;
        std::string                      annot_temp_path;

        std::chrono::steady_clock::time_point last_record_tick{};
    };
    std::array<PerCamera, 2> cams_;

    // ---- 录制队列 ----
    struct RecordItem {
        int     cam_index;
        bool    is_annotated;
        cv::Mat frame;
    };
    std::deque<RecordItem>          rec_queue_;
    std::mutex                      rec_mtx_;
    std::condition_variable         rec_cv_;
    static constexpr size_t         kRecordQueueMax = 256;

    // ---- 线程 ----
    std::array<std::thread, 2>      worker_threads_;
    std::thread                     record_thread_;
    std::atomic<bool>               is_running_{ false };
};