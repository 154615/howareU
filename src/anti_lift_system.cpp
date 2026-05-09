#include "anti_lift_system.h"

#include <algorithm>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace {

    template <typename... Args>
    void Log(Args&&... args) {
        std::ostringstream oss;
        (oss << ... << args);
        std::cout << oss.str() << std::endl;
    }

    inline cv::Rect ClampBox(const cv::Rect& box, int cols, int rows) {
        return box & cv::Rect(0, 0, cols, rows);
    }

    // 把 box 周围扩 expand 像素得到 ROI(光流分析窗口)
    cv::Rect ExpandBox(const cv::Rect& box, int expand, int cols, int rows) {
        cv::Rect r(box.x - expand, box.y - expand,
                   box.width + 2 * expand, box.height + 2 * expand);
        return ClampBox(r, cols, rows);
    }

    // 类别名字符串(给截图/录像名用). 不依赖外部类别表; 直接按规则命名.
    const char* SubKindStr(LiftSubKind k) {
        switch (k) {
            case LiftSubKind::HoleLifted:       return "holelift";
            case LiftSubKind::WheelLifted:      return "wheellift";
            case LiftSubKind::PlatformRollover: return "pltroll";
            case LiftSubKind::WheelRollover:    return "whlroll";
            case LiftSubKind::DroveAway:        return "drove";
            default:                            return "none";
        }
    }

    const char* AlarmStr(LiftAlarmType a) {
        switch (a) {
            case LiftAlarmType::Lifted:      return "lifted";
            case LiftAlarmType::DroveAway:   return "drove";
            case LiftAlarmType::SystemError: return "syserr";
            default:                         return "safe";
        }
    }

}  // namespace

// =========================================================================
// 构造 / 析构
// =========================================================================
AntiLiftSystem::AntiLiftSystem(const AntiLiftConfig& cfg) : cfg_(cfg) {
    std::error_code ec;
    if (!std::filesystem::exists(cfg_.snapshot_dir, ec))
        std::filesystem::create_directories(cfg_.snapshot_dir, ec);
    if (cfg_.enable_record && !std::filesystem::exists(cfg_.record_dir, ec))
        std::filesystem::create_directories(cfg_.record_dir, ec);

    for (int i = 0; i < 2; ++i) {
        slots_[i] = std::make_unique<FrameSlot>();
    }

    detector_ = std::make_unique<Yolov8Detector>(cfg_.detector);
}

AntiLiftSystem::~AntiLiftSystem() { Stop(); }

// =========================================================================
// Start / Stop
// =========================================================================
void AntiLiftSystem::Start(AntiLiftState* out_state) {
    if (is_running_.load()) {
        Log("[AntiLift] Start() 已经在运行");
        return;
    }
    if (!out_state) {
        Log("[AntiLift] Start() out_state 为空, 启动失败");
        return;
    }

    if (!detector_loaded_) {
        std::string err;
        if (!detector_ || !detector_->Load(&err)) {
            Log("[AntiLift] 模型加载失败: ", err);
            return;
        }
        detector_loaded_ = true;
        Log("[AntiLift] 模型加载成功");
    }

    state_ = out_state;
    is_running_ = true;

    for (int i = 0; i < 2; ++i) {
        worker_threads_[i] = std::thread(&AntiLiftSystem::WorkerLoop, this, i);
    }
    if (cfg_.enable_record) {
        record_thread_ = std::thread(&AntiLiftSystem::RecordLoop, this);
    }
    Log("[AntiLift] 已启动 (worker=2, record=", cfg_.enable_record ? "on" : "off", ")");
}

void AntiLiftSystem::Stop() {
    if (!is_running_.exchange(false)) return;

    // 通知所有 worker 退出
    for (int i = 0; i < 2; ++i) {
        if (slots_[i]) {
            std::lock_guard<std::mutex> lock(slots_[i]->mtx);
            slots_[i]->cv.notify_all();
        }
    }
    rec_cv_.notify_all();

    for (auto& t : worker_threads_) if (t.joinable()) t.join();
    if (record_thread_.joinable()) record_thread_.join();

    // 关闭可能在写的 VideoWriter
    for (auto& c : cams_) {
        if (c.vw_raw)   { c.vw_raw->release();   c.vw_raw.reset(); }
        if (c.vw_annot) { c.vw_annot->release(); c.vw_annot.reset(); }
    }

    state_ = nullptr;
    Log("[AntiLift] 已停止");
}

// =========================================================================
// FrameSink 接口
// =========================================================================
void AntiLiftSystem::OnFrame(int cam_index, const cv::Mat& frame) {
    if (cam_index < 0 || cam_index >= 2 || frame.empty()) return;
    auto& slot = slots_[cam_index];

    {
        std::lock_guard<std::mutex> lock(slot->mtx);
        frame.copyTo(slot->frame);
        slot->has_frame = true;
    }
    slot->cv.notify_one();

    // 录制原始帧(只在会话期间)
    if (cfg_.enable_record &&
        state_ && state_->cameras[cam_index].in_session.load()) {
        std::lock_guard<std::mutex> lock(rec_mtx_);
        if (rec_queue_.size() < kRecordQueueMax) {
            rec_queue_.push_back({cam_index, /*is_annotated=*/false, frame.clone()});
            rec_cv_.notify_one();
        }
    }
}

// =========================================================================
// PLC 输入接口
// =========================================================================
void AntiLiftSystem::UpdatePlcInputs(const AntiLiftPlcInputs& in) {
    std::lock_guard<std::mutex> lock(plc_mtx_);
    latest_plc_inputs_ = in;
}

// =========================================================================
// Worker 主循环
// =========================================================================
void AntiLiftSystem::WorkerLoop(int cam_index) {
    auto& slot = slots_[cam_index];
    cv::Mat frame;

    while (is_running_.load()) {
        // 等帧
        {
            std::unique_lock<std::mutex> lock(slot->mtx);
            slot->cv.wait_for(lock, std::chrono::milliseconds(200), [&]{
                return slot->has_frame || !is_running_.load();
            });
            if (!is_running_.load()) break;
            if (!slot->has_frame) {
                // 超时 → 检查会话是否要超时结束
                TryStartSession(cam_index);   // 同时也判结束
                continue;
            }
            slot->frame.copyTo(frame);
            slot->has_frame = false;
        }

        // 1) 会话状态机判定(无论会不会推理都要走)
        TryStartSession(cam_index);

        bool in_session = state_ && state_->cameras[cam_index].in_session.load();
        if (!in_session) {
            // 不在会话, 就不做检测, 不计帧, 不录制
            continue;
        }

        // 2) 在会话: 跑算法
        HandleFrame(cam_index, frame);

        if (state_) {
            state_->cameras[cam_index].frame_count.fetch_add(1, std::memory_order_relaxed);
        }
    }
}

// =========================================================================
// 录制线程
// =========================================================================
void AntiLiftSystem::RecordLoop() {
    using Clock = std::chrono::steady_clock;
    const int frame_period_ms = std::max(1, 1000 / std::max(1, cfg_.record_fps));

    while (is_running_.load()) {
        RecordItem item;
        {
            std::unique_lock<std::mutex> lock(rec_mtx_);
            rec_cv_.wait_for(lock, std::chrono::milliseconds(200), [&]{
                return !rec_queue_.empty() || !is_running_.load();
            });
            if (!is_running_.load() && rec_queue_.empty()) break;
            if (rec_queue_.empty()) continue;
            item = std::move(rec_queue_.front());
            rec_queue_.pop_front();
        }

        auto& cam = cams_[item.cam_index];

        // 节流: 同一类(raw 或 annot) 100ms 内只取一帧, 实现 10fps
        // 这里做简单的"距离上次写入间隔判断". raw 和 annot 共享 last_record_tick
        // 是合理的, 因为它们各自的写入频率天然就是 ~25fps, 节流到 10fps 后是
        // 二者交替写入, 实际每条流大约 5fps —— 对故障复盘足够.
        // 如果需要严格分别 10fps, 可以拆成两个 last_tick.
        auto now = Clock::now();
        auto since_last = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - cam.last_record_tick).count();
        if (since_last < frame_period_ms) {
            continue;  // 丢弃
        }
        cam.last_record_tick = now;

        // 选择对应的 VideoWriter 写入
        auto& vw = item.is_annotated ? cam.vw_annot : cam.vw_raw;
        if (vw && vw->isOpened()) {
            vw->write(item.frame);
        }
    }

    // 退出前清空队列(主要是为了让正在写的文件落盘)
    std::lock_guard<std::mutex> lock(rec_mtx_);
    rec_queue_.clear();
}

// =========================================================================
// 会话状态机
// =========================================================================
void AntiLiftSystem::TryStartSession(int cam_index) {
    using Clock = std::chrono::steady_clock;
    if (!state_) return;

    AntiLiftPlcInputs in;
    {
        std::lock_guard<std::mutex> lock(plc_mtx_);
        in = latest_plc_inputs_;
    }

    auto& cam = cams_[cam_index];
    auto& cam_state = state_->cameras[cam_index];
    bool now_in_session = cam_state.in_session.load();

    // 启动条件: 闭锁 && 着箱 && 起升中
    bool start_cond = (in.lock_status == 1) &&
                      (in.box_landed == 1) &&
                      (in.spd_hoist > 0);

    // 结束条件: 复位按下 || 起升停止持续 session_idle_ms
    bool end_cond_explicit = (in.reset_signal == 1);

    auto now = Clock::now();
    if (now_in_session) {
        // 已经在会话中, 看是否该结束
        if (in.spd_hoist > 0) {
            cam.last_motion_time = now;   // 还在动
        }
        auto idle_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - cam.last_motion_time).count();
        bool end_cond_idle = (idle_ms >= cfg_.session_idle_ms);

        if (end_cond_explicit || end_cond_idle) {
            // 结束: 用当前 last_alarm 作为最终结果
            EndSession(cam_index, cam.last_alarm, cam.last_sub_kind);
        }
    } else {
        // 不在会话, 看是否该启动
        if (start_cond) {
            cam.session_start    = now;
            cam.last_motion_time = now;
            cam.last_alarm       = LiftAlarmType::None;
            cam.last_sub_kind    = LiftSubKind::None;
            cam.features.clear();
            cam_state.alarm.store(LiftAlarmType::None);
            cam_state.sub_kind.store(LiftSubKind::None);
            cam_state.in_session.store(true);

            // 创建 VideoWriter (会话结束时再改名)
            if (cfg_.enable_record) {
                std::string ts = CurrentTimestamp();
                cam.raw_temp_path = cfg_.record_dir +
                    "cam" + std::to_string(cam_index) + "_" + ts + "_INPROGRESS_raw.mp4";
                cam.annot_temp_path = cfg_.record_dir +
                    "cam" + std::to_string(cam_index) + "_" + ts + "_INPROGRESS_annot.mp4";
                int fourcc = cv::VideoWriter::fourcc('m','p','4','v');
                // 分辨率延后到第一帧时再确定 —— 这里先存空指针, 在 HandleFrame 里 lazy init
            }
            Log("[AntiLift] cam", cam_index, " 会话启动");
        }
    }
}

void AntiLiftSystem::EndSession(int cam_index, LiftAlarmType result, LiftSubKind sub) {
    auto& cam = cams_[cam_index];
    auto& cam_state = state_->cameras[cam_index];

    cam_state.alarm.store(result);
    cam_state.sub_kind.store(sub);
    cam_state.in_session.store(false);

    // 关闭录像并改名
    if (cam.vw_raw) {
        cam.vw_raw->release();
        cam.vw_raw.reset();
        std::string final_path = cfg_.record_dir +
            "cam" + std::to_string(cam_index) + "_" + CurrentTimestamp() + "_" +
            AlarmStr(result) + "_" + SubKindStr(sub) + "_raw.mp4";
        std::error_code ec;
        std::filesystem::rename(cam.raw_temp_path, final_path, ec);
    }
    if (cam.vw_annot) {
        cam.vw_annot->release();
        cam.vw_annot.reset();
        std::string final_path = cfg_.record_dir +
            "cam" + std::to_string(cam_index) + "_" + CurrentTimestamp() + "_" +
            AlarmStr(result) + "_" + SubKindStr(sub) + "_annot.mp4";
        std::error_code ec;
        std::filesystem::rename(cam.annot_temp_path, final_path, ec);
    }

    Log("[AntiLift] cam", cam_index, " 会话结束: ",
        AlarmStr(result), "/", SubKindStr(sub));
}

// =========================================================================
// 算法核心
// =========================================================================
void AntiLiftSystem::HandleFrame(int cam_index, cv::Mat& frame) {
    auto& cam = cams_[cam_index];

    // Lazy init VideoWriter
    if (cfg_.enable_record) {
        if (!cam.vw_raw) {
            int fourcc = cv::VideoWriter::fourcc('m','p','4','v');
            cam.vw_raw = std::make_unique<cv::VideoWriter>(
                cam.raw_temp_path, fourcc, cfg_.record_fps,
                cv::Size(frame.cols, frame.rows));
        }
        if (!cam.vw_annot) {
            int fourcc = cv::VideoWriter::fourcc('m','p','4','v');
            cam.vw_annot = std::make_unique<cv::VideoWriter>(
                cam.annot_temp_path, fourcc, cfg_.record_fps,
                cv::Size(frame.cols, frame.rows));
        }
    }

    // 灰度化(光流要灰度图)
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    // 第一帧: 跑检测建锚点
    if (cam.features.empty()) {
        std::vector<OutputParams> dets;
        cv::Mat tmp = frame.clone();
        if (detector_ && detector_->Detect(tmp, dets)) {
            InitFeatures(cam_index, gray, dets);
        }
    } else {
        // 后续帧: 对每个锚点跑光流位移更新
        for (auto& f : cam.features) {
            UpdateFeature(cam_index, gray, f);
        }
    }

    // 评估异常
    LiftSubKind sub = EvaluateAlarm(cam_index, cam.features);
    if (sub != LiftSubKind::None) {
        LiftAlarmType outer = (sub == LiftSubKind::DroveAway)
                              ? LiftAlarmType::DroveAway
                              : LiftAlarmType::Lifted;
        cam.last_alarm    = outer;
        cam.last_sub_kind = sub;
        if (state_) {
            state_->cameras[cam_index].alarm.store(outer);
            state_->cameras[cam_index].sub_kind.store(sub);
        }
    }

    // 调试可视化 + 录制 annotated
    if (cfg_.enable_debug_show || cfg_.enable_record) {
        cv::Mat annot = frame.clone();
        for (const auto& f : cam.features) {
            cv::rectangle(annot, f.target_box, cv::Scalar(0, 255, 0), 2);
            std::ostringstream txt;
            txt << "id=" << f.class_id
                << " dy=" << static_cast<int>(f.total_diff_y);
            cv::putText(annot, txt.str(), f.target_box.tl(),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        }
        if (cfg_.enable_debug_show) {
            std::string win = "AntiLift cam" + std::to_string(cam_index);
            cv::namedWindow(win, cv::WINDOW_NORMAL);
            cv::imshow(win, annot);
            cv::waitKey(1);
        }
        if (cfg_.enable_record) {
            std::lock_guard<std::mutex> lock(rec_mtx_);
            if (rec_queue_.size() < kRecordQueueMax) {
                rec_queue_.push_back({cam_index, true, std::move(annot)});
                rec_cv_.notify_one();
            }
        }
    }
}

void AntiLiftSystem::InitFeatures(int cam_index, const cv::Mat& gray,
                                  const std::vector<OutputParams>& dets) {
    auto& cam = cams_[cam_index];
    const int expand = 30;  // ROI 扩边
    for (const auto& d : dets) {
        FeatureRect f;
        f.class_id   = d.id;
        f.target_box = ClampBox(d.box, gray.cols, gray.rows);
        f.view_box   = ExpandBox(f.target_box, expand, gray.cols, gray.rows);
        f.view_target_box = cv::Rect(
            f.target_box.x - f.view_box.x,
            f.target_box.y - f.view_box.y,
            f.target_box.width, f.target_box.height);
        if (f.view_box.width > 0 && f.view_box.height > 0) {
            f.prev_view = gray(f.view_box).clone();
            cam.features.push_back(std::move(f));
        }
    }
    if (cam.features.empty()) {
        // 没检到, 下一帧再试 —— features 还是空, HandleFrame 还会重试 Detect
    } else {
        Log("[AntiLift] cam", cam_index, " 锚点初始化: ", cam.features.size(), " 个");
    }
}

void AntiLiftSystem::UpdateFeature(int cam_index, const cv::Mat& gray, FeatureRect& f) {
    if (f.view_box.empty() || f.prev_view.empty()) return;
    if (f.view_box.x < 0 || f.view_box.y < 0 ||
        f.view_box.x + f.view_box.width  > gray.cols ||
        f.view_box.y + f.view_box.height > gray.rows) return;

    f.curr_view = gray(f.view_box).clone();

    // 在 prev_view 上的 view_target_box 为初始点集, 用 LK 光流跟踪
    std::vector<cv::Point2f> prev_pts, curr_pts;
    // 在目标框中心建一个稀疏点阵(简单可靠)
    int step = std::max(4, std::min(f.view_target_box.width, f.view_target_box.height) / 6);
    for (int y = f.view_target_box.y; y < f.view_target_box.y + f.view_target_box.height; y += step) {
        for (int x = f.view_target_box.x; x < f.view_target_box.x + f.view_target_box.width; x += step) {
            prev_pts.emplace_back(static_cast<float>(x), static_cast<float>(y));
        }
    }
    if (prev_pts.empty()) return;

    std::vector<uchar> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(f.prev_view, f.curr_view, prev_pts, curr_pts, status, err);

    // 取所有 status==1 的位移均值
    double sx = 0.0, sy = 0.0;
    int n = 0;
    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) {
            sx += curr_pts[i].x - prev_pts[i].x;
            sy += curr_pts[i].y - prev_pts[i].y;
            ++n;
        }
    }
    if (n > 0) {
        f.cur_diff_x = sx / n;
        f.cur_diff_y = sy / n;
        f.total_diff_x += f.cur_diff_x;
        f.total_diff_y += f.cur_diff_y;
    }

    // 更新 prev
    f.prev_view = f.curr_view.clone();

    // 同步把 target_box 平移(供绘制)
    f.target_box.x = ClampBox(
        cv::Rect(f.target_box.x + static_cast<int>(f.cur_diff_x),
                 f.target_box.y + static_cast<int>(f.cur_diff_y),
                 f.target_box.width, f.target_box.height),
        gray.cols, gray.rows).x;
    f.target_box.y = ClampBox(
        cv::Rect(f.target_box.x,
                 f.target_box.y + static_cast<int>(f.cur_diff_y),
                 f.target_box.width, f.target_box.height),
        gray.cols, gray.rows).y;
}

LiftSubKind AntiLiftSystem::EvaluateAlarm(int cam_index,
                                          const std::vector<FeatureRect>& features) {
    // 按类别 id 把 total_diff_y 划分到"锁孔"和"车轮"
    // 类别名约定: 由 cfg_.detector.class_names 决定; 这里按经验命名做匹配
    // 名字含 "hole" → 锁孔, 含 "wheel" → 车轮.
    const auto& names = cfg_.detector.class_names;

    double max_hole_dy  = 0.0, max_wheel_dy = 0.0;
    double max_hole_dx  = 0.0;
    double max_total_dy = 0.0;

    for (const auto& f : features) {
        const std::string& nm = (f.class_id >= 0 && f.class_id < (int)names.size())
                                ? names[f.class_id] : std::string();
        double abs_dy = std::abs(f.total_diff_y);
        double abs_dx = std::abs(f.total_diff_x);

        if (nm.find("hole") != std::string::npos) {
            if (abs_dy > max_hole_dy)  max_hole_dy = abs_dy;
            if (abs_dx > max_hole_dx)  max_hole_dx = abs_dx;
        } else if (nm.find("wheel") != std::string::npos) {
            if (abs_dy > max_wheel_dy) max_wheel_dy = abs_dy;
        }
        if (abs_dy > max_total_dy) max_total_dy = abs_dy;
    }

    // 判定优先级: 侧翻 > 吊起 > 开走
    if (max_hole_dy  > cfg_.limit_rotate_lift_plt &&
        max_wheel_dy < cfg_.limit_rotate_lift_wheel) {
        return LiftSubKind::PlatformRollover;
    }
    if (max_wheel_dy > cfg_.limit_rotate_lift_wheel &&
        max_hole_dy  < cfg_.limit_rotate_lift_plt) {
        return LiftSubKind::WheelRollover;
    }
    if (max_hole_dy > cfg_.limit_hole_y) {
        return LiftSubKind::HoleLifted;
    }
    if (max_wheel_dy > cfg_.limit_wheel_y) {
        return LiftSubKind::WheelLifted;
    }
    if (max_hole_dx > cfg_.limit_horizon_left) {
        return LiftSubKind::DroveAway;
    }
    return LiftSubKind::None;
}

// =========================================================================
// 工具
// =========================================================================
std::string AntiLiftSystem::CurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count() % 1000;
    std::tm tm_buf;
#if defined(_WIN32)
    localtime_s(&tm_buf, &t);
#else
    localtime_r(&t, &tm_buf);
#endif
    std::ostringstream oss;
    oss << std::put_time(&tm_buf, "%Y%m%d_%H%M%S")
        << "_" << std::setw(3) << std::setfill('0') << ms;
    return oss.str();
}

std::string AntiLiftSystem::AlarmKindString(LiftAlarmType t, LiftSubKind k) {
    return std::string(AlarmStr(t)) + "_" + std::string(SubKindStr(k));
}
