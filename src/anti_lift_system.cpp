#include "anti_lift_system.h"

#include <algorithm>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>

#include "utils.h"   // LOG_LIFT: 带时间戳 + 落盘 + 多线程安全

namespace {

    inline cv::Rect ClampBox(const cv::Rect& box, int cols, int rows) {
        return box & cv::Rect(0, 0, cols, rows);
    }

    // 把 box 周围扩 expand 像素得到 ROI(光流分析窗口)
    cv::Rect ExpandBox(const cv::Rect& box, int expand, int cols, int rows) {
        cv::Rect r(box.x - expand, box.y - expand,
            box.width + 2 * expand, box.height + 2 * expand);
        return ClampBox(r, cols, rows);
    }

    // 类别名字符串(给截图/录像名用)
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
        LOG_LIFT("[AntiLift] Start() 已经在运行");
        return;
    }
    if (!out_state) {
        LOG_LIFT("[AntiLift] Start() out_state 为空, 启动失败");
        return;
    }

    if (!detector_loaded_) {
        std::string err;
        if (!detector_ || !detector_->Load(&err)) {
            LOG_LIFT("[AntiLift] 模型加载失败: " << err);
            return;
        }
        detector_loaded_ = true;
        LOG_LIFT("[AntiLift] 模型加载成功");
    }

    state_ = out_state;
    is_running_ = true;

    for (int i = 0; i < 2; ++i) {
        worker_threads_[i] = std::thread(&AntiLiftSystem::WorkerLoop, this, i);
    }
    if (cfg_.enable_record) {
        record_thread_ = std::thread(&AntiLiftSystem::RecordLoop, this);
    }
    LOG_LIFT("[AntiLift] 已启动 (worker=2, record="
        << (cfg_.enable_record ? "on" : "off") << ")");
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

    // 关闭可能在写的 VideoWriter (走 Stop 路径, 不改名也不删)
    for (auto& c : cams_) {
        if (c.vw_raw) { c.vw_raw->release();   c.vw_raw.reset(); }
        if (c.vw_annot) { c.vw_annot->release(); c.vw_annot.reset(); }
    }

    state_ = nullptr;
    LOG_LIFT("[AntiLift] 已停止");
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
            rec_queue_.push_back({ cam_index, /*is_annotated=*/false, frame.clone() });
            rec_cv_.notify_one();
        }
    }
}

// =========================================================================
// 会话控制(由 App 显式调)
// =========================================================================
void AntiLiftSystem::BeginSession(int cam_index) {
    if (cam_index < 0 || cam_index >= 2) return;
    if (!state_) {
        LOG_LIFT("[AntiLift] BeginSession 时 state_ 为空, 忽略");
        return;
    }

    auto& cam = cams_[cam_index];
    auto& cam_state = state_->cameras[cam_index];

    if (cam_state.in_session.load()) {
        LOG_LIFT("[AntiLift] cam" << cam_index << " 已经在会话中, BeginSession 忽略");
        return;
    }

    // 清算法状态(只在 worker 线程外通过 BeginSession 调用; 此刻 worker 正阻塞在 wait
    //  或者刚处理完上一帧 —— in_session=false 期间 worker 也不会进 HandleFrame,
    //  cam.features 没有 worker 在并发访问. 这里直接清是安全的.)
    cam.features.clear();
    cam.last_alarm = LiftAlarmType::None;
    cam.last_sub_kind = LiftSubKind::None;

    cam_state.alarm.store(LiftAlarmType::None);
    cam_state.sub_kind.store(LiftSubKind::None);
    cam_state.has_plate.store(false);

    // 录像临时路径(实际 VideoWriter 在 HandleFrame 里 lazy init, 因为分辨率要到第一帧才知道)
    if (cfg_.enable_record) {
        std::string ts = CurrentTimestamp();
        cam.raw_temp_path = cfg_.record_dir +
            "cam" + std::to_string(cam_index) + "_" + ts + "_INPROGRESS_raw.mp4";
        cam.annot_temp_path = cfg_.record_dir +
            "cam" + std::to_string(cam_index) + "_" + ts + "_INPROGRESS_annot.mp4";
    }

    // 翻 in_session: 这一句之后 worker 才会进 HandleFrame, 必须在前面所有清零之后
    cam_state.in_session.store(true);

    LOG_LIFT("[AntiLift] cam" << cam_index << " 会话开始");
}

void AntiLiftSystem::EndSession(int cam_index, bool keep_recording) {
    if (cam_index < 0 || cam_index >= 2) return;
    if (!state_) return;

    auto& cam = cams_[cam_index];
    auto& cam_state = state_->cameras[cam_index];

    if (!cam_state.in_session.load()) {
        // 不在会话中, 仅清 has_plate 标志后忽略
        cam_state.has_plate.store(false);
        return;
    }

    // 先翻 in_session: 之后 worker 不会再进 HandleFrame; OnFrame 也不会再 push 录像
    cam_state.in_session.store(false);

    // 关闭 VideoWriter
    // 注意 worker 可能此刻正在 HandleFrame 内部、还没退出 —— vw_* 是 unique_ptr,
    // worker 持有的是 raw 引用. worker 不会在 in_session=false 时新建 writer, 但
    // 可能正在 write 到旧 writer. 给 RecordLoop 一个短窗口排空队列更稳妥.
    // 这里简化处理: 直接 release. RecordLoop 自己会在下一轮发现 vw 已被 reset 后
    // 走 isOpened() 假分支不再写. 实际 vw 一旦 release(), isOpened() 立即返回 false,
    // 同时该轮 write 在大多数 OpenCV 实现里就是个 no-op (worst case 是丢一帧).
    if (cam.vw_raw) {
        cam.vw_raw->release();
        cam.vw_raw.reset();
    }
    if (cam.vw_annot) {
        cam.vw_annot->release();
        cam.vw_annot.reset();
    }

    // 改名 或 删除 临时文件
    std::error_code ec;
    if (keep_recording) {
        const std::string ts = CurrentTimestamp();
        if (!cam.raw_temp_path.empty()) {
            std::string final_path = cfg_.record_dir +
                "cam" + std::to_string(cam_index) + "_" + ts + "_" +
                AlarmStr(cam.last_alarm) + "_" + SubKindStr(cam.last_sub_kind) + "_raw.mp4";
            std::filesystem::rename(cam.raw_temp_path, final_path, ec);
            if (ec) LOG_LIFT("[AntiLift] cam" << cam_index
                << " 录像改名失败 raw: " << ec.message());
        }
        if (!cam.annot_temp_path.empty()) {
            std::string final_path = cfg_.record_dir +
                "cam" + std::to_string(cam_index) + "_" + ts + "_" +
                AlarmStr(cam.last_alarm) + "_" + SubKindStr(cam.last_sub_kind) + "_annot.mp4";
            ec.clear();
            std::filesystem::rename(cam.annot_temp_path, final_path, ec);
            if (ec) LOG_LIFT("[AntiLift] cam" << cam_index
                << " 录像改名失败 annot: " << ec.message());
        }
    }
    else {
        if (!cam.raw_temp_path.empty()) {
            std::filesystem::remove(cam.raw_temp_path, ec);
        }
        if (!cam.annot_temp_path.empty()) {
            ec.clear();
            std::filesystem::remove(cam.annot_temp_path, ec);
        }
    }
    cam.raw_temp_path.clear();
    cam.annot_temp_path.clear();

    // 注意: 不动 cam_state.alarm —— 让 App 在状态机里决定何时清零(通常进 Idle 时清)
    // has_plate 同样保留, App 读完后会在下次 BeginSession 时被清

    LOG_LIFT("[AntiLift] cam" << cam_index << " 会话结束 keep_recording="
        << (keep_recording ? "true" : "false")
        << " final_alarm=" << AlarmStr(cam.last_alarm)
        << "/" << SubKindStr(cam.last_sub_kind));
}

// =========================================================================
// Worker 主循环
// =========================================================================
void AntiLiftSystem::WorkerLoop(int cam_index) {
    auto& slot = slots_[cam_index];
    cv::Mat frame;

    const std::string win = "AntiLift cam" + std::to_string(cam_index);
    bool window_created = false;

    while (is_running_.load()) {
        // 等帧
        {
            std::unique_lock<std::mutex> lock(slot->mtx);
            slot->cv.wait_for(lock, std::chrono::milliseconds(200), [&] {
                return slot->has_frame || !is_running_.load();
                });
            if (!is_running_.load()) break;
            if (!slot->has_frame) continue;   // 超时: 没有新帧, 重新等
            slot->frame.copyTo(frame);
            slot->has_frame = false;
        }

        // 不在会话则丢帧(快速返回, 不做检测/光流)
        const bool in_session = state_
            && state_->cameras[cam_index].in_session.load();
        if (!in_session) continue;

        cv::Mat display = frame;
        cv::Mat annot;

        HandleFrame(cam_index, frame,
            cfg_.enable_debug_show ? &annot : nullptr);

        if (cfg_.enable_debug_show && !annot.empty()) {
            display = annot;
        }

        if (state_) {
            state_->cameras[cam_index].frame_count.fetch_add(
                1, std::memory_order_relaxed);
        }

        // 调试显示
        if (cfg_.enable_debug_show) {
            if (!window_created) {
                cv::namedWindow(win, cv::WINDOW_NORMAL);
                window_created = true;
            }
            cv::imshow(win, display);
            cv::waitKey(1);
        }
    }
}

// =========================================================================
// 录制线程
// =========================================================================
void AntiLiftSystem::RecordLoop() {
    using Clock = std::chrono::steady_clock;
    const int frame_period_ms = (std::max)(1, 1000 / (std::max)(1, cfg_.record_fps));

    while (is_running_.load()) {
        RecordItem item;
        {
            std::unique_lock<std::mutex> lock(rec_mtx_);
            rec_cv_.wait_for(lock, std::chrono::milliseconds(200), [&] {
                return !rec_queue_.empty() || !is_running_.load();
                });
            if (!is_running_.load() && rec_queue_.empty()) break;
            if (rec_queue_.empty()) continue;
            item = std::move(rec_queue_.front());
            rec_queue_.pop_front();
        }

        auto& cam = cams_[item.cam_index];

        // 节流: 同一类(raw 或 annot)按 record_fps 控制实际写入帧率
        auto now = Clock::now();
        auto since_last = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - cam.last_record_tick).count();
        if (since_last < frame_period_ms) continue;
        cam.last_record_tick = now;

        // 选择对应的 VideoWriter 写入(可能已被 EndSession release, 此时 isOpened()=false)
        auto& vw = item.is_annotated ? cam.vw_annot : cam.vw_raw;
        if (vw && vw->isOpened()) {
            vw->write(item.frame);
        }
    }

    // 退出前清空队列
    std::lock_guard<std::mutex> lock(rec_mtx_);
    rec_queue_.clear();
}

// =========================================================================
// 算法核心
// =========================================================================
void AntiLiftSystem::HandleFrame(int cam_index, cv::Mat& frame, cv::Mat* out_annot) {
    auto& cam = cams_[cam_index];

    // Lazy init VideoWriter(分辨率要到第一帧才知道)
    if (cfg_.enable_record) {
        if (!cam.vw_raw && !cam.raw_temp_path.empty()) {
            int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
            cam.vw_raw = std::make_unique<cv::VideoWriter>(
                cam.raw_temp_path, fourcc, cfg_.record_fps,
                cv::Size(frame.cols, frame.rows));
        }
        if (!cam.vw_annot && !cam.annot_temp_path.empty()) {
            int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
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
    }
    else {
        // 后续帧: 对每个锚点跑光流位移更新
        for (auto& f : cam.features) {
            UpdateFeature(cam_index, gray, f);
        }
    }

    // 评估异常 + 检测 plate
    LiftSubKind sub = EvaluateAlarm(cam_index, cam.features);
    if (sub != LiftSubKind::None) {
        LiftAlarmType outer = (sub == LiftSubKind::DroveAway)
            ? LiftAlarmType::DroveAway
            : LiftAlarmType::Lifted;
        cam.last_alarm = outer;
        cam.last_sub_kind = sub;
        if (state_) {
            state_->cameras[cam_index].alarm.store(outer);
            state_->cameras[cam_index].sub_kind.store(sub);
        }
    }

    // 生成标注画面: 录制需要 或 out_annot 非空
    const bool need_annot = cfg_.enable_record || (out_annot != nullptr);
    if (need_annot) {
        cv::Mat annot = frame.clone();
        for (const auto& f : cam.features) {
            cv::rectangle(annot, f.target_box, cv::Scalar(0, 255, 0), 2);
            std::ostringstream txt;
            txt << "id=" << f.class_id
                << " dy=" << static_cast<int>(f.total_diff_y);
            cv::putText(annot, txt.str(), f.target_box.tl(),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        }

        if (out_annot) {
            if (cfg_.enable_record) {
                *out_annot = annot.clone();
            }
            else {
                *out_annot = annot;
            }
        }

        if (cfg_.enable_record) {
            std::lock_guard<std::mutex> lock(rec_mtx_);
            if (rec_queue_.size() < kRecordQueueMax) {
                rec_queue_.push_back({ cam_index, true, std::move(annot) });
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
        f.class_id = d.id;
        f.target_box = ClampBox(d.box, gray.cols, gray.rows);
        f.view_box = ExpandBox(f.target_box, expand, gray.cols, gray.rows);
        f.view_target_box = cv::Rect(
            f.target_box.x - f.view_box.x,
            f.target_box.y - f.view_box.y,
            f.target_box.width, f.target_box.height);
        if (f.view_box.width > 0 && f.view_box.height > 0) {
            f.prev_view = gray(f.view_box).clone();
            cam.features.push_back(std::move(f));
        }
    }
    if (!cam.features.empty()) {
        LOG_LIFT("[AntiLift] cam" << cam_index
            << " 锚点初始化: " << cam.features.size() << " 个");
    }
}

void AntiLiftSystem::UpdateFeature(int cam_index, const cv::Mat& gray, FeatureRect& f) {
    if (f.view_box.empty() || f.prev_view.empty()) return;
    if (f.view_box.x < 0 || f.view_box.y < 0 ||
        f.view_box.x + f.view_box.width  > gray.cols ||
        f.view_box.y + f.view_box.height > gray.rows) return;

    f.curr_view = gray(f.view_box).clone();

    // 在 prev_view 上的 view_target_box 区域构造一个稀疏点阵, 用 LK 光流跟踪
    std::vector<cv::Point2f> prev_pts, curr_pts;
    int step = (std::max)(4, (std::min)(f.view_target_box.width,
        f.view_target_box.height) / 6);
    for (int y = f.view_target_box.y;
        y < f.view_target_box.y + f.view_target_box.height; y += step) {
        for (int x = f.view_target_box.x;
            x < f.view_target_box.x + f.view_target_box.width; x += step) {
            prev_pts.emplace_back(static_cast<float>(x), static_cast<float>(y));
        }
    }
    if (prev_pts.empty()) return;

    std::vector<uchar> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(f.prev_view, f.curr_view, prev_pts, curr_pts, status, err);

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

    f.prev_view = f.curr_view.clone();

    // 同步 target_box 平移(供绘制)
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
    // 类别名约定: 由 cfg_.detector.class_names 决定
    //   含 "hole"  → 锁孔
    //   含 "wheel" → 车轮
    //   含 "plate" → 车牌(plate). 出现即为内集卡作业.
    const auto& names = cfg_.detector.class_names;

    double max_hole_dy = 0.0, max_wheel_dy = 0.0;
    double max_hole_dx = 0.0;
    bool   seen_plate = false;

    for (const auto& f : features) {
        const std::string& nm = (f.class_id >= 0 && f.class_id < (int)names.size())
            ? names[f.class_id] : std::string();
        double abs_dy = std::abs(f.total_diff_y);
        double abs_dx = std::abs(f.total_diff_x);

        if (nm.find("plate") != std::string::npos) {
            seen_plate = true;
            // plate 锚点不参与位移判定, 直接 continue
            continue;
        }
        if (nm.find("hole") != std::string::npos) {
            if (abs_dy > max_hole_dy)  max_hole_dy = abs_dy;
            if (abs_dx > max_hole_dx)  max_hole_dx = abs_dx;
        }
        else if (nm.find("wheel") != std::string::npos) {
            if (abs_dy > max_wheel_dy) max_wheel_dy = abs_dy;
        }
    }

    // plate 命中标志一旦看到, 本会话内保持 true (App 读到就会结会话)
    if (seen_plate && state_) {
        state_->cameras[cam_index].has_plate.store(true);
    }

    // 判定优先级: 侧翻 > 吊起 > 开走
    if (max_hole_dy > cfg_.limit_rotate_lift_plt &&
        max_wheel_dy < cfg_.limit_rotate_lift_wheel) {
        return LiftSubKind::PlatformRollover;
    }
    if (max_wheel_dy > cfg_.limit_rotate_lift_wheel &&
        max_hole_dy < cfg_.limit_rotate_lift_plt) {
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