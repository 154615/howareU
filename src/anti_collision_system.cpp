#include "anti_collision_system.h"

#include <algorithm>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "utils.h"   // LOG_AC / LOG_COMMON / SAFE_LOG: 带时间戳 + 落盘 + 多线程安全

namespace {

    const std::vector<cv::Scalar>& ClassColors() {
        static const std::vector<cv::Scalar> kColors = {
            cv::Scalar(0,   0, 255), // 0 红
            cv::Scalar(255, 255, 255), // 1 白
            cv::Scalar(255,   0,   0), // 2 蓝
            cv::Scalar(255, 255,   0), // 3 青
            cv::Scalar(0, 255,   0), // 4 绿
            cv::Scalar(255,   0, 255), // 5 粉
            cv::Scalar(0, 165, 255), // 6 橙
            cv::Scalar(128,   0, 128), // 7 紫
            cv::Scalar(0, 204, 255), // 8 金
        };
        return kColors;
    }

    inline uint64_t SteadyNowMs() {
        return static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count());
    }

    inline cv::Rect ClampBox(const cv::Rect& box, int cols, int rows) {
        return box & cv::Rect(0, 0, cols, rows);
    }

}  // namespace

// =========================================================================
// 构造 / 析构
// =========================================================================
AntiCollisionSystem::AntiCollisionSystem(const AntiCollisionConfig& cfg)
    : cfg_(cfg) {

    std::error_code ec;
    if (!std::filesystem::exists(cfg_.snapshot_dir, ec)) {
        std::filesystem::create_directories(cfg_.snapshot_dir, ec);
    }

    auto far_past = std::chrono::steady_clock::now() - std::chrono::hours(24);
    for (int i = 0; i < 4; ++i) {
        last_stop_time_[i] = far_past;
        last_decel_time_[i] = far_past;
        prev_level_[i] = AlarmLevel::Safe;
        slots_[i] = std::make_unique<FrameSlot>();
    }

    if (cfg_.split_ratio <= 0.0f || cfg_.split_ratio >= 1.0f) {
        LOG_AC("[AntiCollision] split_ratio 配置异常 (" << cfg_.split_ratio
            << ") 已强制设为 0.5");
        cfg_.split_ratio = 0.5f;
    }

    for (int i = 0; i < 4; ++i) {
        regions_[i] = BuildRegion(cfg_.regions[i], cfg_.split_ratio);
    }

    // 创建检测器实例 (此时只构造, 模型加载延后到 Start)
    detector_ = std::make_unique<Yolov8Detector>(cfg_.detector);
}

AntiCollisionSystem::~AntiCollisionSystem() {
    Stop();
}

// =========================================================================
// Start / Stop
// =========================================================================
void AntiCollisionSystem::Start(AntiCollisionState* out_state) {
    if (is_running_.load()) {
        LOG_AC("[AntiCollision] Start() 已经在运行");
        return;
    }
    if (out_state == nullptr) {
        LOG_AC("[AntiCollision] Start() out_state 为空，启动失败");
        return;
    }

    // 严格校验 4 路区域
    bool ok = true;
    for (int i = 0; i < 4; ++i) {
        if (!regions_[i].valid) {
            LOG_AC("[AntiCollision] 启动失败：cam" << (i + 1) << " 区域配置无效");
            ok = false;
        }
    }
    if (!ok) return;

    if (!detector_loaded_) {
        std::string err;
        if (!detector_ || !detector_->Load(&err)) {
            LOG_AC("[AntiCollision] 模型加载失败: " << err);
            return;
        }
        detector_loaded_ = true;
        LOG_AC("[AntiCollision] 模型加载成功");
    }

    state_ = out_state;
    is_running_ = true;

    for (int i = 0; i < 4; ++i) {
        worker_threads_[i] = std::thread(
            &AntiCollisionSystem::WorkerLoop, this, i);
    }

    if (cfg_.retain_days > 0) {
        cleanup_thread_ = std::thread(&AntiCollisionSystem::DiskCleanupLoop, this);
        LOG_AC("[AntiCollision] 磁盘清理线程已启动，保留天数: " << cfg_.retain_days);
    }
}

void AntiCollisionSystem::Stop() {
    if (!is_running_.exchange(false)) return;

    // 唤醒所有 worker
    for (auto& slot : slots_) {
        if (!slot) continue;
        {
            std::lock_guard<std::mutex> lock(slot->mtx);
        }
        slot->cv.notify_all();
    }

    for (auto& t : worker_threads_) {
        if (t.joinable()) t.join();
    }
    if (cleanup_thread_.joinable()) cleanup_thread_.join();

    if (cfg_.enable_debug_show) {
        cv::destroyAllWindows();
    }
}

// =========================================================================
// FrameSink::OnFrame —— 在生产端线程被调用，只做"覆盖+通知"
// =========================================================================
void AntiCollisionSystem::OnFrame(int cam_index, const cv::Mat& frame) {
    if (!is_running_.load()) return;
    if (cam_index < 0 || cam_index >= 4) return;
    if (frame.empty()) return;

    auto& slot = slots_[cam_index];

    // 关键: 必须深拷贝。
    // OnFrame 在生产端 PollLoop 线程被调用,frame 引用的是 PollLoop 的局部变量。
    // 海康 SDK 的 getLatestFrame 用 copyTo 写入同一个 cv::Mat,如果浅拷贝传给我们,
    // 在 worker 处理期间 PollLoop 下一轮 copyTo 会就地覆盖底层像素数据,
    // 造成画面撕裂或"看到下一帧"。深拷贝隔断这种竞态。
    cv::Mat owned;
    frame.copyTo(owned);

    {
        std::lock_guard<std::mutex> lock(slot->mtx);
        slot->frame = std::move(owned);
        slot->has_frame = true;
    }
    slot->cv.notify_one();
}

// =========================================================================
// Worker 主循环
// =========================================================================
void AntiCollisionSystem::WorkerLoop(int cam_index) {
    auto& slot = slots_[cam_index];
    cv::Mat frame;
    std::vector<OutputParams> detections;

    while (is_running_.load()) {
        // 等帧
        {
            std::unique_lock<std::mutex> lock(slot->mtx);
            slot->cv.wait_for(lock, std::chrono::milliseconds(200),
                [&] { return slot->has_frame || !is_running_.load(); });

            if (!is_running_.load()) break;
            if (!slot->has_frame) continue;

            frame = std::move(slot->frame);   // 拿走帧
            slot->has_frame = false;
        }
        if (frame.empty()) continue;

        // OnFrame 已经做过深拷贝,frame 是独占数据,可直接处理
        // (不再需要二次 copyTo)

        auto& status = state_->cameras[cam_index];
        status.has_input.store(true, std::memory_order_relaxed);
        status.last_frame_tick_ms.store(SteadyNowMs(), std::memory_order_relaxed);

        // 推理 (Yolov8Detector 内部自带锁, 多 worker 并发会被串行化)
        detections.clear();
        bool ok = detector_->Detect(frame, detections);
        if (!ok) continue;

        ProcessOneFrame(cam_index, frame, detections);

        status.frame_count.fetch_add(1, std::memory_order_relaxed);

        if (cfg_.enable_debug_show) {
            std::string win = "Camera " + std::to_string(cam_index + 1);
            cv::namedWindow(win, cv::WINDOW_NORMAL);
            cv::imshow(win, frame);
            cv::waitKey(1);
        }
    }
}

// =========================================================================
// 业务处理 (与上一版一致，只是参数 frame 已是 work_frame)
// =========================================================================
void AntiCollisionSystem::ProcessOneFrame(
    int cam_index,
    cv::Mat& frame,
    const std::vector<OutputParams>& detections) {

    const AlarmRegion& region = regions_[cam_index];
    if (!region.valid) return;

    cv::Mat clean_frame = frame.clone();
    const auto& colors = ClassColors();

    // ----- 绘制检测结果 -----
    for (const auto& target : detections) {
        if (target.confidence <= 0.3f) continue;

        cv::Scalar color =
            (target.id >= 0 && target.id < static_cast<int>(colors.size()))
            ? colors[target.id]
            : cv::Scalar(128, 128, 128);

        cv::Rect safe_box = ClampBox(target.box, frame.cols, frame.rows);

        if (safe_box.area() > 0 && target.boxMask.size() == safe_box.size()) {
            cv::Mat roi = frame(safe_box);
            cv::Mat color_roi(roi.size(), roi.type(), color);
            cv::Mat blended;
            cv::addWeighted(roi, 0.5, color_roi, 0.5, 0.0, blended);

            cv::Mat bin_mask;
            cv::threshold(target.boxMask, bin_mask, 127, 255, cv::THRESH_BINARY);
            blended.copyTo(roi, bin_mask);
        }

        cv::rectangle(frame, target.box, color, 1);

        std::ostringstream info;
        info << "ID:" << target.id
            << " CONF:" << std::fixed << std::setprecision(2) << target.confidence
            << " BOX:[" << target.box.x << "," << target.box.y << ","
            << target.box.width << "," << target.box.height << "]";

        cv::Point text_pos = target.box.tl() + cv::Point(0, -5);
        if (text_pos.y < 15) text_pos.y = target.box.y + 15;

        cv::putText(frame, info.str(), text_pos,
            cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 2);
        cv::putText(frame, info.str(), text_pos,
            cv::FONT_HERSHEY_SIMPLEX, 0.4, color, 1);
    }

    // ----- 区域线 -----
    cv::polylines(frame, std::vector<std::vector<cv::Point>>{region.decel_points},
        true, cv::Scalar(0, 255, 255), 2);
    cv::polylines(frame, std::vector<std::vector<cv::Point>>{region.stop_points},
        true, cv::Scalar(0, 0, 255), 2);

    // ----- 入侵判定 -----
    bool current_stop_intruded = false;
    bool current_decel_intruded = false;

    for (const auto& target : detections) {
        if (target.confidence < cfg_.conf_threshold) continue;
        if (target.box.height < 15 || target.box.width < 15) continue;

        cv::Mat align_mask = cv::Mat::zeros(region.decel_mask.size(), CV_8UC1);
        cv::Rect safe_box = ClampBox(target.box,
            region.decel_mask.cols,
            region.decel_mask.rows);
        if (safe_box.area() <= 0) continue;
        if (target.boxMask.size() != safe_box.size()) continue;

        target.boxMask.copyTo(align_mask(safe_box));
        cv::threshold(align_mask, align_mask, 127, 255, cv::THRESH_BINARY);

        cv::Mat stop_inter;
        cv::bitwise_and(region.stop_mask, align_mask, stop_inter);
        if (cv::countNonZero(stop_inter) > cfg_.intrusion_pixel_thresh) {
            current_stop_intruded = true;
            last_stop_time_[cam_index] = std::chrono::steady_clock::now();
            cv::rectangle(frame, target.box, cv::Scalar(0, 0, 255), 4);
            cv::putText(frame, "STOP",
                target.box.tl() + cv::Point(0, 30),
                cv::FONT_HERSHEY_SIMPLEX, 1.2,
                cv::Scalar(0, 0, 255), 3);
            continue;
        }

        cv::Mat decel_inter;
        cv::bitwise_and(region.decel_mask, align_mask, decel_inter);
        if (cv::countNonZero(decel_inter) > cfg_.intrusion_pixel_thresh) {
            current_decel_intruded = true;
            last_decel_time_[cam_index] = std::chrono::steady_clock::now();
            cv::rectangle(frame, target.box, cv::Scalar(0, 255, 255), 4);
            cv::putText(frame, "SLOW",
                target.box.tl() + cv::Point(0, 30),
                cv::FONT_HERSHEY_SIMPLEX, 1.2,
                cv::Scalar(0, 255, 255), 3);
        }
    }

    // ----- 状态机 -----
    auto now = std::chrono::steady_clock::now();
    auto stop_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_stop_time_[cam_index]).count();
    auto decel_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_decel_time_[cam_index]).count();

    bool final_stop = (stop_elapsed < cfg_.alarm_hold_ms);
    bool final_decel = (decel_elapsed < cfg_.alarm_hold_ms) && !final_stop;

    AlarmLevel new_level = AlarmLevel::Safe;
    if (final_stop)       new_level = AlarmLevel::Stop;
    else if (final_decel) new_level = AlarmLevel::Decel;

    AlarmLevel old_level = prev_level_[cam_index];

    bool need_snapshot = false;
    if (new_level == AlarmLevel::Stop && old_level != AlarmLevel::Stop &&
        current_stop_intruded) {
        need_snapshot = true;
    }
    else if (new_level == AlarmLevel::Decel && old_level == AlarmLevel::Safe &&
        current_decel_intruded) {
        need_snapshot = true;
    }

    // 方位标签 (与 anti_collision_app 的方位映射保持一致)
    // cam0/cam2 → 东向, cam1/cam3 → 西向
    const char* dir_tag = (cam_index == 0 || cam_index == 2) ? "东向" : "西向";

    if (need_snapshot) {
        std::string ts = CurrentTimestamp();
        std::string base = cfg_.snapshot_dir + "cam" +
            std::to_string(cam_index + 1) + "_";
        cv::imwrite(base + "draw_" + ts + ".jpg", frame);
        cv::imwrite(base + "raw_" + ts + ".jpg", clean_frame);
        LOG_AC("[报警] 相机 " << (cam_index + 1)
            << " 区域被入侵, 触发"
            << (final_stop ? "急停" : "减速")
            << ", 已截图");
    }

    if (new_level != old_level) {
        std::string trans;
        if (new_level == AlarmLevel::Stop)       trans = "触发急停";
        else if (new_level == AlarmLevel::Decel) trans = "触发减速";
        else                                     trans = "恢复安全 (解除报警)";

        LOG_AC("[状态跃迁] 相机 " << (cam_index + 1)
            << " (" << dir_tag << ") 状态变化 -> " << trans);
        prev_level_[cam_index] = new_level;
    }

    state_->cameras[cam_index].level.store(new_level, std::memory_order_relaxed);
}

// =========================================================================
// 区域构建 / 时间戳 / 清理 (与上一版一致)
// =========================================================================
AlarmRegion AntiCollisionSystem::BuildRegion(const CameraRegionConfig& cfg,
    float split_ratio) {
    AlarmRegion region;
    if (cfg.quad.size() != 4 ||
        cfg.frame_width <= 0 || cfg.frame_height <= 0) {
        return region;
    }

    cv::Point tl = cfg.quad[0], tr = cfg.quad[1];
    cv::Point br = cfg.quad[2], bl = cfg.quad[3];

    cv::Point mid_l(
        tl.x + static_cast<int>((bl.x - tl.x) * split_ratio),
        tl.y + static_cast<int>((bl.y - tl.y) * split_ratio));
    cv::Point mid_r(
        tr.x + static_cast<int>((br.x - tr.x) * split_ratio),
        tr.y + static_cast<int>((br.y - tr.y) * split_ratio));

    region.decel_points = { tl, tr, mid_r, mid_l };
    region.stop_points = { mid_l, mid_r, br, bl };

    region.decel_mask = cv::Mat::zeros(cfg.frame_height, cfg.frame_width, CV_8UC1);
    region.stop_mask = cv::Mat::zeros(cfg.frame_height, cfg.frame_width, CV_8UC1);

    cv::fillPoly(region.decel_mask,
        std::vector<std::vector<cv::Point>>{region.decel_points},
        cv::Scalar(255));
    cv::fillPoly(region.stop_mask,
        std::vector<std::vector<cv::Point>>{region.stop_points},
        cv::Scalar(255));

    region.valid = true;
    return region;
}

std::string AntiCollisionSystem::CurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm_buf{};
#ifdef _WIN32
    localtime_s(&tm_buf, &t);
#else
    localtime_r(&t, &tm_buf);
#endif
    std::ostringstream oss;
    oss << std::put_time(&tm_buf, "%Y%m%d_%H%M%S");
    return oss.str();
}

void AntiCollisionSystem::DiskCleanupLoop() {
    while (is_running_.load()) {
        try {
            auto now = std::filesystem::file_time_type::clock::now();
            for (const auto& dir : cfg_.cleanup_dirs) {
                if (!std::filesystem::exists(dir)) continue;
                for (const auto& entry : std::filesystem::directory_iterator(dir)) {
                    if (!entry.is_regular_file()) continue;
                    auto age_h = std::chrono::duration_cast<std::chrono::hours>(
                        now - entry.last_write_time()).count();
                    if (age_h > cfg_.retain_days * 24) {
                        std::error_code ec;
                        std::filesystem::remove(entry.path(), ec);
                    }
                }
            }
        }
        catch (const std::filesystem::filesystem_error& e) {
            LOG_AC("[AntiCollision] 清理线程异常: " << e.what());
        }
        for (int i = 0; i < 360 && is_running_.load(); ++i) {
            std::this_thread::sleep_for(std::chrono::seconds(10));
        }
    }
}