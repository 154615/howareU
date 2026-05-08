#include "data_sample.h"
#include "utils.h"
#include <chrono>
#include <thread>
#include <filesystem>

namespace fs = std::filesystem;

TrafficMonitor::TrafficMonitor(const std::string& url)
    : rtsp_url(url), running(true), has_captured(false) {
}

void TrafficMonitor::save_frame(const cv::Mat& frame, int cam_id, int height) {
    // 1. 定義基礎路徑與完整目錄路徑
    std::string base_dir = "saved";
    std::string sub_dir = base_dir + "/cam_" + std::to_string(cam_id);

    try {
        // 2. 檢查並自動創建目錄 (create_directories 會遞迴建立所有不存在的層級)
        if (!fs::exists(sub_dir)) {
            fs::create_directories(sub_dir);
            Log("Created directory: " + sub_dir);
        }

        // 3. 生成完整檔案路徑
        std::string filename = sub_dir + "/capture_" + get_current_time() + "_H" + std::to_string(height) + ".jpg";

        // 4. 儲存圖片
        if (cv::imwrite(filename, frame)) {
            Log("Trigger Capture Success: " + filename);
        }
        else {
            Log("Error: cv::imwrite failed to save " + filename);
        }
    }
    catch (const fs::filesystem_error& e) {
        Log("Filesystem Error: " + std::string(e.what()));
    }
}

void TrafficMonitor::monitor_loop(uint16_t* rcv_buf, int camera_id) {
    cv::VideoCapture cap;
    cv::Mat frame;

    while (running) {
        uint16_t trolley_pos = rcv_buf[2];   // 小车位置
        uint16_t hoist_height = rcv_buf[3];   // 起升高度
        uint16_t lock_status = rcv_buf[4];   // 闭锁状态 (1闭)
        uint16_t truck_pos = rcv_buf[12];  // 外集卡位置

        // --- 逻辑1：提前预热连接 ---
        if (trolley_pos < 100) {
            if (!cap.isOpened()) {
                Log("Trolley < 100, connecting camera " + std::to_string(camera_id));
                cap.open(rtsp_url, cv::CAP_FFMPEG);
                cap.set(cv::CAP_PROP_BUFFERSIZE, 1); // 减少延迟
            }
        }
        else if (trolley_pos > 150) {
            if (cap.isOpened()) {
                cap.release();
                has_captured = false; // 离开作业区，重置抓拍标志
                Log("Trolley > 150, releasing camera " + std::to_string(camera_id));
            }
        }

        // --- 逻辑2：视频流处理与抓拍 ---
        if (cap.isOpened()) {
            cap >> frame; // 持续取帧，清空缓冲区
            if (frame.empty()) continue;

            // 检查抓拍条件
            bool condition = (truck_pos != 0) &&
                (trolley_pos < 100) &&
                (lock_status == 1) &&
                (hoist_height >= 450 && hoist_height <= 480);

            if (condition && !has_captured) {
                save_frame(frame, camera_id, hoist_height);
                has_captured = true; // 锁定，直到高度离开区间或小车离开
            }

            // 如果高度离开触发区间，重置锁定，允许下次抓拍（如果需要单次作业多次抓拍可微调此处）
            if (has_captured && (hoist_height < 400 || hoist_height > 520)) {
                has_captured = false;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
}

void TrafficMonitor::stop() { running = false; }