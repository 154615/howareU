#pragma once
#ifndef DATA_SAMPLE_H
#define DATA_SAMPLE_H

#include <opencv2/opencv.hpp>
#include <string>
#include <mutex>
#include <atomic>

// 封装业务监控逻辑
class TrafficMonitor {
private:
    std::string rtsp_url;
    std::atomic<bool> running;
    bool has_captured; // 抓拍锁定，防止在区间内重复抓拍

    void save_frame(const cv::Mat& frame, int cam_id, int height);

public:
    TrafficMonitor(const std::string& url);
    // 监控循环：传入PLC数据缓存指针
    void monitor_loop(uint16_t* rcv_buf, int camera_id);
    void stop();
};

#endif
