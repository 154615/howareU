/**
 * @file main.cpp
 * @brief 应用层装配示例
 *        4 路 CameraSource (生产端) + 多消费者订阅 (FrameSink)
 */

#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include "anti_collision_system.h"
#include "camera_source.h"

// =========================================================================
// 示例：另一个消费者 —— UI 预览（仅 cam1）
// 演示如何添加多消费者：实现 FrameSink，AddSink 即可
// =========================================================================
class UiPreviewSink : public FrameSink {
public:
    explicit UiPreviewSink(int target_cam) : target_(target_cam) {}

    void OnFrame(int cam_index, const cv::Mat& frame) override {
        if (cam_index != target_) return;
        // 注意：OnFrame 在生产端轮询线程被调用；imshow 必须在主线程,
        // 实战中应把 frame 拷贝入队由主线程取出再 imshow
        std::lock_guard<std::mutex> lock(mtx_);
        frame.copyTo(latest_);
        has_frame_ = true;
    }

    bool TryGetLatest(cv::Mat& out) {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!has_frame_) return false;
        latest_.copyTo(out);
        return true;
    }

private:
    int                target_;
    std::mutex         mtx_;
    cv::Mat            latest_;
    bool               has_frame_ = false;
};

int main() {
    // ====================================================================
    // 1. 防撞模块配置
    // ====================================================================
    AntiCollisionConfig acs_cfg;
    acs_cfg.model_path        = "./model/yolov8_seg.onnx";
    acs_cfg.split_ratio       = 0.5f;
    acs_cfg.retain_days       = 7;
    acs_cfg.enable_debug_show = false;

    const int W = 1920, H = 1080;
    for (int i = 0; i < 4; ++i) {
        acs_cfg.regions[i] = {
            {{100,100},{1820,100},{1820,980},{100,980}}, W, H
        };
    }

    AntiCollisionSystem acs(acs_cfg);
    AntiCollisionState  state;
    acs.Start(&state);

    // ====================================================================
    // 2. UI 预览消费者（演示多消费者）
    // ====================================================================
    UiPreviewSink ui_sink(/*cam_index=*/0);

    // ====================================================================
    // 3. 4 路相机源
    // ====================================================================
    std::vector<CameraSourceConfig> cam_cfgs(4);
    cam_cfgs[0] = { "192.168.1.64", 8000, "admin", "Hc@RTG210", 1, 0 };
    cam_cfgs[1] = { "192.168.1.65", 8000, "admin", "Hc@RTG210", 1, 1 };
    cam_cfgs[2] = { "192.168.1.66", 8000, "admin", "Hc@RTG210", 1, 2 };
    cam_cfgs[3] = { "192.168.1.67", 8000, "admin", "Hc@RTG210", 1, 3 };

    std::vector<std::unique_ptr<CameraSource>> sources;
    for (auto& c : cam_cfgs) {
        auto src = std::make_unique<CameraSource>(c);
        src->AddSink(&acs);          // 防撞订阅
        sources.push_back(std::move(src));
    }
    sources[0]->AddSink(&ui_sink);   // UI 仅订阅 cam1（运行期 AddSink 也可）

    for (auto& src : sources) src->Start();

    // ====================================================================
    // 4. 主循环
    // ====================================================================
    std::cout << "==============================================\n"
              << " [Q] 退出  [C] 重连 cam1  [X] 断开 cam1\n"
              << " [W/A/S/D] cam1 上下左右 5 度\n"
              << " [1/2/3/4] cam1 变焦 1x/2x/3x/4x\n"
              << "==============================================\n";

    cv::namedWindow("Preview cam1", cv::WINDOW_NORMAL);
    cv::resizeWindow("Preview cam1", 960, 540);

    while (true) {
        cv::Mat preview;
        if (ui_sink.TryGetLatest(preview)) {
            // 叠加状态文字
            for (int i = 0; i < 4; ++i) {
                auto level = state.cameras[i].level.load();
                cv::Scalar color = (level == AlarmLevel::Stop)  ? cv::Scalar(0,0,255)
                                  : (level == AlarmLevel::Decel) ? cv::Scalar(0,255,255)
                                                                  : cv::Scalar(0,255,0);
                const char* tag = (level == AlarmLevel::Stop)  ? "STOP"
                                : (level == AlarmLevel::Decel) ? "SLOW"
                                                                : "SAFE";
                cv::putText(preview,
                    "cam" + std::to_string(i+1) + ": " + tag,
                    cv::Point(20, 30 + i * 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, color, 2);
            }
            cv::imshow("Preview cam1", preview);
        }

        int key = cv::waitKey(33) & 0xFF;
        if (key == 'q' || key == 'Q' || key == 27) break;

        // PTZ 走抽象接口；不再耦合 HikvisionCamera
        IPtzControl* ptz = sources[0]->Ptz();

        if (ptz && ptz->HasPan()) {
            if      (key == 'a' || key == 'A') ptz->Move(-5.0f, 0.0f);
            else if (key == 'd' || key == 'D') ptz->Move( 5.0f, 0.0f);
            else if (key == 'w' || key == 'W') ptz->Move( 0.0f, 5.0f);
            else if (key == 's' || key == 'S') ptz->Move( 0.0f,-5.0f);
        }
        if (ptz && ptz->HasZoom()) {
            if      (key == '1') ptz->Zoom(1.0f);
            else if (key == '2') ptz->Zoom(2.0f);
            else if (key == '3') ptz->Zoom(3.0f);
            else if (key == '4') ptz->Zoom(4.0f);
        }

        if      (key == 'c' || key == 'C') sources[0]->RequestConnect();
        else if (key == 'x' || key == 'X') sources[0]->RequestDisconnect();
    }

    // ====================================================================
    // 5. 退出（析构会自动 Stop；下面是显式版）
    // ====================================================================
    std::cout << "[SYS] 退出中..." << std::endl;
    for (auto& src : sources) src->Stop();   // 先停生产
    acs.Stop();                              // 再停消费
    cv::destroyAllWindows();
    return 0;
}
