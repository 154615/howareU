/**
 * @file main.cpp
 * @brief 客户端业务层代码 - 极其整洁的业务循环 (非同步連線版)
 */

#include "HikvisionCamera.h"
#include <iostream>
#include <thread>
#include <atomic>

int main() {
    HikvisionCamera camera;

    cv::namedWindow("Smart Vision AI Platform", cv::WINDOW_NORMAL);
    cv::resizeWindow("Smart Vision AI Platform", 1280, 720);

    std::cout << "==========================================" << std::endl;
    std::cout << " 智能控制面板 (請在影像視窗內按下按鍵):" << std::endl;
    std::cout << " [W] 上仰 5 度   [S] 下俯 5 度" << std::endl;
    std::cout << " [A] 左轉 5 度   [D] 右轉 5 度" << std::endl;
    std::cout << " [1/2/3/4] 絕對變焦 (1x / 2x / 3x / 4x)" << std::endl;
    std::cout << " [C] 連接相機    [X] 斷開連接" << std::endl;
    std::cout << " [Q] 退出程式" << std::endl;
    std::cout << "==========================================" << std::endl;

    cv::Mat frame;

    // 初始化本地状态位
    bool lastConnectionState = false;
    bool isManualDisconnect = false;

    // 【新增】原子變數，確保同一時間只有一個連線執行緒在運作
    std::atomic<bool> isConnecting(false);

    // 【新增】非同步連線的 Lambda 函數
    auto asyncConnect = [&]() {
        if (isConnecting.load()) {
            std::cout << "[SYS] 已經在連接中，請勿重複操作..." << std::endl;
            return;
        }

        isConnecting.store(true);
        isManualDisconnect = false; // 重置手動斷線標記

        // 分配子執行緒進行背景連線 (Detach 放後台執行)
        std::thread([&]() {
            std::cout << "[SYS] [子執行緒] 正在連接相機並建立流媒體通道..." << std::endl;
            bool success = camera.connect("192.168.1.64", 8000, "admin", "Hc@RTG210");
            if (!success) {
                std::cerr << "[SYS] [子執行緒] 相機連接失敗，請檢查網路！" << std::endl;
            }
            // 無論成功或失敗，結束連線狀態
            isConnecting.store(false);
            }).detach();
        };

    // 啟動時立刻觸發一次背景連線
    asyncConnect();

    // 主循環 (UI 與事件監聽，永不阻塞)
    while (true) {
        // 1. 获取并监听状态位变化
        bool currentStatus = camera.isStreamConnected();
        if (currentStatus != lastConnectionState) {
            std::cout << "\n==========================================" << std::endl;
            if (currentStatus) {
                std::cout << "[STATUS] 视频流连接状态位: 1 (重连成功，画面恢复)" << std::endl;
                isManualDisconnect = false;
            }
            else {
                if (isManualDisconnect) {
                    std::cout << "[STATUS] 视频流连接状态位: 0 (已手动断开连接)" << std::endl;
                }
                else {
                    std::cout << "[STATUS] 视频流连接状态位: 0 (已断线，等待网络)" << std::endl;
                }
            }
            std::cout << "==========================================\n" << std::endl;
            lastConnectionState = currentStatus;
        }

        // 2. 获取画面与渲染 UI
        if (!currentStatus || !camera.getLatestFrame(frame)) {
            // 如果处于断开状态，或者拿不到画面，创建/保持一个黑底提示画布
            if (frame.empty()) {
                frame = cv::Mat::zeros(720, 1280, CV_8UC3);
            }
            // 增加半透明黑底避免文字看不清
            cv::rectangle(frame, cv::Point(0, 0), cv::Point(1280, 150), cv::Scalar(0, 0, 0), -1);

            // 【修改】加入正在連接中的 UI 提示邏輯
            if (isConnecting.load()) {
                cv::putText(frame, "SYSTEM CONNECTING - PLEASE WAIT...",
                    cv::Point(50, 100), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 255, 255), 3); // 黃色提示
            }
            else if (isManualDisconnect) {
                cv::putText(frame, "SYSTEM OFFLINE - MANUALLY DISCONNECTED",
                    cv::Point(50, 100), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 165, 255), 3); // 橙色提示
            }
            else {
                cv::putText(frame, "NETWORK DISCONNECTED - WAITING FOR SIGNAL...",
                    cv::Point(50, 100), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 0, 255), 3);   // 红色提示
            }
            cv::imshow("Smart Vision AI Platform", frame);

            // 断线/连接中状态下，短暂休眠并捕获按键以支持手动重连
            int offlineKey = cv::waitKey(50) & 0xFF;
            if (offlineKey == 'q' || offlineKey == 'Q' || offlineKey == 27) {
                break;
            }
            else if (offlineKey == 'c' || offlineKey == 'C') {
                std::cout << "[SYS] 手動嘗試重新連接..." << std::endl;
                asyncConnect(); // 呼叫非同步連線
            }
            continue;
        }

        // 走到这里说明状态位正常 (为1)，进行业务渲染
        if (!frame.empty()) {
            cv::drawMarker(frame, cv::Point(frame.cols / 2, frame.rows / 2), cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 20, 2);
            cv::imshow("Smart Vision AI Platform", frame);
        }

        // 3. 交互处理
        int key = cv::waitKey(1) & 0xFF;
        if (key == 255) continue;

        if (key == 'q' || key == 'Q' || key == 27) {
            break;
        }
        else if (key == 'a' || key == 'A') {
            camera.setRelativeAngle(-5.0f, 0.0f);
        }
        else if (key == 'd' || key == 'D') {
            camera.setRelativeAngle(5.0f, 0.0f);
        }
        else if (key == 'w' || key == 'W') {
            camera.setRelativeAngle(0.0f, 5.0f);
        }
        else if (key == 's' || key == 'S') {
            camera.setRelativeAngle(0.0f, -5.0f);
        }
        else if (key == '1') {
            camera.setAbsoluteZoom(1);
        }
        else if (key == '2') {
            camera.setAbsoluteZoom(2);
        }
        else if (key == '3') {
            camera.setAbsoluteZoom(3);
        }
        else if (key == '4') {
            camera.setAbsoluteZoom(4);
        }
        else if (key == 'c' || key == 'C') {
            std::cout << "[SYS] 嘗試重新連接..." << std::endl;
            asyncConnect(); // 呼叫非同步連線
        }
        else if (key == 'x' || key == 'X') {
            if (isConnecting.load()) {
                std::cout << "[SYS] 正在連線中，請稍後再試..." << std::endl;
            }
            else {
                std::cout << "[SYS] 手動斷開連接..." << std::endl;
                isManualDisconnect = true;
                camera.disconnect();
            }
        }
        else if (key == ' ') {
            camera.stopAllActions();
        }
    }

    std::cout << "[SYS] 正在安全退出系统..." << std::endl;
    // 確保退出時關閉 SDK
    camera.disconnect();
    cv::destroyAllWindows();
    return 0;
}