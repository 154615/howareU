/**
 * @file main.cpp
 * @brief 大车防撞主程序入口（极简版）
 *        所有业务装配（生产/消费/PLC）由 AntiCollisionApp 负责
 */

#include <chrono>
#include <cstdint>
#include <thread>

#include "anti_collision_app.h"
#include "config_loader.h"
#include "utils.h"   // SAFE_LOG

 // 重启信号 —— 由其它模块（如 PLC 收到指令时）置位
uint16_t restart_flag = 0;

int main() {
    SAFE_LOG("********************** 大车防撞主程序启动 **********************");

    // 1. 加载配置
    AntiCollisionAppConfig cfg;
    if (!LoadConfigFromJson("./anti_collision_VS.json", cfg)) {
        SAFE_LOG("[Main] 配置加载失败,退出");
        return 1;
    }

    // 2. 启动 App（内部完成：PLC + 4 路取流 + 4 路检测 + PLC 发布）
    AntiCollisionApp app;
    if (!app.Configure(cfg) || !app.Start()) {
        SAFE_LOG("[Main] App 启动失败,退出");
        return 1;
    }

    // 3. 主线程监听重启信号
    while (true) {
        if (restart_flag == 1) {
            SAFE_LOG("[Main] 收到重启信号,准备安全退出...");
            app.Stop();
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }

    return 0;
}