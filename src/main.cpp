// =========================================================================
// main.cpp —— 顶层装配
// -------------------------------------------------------------------------
// 启动顺序:
//   1) 加载配置(json → AppBundleConfig)
//   2) 启动 PLC IO(共享 buffer 提供者)
//   3) 启动各 App, 注入共享 buffer
//   4) 主循环等待退出
// 关闭顺序与启动相反.
// =========================================================================

#include <atomic>
#include <chrono>
#include <csignal>
#include <thread>

#include "anti_collision_app.h"
#include "anti_lift_app.h"
#include "config_loader.h"
#include "plc_io.h"
#include "utils.h"   // LOG_COMMON

namespace {
    std::atomic<bool> g_quit{ false };
    void HandleSignal(int) { g_quit.store(true); }
}

int main(int argc, char* argv[]) {
    std::signal(SIGINT, HandleSignal);
    std::signal(SIGTERM, HandleSignal);

    const std::string json_path = (argc > 1) ? argv[1] : "./anti_collision_VS.json";

    // ---- 1) 加载配置 ----
    AppBundleConfig cfg;
    if (!LoadConfigFromJson(json_path, cfg)) {
        LOG_COMMON("[main] 配置加载失败");
        return 1;
    }

    // ---- 2) PLC IO ----
    PlcIoManager plc_io(cfg.plc_io);
    plc_io.Start();

    // ---- 3) 各 App ----
    std::unique_ptr<AntiCollisionApp> ac;
    if (cfg.enable_anti_collision) {
        ac = std::make_unique<AntiCollisionApp>();
        if (!ac->Configure(cfg.anti_collision,
            plc_io.RcvBuffer(),
            plc_io.SendBuffer())) {
            LOG_COMMON("[main] AntiCollisionApp::Configure 失败");
            return 1;
        }
        ac->Start();
    }

    std::unique_ptr<AntiLiftApp> lift;
    if (cfg.enable_anti_lift) {
        lift = std::make_unique<AntiLiftApp>();
        if (!lift->Configure(cfg.anti_lift,
            plc_io.RcvBuffer(),
            plc_io.SendBuffer())) {
            LOG_COMMON("[main] AntiLiftApp::Configure 失败");
            return 1;
        }
        lift->Start();
    }

    LOG_COMMON("[main] 系统启动完成, Ctrl+C 退出");

    // ---- 4) 主循环 ----
    while (!g_quit.load()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    LOG_COMMON("[main] 收到退出信号, 关闭系统...");

    // ---- 关闭顺序: 反向 ----
    if (lift) lift->Stop();
    if (ac)   ac->Stop();
    plc_io.Stop();

    LOG_COMMON("[main] 已安全退出");
    return 0;
}