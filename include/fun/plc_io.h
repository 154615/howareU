#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "modbus_cfg.h"   // Plc_interact, RCV_BUF_LEN, SEND_BUF_LEN, PORT, SLAVE_ID

// =========================================================================
// plc_io.h —— PLC 通讯模块: 集中式发布/订阅缓冲 + 收发 IO 线程
// -------------------------------------------------------------------------
// 模块定位:
//   把 PLC 通讯从应用层(App) 抽离, 提供两个全局可见的"消息总线":
//     - PlcReceiveBuffer:  PLC → 多 App  (订阅)
//     - PlcSendBuffer  :  多 App → PLC  (发布)
//   再用一个 PlcIoManager 持有它们 + 两条 IO 线程 + 心跳, 周期性
//   把 PLC 设备和缓冲区双向同步.
//
// 为什么用缓冲区, 而不让每个 App 各自调 modbus_write_registers:
//   - 避免多 App 写 PLC 时的顺序/竞态问题(虽然不一定真出问题)
//   - 让"业务侧 → PLC"之间天然解耦: App 想写就写, IO 线程统一打包下发
//   - 心跳由 IO 线程自己维护, 不与业务耦合
//   - 多 App 共享一个 PlcReceiveBuffer, 各取各的字段, 不重复发起 modbus 读
//   - 测试场景下"读真 PLC、写本地"很容易实现: rcv_ip 和 send_ip 各填一个
//
// 装配示意(在 main 里):
//   PlcIoManager::Config cfg;
//   cfg.rcv_ip  = "172.16.178.4";
//   cfg.send_ip = "127.0.0.1";       // 测试时分离; 生产环境填同一个
//   PlcIoManager plc_io(cfg);
//   plc_io.Start();
//
//   AntiCollisionApp ac;  ac.Configure(ac_cfg, plc_io.RcvBuffer(), plc_io.SendBuffer());
//   AntiLiftApp     lift; lift.Configure(lift_cfg, plc_io.RcvBuffer(), plc_io.SendBuffer());
//
// 线程安全:
//   - 两个 Buffer 的所有 public 方法都有内部互斥锁保护, 任意线程任意时刻可调
//   - PlcIoManager::Start/Stop 仅由 main 调用, 不要并发
// =========================================================================


// -------------------------------------------------------------------------
// PlcReceiveBuffer —— 接收缓冲(PLC → 多 App)
//
// 数据流向: PlcIoManager 的接收线程 50ms 拉一次 PLC 寄存器, SetAll 进来;
//           各 App 通过 Get / Snapshot 读出自己关心的字段.
// -------------------------------------------------------------------------
class PlcReceiveBuffer {
public:
    PlcReceiveBuffer() = default;

    PlcReceiveBuffer(const PlcReceiveBuffer&)            = delete;
    PlcReceiveBuffer& operator=(const PlcReceiveBuffer&) = delete;

    // ---------------------------------------------------------------------
    // SetAll() —— 整段写入(PlcIoManager 内部用, 业务方一般不该调)
    // ---------------------------------------------------------------------
    // 入参:
    //     src   源地址, 长度 >= len 的 uint16_t 数组指针
    //     len   要写入的寄存器个数; 超过 RCV_BUF_LEN 的部分会被丢弃
    // 阻塞性: 不阻塞, 仅一次内存拷贝
    // 线程安全: 安全
    void SetAll(const uint16_t* src, int len);

    // ---------------------------------------------------------------------
    // Get() —— 读单个寄存器
    // ---------------------------------------------------------------------
    // 入参:
    //     reg_index    PlcRcv:: 命名空间下的常量(强烈推荐), 越界返回 0
    // 返回:    该寄存器最新已知值
    // 阻塞性: 不阻塞
    // 线程安全: 安全
    uint16_t Get(int reg_index) const;

    // ---------------------------------------------------------------------
    // Snapshot() —— 整段一次性快照
    // ---------------------------------------------------------------------
    // 适合 App 一次性读多个相关字段(如防吊起的"小车位置 + 起升高度 + 着箱状态"),
    // 用快照能保证读到的是一组"瞬时一致"的数据.
    // 返回值: 当前缓冲区的副本
    std::array<uint16_t, RCV_BUF_LEN> Snapshot() const;

private:
    mutable std::mutex                     mtx_;
    std::array<uint16_t, RCV_BUF_LEN>      buf_{};
};


// -------------------------------------------------------------------------
// PlcSendBuffer —— 发送缓冲(多 App → PLC)
//
// 数据流向: 多个 App 通过 Set / SetMany 把自己的输出写进来;
//           PlcIoManager 的发送线程每 50ms Snapshot 一次, 统一发到 PLC.
// 心跳约定: 业务方不要写 PlcSend::Heartbeat(0); 这位由 IO 线程独占维护.
//           即使误写也不会崩, 只是会在下个发送周期被覆盖.
// -------------------------------------------------------------------------
class PlcSendBuffer {
public:
    PlcSendBuffer() = default;

    PlcSendBuffer(const PlcSendBuffer&)            = delete;
    PlcSendBuffer& operator=(const PlcSendBuffer&) = delete;

    // ---------------------------------------------------------------------
    // Set() —— 写单个寄存器
    // ---------------------------------------------------------------------
    // 入参:
    //     reg_index    PlcSend:: 下的常量; 越界静默忽略
    //     value        要写入的 16 位值
    // 阻塞性: 不阻塞
    // 线程安全: 安全
    // 副作用: 仅更新缓冲区, 真正下发由 PlcIoManager 的发送线程负责
    void Set(int reg_index, uint16_t value);

    // ---------------------------------------------------------------------
    // SetMany() —— 整段写入
    // ---------------------------------------------------------------------
    // 用于一次性写多个连续寄存器(如同时更新 4 路相机通讯状态).
    // 入参:
    //     start    起始下标(包含)
    //     src      数据源指针
    //     len      要写入的寄存器个数; 越界部分被截断
    void SetMany(int start, const uint16_t* src, int len);

    // ---------------------------------------------------------------------
    // Snapshot() —— PlcIoManager 发送线程内部用; 取完整副本
    // ---------------------------------------------------------------------
    std::array<uint16_t, SEND_BUF_LEN> Snapshot() const;

private:
    mutable std::mutex                       mtx_;
    std::array<uint16_t, SEND_BUF_LEN>       buf_{};
};


// -------------------------------------------------------------------------
// PlcIoManager —— PLC 通讯总管, 持有两个 Buffer 和两条 IO 线程
//
// 生命周期:
//   构造(只复制配置, 不连接) → Start() → 跑 → Stop() → 析构
//
// 线程模型:
//   Start 之后内部跑两条线程:
//     - 接收线程(rcv_thread_):  每 rcv_interval_ms  从 plc_rcv_  整段读
//                               寄存器 → SetAll 进 rcv_buf_
//     - 发送线程(send_thread_): 每 send_interval_ms 取 send_buf_ 快照,
//                               叠上自维护的心跳 → 整段写到 plc_send_
//
// 测试场景"读真 PLC、写本地":
//   cfg.rcv_ip  = "172.16.178.4";   // 真 PLC
//   cfg.send_ip = "127.0.0.1";      // 本地 modbus slave 监听
//   两个 Plc_interact 实例独立运行, 互不影响
// -------------------------------------------------------------------------
class PlcIoManager {
public:
    struct Config {
        // ===== 通讯端点 =====
        // PLC 设备 IP. 两个 IP 通常一致(生产); 测试时可分别填 真PLC / 本地.
        std::string rcv_ip;     // 读 PLC 的目标 IP(必填)
        std::string send_ip;    // 写 PLC 的目标 IP(必填)

        int port     = PORT;        // modbus TCP 端口, 默认 502
        int slave_id = SLAVE_ID;    // 从机 ID, 默认 1

        // ===== 节拍 =====
        int rcv_interval_ms       = 50;    // 接收线程拉数据周期
        int send_interval_ms      = 50;    // 发送线程下发数据周期
        int reconnect_interval_ms = 3000;  // 任一连接断开后重试间隔

        // ===== 是否启用 =====
        // 调试场景不接 PLC 时, 把此项置 false; Start() 立即返回 true 但不起线程,
        // 业务侧仍可正常读写 buffer(读到全 0; 写入只在内存生效).
        bool enable = true;
    };

    // 构造: 仅复制配置, 不连接, 不起线程, 不抛异常.
    explicit PlcIoManager(const Config& cfg);

    // 析构: 自动 Stop().
    ~PlcIoManager();

    PlcIoManager(const PlcIoManager&)            = delete;
    PlcIoManager& operator=(const PlcIoManager&) = delete;

    // ---------------------------------------------------------------------
    // Start() —— 起两条 IO 线程
    // ---------------------------------------------------------------------
    // 行为:
    //   - 创建 plc_rcv_ / plc_send_ 两个 Plc_interact 实例(可能因网络
    //     失败, 内部会进入重连循环, 不阻塞)
    //   - 起 rcv_thread_ 和 send_thread_
    // 返回:
    //     true   线程已起(此刻可能尚未连上 PLC, 但 Buffer 接口立即可用)
    //     false  已经在运行, 重复调用
    // 阻塞性: 不阻塞; 网络握手在线程里异步进行
    bool Start();

    // ---------------------------------------------------------------------
    // Stop() —— 停止 IO, join 两条线程, 释放 Plc_interact
    // ---------------------------------------------------------------------
    // 阻塞性: 阻塞, 上限 ~ 一个发送周期
    // 幂等性: 重复调用安全
    void Stop();

    // ---------------------------------------------------------------------
    // Buffer 取得 —— 注入给各 App 用
    // ---------------------------------------------------------------------
    // 返回的指针生命周期与本 PlcIoManager 一致; 调用方不释放, 不缓存超过
    // 本对象寿命的位置.
    PlcReceiveBuffer* RcvBuffer()  { return &rcv_buf_; }
    PlcSendBuffer*    SendBuffer() { return &send_buf_; }

    // 当前两条连接是否处于"健康"(初始为 false, 首次成功通讯后 true)
    bool RcvConnected()  const { return rcv_connected_.load();  }
    bool SendConnected() const { return send_connected_.load(); }

private:
    void RcvLoop();    // 接收线程主循环
    void SendLoop();   // 发送线程主循环

    Config            cfg_;

    // 共享 Buffer
    PlcReceiveBuffer  rcv_buf_;
    PlcSendBuffer     send_buf_;

    // PLC 句柄. 用 unique_ptr, Stop 时统一释放.
    std::unique_ptr<Plc_interact> plc_rcv_;
    std::unique_ptr<Plc_interact> plc_send_;

    // IO 线程
    std::thread       rcv_thread_;
    std::thread       send_thread_;
    std::atomic<bool> running_{ false };

    // 健康状态
    std::atomic<bool> rcv_connected_{ false };
    std::atomic<bool> send_connected_{ false };

    // 心跳计数, 每发送周期 +1, 0~65535 滚动
    std::atomic<uint16_t> heartbeat_{ 0 };
};
