#include "plc_io.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <sstream>

#include "plc_register_map.h"   // PlcSend::Heartbeat
#include "utils.h"              // LOG_COMMON

// =========================================================================
// PlcReceiveBuffer
// =========================================================================
void PlcReceiveBuffer::SetAll(const uint16_t* src, int len) {
    if (!src || len <= 0) return;
    int n = std::min<int>(len, RCV_BUF_LEN);
    std::lock_guard<std::mutex> lock(mtx_);
    std::memcpy(buf_.data(), src, n * sizeof(uint16_t));
}

uint16_t PlcReceiveBuffer::Get(int reg_index) const {
    if (reg_index < 0 || reg_index >= RCV_BUF_LEN) return 0;
    std::lock_guard<std::mutex> lock(mtx_);
    return buf_[reg_index];
}

std::array<uint16_t, RCV_BUF_LEN> PlcReceiveBuffer::Snapshot() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return buf_;
}

// =========================================================================
// PlcSendBuffer
// =========================================================================
void PlcSendBuffer::Set(int reg_index, uint16_t value) {
    if (reg_index < 0 || reg_index >= SEND_BUF_LEN) return;
    std::lock_guard<std::mutex> lock(mtx_);
    buf_[reg_index] = value;
}

void PlcSendBuffer::SetMany(int start, const uint16_t* src, int len) {
    if (!src || len <= 0 || start < 0 || start >= SEND_BUF_LEN) return;
    int n = std::min<int>(len, SEND_BUF_LEN - start);
    std::lock_guard<std::mutex> lock(mtx_);
    std::memcpy(buf_.data() + start, src, n * sizeof(uint16_t));
}

std::array<uint16_t, SEND_BUF_LEN> PlcSendBuffer::Snapshot() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return buf_;
}

// =========================================================================
// PlcIoManager
// =========================================================================
PlcIoManager::PlcIoManager(const Config& cfg) : cfg_(cfg) {}

PlcIoManager::~PlcIoManager() { Stop(); }

bool PlcIoManager::Start() {
    if (running_.exchange(true)) {
        LOG_COMMON("[PlcIoManager] 已经在运行, 忽略重复 Start");
        return false;
    }

    if (!cfg_.enable) {
        LOG_COMMON("[PlcIoManager] enable=false, 不起 IO 线程, Buffer 仅在内存生效");
        return true;
    }

    // 创建两个 Plc_interact 实例 —— 它们的构造允许失败(connect_flag=0).
    // RcvLoop / SendLoop 内部会 connect_check 自动重连.
    plc_rcv_ = std::make_unique<Plc_interact>(cfg_.rcv_ip);
    plc_send_ = std::make_unique<Plc_interact>(cfg_.send_ip);

    rcv_thread_ = std::thread(&PlcIoManager::RcvLoop, this);
    send_thread_ = std::thread(&PlcIoManager::SendLoop, this);

    LOG_COMMON("[PlcIoManager] 已启动 | rcv_ip=" << cfg_.rcv_ip
        << " | send_ip=" << cfg_.send_ip
        << " | rcv=" << cfg_.rcv_interval_ms << "ms"
        << " | send=" << cfg_.send_interval_ms << "ms");
    return true;
}

void PlcIoManager::Stop() {
    if (!running_.exchange(false)) return;

    if (rcv_thread_.joinable())  rcv_thread_.join();
    if (send_thread_.joinable()) send_thread_.join();

    plc_rcv_.reset();
    plc_send_.reset();
    rcv_connected_.store(false);
    send_connected_.store(false);
    LOG_COMMON("[PlcIoManager] 已停止");
}

// -------------------------------------------------------------------------
// 接收线程: 周期把 PLC 寄存器读到 rcv_buf_
//
// 状态机:
//   每个循环:
//     1) 若 plc_rcv_->is_connected() 为 false → connect_check 单次重试
//     2) 调 get_modbus_data 单次读
//     3) 成功 → SetAll 进 buffer, 标记 rcv_connected_=true
//     4) 失败 → 不进 buffer, 等下一周期或下次重连
//   sleep rcv_interval_ms 进入下一周期
//
// 注: 重连失败时会 sleep 完整 rcv_interval_ms 才重试, 这是有意的 ——
//     避免 PLC 离线时疯狂重试塞满 CPU.
// -------------------------------------------------------------------------
void PlcIoManager::RcvLoop() {
    while (running_.load()) {
        bool ok = false;
        if (plc_rcv_) {
            // 1) 必要时重连
            if (!plc_rcv_->is_connected()) {
                plc_rcv_->connect_check();
            }
            // 2) 单次读
            if (plc_rcv_->is_connected()) {
                if (plc_rcv_->get_modbus_data()) {
                    uint16_t* p = plc_rcv_->get_rcv_buf();
                    if (p) {
                        rcv_buf_.SetAll(p, RCV_BUF_LEN);
                        ok = true;
                    }
                }
            }
        }
        rcv_connected_.store(ok);

        std::this_thread::sleep_for(std::chrono::milliseconds(cfg_.rcv_interval_ms));
    }
}

// -------------------------------------------------------------------------
// 发送线程: 周期把 send_buf_ 快照写到 PLC, 顺手注入心跳
//
// 状态机与接收线程对称:
//   1) 必要时 connect_check
//   2) 取 send_buf_ 快照, 注入心跳
//   3) modbus_write_registers 单次下发
//   4) 失败 → 把 client 端的 connect_flag 置 0(走 connect_check 流程)
//
// 心跳维护:
//   heartbeat_ 是 atomic<uint16_t>, 每周期 +1, 自然在 65535 处回滚.
// -------------------------------------------------------------------------
void PlcIoManager::SendLoop() {
    while (running_.load()) {
        bool ok = false;
        if (plc_send_) {
            // 1) 必要时重连
            if (!plc_send_->is_connected()) {
                plc_send_->connect_check();
            }

            if (plc_send_->is_connected()) {
                // 2) 取快照 + 注入心跳
                auto snap = send_buf_.Snapshot();
                uint16_t hb = heartbeat_.fetch_add(1) + 1;
                snap[PlcSend::Heartbeat] = hb;

                // 3) 单次写
                modbus_t* mb = plc_send_->get_client();
                if (mb) {
                    int rc = modbus_write_registers(mb, SEND_ADDR, SEND_BUF_LEN, snap.data());
                    if (rc == SEND_BUF_LEN) {
                        ok = true;
                    }
                    else {
                        // 失败 → 标记断开, 下个周期 connect_check 会重建
                        // 注: Plc_interact 的 connect_flag 是 private, 通过下次
                        //     get_modbus_data 失败/connect_check 路径自然恢复.
                        //     这里只能间接强制重连 —— 释放并新建 plc_send_.
                        LOG_COMMON("[PlcIoManager] SendLoop 写入失败, 重置发送连接");
                        plc_send_.reset();
                        plc_send_ = std::make_unique<Plc_interact>(cfg_.send_ip);
                    }
                }
            }
        }
        send_connected_.store(ok);

        std::this_thread::sleep_for(std::chrono::milliseconds(cfg_.send_interval_ms));
    }
}