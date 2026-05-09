#pragma once
#ifndef MODBUS_CFG_H
#define MODBUS_CFG_H

#include <iostream>
#include <string>
#include <libmodbus/modbus.h>

// =========================================================================
// modbus_cfg.h —— Modbus TCP 通讯底层封装(Plc_interact 类)
// -------------------------------------------------------------------------
// 模块定位:
//   纯通讯层. 单次读 / 单次连接检查 / 句柄管理. 不持有线程, 不死循环.
//   节拍由调用方(PlcIoManager) 控制.
//
// 历史变更:
//   - 2026-05  与 PlcIoManager 配套大改:
//              * 删除 8 个语义级 send_xxx 函数(语义由 PlcSend:: 常量表 + Buffer 接管)
//              * 删除 send_heart_beat 死循环(心跳由 PlcIoManager 自己维护)
//              * get_modbus_data / connect_check 从死循环改为单次执行 + 返回 bool
//              * 调用方需自己控制循环节拍(PlcIoManager::RcvLoop / SendLoop)
//
// 与 plc_register_map.h 的分工:
//   - modbus_cfg.h        通讯类 + 连接信息(IP/端口/总数), 不知"语义"
//   - plc_register_map.h  寄存器下标 → 业务语义, 不知"通讯"
//
// 调用约定(配套 PlcIoManager):
//   - 业务侧不直接持有 Plc_interact, 只通过 PlcReceiveBuffer / PlcSendBuffer
//     间接读写
//   - PlcIoManager 在它的两条 IO 线程里:
//       * 周期性调 get_modbus_data() 读一次 PLC, 失败时调 connect_check()
//       * 直接用 get_client() 拿 modbus_t* 调 modbus_write_registers 写一次
// =========================================================================

#define IP			"172.16.178.4"   // 默认 PLC IP, 可被 json 覆盖
#define IP_LOCAL	"127.0.0.1"      // 本地监听用(测试场景)
#define PORT		502
#define SLAVE_ID	1
// 重连尝试次数上限(保留宏供工程其它地方使用)
#define RECONNECT_TIMES 100
// 读取 / 发送起始地址
#define READ_ADDR 2000
#define SEND_ADDR 2300
// 读取 / 发送寄存器个数
#define RCV_BUF_LEN 20
#define SEND_BUF_LEN 20

class Plc_interact {
private:
    modbus_t* client = nullptr;   // libmodbus 句柄, 失败时为 null
    uint16_t     connect_flag = 0;         // 0=未连接/已断开, 1=已连接
    std::string  m_ip;                     // 该实例对应的 IP

    // 接收缓冲. 由 get_modbus_data() 写入; 上层 PlcIoManager 通过
    // get_rcv_buf() 拿首地址再 SetAll 到共享 buffer.
    uint16_t rcv_buf[RCV_BUF_LEN] = { 0 };

public:
    // 构造: 立即尝试连接一次. 失败时 connect_flag=0, 不抛异常.
    // 上层 PlcIoManager 在 IO 线程里看到失败时会调 connect_check() 重试.
    Plc_interact(std::string ip) : m_ip(ip) {
        client = connect_modbus(m_ip.c_str(), PORT, SLAVE_ID);
        connect_flag = (client ? 1 : 0);
    }

    ~Plc_interact() {
        if (client) {
            modbus_close(client);
            modbus_free(client);
            client = nullptr;
        }
    }

    Plc_interact(const Plc_interact&) = delete;
    Plc_interact& operator=(const Plc_interact&) = delete;

    // ===== 连接管理 =====
    // ---------------------------------------------------------------------
    // connect_modbus —— 单次尝试建立连接, 不重试
    // ---------------------------------------------------------------------
    // 入参:
    //     ip / port / server_id  目标 PLC 地址
    // 返回: 成功的 modbus_t* 句柄; 失败 nullptr.
    // 副作用: 成功时把 connect_flag 置 1.
    modbus_t* connect_modbus(const char* ip, int port, int server_id);

    // ---------------------------------------------------------------------
    // connect_check —— 单次连接自检 + 必要时重建
    // ---------------------------------------------------------------------
    // 行为:
    //   - 若 connect_flag != 0 且 client 仍然有效: 立即返回 true
    //   - 若 connect_flag == 0: 释放旧 client, 重新调一次 connect_modbus
    //                           成功 → 返回 true, 失败 → 返回 false
    // 阻塞性: 不死循环, 不内部 sleep. 仅一次系统调用的耗时.
    // 调用方: PlcIoManager 在 RcvLoop / SendLoop 检测到失败时调.
    bool connect_check();

    // ===== 单次读 =====
    // ---------------------------------------------------------------------
    // get_modbus_data —— 整段读 RCV_BUF_LEN 个寄存器到 rcv_buf
    // ---------------------------------------------------------------------
    // 返回:
    //     true   读取成功, 数据在 rcv_buf
    //     false  client 为空 / 未连接 / modbus_read_registers 失败
    //            (内部置 connect_flag=0, 上层下次循环会调 connect_check)
    // 阻塞: 同步, 仅一次 modbus 请求时长(几 ms).
    bool get_modbus_data();

    // ===== 访问器 =====
    // 取最近一次成功 get_modbus_data 后的接收缓冲首地址(长度 RCV_BUF_LEN).
    uint16_t* get_rcv_buf();

    // 取底层 modbus_t*. 上层 PlcIoManager 直接 modbus_write_registers 写整段.
    modbus_t* get_client();

    // 当前是否处于"已连接"状态.
    bool is_connected() const { return connect_flag != 0; }

    // 调试: 把 rcv_buf 全部打印到日志.
    void output_rcv_data();
};

#endif