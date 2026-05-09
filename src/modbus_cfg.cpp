#include <modbus_cfg.h>
#include <fun/utils.h>

#include <cerrno>
#include <cstdio>

using namespace std;

// =========================================================================
// 与上一版的核心差异:
//   - 删除 8 个 send_xxx 业务函数(语义由 PlcSend:: + Buffer 接管)
//   - 删除 send_heart_beat() 死循环(心跳由 PlcIoManager 维护)
//   - get_modbus_data() / connect_check() 改成单次执行 + 返回 bool
//   - 节拍由 PlcIoManager 控制
// =========================================================================

modbus_t* Plc_interact::get_client() {
    return client;
}

uint16_t* Plc_interact::get_rcv_buf() {
    return rcv_buf;
}

// =========================================================================
// connect_modbus —— 单次建立连接, 不重试
// 失败时正确释放 modbus_t, 不泄漏
// =========================================================================
modbus_t* Plc_interact::connect_modbus(const char* ip, int port, int server_id) {
    modbus_t* c = modbus_new_tcp(ip, port);
    if (!c) {
        fprintf(stderr, "modbus_new_tcp failed\n");
        return nullptr;
    }

    if (modbus_set_slave(c, server_id) != 0) {
        fprintf(stderr, "modbus_set_slave failed: %s\n", modbus_strerror(errno));
        modbus_free(c);
        return nullptr;
    }

    if (modbus_connect(c) == -1) {
        fprintf(stderr, "Connection failed (%s:%d): %s\n",
            ip, port, modbus_strerror(errno));
        modbus_free(c);
        return nullptr;
    }

    printf("[PLC] 连接成功 %s:%d\n", ip, port);
    connect_flag = 1;
    return c;
}

// =========================================================================
// connect_check —— 单次自检 + 必要时重建
//
// 关键变化:
//   - 不再死循环, 不再 sleep
//   - 用 m_ip(构造时保存)而不是 IP 宏, 测试 127.0.0.1 时不会跳到生产 IP
//   - 返回 bool, 调用方据此决定是否进入下一周期
// =========================================================================
bool Plc_interact::connect_check() {
    if (connect_flag != 0 && client) {
        return true;
    }

    // 释放旧句柄(若有)
    if (client) {
        modbus_close(client);
        modbus_free(client);
        client = nullptr;
    }

    client = connect_modbus(m_ip.c_str(), PORT, SLAVE_ID);
    if (!client) {
        printf("[PLC] *****Reconnecting %s ...*****\n", m_ip.c_str());
        return false;
    }
    return true;
}

// =========================================================================
// get_modbus_data —— 单次整段读
//
// 返回 false 时已把 connect_flag 置 0, 上层下个周期会 connect_check 重连
// =========================================================================
bool Plc_interact::get_modbus_data() {
    if (connect_flag == 0 || !client) {
        return false;
    }

    if (modbus_read_registers(client, READ_ADDR, RCV_BUF_LEN, rcv_buf) == -1) {
        printf("[PLC] Modbus read err: %s\n", modbus_strerror(errno));
        connect_flag = 0;
        return false;
    }
    return true;
}

// =========================================================================
// 调试: 打印 rcv_buf
// =========================================================================
void Plc_interact::output_rcv_data() {
    Log("************接收数据:" +
        to_string(rcv_buf[0]) + "," + to_string(rcv_buf[1]) + "," +
        to_string(rcv_buf[2]) + "," + to_string(rcv_buf[3]) + "," +
        to_string(rcv_buf[4]) + "," + to_string(rcv_buf[5]));
}