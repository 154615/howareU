#include <modbus_cfg.h>
#include <fun/utils.h>
#include <thread>
#include <chrono>


using namespace std;


modbus_t* Plc_interact::get_client() {
    return client;
}

// =========================================================================
// connect_modbus
//
// 修复点:
//   - 原 while (rc != 0); 死循环改为失败返回 nullptr
//   - 原内部 while(1) 死循环改为单次尝试,由调用方决定是否重试
//     (避免构造函数无限阻塞,也避免 connect_check 的"重连套娃")
//   - 失败时正确释放 modbus_t,避免泄漏
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
// connect_check
//
// 修复点:
//   - 重连时使用构造时保存的 m_ip,而不是硬编码的 IP 宏
//     (这样测试本地 127.0.0.1 时,断线重连不会跳到生产 IP)
//   - 重连前先关闭并释放旧连接,避免 modbus_t 泄漏
// =========================================================================
void Plc_interact::connect_check() {
    while (1) {
        if (connect_flag == 0) {
            // 释放旧连接(如果有)
            if (client) {
                modbus_close(client);
                modbus_free(client);
                client = nullptr;
            }
            client = connect_modbus(m_ip.c_str(), PORT, SLAVE_ID);
            if (!client) {
                printf("[PLC] *****Reconnecting %s ...*****\n", m_ip.c_str());
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    }
}


// ==== 相机通讯状态发送 (0断开,1通讯正常) ====
void Plc_interact::send_SeaEastCamera_status(uint16_t status) { send_buf[1] = status; }
void Plc_interact::send_SeaWestCamera_status(uint16_t status) { send_buf[2] = status; }
void Plc_interact::send_LandEastCamera_status(uint16_t status) { send_buf[3] = status; }
void Plc_interact::send_LandWestCamera_status(uint16_t status) { send_buf[4] = status; }

// ==== 大车视觉防撞目标距离发送 (单位: cm) ====
void Plc_interact::send_EastDistance(uint16_t dist_cm) { send_buf[5] = dist_cm; }
void Plc_interact::send_WestDistance(uint16_t dist_cm) { send_buf[6] = dist_cm; }

// ==== 大车动作控制发送 ====
// 0不限速,1东向限速,2西向限速,3两侧限速
void Plc_interact::send_SpeedLimit_Direction(uint16_t dir) { send_buf[7] = dir; }
// 0无动作,1东向停止,2西向停止,3双向停止
void Plc_interact::send_Stop_Direction(uint16_t dir) { send_buf[8] = dir; }


// =========================================================================
// send_heart_beat
//
// 修复点:
//   - 写入失败时关闭旧连接,避免与 connect_check 中重连的连接冲突
// =========================================================================
void Plc_interact::send_heart_beat() {
    send_buf[0] = 0;
    uint16_t counter = 0;

    while (1) {
        // 1. 断线状态下,挂起当前线程,等待后台重连成功
        if (connect_flag == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        // 2. 频率控制:每两轮(约 50ms)递增一次心跳值
        counter++;
        if (counter >= 2) {
            send_buf[0]++;
            counter = 0;
        }

        // 3. 边界安全:到上限归零
        if (send_buf[0] >= 32768) {
            send_buf[0] = 0;
        }

        // 4. Modbus 发送与错误处理
        if (!client ||
            modbus_write_registers(client, SEND_ADDR, SEND_BUF_LEN, send_buf) == -1) {
            printf("[PLC] Heart beat send err.\n");
            connect_flag = 0;   // connect_check 会负责重连和释放旧连接
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        else {
            std::this_thread::sleep_for(std::chrono::milliseconds(HEART_FREQ));
        }
    }
}

// =========================================================================
// get_modbus_data
//
// 修复点:
//   - 读取失败时同样置 connect_flag = 0,旧连接由 connect_check 负责释放
//   - client 空指针保护
// =========================================================================
void Plc_interact::get_modbus_data() {
    while (1) {
        if (connect_flag == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        if (!client ||
            modbus_read_registers(client, READ_ADDR, RCV_BUF_LEN, rcv_buf) == -1) {
            printf("[PLC] Modbus read err.\n");
            connect_flag = 0;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}


uint16_t* Plc_interact::get_rcv_buf() {
    return rcv_buf;
}


void Plc_interact::output_rcv_data() {
    Log("************接收数据:" + to_string(rcv_buf[0]) + "," + to_string(rcv_buf[1]) + "," +
        to_string(rcv_buf[2]) + "," + to_string(rcv_buf[3]) + "," + to_string(rcv_buf[4]) +
        "," + to_string(rcv_buf[5]));
}