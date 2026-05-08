#pragma once
#ifndef MODBUS_CFG_H
#define MODBUS_CFG_H

#include <iostream>
#include <libmodbus/modbus.h>


#define IP			"172.16.178.4"
#define IP_LOCAL	"127.0.0.1"
#define PORT		502
#define SLAVE_ID	1
#define HEART_FREQ	25
// 重连次数
#define RECONNECT_TIMES 100
// 读取发送起始地址
#define READ_ADDR 2000
#define SEND_ADDR 2300
// 读取发送寄存器个数
#define RCV_BUF_LEN 20
#define SEND_BUF_LEN 20


class Plc_interact {
private:
    modbus_t* client;
    uint16_t connect_flag = 0;
    std::string m_ip; // 新增：保存此实例对应的真实 IP
    /***读取PLC数据		所有数据均为四字节
    * 0 心跳信号		0-32768
    * 1 大车位置	单位cm
    * 2 小车位置	单位cm
    * 3 起升高度	单位cm
    * 4 开闭锁状态	0开锁，1闭锁
    * 5 大车速度	单位mm/s
    * 6 小车速度	单位mm/s
    * 7 起升速度	单位mm/s
    * 8 着箱状态	0未着箱，1着箱
    * 9 吊具尺寸	20,40,45
    * 10 旁路复位信号		0默认，1按下复位按钮
    *
    * 12 外集卡放箱位置		1前20尺，2中20尺或40尺 3后20尺
    * 13 集卡引导启动		0不启动，1启动
    * 14 作业类型			0默认，1收箱前20，2收箱后20，3收箱40，4发箱前20，5发箱后20，6发箱40
    **/
    uint16_t rcv_buf[RCV_BUF_LEN] = { 0 };

    /****
    * 发送到PLC
     0   心跳                         0-65535
     1   海左相机通讯状态           0断开，1通讯正常
     2   海右相机通讯状态           0断开，1通讯正常
     3   陆左相机通讯状态           0断开，1通讯正常
     4   陆右相机通讯状态           0断开，1通讯正常
     5   左侧大车视觉防撞目标距离   单位cm
     6   右侧大车视觉防撞目标距离   单位cm
     7   大车防撞限速方向           0不限速，1左向限速，2右向限速，3两侧限速
     8   大车停止信号               0无动作，1左向停止，2右向停止，3双向停止

    ****/
    uint16_t send_buf[SEND_BUF_LEN] = { 0 };

public:
    Plc_interact(std::string ip) : m_ip(ip) { // 保存传入的 IP
        client = connect_modbus(m_ip.c_str(), PORT, SLAVE_ID);
        // 【关键】移除 while (!client); 允许构造函数失败并返回
        if (client) connect_flag = 1;
        else connect_flag = 0;
    }
    /*** 分配线程循环检查连接标志位，断线重连，心跳发送失败三次断线重连 ***/
    void connect_check();
    modbus_t* connect_modbus(const char* ip, int port, int server_id);

    // ==== 相机通讯状态发送 (0断开，1通讯正常) ====
    void send_SeaEastCamera_status(uint16_t status);
    void send_SeaWestCamera_status(uint16_t status);
    void send_LandEastCamera_status(uint16_t status);
    void send_LandWestCamera_status(uint16_t status);

    // ==== 大车视觉防撞目标距离发送 (单位：cm) ====
    void send_EastDistance(uint16_t dist_cm);
    void send_WestDistance(uint16_t dist_cm);

    // ==== 大车动作控制发送 ====
    // 0不限速，1东向限速，2西向限速，3两侧限速
    void send_SpeedLimit_Direction(uint16_t dir);
    // 0无动作，1东向停止，2西向停止，3双向停止
    void send_Stop_Direction(uint16_t dir);

    // 更新并发送心跳，分配线程调用
    void send_heart_beat();

    /****** modbus读取 ******/
    void get_modbus_data();
    // 整串读取modbus数据，失败返回false，成功true
    uint16_t* get_rcv_buf();
    /*****/
    modbus_t* get_client();

    void output_rcv_data();
};

#endif
