#pragma once

// =========================================================================
// plc_register_map.h —— PLC 寄存器下标集中映射
// -------------------------------------------------------------------------
// 用途:
//   把 PLC 寄存器的"语义 → 下标"映射收口在这一个文件里. 所有业务代码
//   严禁直接写裸数字下标(如 buf[8]), 必须通过 PlcRcv:: / PlcSend:: 命名空间
//   下的常量来引用. 当 PLC 工程师调整接口顺序时, 只动这一个文件.
//
// 与 modbus_cfg.h 的分工:
//   - modbus_cfg.h    持有 Plc_interact 类、连接信息、寄存器总数等"通讯层"的东西
//   - plc_register_map.h  仅描述"寄存器下标 → 业务语义"的映射, 不依赖 modbus
//
// 维护规则:
//   - 当 PLC 端调整某寄存器位置: 改这里对应常量的数值即可, 业务代码无需改
//   - 当 PLC 端新增寄存器: 在对应 namespace 下新增常量, 名字使用 PascalCase
//   - 必须配套更新 RCV_BUF_LEN / SEND_BUF_LEN(在 modbus_cfg.h)
// =========================================================================


// -------------------------------------------------------------------------
// 接收(rcv): PLC → 软件
// -------------------------------------------------------------------------
//   下标含义来源: modbus_cfg.h 中 rcv_buf 的注释表
//   字段名遵循"业务名 + 单位/状态"风格, 不用拼音
namespace PlcRcv {
    constexpr int Heartbeat       = 0;   // PLC 心跳, 0~32768 持续递增
    constexpr int PosBigCar       = 1;   // 大车位置, 单位 cm
    constexpr int PosTrolley      = 2;   // 小车位置, 单位 cm
    constexpr int HoistHeight     = 3;   // 起升高度, 单位 cm
    constexpr int LockStatus      = 4;   // 开闭锁状态: 0=开锁, 1=闭锁
    constexpr int SpdBigCar       = 5;   // 大车速度, 单位 mm/s
    constexpr int SpdTrolley      = 6;   // 小车速度, 单位 mm/s
    constexpr int SpdHoist        = 7;   // 起升速度, 单位 mm/s, 有正负(int16 解读)
    constexpr int BoxLanded       = 8;   // 着箱状态: 0=未着箱, 1=着箱
    constexpr int SpreaderSize    = 9;   // 吊具尺寸: 20 / 40 / 45
    constexpr int ResetSignal     = 10;  // 旁路复位信号: 0=默认, 1=按下复位按钮
    // 11 保留
    constexpr int TruckBoxPos     = 12;  // 外集卡放箱位置: 1=前20尺, 2=中(20/40), 3=后20尺
    constexpr int TruckGuideStart = 13;  // 集卡引导启动: 0=不启动, 1=启动
    constexpr int JobType         = 14;  // 作业类型: 0=默认, 1~3=收箱, 4~6=发箱
}

// -------------------------------------------------------------------------
// 发送(send): 软件 → PLC
// -------------------------------------------------------------------------
//   注: 下标 0 心跳由 PlcIoManager 内部维护, 业务代码不要直接 Set(Heartbeat).
namespace PlcSend {
    constexpr int Heartbeat          = 0;   // 软件心跳, 0~65535 递增, 由 IO 线程自动维护

    // ---- 4 路相机通讯状态: 0=断开, 1=通讯正常 ----
    constexpr int CamSeaEastStatus   = 1;   // 海东
    constexpr int CamSeaWestStatus   = 2;   // 海西
    constexpr int CamLandEastStatus  = 3;   // 陆东
    constexpr int CamLandWestStatus  = 4;   // 陆西

    // ---- 大车防撞距离 ----
    constexpr int DistEast           = 5;   // 东侧目标距离, cm
    constexpr int DistWest           = 6;   // 西侧目标距离, cm

    // ---- 大车防撞动作 ----
    constexpr int SpeedLimitDir      = 7;   // 限速方向: 0=不限, 1=东向, 2=西向, 3=双侧
    constexpr int StopDir            = 8;   // 停止方向: 0=无, 1=东向, 2=西向, 3=双向

    // ---- 防吊起 ----
    constexpr int LiftAlarm          = 9;   // 0=安全, 1=吊起(含侧翻), 2=车辆开走, 3=程序异常
}
