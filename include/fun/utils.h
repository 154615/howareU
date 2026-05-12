#pragma once
#ifndef UTILS_H
#define UTILS_H
#include <iostream>
#include <json/json.h>
#include <string>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <filesystem>
#include <fun/modbus_cfg.h>
#include <mutex>


uint16_t int2uint16_t(int v);
// float保留两位小数转uint16，float整数部分需要小于100
uint16_t float2uint16_t(float v);
// 

void Log(std::string s);
void Log_no_date(std::string s);


// 读取Json文件，参数：JSON文件名，键名
std::string read_String_Json(std::string json_file, std::string root_name);
int read_Int_Json(std::string json_file, std::string root_name);
float read_Float_Json(std::string json_file, std::string root_name);
void write_Float_Json(std::string jsonFileName, std::string root_name, std::string root_name2, float data);

int getFileNumInData(std::string path);
std::string get_current_time();
std::string num2str(double num);



// 全局日志锁，保证多线程写入时控制台和文件都不会乱码
inline std::mutex& GetLogMutex() {
    static std::mutex log_mutex;
    return log_mutex;
}

// ============================================================================
// 日志通道
// ----------------------------------------------------------------------------
// 三个业务通道, 落盘到不同目录, 控制台合并输出(带前缀区分):
//
//   LOG_AC(msg)     → save_log/anti_collision/Log_YYYY-MM-DD.txt   控制台 [AC]
//   LOG_LIFT(msg)   → save_log/anti_lift/Log_YYYY-MM-DD.txt        控制台 [LIFT]
//   LOG_COMMON(msg) → save_log/common/Log_YYYY-MM-DD.txt           控制台 [COMMON]
//
// 用法 (跟旧 SAFE_LOG 完全一致, 支持流式 <<):
//   LOG_AC("[报警] 相机 " << (cam_index+1) << " 进入 " << level);
//   LOG_LIFT("[会话] cam" << cam_index << " 结束: " << reason);
//   LOG_COMMON("[main] 系统启动完成");
//
// SAFE_LOG 保留为 LOG_COMMON 的别名, 现有调用零改动.
// ============================================================================

enum class LogChannel { Common, AntiCollision, AntiLift };

// 通道 → 目录 / 控制台前缀 的映射. 改这里就改了全程.
inline const char* LogChannelDir(LogChannel ch) {
    switch (ch) {
    case LogChannel::AntiCollision: return "./save_log/anti_collision/";
    case LogChannel::AntiLift:      return "./save_log/anti_lift/";
    case LogChannel::Common:
    default:                        return "./save_log/common/";
    }
}
inline const char* LogChannelTag(LogChannel ch) {
    switch (ch) {
    case LogChannel::AntiCollision: return "[AC] ";
    case LogChannel::AntiLift:      return "[LIFT] ";
    case LogChannel::Common:
    default:                        return "[COMMON] ";
    }
}

// 核心写入函数: 单次调用 = 一行带时间戳 + 通道前缀, 同步写控制台 + 当日通道文件.
inline void WriteLogChannel(LogChannel ch, const std::string& message) {
    std::lock_guard<std::mutex> lock(GetLogMutex());

    // 1. 时间戳
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss_time;
    ss_time << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %H:%M:%S");
    std::string time_str = ss_time.str();

    std::stringstream ss_date;
    ss_date << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d");
    std::string date_str = ss_date.str();

    // 2. 目录懒创建
    const std::string log_dir = LogChannelDir(ch);
    std::error_code ec;
    if (!std::filesystem::exists(log_dir, ec)) {
        std::filesystem::create_directories(log_dir, ec);
    }

    // 3. 拼装最终行
    //    控制台: [AC] [时间] 内容   ← 通道前缀在最前面, 一眼看清归属
    //    文件:   [时间] 内容        ← 目录已经分了, 不重复加通道前缀
    const std::string console_line = std::string(LogChannelTag(ch)) + "[" + time_str + "] " + message;
    const std::string file_line = "[" + time_str + "] " + message;

    // 4. 控制台
    std::cout << console_line << std::endl;

    // 5. 当日文件
    const std::string file_path = log_dir + "Log_" + date_str + ".txt";
    std::ofstream log_file(file_path, std::ios::app);
    if (log_file.is_open()) {
        log_file << file_line << std::endl;
        log_file.close();
    }
    else {
        std::cerr << "[严重错误] 无法打开日志文件进行写入: " << file_path << std::endl;
    }
}

// ----------------------------------------------------------------------------
// 旧 API 保留: WriteLogToFileAndConsole 仍可用, 等价于写 Common 通道.
// 老代码里直接调它的地方不需要改.
// ----------------------------------------------------------------------------
inline void WriteLogToFileAndConsole(const std::string& message) {
    WriteLogChannel(LogChannel::Common, message);
}

// ----------------------------------------------------------------------------
// 三个业务宏: 流式 << 用法, 跟旧 SAFE_LOG 完全一致.
// ----------------------------------------------------------------------------
#define LOG_AC(msg) do { \
    std::ostringstream _oss_; \
    _oss_ << msg; \
    WriteLogChannel(LogChannel::AntiCollision, _oss_.str()); \
} while(0)

#define LOG_LIFT(msg) do { \
    std::ostringstream _oss_; \
    _oss_ << msg; \
    WriteLogChannel(LogChannel::AntiLift, _oss_.str()); \
} while(0)

#define LOG_COMMON(msg) do { \
    std::ostringstream _oss_; \
    _oss_ << msg; \
    WriteLogChannel(LogChannel::Common, _oss_.str()); \
} while(0)

// 兼容旧代码: SAFE_LOG = LOG_COMMON. 不删, 让任何 #include "utils.h" 的旧 cpp 都能继续编.
#define SAFE_LOG(msg) LOG_COMMON(msg)

#endif