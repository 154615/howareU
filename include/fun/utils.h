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

// 核心写入函数
inline void WriteLogToFileAndConsole(const std::string& message) {
    std::lock_guard<std::mutex> lock(GetLogMutex());

    // 1. 获取当前系统时间
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    // 2. 格式化时间戳 (用于每行日志的开头，精确到秒)
    std::stringstream ss_time;
    ss_time << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %H:%M:%S");
    std::string time_str = ss_time.str();

    // 3. 格式化日期 (用于生成当天的日志文件名)
    std::stringstream ss_date;
    ss_date << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d");
    std::string date_str = ss_date.str();

    // 4. 确保日志目录存在
    std::string log_dir = "./save_log/logs/";
    std::error_code ec; // 避免 filesystem 抛出异常导致程序崩溃
    if (!std::filesystem::exists(log_dir, ec)) {
        std::filesystem::create_directories(log_dir, ec);
    }

    // 5. 拼装最终的日志文本
    std::string final_message = "[" + time_str + "] " + message;

    // 6. 同步输出到控制台
    std::cout << final_message << std::endl;

    // 7. 同步追加写入到当天的 txt 文件中
    std::string file_path = log_dir + "Log_" + date_str + ".txt"; // 例如：Log_2026-03-30.txt
    std::ofstream log_file(file_path, std::ios::app); // ios::app 表示追加模式 (Append)

    if (log_file.is_open()) {
        log_file << final_message << std::endl;
        log_file.close(); // 写入完毕立即关闭，防止系统断电导致缓存丢失
    }
    else {
        std::cerr << "[严重错误] 无法打开日志文件进行写入: " << file_path << std::endl;
    }
}

// 宏定义包装：利用 std::ostringstream 支持原生的流式输入 (<<)
#define SAFE_LOG(msg) do { \
    std::ostringstream _oss_; \
    _oss_ << msg; \
    WriteLogToFileAndConsole(_oss_.str()); \
} while(0)

#endif
