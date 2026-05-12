#include <fun/utils.h>
#include <filesystem>

uint16_t int2uint16_t(int v) {
    uint16_t data;
    unsigned char vv[2] = { 0, 0 };
    vv[0] |= v;
    vv[1] |= v >> 8;
    memcpy(&data, vv, 2);
    return data;
}

uint16_t float2uint16_t(float v) {
    uint16_t data;
    int a = int(v * 100);
    unsigned char vv[2] = { 0, 0 };
    vv[0] |= a >> 8;
    vv[1] |= a;
    memcpy(&data, vv, 2);
    return data;
}

// 兼容旧 API: 转发到统一通道日志(Common).
// 老路径 LOG/log2.txt 不再使用, 新落盘到 ./save_log/common/Log_YYYY-MM-DD.txt.
void Log(std::string s) {
    WriteLogChannel(LogChannel::Common, s);
}

void Log_no_date(std::string s) {
    // 旧函数本意是"无时间戳", 但新通道全部带时间戳;
    // 保留接口避免编译错误, 行为与 Log() 一致.
    WriteLogChannel(LogChannel::Common, s);
}

std::string read_String_Json(std::string json_file, std::string root_name)
{
    Json::Reader reader;
    Json::Value root;
    std::string name;

    //从文件中读取，保证当前文件有demo.json文件  
    std::ifstream in(json_file, std::ios::binary);

    if (!in.is_open())
    {
        LOG_COMMON("[utils] Error opening file");
        return "0";
    }
    //读取根节点信息
    if (reader.parse(in, root))
        name = root[root_name].asString();
    else
        LOG_COMMON("[utils] parse error");
    in.close();
    return name;
}

int read_Int_Json(std::string json_file, std::string root_name)
{
    Json::Reader reader;
    Json::Value root;
    int data = 0;

    //从文件中读取，保证当前文件有demo.json文件  
    std::ifstream in(json_file, std::ios::binary);

    if (!in.is_open())
    {
        LOG_COMMON("[utils] Error opening file");
        return 0;
    }
    //读取根节点信息
    if (reader.parse(in, root))
        data = root[root_name].asInt();
    else
        LOG_COMMON("[utils] parse error");

    in.close();
    return data;
}

float read_Float_Json(std::string json_file, std::string root_name)
{
    Json::Reader reader;
    Json::Value root;
    float data = 0;

    //从文件中读取，保证当前文件有json文件  
    std::ifstream in(json_file, std::ios::binary);

    if (!in.is_open())
    {
        LOG_COMMON("[utils] Error opening file");
        return 0;
    }
    //读取根节点信息
    if (reader.parse(in, root))
        data = root[root_name].asFloat();
    else
        LOG_COMMON("[utils] parse error");

    in.close();
    return data;
}

void write_Float_Json(std::string jsonFileName, std::string root_name, std::string root_name2, float data)
{
    // 读打开文件
    std::ifstream jsonFile(jsonFileName, std::ios::binary);
    Json::Value jsonInfo;
    if (!jsonFile.is_open())
    {
        LOG_COMMON("[utils] File open error.");
        return;
    }
    else
    {
        // 读取保存到json value
        Json::Reader reader;
        reader.parse(jsonFile, jsonInfo);
        jsonFile.close();
        // 写打开文件，修改json value，写入保存，关闭
        std::ofstream jsonFile(jsonFileName, std::ios::out);
        jsonInfo[root_name][root_name2] = Json::Value(data);
        Json::StyledWriter sw;
        jsonFile << sw.write(jsonInfo);
        jsonFile.close();
    }
}


int getFileNumInData(std::string path)
{
    std::filesystem::directory_iterator list(path);
    int i = 0;
    for (auto& it : list)
    {
        i++;
        // std::cout << it.path().filename() << std::endl;
    }
    return i;
}

std::string get_current_time() {
    std::time_t raw_time = std::time(nullptr);

    // 使用线程安全的本地时间转换
    std::tm tm_struct;
#if defined(_WIN32)
    localtime_s(&tm_struct, &raw_time);  // Windows 安全版本
#else
    localtime_r(&raw_time, &tm_struct);  // Linux/macOS 安全版本
#endif

    // 格式化输出（缓冲方式安全）
    char buffer[80];
    strftime(buffer, sizeof(buffer), "%F %H_%M_%S", &tm_struct);
    return std::string(buffer);
}

std::string num2str(double num) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << num; // 固定小数点，保留2位
    std::string result = ss.str();
    return result;
}