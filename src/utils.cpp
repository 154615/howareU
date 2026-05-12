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

// =========================================================================
// 新接口: 节点式 JSON 读取
// =========================================================================

bool LoadJsonFile(const std::string& json_path, Json::Value& root) {
    std::ifstream in(json_path, std::ios::binary);
    if (!in.is_open()) {
        LOG_COMMON("[utils] LoadJsonFile: 无法打开 " << json_path);
        return false;
    }
    Json::CharReaderBuilder builder;
    // 关键: 允许注释. 业务 json 里有大量 // 注释, 必须开
    builder["allowComments"] = true;
    builder["collectComments"] = false;

    std::string errs;
    Json::Value tmp;
    bool ok = Json::parseFromStream(builder, in, &tmp, &errs);
    in.close();
    if (!ok) {
        LOG_COMMON("[utils] LoadJsonFile: parse 失败 " << json_path << " : " << errs);
        return false;
    }
    root = std::move(tmp);
    return true;
}

bool HasMember(const Json::Value& node, const std::string& key) {
    return node.isObject() && node.isMember(key);
}

std::string GetJsonString(const Json::Value& node,
    const std::string& key,
    const std::string& def_value) {
    if (!HasMember(node, key)) return def_value;
    const Json::Value& v = node[key];
    if (!v.isString()) return def_value;
    return v.asString();
}

int GetJsonInt(const Json::Value& node,
    const std::string& key,
    int def_value) {
    if (!HasMember(node, key)) return def_value;
    const Json::Value& v = node[key];
    // 允许 int / uint / 布尔, 不允许字符串 → 避免静默吃下错填的字符串
    if (v.isIntegral()) return v.asInt();
    if (v.isBool())     return v.asBool() ? 1 : 0;
    return def_value;
}

float GetJsonFloat(const Json::Value& node,
    const std::string& key,
    float def_value) {
    if (!HasMember(node, key)) return def_value;
    const Json::Value& v = node[key];
    if (v.isNumeric()) return v.asFloat();
    return def_value;
}

bool GetJsonBool(const Json::Value& node,
    const std::string& key,
    bool def_value) {
    if (!HasMember(node, key)) return def_value;
    const Json::Value& v = node[key];
    if (v.isBool())     return v.asBool();
    if (v.isIntegral()) return v.asInt() != 0;
    return def_value;
}

// =========================================================================
// 旧接口(兼容层): 转发到新接口
// =========================================================================
// 注: 这几个函数每次调用都会重新打开文件 + 解析整棵树, 性能不如新接口.
//     新代码请用 LoadJsonFile + Get*. 这里只保留兼容.
//
// 还原旧行为时有两个细节:
//   - 老的 root_name 是顶层 key, 因此查的是 root[root_name]
//   - 旧版打开失败时返回 "0" / 0; 这里保留同样的语义
// =========================================================================

std::string read_String_Json(std::string json_file, std::string root_name)
{
    Json::Value root;
    if (!LoadJsonFile(json_file, root)) {
        return "0";
    }
    if (!HasMember(root, root_name)) return "";
    const Json::Value& v = root[root_name];
    if (!v.isString()) return "";
    return v.asString();
}

int read_Int_Json(std::string json_file, std::string root_name)
{
    Json::Value root;
    if (!LoadJsonFile(json_file, root)) {
        return 0;
    }
    return GetJsonInt(root, root_name, 0);
}

float read_Float_Json(std::string json_file, std::string root_name)
{
    Json::Value root;
    if (!LoadJsonFile(json_file, root)) {
        return 0;
    }
    return GetJsonFloat(root, root_name, 0.0f);
}

void write_Float_Json(std::string jsonFileName, std::string root_name, std::string root_name2, float data)
{
    // 读
    Json::Value jsonInfo;
    if (!LoadJsonFile(jsonFileName, jsonInfo)) {
        return;
    }
    // 改
    jsonInfo[root_name][root_name2] = Json::Value(data);
    // 写
    std::ofstream jsonFile(jsonFileName, std::ios::out);
    if (!jsonFile.is_open()) {
        LOG_COMMON("[utils] write_Float_Json: 无法打开写入 " << jsonFileName);
        return;
    }
    Json::StyledWriter sw;
    jsonFile << sw.write(jsonInfo);
    jsonFile.close();
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