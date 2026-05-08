#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "yolov8_onnx.h"
#include "yolov8_seg_onnx.h"
#include "yolov8_utils.h"   // OutputParams

// =========================================================================
// yolo_detector.h —— YOLOv8 检测/分割模型的统一封装层
// -------------------------------------------------------------------------
// 模块定位:
//   算法消费层(第三层)的子模块. 把 Yolov8Onnx 和 Yolov8SegOnnx 两个
//   底层类合并成一个统一接口, 同时把"模型路径 + imgsz + 阈值 + 类别名
//   + CUDA 设置"全部参数化, 可以一份程序加载多套模型.
//
// 解决的问题:
//   旧方式必须直接持有 Yolov8Onnx / Yolov8SegOnnx, 类别和阈值在头里
//   硬编码, 一个进程只能跑一套模型. 现在每实例化一个 Yolov8Detector 就
//   是一套独立模型 + 独立参数, 同进程可共存多套(例如 cam0 跑集装箱
//   分割, cam1 跑人员检测).
//
// 关键约束:
//   - 一个 Yolov8Detector 只代表一个模型. 两个不同的模型 = 两个对象.
//   - 检测版(Detect) vs 分割版(Seg) 由 cfg.task 决定; 内部根据 task
//     选 Yolov8Onnx 或 Yolov8SegOnnx, 对外接口完全一致.
//   - mask_threshold 仅 Seg 任务用, Detect 时被忽略.
//
// 典型用法(单模型):
//     Yolov8DetectorConfig cfg;
//     cfg.task           = TaskType::Seg;
//     cfg.model_path     = "container_seg.onnx";
//     cfg.img_width      = 640;
//     cfg.img_height     = 640;
//     cfg.conf_threshold = 0.25f;
//     cfg.nms_threshold  = 0.7f;
//     cfg.mask_threshold = 0.8f;
//     cfg.class_names    = {"driver","container20", ...};
//     cfg.use_cuda       = true;
//
//     Yolov8Detector det(cfg);
//     std::string err;
//     if (!det.Load(&err)) { /* err 里有原因 */ return; }
//
//     std::vector<OutputParams> out;
//     det.Detect(frame, out);   // 多线程并发安全
//
// 典型用法(同进程多模型):
//     Yolov8Detector seg_det(seg_cfg);    seg_det.Load();
//     Yolov8Detector person_det(det_cfg); person_det.Load();
//     // 互不干扰, 各自带锁
//
// 线程安全:
//   - 构造 / Load() 不可并发, 视为初始化阶段.
//   - Detect() 内部加锁, 多线程并发会被串行化(底层 ORT Session 不保证
//     全线程安全, 这里统一收口).
//   - 查询函数 (IsLoaded / ClassNames / Task) 是无锁只读, 可任意并发.
// =========================================================================

// -------------------------------------------------------------------------
// TaskType —— 区分检测版(yolov8_onnx) 还是分割版(yolov8_seg_onnx)
// -------------------------------------------------------------------------
enum class TaskType {
    Detect,   // 走 Yolov8Onnx,    输出 box + class + conf
    Seg       // 走 Yolov8SegOnnx, 在 Detect 输出基础上加上每目标 mask
};

// -------------------------------------------------------------------------
// Yolov8DetectorConfig —— 一套模型的全部参数
//
// 实例化一个 detector 必须先填好这个结构体. 所有字段都有合理默认值,
// 除了 model_path 和 class_names 必填.
// -------------------------------------------------------------------------
struct Yolov8DetectorConfig {
    // 任务类型: Detect 或 Seg. 决定底层用哪个原始类.
    TaskType    task = TaskType::Seg;

    // ONNX 模型文件路径(必填). 相对路径基于 exe 工作目录.
    std::string model_path;

    // 网络输入尺寸. 必须与导出 ONNX 时的 imgsz 匹配, 否则推理会报错或精度跌.
    int   img_width = 640;
    int   img_height = 640;

    // 类别置信度阈值. NMS 之前的过滤; 越低召回越高, 误检越多.
    float conf_threshold = 0.25f;

    // NMS IOU 阈值. 越大越倾向保留重叠框, 越小越激进抑制.
    float nms_threshold = 0.7f;

    // 分割掩码二值化阈值, 仅 task=Seg 时生效. Detect 任务下被忽略.
    float mask_threshold = 0.8f;

    // 类别名表(必填, 顺序必须与训练时 names 一致).
    // 类别数 = class_names.size(), ONNX 输出的 class_id 是这个表的下标.
    std::vector<std::string> class_names;

    // ===== 推理后端 =====
    bool use_cuda = true;   // false 时走 CPU 推理(慢一个数量级)
    int  cuda_id = 0;      // CUDA 设备号(多卡时用)
    bool warmup = true;   // GPU 模式下 Load 时跑一次空推理预热, 消除首帧抖动

    // ---------------------------------------------------------------------
    // Validate —— 配置合法性检查
    // -----
    // 在 Load() 内部会被自动调用; 也可以提前手动调一次拿到错误信息.
    // 入参:
    //     err   非空时, 校验失败的具体原因写到这里.
    // 返回:
    //     true  全部字段合法
    //     false 至少有一项非法; *err 给出原因
    // ---------------------------------------------------------------------
    bool Validate(std::string* err = nullptr) const;
};

// -------------------------------------------------------------------------
// Yolov8Detector —— 模型的对外门面
// -------------------------------------------------------------------------
class Yolov8Detector {
public:
    // 构造: 只复制配置, 不读模型, 不分配 GPU 资源. 不抛异常.
    // 入参:
    //     cfg   见 Yolov8DetectorConfig 注释
    explicit Yolov8Detector(const Yolov8DetectorConfig& cfg);

    // 析构: 释放底层 Yolov8Onnx / Yolov8SegOnnx, 进而释放 ORT Session 和
    // GPU 显存. 阻塞 ~ 几毫秒.
    ~Yolov8Detector();

    Yolov8Detector(const Yolov8Detector&) = delete;
    Yolov8Detector& operator=(const Yolov8Detector&) = delete;

    // ---------------------------------------------------------------------
    // Load() —— 加载模型(必须在 Detect 之前调用一次, 且仅一次)
    // ---------------------------------------------------------------------
    // 功能:
    //     1) 校验配置(等价于先调 cfg.Validate)
    //     2) new 出对应的底层类(Yolov8Onnx 或 Yolov8SegOnnx)
    //     3) 把 cfg 中的 imgsz / 阈值 / 类别名注入底层类的 public 字段
    //     4) 调底层 ReadModel(model_path, use_cuda, cuda_id, warmup)
    // 入参:
    //     err   非空时, 失败原因写到这里(如 "model_path 为空" /
    //           "底层 ReadModel 失败: xxx.onnx")
    // 返回:
    //     true  加载成功; IsLoaded() 之后会一直返回 true
    //     false 加载失败; 内部状态回滚到未加载, 可以再尝试 Load
    // 阻塞性: 阻塞. 模型大小 + 是否 warmup 决定耗时(一般几百 ms ~ 几秒).
    // 线程安全: 不可并发. 视为单线程初始化阶段.
    bool Load(std::string* err = nullptr);

    // 是否已经成功 Load. 无锁只读.
    bool IsLoaded() const { return loaded_; }

    // ---------------------------------------------------------------------
    // Detect() —— 单帧推理
    // ---------------------------------------------------------------------
    // 功能: 对 frame 跑一次前向, 把检测结果写入 out.
    // 入参:
    //     frame  输入图像, BGR 三通道 CV_8UC3, 任意分辨率(内部会 letterbox
    //            缩放到 img_width × img_height). 注意原始类的 OnnxDetect
    //            签名是 cv::Mat& 非 const, 内部可能就地修改, 调用方如需保留
    //            原图请自行 clone.
    //     out    输出 vector<OutputParams>, 每元素一个目标:
    //              .id          类别 id(class_names 的下标)
    //              .confidence  置信度
    //              .box         检测框(基于原图坐标)
    //              .boxMask     仅 Seg 任务有, 实例 mask, 与原图同尺寸
    //            调用方应在调用前清空(函数内部不保证清空).
    // 返回:
    //     true   推理成功(out 可能为空, 表示该帧无目标)
    //     false  尚未 Load / frame 为空 / 底层推理失败
    // 阻塞性: 阻塞, 同步推理. GPU 推理一般 5~50ms / 帧.
    // 线程安全: 安全. 内部 mutex 串行化; 多线程同时调时排队执行, 不会崩.
    bool Detect(cv::Mat& frame, std::vector<OutputParams>& out);

    // 类别名查询, 用于绘制标签 / 写日志. 引用生命周期与本对象一致.
    const std::vector<std::string>& ClassNames() const { return cfg_.class_names; }

    // 当前模型的任务类型.
    TaskType Task() const { return cfg_.task; }

private:
    // 启动配置. Load 之后不应再修改.
    Yolov8DetectorConfig cfg_;

    // 是否已成功 Load. 控制 Detect() 的早返.
    bool                 loaded_ = false;

    // 推理串行化锁. 多线程并发调 Detect 时排队执行.
    std::mutex           mtx_;

    // 底层实例 —— 二选一; 由 cfg_.task 决定哪个被 new 出来, 另一个保持 nullptr.
    std::unique_ptr<Yolov8Onnx>    det_;   // task == Detect 时使用
    std::unique_ptr<Yolov8SegOnnx> seg_;   // task == Seg    时使用
};