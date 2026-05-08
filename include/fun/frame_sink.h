#pragma once

#include <opencv2/opencv.hpp>

// =========================================================================
// 通用帧消费者接口
//
// 任何想从 CameraSource 接收帧的模块（防撞、录像、UI 预览、目标跟踪…）
// 实现这个接口即可。CameraSource 不关心具体类型，只调 OnFrame()。
//
// 实现方注意：
//   - OnFrame 在 CameraSource 的轮询线程中被调用，必须尽快返回
//   - 重活（推理、编码、磁盘 IO）应在 OnFrame 里入队并交给自己的线程做
//   - frame 是 const 引用，如要保留请自行 clone()
// =========================================================================
class FrameSink {
public:
    virtual ~FrameSink() = default;

    // cam_index: 该路相机在订阅它的 CameraSource 中的逻辑索引
    //            (含义由上层装配方约定，通常 0~3)
    // frame:     当前最新帧（BGR）
    virtual void OnFrame(int cam_index, const cv::Mat& frame) = 0;
};
