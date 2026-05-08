#pragma once

// =========================================================================
// PTZ (Pan-Tilt-Zoom) 控制抽象接口
//
// 不是所有相机都支持云台/变焦：
//   - 球机：通常都支持
//   - 枪机/固定相机：不支持
//   - USB 摄像头：不支持
//   - 文件回放/RTSP 源：不支持
//
// CameraSource 通过 Ptz() 返回 IPtzControl*：
//   - nullptr —— 该路相机无 PTZ 能力
//   - 非空    —— 调用者可进一步用 HasPan()/HasZoom() 查询细分能力
//
// 调用方建议模式：
//     if (auto* ptz = source->Ptz()) {
//         if (ptz->HasPan())  ptz->Move(5.0f, 0.0f);
//         if (ptz->HasZoom()) ptz->Zoom(2.0f);
//     }
// =========================================================================
class IPtzControl {
public:
    virtual ~IPtzControl() = default;

    // ---- 能力查询 ----
    virtual bool HasPan()  const = 0;   // 是否支持水平/垂直旋转
    virtual bool HasZoom() const = 0;   // 是否支持光学变焦

    // ---- 相对运动（增量） ----
    // pan_deg:  正=右转, 负=左转，单位度
    // tilt_deg: 正=上仰, 负=下俯，单位度
    // 返回 false 表示不支持或失败
    virtual bool Move(float pan_deg, float tilt_deg) = 0;

    // ---- 绝对位置 ----
    virtual bool MoveTo(float pan_deg, float tilt_deg) = 0;

    // ---- 变焦倍率（绝对） ----
    virtual bool Zoom(float multiplier) = 0;

    // ---- 停止所有运动 ----
    virtual void StopAll() = 0;
};
