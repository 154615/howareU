/**
 * @file HikvisionCamera.h
 * @brief 海康相机高可用控制库 (纯 SDK 取流解码 - 并发安全优化版)
 * ##########################
 * ##########################海康相机半球机驱动逻辑，勿动
 *
 * 【新增】connect(... , bool only_login)
 *   only_login = true  : 仅登录 + 启动 PTZ 通道, 不调用 NET_DVR_RealPlay
 *                        (用于把取流交给 OpenCV cudacodec 走 RTSP)
 *   only_login = false : 原行为, 登录后立即 RealPlay 开 SDK 软解码
 */

#ifndef HIKVISION_CAMERA_H
#define HIKVISION_CAMERA_H

#include <iostream>
#include <string>
#include <mutex>
#include <atomic>
#include <opencv2/opencv.hpp>
#include "HCNetSDK.h"
#include "plaympeg4.h"

class HikvisionCamera {
public:
    HikvisionCamera();
    ~HikvisionCamera();

    // ==========================================
    // 1. 连接与状态监控
    // ==========================================
    // only_login = true  → 只登录, 不开取流（PTZ 仍可用）
    // only_login = false → 旧行为, 登录后立即开 SDK 软解码取流
    bool connect(const std::string& ip, int port,
        const std::string& user, const std::string& pwd,
        int channel = 1, bool only_login = false);

    void disconnect();

    bool isStreamConnected() const;
    void markDisconnected();

    // 仅当 only_login=true 时, 上层可显式拉起 SDK 取流(软解兜底)
    bool startRealPlay();
    void stopRealPlay();
    bool isRealPlaying() const { return m_realPlayHandle >= 0; }

    // ==========================================
    // 2. 画面取流 (业务层调用)
    // ==========================================
    bool getLatestFrame(cv::Mat& outFrame);

    // ==========================================
    // 3. 云台与变焦控制 (PTZ)
    // ==========================================
    void turnLeft(unsigned int speed = 4);
    void turnRight(unsigned int speed = 4);
    void tiltUp(unsigned int speed = 4);
    void tiltDown(unsigned int speed = 4);
    void zoomIn(unsigned int speed = 4);
    void zoomOut(unsigned int speed = 4);
    void stopAllActions();

    bool setAbsoluteAngle(float pan, float tilt);
    bool setRelativeAngle(float panOffset, float tiltOffset);
    bool setAbsoluteZoom(float zoomMultiplier);

    // ==========================================
    // 内部 SDK 回调处理接口
    // ==========================================
    void processRealData(DWORD dwDataType, BYTE* pBuffer, DWORD dwBufSize);
    void processDecodedFrame(char* pBuf, long nSize, FRAME_INFO* pFrameInfo);

private:
    long m_userID;
    long m_realPlayHandle;
    long m_playPort;
    int  m_channel;

    std::mutex        m_connMutex;
    std::mutex        m_ptzMutex;
    std::mutex        m_frameMutex;
    cv::Mat           m_latestFrame;

    std::atomic<bool> m_isStreamConnected;

    bool sendPTZCommand(unsigned int command, bool start, unsigned int speed);

    static void CALLBACK g_RealDataCallBack(LONG lRealHandle, DWORD dwDataType, BYTE* pBuffer, DWORD dwBufSize, void* pUser);
    static void CALLBACK g_DecCallBack(long nPort, char* pBuf, long nSize, FRAME_INFO* pFrameInfo, long nReserved1, long nReserved2);
};

#endif // HIKVISION_CAMERA_H