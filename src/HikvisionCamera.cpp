/**
 * @file HikvisionCamera.cpp
 * @brief 海康相机控制库实现 (SDK 原生解码流 - 并发安全优化版)
 *
 * 【新增】only_login 模式：只登录, 不开 RealPlay; PTZ 仍可用.
 *        startRealPlay()/stopRealPlay() 用于软解兜底.
 */

#include "HikvisionCamera.h"
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <thread>
#include <chrono>

#include "utils.h"   // LOG_COMMON

static std::unordered_map<long, HikvisionCamera*> g_portToInstanceMap;
static std::unordered_map<long, HikvisionCamera*> g_userToInstanceMap;
static std::mutex g_mapMutex;

static void CALLBACK g_ExceptionCallBack(DWORD dwType, LONG lUserID, LONG lHandle, void* pUser) {
    std::lock_guard<std::mutex> lock(g_mapMutex);
    auto it = g_userToInstanceMap.find(lUserID);
    if (it != g_userToInstanceMap.end()) {
        it->second->markDisconnected();
    }
}

static WORD FloatToBCD(float val) {
    val = (std::max)(0.0f, (std::min)(val, 3599.9f));
    int v = (int)(val * 10.0f + 0.5f); WORD bcd = 0; int shift = 0;
    while (v > 0 && shift < 16) { bcd |= ((v % 10) << shift); v /= 10; shift += 4; }
    return bcd;
}
static float BCDToFloat(WORD bcd) {
    float v = 0; float mul = 0.1f;
    while (bcd > 0) { v += (bcd & 0xF) * mul; bcd >>= 4; mul *= 10.0f; }
    return v;
}

HikvisionCamera::HikvisionCamera() :
    m_userID(-1), m_realPlayHandle(-1), m_playPort(-1), m_channel(1), m_isStreamConnected(false) {

    NET_DVR_Init();
    NET_DVR_SetConnectTime(2000, 1);
    NET_DVR_SetReconnect(10000, true);
    NET_DVR_SetExceptionCallBack_V30(0, NULL, g_ExceptionCallBack, NULL);
}

HikvisionCamera::~HikvisionCamera() {
    disconnect();
    NET_DVR_Cleanup();
}

bool HikvisionCamera::connect(const std::string& ip, int port,
    const std::string& user, const std::string& pwd,
    int channel, bool only_login) {
    std::lock_guard<std::mutex> lock(m_connMutex);

    m_channel = channel;
    m_isStreamConnected.store(false, std::memory_order_release);

    NET_DVR_USER_LOGIN_INFO loginInfo = { 0 };
    NET_DVR_DEVICEINFO_V40 deviceInfo = { 0 };

    loginInfo.bUseAsynLogin = 0;
    strncpy(loginInfo.sDeviceAddress, ip.c_str(), NET_DVR_DEV_ADDRESS_MAX_LEN - 1);
    loginInfo.sDeviceAddress[NET_DVR_DEV_ADDRESS_MAX_LEN - 1] = '\0';
    loginInfo.wPort = port;
    strncpy(loginInfo.sUserName, user.c_str(), NET_DVR_LOGIN_USERNAME_MAX_LEN - 1);
    loginInfo.sUserName[NET_DVR_LOGIN_USERNAME_MAX_LEN - 1] = '\0';
    strncpy(loginInfo.sPassword, pwd.c_str(), NET_DVR_LOGIN_PASSWD_MAX_LEN - 1);
    loginInfo.sPassword[NET_DVR_LOGIN_PASSWD_MAX_LEN - 1] = '\0';

    m_userID = NET_DVR_Login_V40(&loginInfo, &deviceInfo);
    if (m_userID < 0) {
        LOG_COMMON("[Hikvision] SDK Login failed! Code: " << NET_DVR_GetLastError());
        return false;
    }
    LOG_COMMON("[Hikvision] Device SDK Connected. UserID: " << m_userID
        << " | Channel: " << m_channel
        << (only_login ? " | Mode: PTZ-only" : " | Mode: SDK-stream"));

    {
        std::lock_guard<std::mutex> mapLock(g_mapMutex);
        g_userToInstanceMap[m_userID] = this;
    }

    if (only_login) {
        return true;
    }

    NET_DVR_PREVIEWINFO previewInfo = { 0 };
    previewInfo.hPlayWnd = NULL;
    previewInfo.lChannel = m_channel;
    previewInfo.dwStreamType = 0;
    previewInfo.dwLinkMode = 0;
    previewInfo.bBlocked = 1;

    m_realPlayHandle = NET_DVR_RealPlay_V40(m_userID, &previewInfo, g_RealDataCallBack, this);
    if (m_realPlayHandle < 0) {
        LOG_COMMON("[Hikvision] RealPlay failed! Code: " << NET_DVR_GetLastError());
        NET_DVR_Logout(m_userID);
        m_userID = -1;
        return false;
    }
    return true;
}

bool HikvisionCamera::startRealPlay() {
    std::lock_guard<std::mutex> lock(m_connMutex);
    if (m_userID < 0) return false;
    if (m_realPlayHandle >= 0) return true;

    NET_DVR_PREVIEWINFO previewInfo = { 0 };
    previewInfo.hPlayWnd = NULL;
    previewInfo.lChannel = m_channel;
    previewInfo.dwStreamType = 0;
    previewInfo.dwLinkMode = 0;
    previewInfo.bBlocked = 1;

    m_realPlayHandle = NET_DVR_RealPlay_V40(m_userID, &previewInfo, g_RealDataCallBack, this);
    if (m_realPlayHandle < 0) {
        LOG_COMMON("[Hikvision] startRealPlay failed! Code: " << NET_DVR_GetLastError());
        return false;
    }
    LOG_COMMON("[Hikvision] SDK soft-decode RealPlay started (fallback)");
    return true;
}

void HikvisionCamera::stopRealPlay() {
    std::lock_guard<std::mutex> lock(m_connMutex);
    if (m_realPlayHandle >= 0) {
        NET_DVR_StopRealPlay(m_realPlayHandle);
        m_realPlayHandle = -1;
    }
    if (m_playPort >= 0) {
        PlayM4_Stop(m_playPort);
        PlayM4_CloseStream(m_playPort);
        PlayM4_FreePort(m_playPort);
        {
            std::lock_guard<std::mutex> mapLock(g_mapMutex);
            g_portToInstanceMap.erase(m_playPort);
        }
        m_playPort = -1;
    }
    m_isStreamConnected.store(false, std::memory_order_release);
}

void HikvisionCamera::disconnect() {
    std::lock_guard<std::mutex> lock(m_connMutex);

    if (m_realPlayHandle >= 0) {
        NET_DVR_StopRealPlay(m_realPlayHandle);
        m_realPlayHandle = -1;
    }

    if (m_playPort >= 0) {
        PlayM4_Stop(m_playPort);
        PlayM4_CloseStream(m_playPort);
        PlayM4_FreePort(m_playPort);
        {
            std::lock_guard<std::mutex> mapLock(g_mapMutex);
            g_portToInstanceMap.erase(m_playPort);
        }
        m_playPort = -1;
    }

    if (m_userID >= 0) {
        stopAllActions();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        NET_DVR_Logout(m_userID);
        {
            std::lock_guard<std::mutex> mapLock(g_mapMutex);
            g_userToInstanceMap.erase(m_userID);
        }
        LOG_COMMON("[Hikvision] Device SDK disconnected.");
        m_userID = -1;
    }

    m_isStreamConnected.store(false, std::memory_order_release);
}

bool HikvisionCamera::isStreamConnected() const {
    return m_isStreamConnected.load(std::memory_order_acquire);
}

void HikvisionCamera::markDisconnected() {
    m_isStreamConnected.store(false, std::memory_order_release);
}

bool HikvisionCamera::getLatestFrame(cv::Mat& outFrame) {
    std::lock_guard<std::mutex> lock(m_frameMutex);
    if (m_latestFrame.empty()) return false;
    m_latestFrame.copyTo(outFrame);
    return true;
}

void CALLBACK HikvisionCamera::g_RealDataCallBack(LONG lRealHandle, DWORD dwDataType, BYTE* pBuffer, DWORD dwBufSize, void* pUser) {
    HikvisionCamera* camera = static_cast<HikvisionCamera*>(pUser);
    if (camera) camera->processRealData(dwDataType, pBuffer, dwBufSize);
}

void HikvisionCamera::processRealData(DWORD dwDataType, BYTE* pBuffer, DWORD dwBufSize) {
    switch (dwDataType) {
    case NET_DVR_SYSHEAD:
        if (m_playPort >= 0) {
            PlayM4_Stop(m_playPort);
            PlayM4_CloseStream(m_playPort);
            PlayM4_FreePort(m_playPort);
            {
                std::lock_guard<std::mutex> lock(g_mapMutex);
                g_portToInstanceMap.erase(m_playPort);
            }
            m_playPort = -1;
        }

        if (PlayM4_GetPort(&m_playPort)) {
            {
                std::lock_guard<std::mutex> lock(g_mapMutex);
                g_portToInstanceMap[m_playPort] = this;
            }

            PlayM4_SetStreamOpenMode(m_playPort, STREAME_REALTIME);
            if (PlayM4_OpenStream(m_playPort, pBuffer, dwBufSize, 1024 * 1024)) {
                if (!PlayM4_SetDecCallBack(m_playPort, g_DecCallBack) || !PlayM4_Play(m_playPort, NULL)) {
                    LOG_COMMON("[Hikvision] PlayM4 Setup failed! Cleaning up port...");
                    PlayM4_CloseStream(m_playPort);
                    PlayM4_FreePort(m_playPort);
                    {
                        std::lock_guard<std::mutex> lock(g_mapMutex);
                        g_portToInstanceMap.erase(m_playPort);
                    }
                    m_playPort = -1;
                }
            }
            else {
                PlayM4_FreePort(m_playPort);
                {
                    std::lock_guard<std::mutex> lock(g_mapMutex);
                    g_portToInstanceMap.erase(m_playPort);
                }
                m_playPort = -1;
            }
        }
        break;

    case NET_DVR_STREAMDATA:
        if (dwBufSize > 0 && m_playPort >= 0) {
            PlayM4_InputData(m_playPort, pBuffer, dwBufSize);
        }
        break;
    }
}

void CALLBACK HikvisionCamera::g_DecCallBack(long nPort, char* pBuf, long nSize, FRAME_INFO* pFrameInfo, long nReserved1, long nReserved2) {
    HikvisionCamera* camera = nullptr;
    {
        std::lock_guard<std::mutex> lock(g_mapMutex);
        auto it = g_portToInstanceMap.find(nPort);
        if (it != g_portToInstanceMap.end()) camera = it->second;
    }
    if (camera) camera->processDecodedFrame(pBuf, nSize, pFrameInfo);
}

void HikvisionCamera::processDecodedFrame(char* pBuf, long nSize, FRAME_INFO* pFrameInfo) {
    if (pFrameInfo->nType == T_YV12) {
        cv::Mat yuvMat(pFrameInfo->nHeight + pFrameInfo->nHeight / 2, pFrameInfo->nWidth, CV_8UC1, (unsigned char*)pBuf);
        cv::Mat bgrMat;
        cv::cvtColor(yuvMat, bgrMat, cv::COLOR_YUV2BGR_YV12);

        {
            std::lock_guard<std::mutex> lock(m_frameMutex);
            m_latestFrame = bgrMat;
        }

        if (!m_isStreamConnected.load(std::memory_order_acquire)) {
            m_isStreamConnected.store(true, std::memory_order_release);
        }
    }
}

// ==========================================
// 云台控制相关 (保持原样)
// ==========================================
bool HikvisionCamera::sendPTZCommand(unsigned int command, bool start, unsigned int speed) {
    std::lock_guard<std::mutex> lock(m_ptzMutex);
    if (m_userID < 0) return false;
    DWORD action = start ? 0 : 1;
    return NET_DVR_PTZControlWithSpeed_Other(m_userID, m_channel, command, action, speed) == TRUE;
}
void HikvisionCamera::turnLeft(unsigned int speed) { sendPTZCommand(PAN_LEFT, true, speed); }
void HikvisionCamera::turnRight(unsigned int speed) { sendPTZCommand(PAN_RIGHT, true, speed); }
void HikvisionCamera::tiltUp(unsigned int speed) { sendPTZCommand(TILT_UP, true, speed); }
void HikvisionCamera::tiltDown(unsigned int speed) { sendPTZCommand(TILT_DOWN, true, speed); }
void HikvisionCamera::zoomIn(unsigned int speed) { sendPTZCommand(ZOOM_IN, true, speed); }
void HikvisionCamera::zoomOut(unsigned int speed) { sendPTZCommand(ZOOM_OUT, true, speed); }
void HikvisionCamera::stopAllActions() {
    sendPTZCommand(PAN_LEFT, false, 0); sendPTZCommand(PAN_RIGHT, false, 0);
    sendPTZCommand(TILT_UP, false, 0);  sendPTZCommand(TILT_DOWN, false, 0);
    sendPTZCommand(ZOOM_IN, false, 0);  sendPTZCommand(ZOOM_OUT, false, 0);
}

bool HikvisionCamera::setAbsoluteAngle(float pan, float tilt) {
    std::lock_guard<std::mutex> lock(m_ptzMutex);
    if (m_userID < 0) return false;
    NET_DVR_PTZPOS currentPtz = { 0 }; DWORD bytesReturned = 0;
    NET_DVR_GetDVRConfig(m_userID, NET_DVR_GET_PTZPOS, m_channel, &currentPtz, sizeof(currentPtz), &bytesReturned);
    NET_DVR_PTZPOS ptzPos = { 0 }; ptzPos.wAction = 1;
    ptzPos.wPanPos = FloatToBCD(pan); ptzPos.wTiltPos = FloatToBCD(tilt); ptzPos.wZoomPos = currentPtz.wZoomPos;
    return NET_DVR_SetDVRConfig(m_userID, NET_DVR_SET_PTZPOS, m_channel, &ptzPos, sizeof(ptzPos)) == TRUE;
}

bool HikvisionCamera::setRelativeAngle(float panOffset, float tiltOffset) {
    std::lock_guard<std::mutex> lock(m_ptzMutex);
    if (m_userID < 0) return false;
    NET_DVR_PTZPOS currentPtz = { 0 }; DWORD bytesReturned = 0;
    if (!NET_DVR_GetDVRConfig(m_userID, NET_DVR_GET_PTZPOS, m_channel, &currentPtz, sizeof(currentPtz), &bytesReturned)) return false;

    float curPan = BCDToFloat(currentPtz.wPanPos); float curTilt = BCDToFloat(currentPtz.wTiltPos);
    float newPan = std::fmod(curPan + panOffset + 3600.0f, 360.0f);
    float newTilt = std::clamp(curTilt + tiltOffset, 0.0f, 90.0f);

    NET_DVR_PTZPOS ptzPos = { 0 }; ptzPos.wAction = 1;
    ptzPos.wPanPos = FloatToBCD(newPan); ptzPos.wTiltPos = FloatToBCD(newTilt); ptzPos.wZoomPos = currentPtz.wZoomPos;
    return NET_DVR_SetDVRConfig(m_userID, NET_DVR_SET_PTZPOS, m_channel, &ptzPos, sizeof(ptzPos)) == TRUE;
}

bool HikvisionCamera::setAbsoluteZoom(float zoomMultiplier) {
    std::lock_guard<std::mutex> lock(m_ptzMutex);
    if (m_userID < 0) return false;
    NET_DVR_PTZPOS currentPtz = { 0 }; DWORD bytesReturned = 0;
    if (!NET_DVR_GetDVRConfig(m_userID, NET_DVR_GET_PTZPOS, m_channel, &currentPtz, sizeof(currentPtz), &bytesReturned)) return false;
    NET_DVR_PTZPOS ptzPos = { 0 }; ptzPos.wAction = 1;
    ptzPos.wPanPos = currentPtz.wPanPos; ptzPos.wTiltPos = currentPtz.wTiltPos; ptzPos.wZoomPos = FloatToBCD(zoomMultiplier);
    return NET_DVR_SetDVRConfig(m_userID, NET_DVR_SET_PTZPOS, m_channel, &ptzPos, sizeof(ptzPos)) == TRUE;
}