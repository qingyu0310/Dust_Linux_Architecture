/**
 * @file hikrobot.cpp
 * @author qingyu
 * @brief 海康 MVS 采集实现 — USB 枚举、BGR 转换和异常回滚
 * @version 0.1
 * @date 2026-10-12
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "hikrobot.hpp"
#include "MvCameraControl.h"
#include "framework/cmd/shell/log/log.hpp"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <utility>

namespace modules::camera
{

namespace
{

std::mutex   runtime_mutex;         // SDK 运行时引用保护
unsigned int runtime_users = 0;     // 已初始化的相机实例数量

/**
 * @brief 将 SDK 错误转为包含操作名称和错误码的异常
 *
 * @param code SDK 返回值
 * @param operation 操作名称，用于定位失败节点
 * @throws std::runtime_error 返回值不是 MV_OK
 */
void Check(int code, const char* operation)
{
    if (code != MV_OK)
    {
        char message[160];
        std::snprintf(message, sizeof(message), "%s failed: 0x%08x", operation, static_cast<unsigned int>(code));
        const auto value = static_cast<std::uint32_t>(code);
        // 掉线、USB 读写失败、无数据和坏帧归为设备故障：允许上层重连；其余按不可恢复上报
        const bool recoverable = value == MV_E_DEV_OFFLINE || value == MV_E_NORESPONSE ||
                                 value == MV_E_USB_READ    || value == MV_E_USB_WRITE  || value == MV_E_USB_DEVICE ||
                                 value == MV_E_NODATA      || value == MV_E_ABNORMAL_IMAGE;
        throw HikRobotError(message, value, recoverable);
    }
}

/**
 * @brief 获取 SDK 运行时引用，首个相机实例执行初始化
 *
 * 运行时随进程存活（见 ReleaseRuntime），所以进程内 MV_CC_Initialize 只调用一次，
 * 之后的 Acquire 只增加计数。
 *
 * @throws std::runtime_error SDK 初始化失败
 */
void AcquireRuntime()
{
    std::lock_guard<std::mutex> lock(runtime_mutex);
    if (runtime_users == 0)
    {
        Check(MV_CC_Initialize(), "MV_CC_Initialize");
    }
    ++runtime_users;
}

/**
 * @brief 释放 SDK 运行时引用
 *
 * 调用方必须已持有一次引用；引用计数由互斥锁保护。
 *
 * 计数归零也**不**调用 MV_CC_Finalize：设备没插时的启动等待、掉线后的重连都会
 * 反复打开失败，那种场景下每秒一次的 Initialize/Finalize 来回会把 MVS 运行时拖崩
 * （实测约十几次来回后进程段错误）。运行时随进程存活，由操作系统回收，
 * 下一个实例再次 Acquire 时不会重复初始化 SDK。
 *
 * @return 恒为 true；保留返回值以便与调用方的错误处理保持形状
 */
bool ReleaseRuntime()
{
    std::lock_guard<std::mutex> lock(runtime_mutex);
    if (runtime_users > 0)
    {
        --runtime_users;
    }
    return true;
}

/**
 * @brief 解析一个十六进制 USB ID
 *
 * @param value 一至四位十六进制字符串，不含前缀
 * @return USB 厂商或产品 ID
 * @throws std::invalid_argument 字符串为空、过长或包含非法字符
 */
unsigned int ParseId(const std::string& value)
{
    if (value.empty() || value.size() > 4 || value.find_first_not_of("0123456789abcdefABCDEF") != std::string::npos)
    {
        throw std::invalid_argument("vid_pid must contain two hexadecimal USB IDs");
    }
    return static_cast<unsigned int>(std::stoul(value, nullptr, 16));
}

/**
 * @brief SDK 帧缓存作用域保护，转换异常时仍归还缓存
 */
struct BufferGuard
{
    void* handle;           // 取得缓存的设备句柄
    MV_FRAME_OUT* frame;    // 待归还缓存；显式归还后清空，避免重复释放

    /**
     * @brief 归还尚未显式释放的 SDK 缓存
     */
    ~BufferGuard()
    {
        if (frame != nullptr) {
            MV_CC_FreeImageBuffer(handle, frame);
        }
    }
};

/**
 * @brief 读取 sysfs 里的十六进制属性并转成小写
 *
 * @param path 属性文件路径，例如 /sys/bus/usb/devices/1-2/idVendor
 * @return 小写属性文本；读不到返回空串
 */
std::string ReadUsbId(const std::filesystem::path& path)
{
    std::ifstream input(path);
    std::string   value;

    if (!(input >> value)) {
        return {};
    }
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character)
    {
        return static_cast<char>(std::tolower(character));
    });

    return value;
}

} // namespace

/**
 * @brief 初始化 SDK、匹配 USB 设备并开始采集
 *
 * 多个设备匹配时须指定序列号；失败时释放部分初始化资源后重新抛出异常。
 *
 * @param config 相机配置，曝光单位为 ms，增益单位为 dB，帧率单位为 Hz
 * @throws std::logic_error 当前实例已经初始化
 * @throws std::invalid_argument 配置参数无效
 * @throws std::runtime_error 设备匹配或 SDK 操作失败
 */
void HikRobot::Init(const Config& config)
{
    if (handle_ != nullptr || runtime_)
    {
        throw std::logic_error("HikRobot is already open");
    }
    ValidateConfig(config);

    const auto separator = config.vid_pid.find(':');
    const auto vid    = ParseId(config.vid_pid.substr(0, separator));
    const auto pid    = ParseId(config.vid_pid.substr(separator + 1));

    // 打开设备前先记下请求配置：这次打不开时，重连还要用它，不能只认"成功过的那份"
    config_ = config;

    try
    {
        AcquireRuntime();
        runtime_ = true;

        MV_CC_DEVICE_INFO_LIST devices{};
        Check(MV_CC_EnumDevices(MV_USB_DEVICE, &devices), "MV_CC_EnumDevices");

        MV_CC_DEVICE_INFO* selected = nullptr;
        for (unsigned int i = 0; i < devices.nDeviceNum; ++i)
        {
            auto* device = devices.pDeviceInfo[i];
            if (device == nullptr || device->nTLayerType != MV_USB_DEVICE) {
                continue;
            }

            const auto& usb = device->SpecialInfo.stUsb3VInfo;
            const std::string serial(reinterpret_cast<const char*>(usb.chSerialNumber), strnlen(reinterpret_cast<const char*>(usb.chSerialNumber),  sizeof(usb.chSerialNumber)));

            if (usb.idVendor != vid || usb.idProduct != pid || (!config.serial_number.empty() && serial != config.serial_number))  {
                continue;
            }
            if (selected != nullptr) {
                throw std::runtime_error("multiple HikRobot cameras match; specify serial_number");
            }
            selected = device;
        }

        if (selected == nullptr)
        {
            throw HikRobotError("configured HikRobot USB camera not found", 0, true);
        }

        Check(MV_CC_CreateHandle(&handle_, selected), "MV_CC_CreateHandle");
        Check(MV_CC_OpenDevice(handle_), "MV_CC_OpenDevice");

        opened_ = true;
        Configure(config);

        Check(MV_CC_StartGrabbing(handle_), "MV_CC_StartGrabbing");

        const auto exposure_range = GetExposureRange();
        const auto gain_range     = GetGainRange();

        {
            std::lock_guard<std::mutex> lock(parameters_.mutex);
            parameters_.exposure_us = config.exposure_ms * 1000.0;
            parameters_.gain_db = config.gain;
            parameters_.exposure_min_us = exposure_range.minimum * 1000.0;
            parameters_.exposure_max_us = exposure_range.maximum * 1000.0;
            parameters_.gain_min_db = gain_range.minimum;
            parameters_.gain_max_db = gain_range.maximum;
        }

        grabbing_     = true;
        timeout_ms_   = config.timeout_ms;
        frame_number_ = 0;

        DUST_LOG_OK("HikRobot opened, exposure=%.3f ms gain=%.2f fps=%.1f", config.exposure_ms, config.gain, config.frame_rate);
    }
    catch (...)
    {
        // 回滚失败按不可恢复上报：句柄状态已不确定，不放上层继续重连
        if (!Stop())
        {
            throw HikRobotError("HikRobot initialization rollback failed", 0, false);
        }
        throw;
    }
}

/**
 * @brief 限时获取一帧，转换为独立持有内存的 BGR 图像
 *
 * 超时不会返回上一次图像。输出提交前归还 SDK 缓存，异常路径由作用域保护归还。
 * SDK 等待时限不包含像素转换耗时。
 *
 * @param image CV_8UC3 BGR 图像；超时或失败时清空
 * @param timestamp 主机取出 SDK 帧的 steady_clock 时间，非硬件曝光时间；失败时清零
 * @return true 获得有效图像；false SDK 等待超时
 * @throws std::logic_error 相机尚未开始采集
 * @throws std::runtime_error SDK 操作失败、帧数据无效或转换结果不完整
 */
bool HikRobot::Read(cv::Mat& image, std::chrono::steady_clock::time_point& timestamp)
{
    image.release();

    timestamp = {};

    if (!grabbing_)
    {
        throw std::logic_error("HikRobot is not grabbing");
    }

    MV_FRAME_OUT raw{};
    const auto result = MV_CC_GetImageBuffer(handle_, &raw, timeout_ms_);

    if (static_cast<unsigned int>(result) == MV_E_NODATA) {
        return false;
    }
    Check(result, "MV_CC_GetImageBuffer");

    BufferGuard buffer {
        handle_,
        &raw
    };

    const auto  received         = std::chrono::steady_clock::now();
    const auto& info = raw.stFrameInfo;

    if (raw.pBufAddr == nullptr || info.nWidth == 0 || info.nHeight == 0 ||
        static_cast<std::uint64_t>(info.nWidth) * info.nHeight * 3 > std::numeric_limits<unsigned int>::max() || info.nLostPacket != 0)
    {
        // 掉包或字段异常都按坏帧处理：这一帧不可信，抛给上层决定重连
        throw HikRobotError("invalid or incomplete HikRobot image buffer", 0, true);
    }
    cv::Mat bgr(info.nHeight, info.nWidth, CV_8UC3);

    MV_CC_PIXEL_CONVERT_PARAM_EX conversion{};
    conversion.nWidth         = info.nWidth;
    conversion.nHeight        = info.nHeight;
    conversion.pSrcData       = raw.pBufAddr;
    conversion.nSrcDataLen    = info.nFrameLen;
    conversion.enSrcPixelType = info.enPixelType;
    conversion.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
    conversion.pDstBuffer     = bgr.data;
    conversion.nDstBufferSize = static_cast<unsigned int>(bgr.total() * bgr.elemSize());

    Check(MV_CC_ConvertPixelTypeEx(handle_, &conversion), "MV_CC_ConvertPixelTypeEx");

    if (conversion.nDstLen != conversion.nDstBufferSize)
    {
        // 实际写入长度与预留缓冲不符：转换没走完，图不能用
        throw HikRobotError("HikRobot BGR conversion returned an incomplete image", 0, true);
    }

    const auto sequence = info.nFrameNum;
    const auto released = MV_CC_FreeImageBuffer(handle_, &raw);

    buffer.frame  = nullptr; // 已显式归还，不再由析构重复释放

    Check(released, "MV_CC_FreeImageBuffer");

    image         = std::move(bgr);
    timestamp     = received;
    frame_number_ = sequence; // 只在成功路径更新：对外表示最近一帧有效帧的帧号

    return true;
}

/**
 * @brief 依次停止取流、关闭设备、销毁句柄并释放 SDK 引用
 *
 * 可重复调用；一个清理步骤失败后仍继续执行后续步骤。
 * 调用方须先等待读取结束，不得与 Read 并发调用。
 *
 * @return true 所有清理步骤成功；false 至少一个 SDK 清理步骤失败
 */
bool HikRobot::Stop() noexcept
{
    bool success = true;
    if (grabbing_)
    {
        success   = MV_CC_StopGrabbing(handle_) == MV_OK && success;
        grabbing_ = false;
    }
    if (opened_)
    {
        success = MV_CC_CloseDevice(handle_) == MV_OK && success;
        opened_ = false;
    }
    if (handle_ != nullptr)
    {
        success = MV_CC_DestroyHandle(handle_) == MV_OK && success;
        handle_ = nullptr;
    }
    if (runtime_)
    {
        success  = ReleaseRuntime() && success;
        runtime_ = false;
    }
    return success;
}

/**
 * @brief 重新应用最近成功配置，保留配置到初始化结果确认
 */
void HikRobot::Reopen()
{
    const Config config = config_;
    Init(config);
}

/**
 * @brief 探测配置的 USB 相机是否已被系统枚举到
 *
 * 直接读 sysfs 的 idVendor / idProduct，不初始化 MVS 运行时：设备没插时反复
 * Initialize/Finalize 会把 SDK 拖进不稳定状态，启动等待必须先用它挡在前面。
 *
 * @param config 相机配置，只用其中的 vid_pid
 * @return true 至少有一台 VID:PID 匹配的 USB 设备；没有 sysfs 时也返回 true
 * @throws std::invalid_argument vid_pid 格式非法
 */
bool HikRobot::ProbeUsb(const Config& config)
{
    const auto separator = config.vid_pid.find(':');
    if (separator == std::string::npos)
    {
        throw std::invalid_argument("vid_pid must use the form 2bdf:0001");
    }

    // sysfs 属性是小写四位十六进制（例如 2bdf、0001），按同样格式生成待比较文本
    char vendor[8]  = {};
    char product[8] = {};
    std::snprintf(vendor,  sizeof(vendor),  "%04x", ParseId(config.vid_pid.substr(0, separator)));
    std::snprintf(product, sizeof(product), "%04x", ParseId(config.vid_pid.substr(separator + 1)));

    const std::filesystem::path root {"/sys/bus/usb/devices"};
    std::error_code error;
    if (!std::filesystem::exists(root, error))
    {
        return true; // 没有 sysfs（容器等）：不拦路，交给 SDK 自己判断
    }

    // 目录里还含 root hub 和各级 hub，只按 VID:PID 比；属性读不到时比较串为空，自然不匹配
    for (const auto& entry : std::filesystem::directory_iterator(root, error))
    {
        if (ReadUsbId(entry.path() / "idVendor") == vendor && ReadUsbId(entry.path() / "idProduct") == product) {
            return true;
        }
    }
    return false;
}

/**
 * @brief 查询当前设备曝光节点范围，SDK 微秒转换为毫秒
 * @return 当前设备允许的曝光范围，单位 ms
 * @throws std::exception 设备未打开或 SDK 查询失败
 */
HikRobot::Range HikRobot::GetExposureRange()
{
    if (!opened_) {
        throw std::logic_error("HikRobot is not open");
    }

    MVCC_FLOATVALUE value{};
    Check(MV_CC_GetFloatValue(handle_, "ExposureTime", &value), "Get ExposureTime");
    return {value.fMin / 1000.0, value.fMax / 1000.0}; // SDK 曝光节点是 μs，对外统一用 ms
}

/**
 * @brief 查询当前设备增益节点范围
 * @return 当前设备允许的增益范围，单位 dB
 * @throws std::exception 设备未打开或 SDK 查询失败
 */
HikRobot::Range HikRobot::GetGainRange()
{
    if (!opened_) {
        throw std::logic_error("HikRobot is not open");
    }

    MVCC_FLOATVALUE value{};
    Check(MV_CC_GetFloatValue(handle_, "Gain", &value), "Get Gain");
    return {value.fMin, value.fMax};
}

/**
 * @brief 校验并写入手动曝光节点，成功后提交当前值和重连配置
 * @param exposure_ms 曝光时间，单位 ms
 * @throws std::exception 设备未打开或 SDK 写入失败
 */
void HikRobot::SetExposure(double exposure_ms)
{
    const double exposure_us = exposure_ms * 1000.0;
    std::lock_guard<std::mutex> lock(parameters_.mutex); // 与 var 读值共用：范围、当前值、设备写入必须一致

    if (!opened_)
    {
        throw std::logic_error("HikRobot is not open");
    }
    if (!std::isfinite(exposure_us) || exposure_us < parameters_.exposure_min_us ||
        exposure_us > parameters_.exposure_max_us)
    {
        throw std::invalid_argument("exposure is outside current device range");
    }

    Check(MV_CC_SetFloatValue(handle_, "ExposureTime", static_cast<float>(exposure_us)), "ExposureTime");

    // 设备写入成功后才提交：Reopen 复现的是最近一次成功配置，不是被拒绝的请求
    config_.exposure_ms = exposure_ms;
    parameters_.exposure_us = exposure_us;
}

/**
 * @brief 校验并写入手动增益节点，成功后提交当前值和重连配置
 * @param gain_db 增益，单位 dB
 * @throws std::exception 设备未打开或 SDK 写入失败
 */
void HikRobot::SetGain(double gain_db)
{
    std::lock_guard<std::mutex> lock(parameters_.mutex);

    if (!opened_)
    {
        throw std::logic_error("HikRobot is not open");
    }
    if (!std::isfinite(gain_db) || gain_db < parameters_.gain_min_db ||
        gain_db > parameters_.gain_max_db)
    {
        throw std::invalid_argument("gain is outside current device range");
    }

    Check(MV_CC_SetFloatValue(handle_, "Gain", static_cast<float>(gain_db)), "Gain");

    // 同上：写入成功才更新当前值，失败时重连配置保持上一次成功的值
    config_.gain = gain_db;
    parameters_.gain_db = gain_db;
}

/**
 * @brief 检查配置数值及 USB ID 格式
 *
 * 这里只检查数值类型和通用范围，具体设备范围由 SDK 节点校验。
 *
 * @param config 调用方提供的相机配置
 * @throws std::invalid_argument 数值或 USB ID 格式无效
 */
void HikRobot::ValidateConfig(const Config& config)
{
    if (!std::isfinite(config.exposure_ms) || config.exposure_ms <= 0.0 || config.exposure_ms > std::numeric_limits<float>::max() / 1000.0 ||
        !std::isfinite(config.gain) || config.gain < 0.0 || config.gain > std::numeric_limits<float>::max() || !std::isfinite(config.frame_rate) ||
        config.frame_rate <= 0.0 || config.frame_rate > std::numeric_limits<float>::max() || config.timeout_ms == 0 || config.timeout_ms > 1000)
    {
        throw std::invalid_argument("invalid HikRobot exposure/gain/frame_rate/timeout");
    }

    const auto separator = config.vid_pid.find(':');
    if (separator == std::string::npos)
    {
        throw std::invalid_argument("vid_pid must use the form 2bdf:0001");
    }

    ParseId(config.vid_pid.substr(0, separator));
    ParseId(config.vid_pid.substr(separator + 1));
}

/**
 * @brief 配置自由运行、手动曝光和增益，使用 SDK 最新帧策略
 *
 * @param config 已通过通用校验的相机配置
 * @throws std::runtime_error 相机节点写入或采集策略设置失败
 */
void HikRobot::Configure(const Config& config)
{
    // 先关自动模式再写数值：自动曝光/增益开着时手动写入会被设备覆盖
    Check(MV_CC_SetEnumValue (handle_, "AcquisitionMode", MV_ACQ_MODE_CONTINUOUS),    "AcquisitionMode");
    Check(MV_CC_SetEnumValue (handle_, "TriggerMode",     MV_TRIGGER_MODE_OFF),       "TriggerMode");
    Check(MV_CC_SetEnumValue (handle_, "ExposureAuto",    MV_EXPOSURE_AUTO_MODE_OFF), "ExposureAuto");
    Check(MV_CC_SetEnumValue (handle_, "GainAuto",        MV_GAIN_MODE_OFF),          "GainAuto");

    Check(MV_CC_SetFloatValue(handle_, "ExposureTime",    static_cast<float>(config.exposure_ms * 1000.0)), "ExposureTime");
    Check(MV_CC_SetFloatValue(handle_, "Gain",            static_cast<float>(config.gain)),                 "Gain");

    Check(MV_CC_SetBoolValue (handle_, "AcquisitionFrameRateEnable", true), "AcquisitionFrameRateEnable");

    Check(MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", static_cast<float>(config.frame_rate)), "AcquisitionFrameRate");

    // 图像节点 3：SDK 内部缓冲深度；配合 LatestImagesOnly，取到的永远是最新帧而不是排队的旧帧
    Check(MV_CC_SetImageNodeNum(handle_, 3), "MV_CC_SetImageNodeNum");

    Check(MV_CC_SetGrabStrategy(handle_, MV_GrabStrategy_LatestImagesOnly), "MV_CC_SetGrabStrategy");
}

} // namespace modules::camera
