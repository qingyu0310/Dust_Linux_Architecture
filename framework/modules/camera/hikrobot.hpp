/**
 * @file hikrobot.hpp
 * @author qingyu
 * @brief 海康 USB 相机模块 — 参数配置、限时读帧与采集资源管理
 * @version 0.1
 * @date 2026-10-12
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <chrono>
#include <cstdint>
#include <opencv2/core.hpp>
#include <string>
#include <stdexcept>
#include <mutex>

namespace modules::camera
{

/**
 * @brief 相机故障信息，保留 SDK 错误码并区分可恢复设备故障
 */
class HikRobotError final : public std::runtime_error
{
public:
    /**
     * @brief 构造故障，非 SDK 故障的 code 使用 0
     */
    HikRobotError(const std::string& message, std::uint32_t code, bool recoverable) : std::runtime_error(message), code_(code), recoverable_(recoverable) {}

    /**
     * @brief 返回原始 SDK 错误码，模块自身故障为 0
     */
    std::uint32_t GetCode() const noexcept
    {
        return code_;
    }

    /**
     * @brief 查询是否允许业务层重新初始化设备
     */
    bool IsRecoverable() const noexcept
    {
        return recoverable_;
    }

private:
    std::uint32_t code_;    // SDK 返回码；0 表示模块自身检测到的故障
    bool recoverable_;     // 是否允许业务尝试重新初始化
};

/**
 * @brief 海康 USB 相机，参考 Dust_infantry_vision 的 HikRobot 采集流程
 *
 * 调用方提供配置并组织采集；接口须串行调用，读取结束后才能释放设备。
 */
class HikRobot final
{
public:
    /**
     * @brief 相机配置，取值须满足具体设备的节点范围
     */
    struct Config
    {
        double       exposure_ms = 5.0;                 // 曝光时间，单位 ms
        double       gain        = 0.0;                 // 相机增益，单位 dB
        double       frame_rate  = 30.0;                // 请求采集帧率，单位 Hz
        std::string  vid_pid     = "2bdf:0001";         // USB 厂商与产品 ID，十六进制
        std::string  serial_number;                     // 空串要求仅有一个匹配设备
        unsigned int timeout_ms = 100;                  // SDK 取帧等待，单位 ms，范围 [1, 1000]
    };

    /**
     * @brief 创建未初始化的相机对象，不访问设备
     */
    HikRobot() = default;

    /**
     * @brief 析构时释放相机及 SDK 引用
     */
    ~HikRobot() { Stop(); }
    HikRobot(const HikRobot&)            = delete;
    HikRobot& operator=(const HikRobot&) = delete;

    /**
     * @brief 类持有的最近成功调参状态，通知型 var 直接绑定
     */
    struct Parameters
    {
        double exposure_us     = 5000.0;    // 曝光，单位 μs
        double exposure_min_us = 0.0;       // 设备曝光下限，单位 μs
        double exposure_max_us = 0.0;       // 设备曝光上限，单位 μs
        double gain_db         = 0.0;       // 增益，单位 dB
        double gain_min_db     = 0.0;       // 设备增益下限，单位 dB
        double gain_max_db     = 0.0;       // 设备增益上限，单位 dB
        
        std::mutex mutex;                   // 保护成功值和动态范围
    };

    /**
     * @brief 提供通知型 var 绑定状态，禁止绕过 setter 直接修改
     */
    Parameters& GetParameters() noexcept
    {
        return parameters_;
    }

    /**
     * @brief 使用类内最近成功配置重新初始化，调用方须先 Stop()
     */
    void Reopen();

    /**
     * @brief 探测配置的 USB 相机是否已被系统枚举到
     *
     * 只读 /sys/bus/usb/devices，不初始化 MVS 运行时：设备没插时反复
     * MV_CC_Initialize / MV_CC_Finalize 会把 SDK 拖进不稳定状态，
     * 启动等待应当先用它等到设备出现，再调用 Init()。
     *
     * @param config 相机配置，只用其中的 vid_pid
     * @return true 至少有一台 VID:PID 匹配的 USB 设备；探测不可用时也返回 true，交给 SDK 判断
     * @throws std::invalid_argument vid_pid 格式非法
     */
    static bool ProbeUsb(const Config& config);

    void Init(const Config& config);

    bool Read(cv::Mat& image, std::chrono::steady_clock::time_point& timestamp);

    bool Stop() noexcept;

    /**
     * @brief 相机浮点节点范围，与参数使用相同单位
     */
    struct Range
    {
        double minimum;
        double maximum;
    };

    /**
     * @brief 获取曝光范围，单位 ms；须串行调用
     * @return 当前设备范围
     */
    Range GetExposureRange();

    /**
     * @brief 获取增益范围，单位 dB；须串行调用
     * @return 当前设备范围
     */
    Range GetGainRange();

    /**
     * @brief 设置曝光并提交成功状态，仅由相机工作线程调用
     * @param exposure_ms 曝光，单位 ms
     * @throws std::exception 设备未打开、数值越界或 SDK 写入失败
     */
    void SetExposure(double exposure_ms);

    /**
     * @brief 设置增益并提交成功状态，仅由相机工作线程调用
     * @param gain_db 增益，单位 dB
     * @throws std::exception 设备未打开、数值越界或 SDK 写入失败
     */
    void SetGain(double gain_db);

    /**
     * @brief 查询连续采集状态
     *
     * @return true 本实例已经开始采集；false 尚未初始化或已经停止
     */
    bool IsOpen() const noexcept
    {
        return grabbing_;
    }

    /**
     * @brief 返回最近成功读取的设备帧号
     *
     * @return 最近有效帧号，初始化后尚未读取时为 0
     */
    std::uint32_t GetFrameNumber() const noexcept
    {
        return frame_number_;
    }

private:
    Parameters parameters_ {};                  // 参数及范围由相机实例持有
    Config config_ {};                          // 最近一次请求的配置：打开设备前就记下，重连仍可用

    void* handle_               = nullptr;      // MVS 设备句柄
    bool  opened_               = false;        // 设备已打开
    bool  grabbing_             = false;        // 连续采集已开始
    bool  runtime_              = false;        // 本实例持有 SDK 运行时引用

    unsigned int  timeout_ms_   = 100;          // SDK 取帧等待时间，单位 ms
    std::uint32_t frame_number_ = 0;            // 最近有效帧号

    void Configure(const Config& config);
    static void ValidateConfig(const Config& config);
};

} // namespace modules::camera
