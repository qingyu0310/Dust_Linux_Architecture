/**
 * @file trd_ros2.cpp
 * @author qingyu
 * @brief ROS 2 转发线程 —— ROS 2 与 EtherCAT 双向 PDO 桥接
 * @version 0.1
 * @date 2026-11-18
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "Init_entry.hpp"
#include "log.hpp"
#include "thread.hpp"
#include "framework/topic/ecat_rx/ecat_rx.hpp"

#include <algorithm>
#include <cstdio>
#include <iterator>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

namespace thread::ros2
{

using namespace init;
using namespace topic;

static Thread thread_ {};                                                       		// 转发线程
static Subscriber<ecat_rx::Message> subscriber_ {ecat_rx::channel_}; 			// PDO 订阅者

static std::shared_ptr<rclcpp::Context> context_ {};                                    // 独立 ROS 2 上下文
static rclcpp::Node::SharedPtr node_ {};                                               	// 发布节点
static rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_ {};      // PDO 发布器
static rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr command_subscription_ {};
static std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_ {};
static bool command_size_invalid_ = false;
static uint64_t command_count_ = 0;                                                   // ROS 下行回调次数
static std::chrono::steady_clock::time_point last_command_log_ {};                     // 下行日志限频
static std::chrono::steady_clock::time_point last_check_log_ {};                       // 订阅状态日志限频
static std_msgs::msg::UInt8MultiArray message_ {};                                    	// 复用发送消息

/**
 * @brief 接收外部进程的完整命令 PDO，发布给 EtherCAT 线程
 */
static void OnCommand(const std_msgs::msg::UInt8MultiArray::ConstSharedPtr& message)
{
    ++command_count_;
    ecat_tx::Message command {};
    if (message->data.size() != sizeof(command.data))
    {
        // 错误长度也发布失效标记，立即停止沿用上一条命令。
        if (!command_size_invalid_)
        {
            DUST_LOG_WRN("ros2: /ecat/tx_pdo requires 256 bytes, received=%zu", message->data.size());
        }
        command_size_invalid_ = true;
        ecat_tx::channel_.Publish(command);
        return;
    }

    command_size_invalid_ = false;
    std::copy(message->data.begin(), message->data.end(), std::begin(command.data));
    command.received_at = std::chrono::steady_clock::now();
    ecat_tx::channel_.Publish(command);

    const auto now = std::chrono::steady_clock::now();
    if (last_command_log_ == std::chrono::steady_clock::time_point {} || now - last_command_log_ >= std::chrono::seconds(1))
    {
        last_command_log_ = now;
        char bytes[46] {};
        for (size_t i = 0; i < 15; ++i)
        {
            std::snprintf(bytes + i * 3, sizeof(bytes) - i * 3, "%02X ", static_cast<unsigned>(command.data[i]));
        }
        DUST_LOG_INF("ros2: command received count=%llu len=%zu first15=%s", static_cast<unsigned long long>(command_count_), message->data.size(), bytes);
    }
}

/**
 * @brief 任务单拍：执行 ROS 订阅回调，同时转发 MCU 最新 PDO
 *
 * @param context 未使用
 * @return Continue=本拍完成；Exit=收到停止请求
 */
static Thread::TaskResult Task(void* context)
{
    (void)context;

    if (thread_.StopRequested())
    {
        return Thread::TaskResult::Exit;
    }

    executor_->spin_some(std::chrono::milliseconds(1));

    const auto now = std::chrono::steady_clock::now();
    if (last_check_log_ == std::chrono::steady_clock::time_point {} || now - last_check_log_ >= std::chrono::seconds(5))
    {
        last_check_log_ = now;
        DUST_LOG_INF("ros2: /ecat/tx_pdo matched_publishers=%zu received=%llu", command_subscription_->get_publisher_count(), static_cast<unsigned long long>(command_count_));
    }

    // 带超时等待，MCU 停止上报时仍需继续处理 ROS 2 下行命令。
    ecat_rx::Message received {};
    if (subscriber_.Wait(received, std::chrono::milliseconds(1)) && !thread_.StopRequested())
    {
        std::copy(std::begin(received.data), std::end(received.data), message_.data.begin());
        publisher_->publish(message_);
        DUST_LOG_DBG("ros2", "published=%llu dropped=%llu", static_cast<unsigned long long>(subscriber_.Taken()), static_cast<unsigned long long>(subscriber_.Dropped()));
    }

    return Thread::TaskResult::Continue;
}

/**
 * @brief 发布失败隔离本线程，保留 EtherCAT 和调试台
 *
 * @param error 发布异常
 * @param thread 出故障的线程，本钩子不使用
 * @param context 未使用
 * @return Stop=交给框架隔离处理
 */
static RecoveryAction OnRetry(const std::exception& error, Thread& thread, void* context)
{
    (void)thread;
    (void)context;

    DUST_LOG_ERR("ros2: publish failed: %s", error.what());
    return RecoveryAction::Stop;
}

/**
 * @brief Join 后释放；支持初始化中途失败和重复调用
 */
void Release()
{
    if (executor_ && node_)
    {
        executor_->remove_node(node_);
    }
    executor_.reset();
    command_subscription_.reset();
    publisher_.reset();
    node_.reset();
    if (context_ && context_->is_valid())
    {
        context_->shutdown("dust ros2 release");
    }
    context_.reset();
}

/**
 * @brief 创建独立 Context、节点和发布器，信号仍由原框架管理
 * @return true=初始化成功
 */
bool thread_init()
{
    try
    {
        context_ = std::make_shared<rclcpp::Context>();
        context_->init(0, nullptr);
        rclcpp::NodeOptions options;
        options.context(context_);
        node_ = std::make_shared<rclcpp::Node>("dust_ecat_bridge", options);
		
        publisher_ = node_->create_publisher<std_msgs::msg::UInt8MultiArray>("/ecat/rx_pdo", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile());
        command_subscription_ = node_->create_subscription<std_msgs::msg::UInt8MultiArray>("/ecat/tx_pdo", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(), OnCommand);

        rclcpp::ExecutorOptions executor_options;
        executor_options.context = context_;
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(executor_options);
        executor_->add_node(node_);
        message_.data.resize(256);
		
        DUST_LOG_OK("ros2: publish /ecat/rx_pdo, subscribe /ecat/tx_pdo, 256 bytes, best effort");

        return true;
    }
    catch (const std::exception& error)
    {
        DUST_LOG_ERR("ros2: init failed: %s", error.what());
        Release();
        return false;
    }
}

/**
 * @brief 启动转发线程；初始化失败时跳过，不影响其他业务
 * @return true=启动完成或已跳过
 */
bool thread_start()
{
    // 初始化失败时跳过转发，不能使框架停止其他业务线程。
    if (!publisher_)
    {
        DUST_LOG_WRN("ros2: forwarding disabled after init failure");
        return true;
    }

    thread_.Start(Task, DUST_MSEC(0), nullptr, "ros2", ThreadFaultConfig{OnRetry, nullptr, ThreadFaultPolicy::Isolate, false});

    return true;
}

/**
 * @brief 通知转发线程停止，关闭通道以唤醒订阅等待
 */
void thread_stop()
{
    thread_.RequestStop();
    ecat_rx::channel_.Close();
}

/**
 * @brief 等待转发线程退出，资源由 Release 统一释放
 */
void thread_join()
{
    thread_.Join();
}

REGISTER_INIT(thread_init, LateInit, Mid, ContinueOnFail, Release, "ros2_init");
REGISTER_THREAD(thread_start, LateThread, thread_stop, thread_join, "ros2_start");

} // namespace thread::ros2
