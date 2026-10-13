/**
 * @file channel.hpp
 * @author qingyu
 * @brief 进程内数据通道：发布订阅机制，对应子树的 zbus 通道
 * @version 0.1
 * @date 2026-10-13
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <utility>

namespace topic
{

/**
 * @brief 数据通道：单槽最新消息 + 全局序号，多个订阅者各自持游标
 *
 * 发布方覆盖写槽，不做像素/大对象拷贝（消息按值移入）；订阅方各自记录"取到第几条"，
 * 因此多个订阅者互不影响，内存只有槽内那一份。
 * 订阅者只在有新消息或通道关闭时被唤醒，空闲时不产生定时唤醒。
 *
 * @note 锁保护的是槽本身（消息句柄、序号、有效标志、关闭标志）：发布与取帧完全互斥，
 *       读者不会读到写了一半的槽。**槽内消息的内容不在保护范围内**——消息按值浅拷贝、
 *       内容由引用计数共享（如 cv::Mat 的像素），前提是发布方每次新建内容、不原地改写。
 * @note 槽深为 1 是有意的：这里只保存"最新值"，每个订阅者的队列就是它自己的游标。
 *       没有环形缓冲就没有空满判定，也就没有写者套圈读者的问题；代价是订阅者慢就丢最旧。
 *       需要"一条不漏"的订阅者应各自配有界队列，不要靠加深通道解决。
 */
template <typename Message>
class Channel
{
public:
    /**
     * @brief 发布一条消息，覆盖槽内旧消息并唤醒等待中的订阅者
     *
     * 只允许一个发布者（由拥有该数据的线程调用）。
     *
     * @param message 新消息，按值接收后移入槽内
     */
    void Publish(Message message)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            message_ = std::move(message);
            published_ = ++sequence_;
            valid_ = true;
        }
        condition_.notify_all();
    }

    /**
     * @brief 关闭通道并唤醒所有等待者，由停止流程调用
     *
     * 关闭后等待中的 Subscriber::Wait 立即返回 false，据此退出。
     */
    void Close()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            closed_ = true;
        }
        condition_.notify_all();
    }

    /**
     * @brief 释放槽内消息但保留序号，用于断线重连与退出清理
     *
     * 置空有效标志：通用消息类型没有"空值"概念，靠这个标志保证订阅者
     * 不会把清空后的槽当成新消息。
     */
    void Clear()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        message_ = Message{};
        valid_ = false;
    }

    /**
     * @brief 已发布的消息条数
     *
     * @return 发布计数；与槽内序号一致
     */
    std::uint64_t Published() const
    {
        return published_.load();
    }

private:
    template <typename M>
    friend class Subscriber;

    std::mutex mutex_;                              // 保护槽、序号与关闭标志
    std::condition_variable condition_;             // 有新消息或关闭时唤醒订阅者
    Message message_ {};                            // 槽内最新消息，取走后不清空
    std::uint64_t sequence_ = 0;                    // 槽内消息序号，从 1 开始
    std::atomic<std::uint64_t> published_ {0};   // 发布计数，无锁读取
    bool valid_ = false;                            // 槽内有有效消息（Clear 后为 false）
    bool closed_ = false;                           // 通道已关闭，等待者不再挂起
};

/**
 * @brief 通道订阅者：持有自己的游标，同一通道可以有多个
 *
 * 在订阅它的线程里声明（本工程规矩：状态声明在拥有它的线程）。构造只保存通道引用，
 * 不访问通道内容，因此可以安全地在跨翻译单元的静态初始化阶段构造。
 *
 * @note 一个订阅者只允许一条线程使用：游标与计数不加锁，由声明它的那条线程独占。
 *       要两个消费者就声明两个订阅者，不要共用。
 */
template <typename Message>
class Subscriber
{
public:
    /**
     * @brief 绑定通道，游标从 0 开始
     *
     * @param channel 要订阅的通道，须比本对象活得久
     */
    explicit Subscriber(Channel<Message>& channel) : channel_(channel)
    {
    }

    /**
     * @brief 阻塞等待新消息或通道关闭
     *
     * 消费主入口：没有新消息就一直等待，不轮询、不带超时。
     *
     * @note 中途加入的订阅者先用 AlignToLatest() 对齐，否则第一次调用会立刻拿到
     *       槽内已有的消息。
     *
     * @param message 输出新消息
     * @return true 取到新消息；false 通道已关闭且没有可用消息
     */
    bool Wait(Message& message)
    {
        std::unique_lock<std::mutex> lock(channel_.mutex_);
        channel_.condition_.wait(lock, [this]
        {
            return channel_.closed_ || channel_.sequence_ > sequence_;
        });
        return TakeLocked(message);
    }

    /**
     * @brief 阻塞等待新消息或通道关闭，带超时
     *
     * 只给"需要周期性做别的事"的调用方使用（例如主线程要泵 Qt 事件）。
     * 普通消费者用不带超时的 Wait：带周期超时等于把定时唤醒装回消费者身上。
     *
     * @param message 输出新消息
     * @param timeout 最长等待时间
     * @return true 取到新消息；false 超时或通道已关闭
     */
    bool Wait(Message& message, std::chrono::milliseconds timeout)
    {
        std::unique_lock<std::mutex> lock(channel_.mutex_);
        channel_.condition_.wait_for(lock, timeout, [this]
        {
            return channel_.closed_ || channel_.sequence_ > sequence_;
        });
        return TakeLocked(message);
    }

    /**
     * @brief 取最新消息，没有更新的消息时立即返回
     *
     * 只给"本次醒来是因为别的原因"的调用方使用（例如同时等待输入与消息的线程）。
     * 业务消费者用 Wait，写成周期调用就等于把轮询复制到每个订阅者身上。
     *
     * @param message 输出新消息
     * @return true 取到新消息；false 没有新消息
     */
    bool TakeLatest(Message& message)
    {
        std::lock_guard<std::mutex> lock(channel_.mutex_);
        return TakeLocked(message);
    }

    /**
     * @brief 把游标对齐到当前发布计数，跳过已经存在的那条消息
     */
    void AlignToLatest()
    {
        sequence_ = channel_.Published();
    }

    /**
     * @brief 本订阅者取到的消息条数
     */
    std::uint64_t Taken() const
    {
        return taken_;
    }

    /**
     * @brief 本订阅者来不及取、被新消息覆盖掉的条数
     */
    std::uint64_t Dropped() const
    {
        return dropped_;
    }

private:
    /**
     * @brief 持锁取消息：调用方已持有通道互斥锁
     */
    bool TakeLocked(Message& message)
    {
        if (channel_.sequence_ <= sequence_ || !channel_.valid_)
        {
            return false;
        }
        dropped_ += channel_.sequence_ - sequence_ - 1;
        sequence_ = channel_.sequence_;
        message = channel_.message_;
        ++taken_;
        return true;
    }

    Channel<Message>& channel_;        // 被订阅的通道
    std::uint64_t sequence_ = 0;       // 本订阅者已取到的序号
    std::uint64_t taken_ = 0;          // 取到条数
    std::uint64_t dropped_ = 0;        // 被覆盖掉的条数
};

} // namespace topic
