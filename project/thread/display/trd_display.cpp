/**
 * @file trd_display.cpp
 * @author qingyu
 * @brief 显示线程 — 订阅相机话题，在主线程回调里显示最新帧
 * @version 0.1
 * @date 2026-10-13
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "framework/cmd/config/config.hpp"
#include "framework/cmd/shell/var.hpp"
#include "framework/init/Init_entry.hpp"
#include "framework/init/System_startup.hpp"
#include "framework/topic/camera_to/camera_to.hpp"
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <opencv2/highgui.hpp>
#include <stdexcept>

namespace thread::display
{

using namespace topic::camera_to;

static Subscriber sub_camera_to {pub_camera_to_};   // 本线程的帧订阅者，只由主线程回调使用
static bool window_open_ = false;                            // 窗口状态，仅主线程访问
static int  wait_ms_ = 50;                                   // 无新帧时回调最长等待，只为泵 Qt 事件

static bool show_ = true;                                    // 显示开关：true 显示，false 不显示
static std::mutex show_mutex_;                               // 保护显示开关，并作为条件变量的锁
static std::condition_variable show_condition_;              // 开关变化时唤醒主线程回调

static constexpr char kWindowName[] = "Camera";

/**
 * @brief 读取显示开关，主线程回调使用
 *
 * @return true 需要显示
 */
static bool ShowWanted()
{
    std::lock_guard<std::mutex> lock(show_mutex_);
    return show_;
}

/**
 * @brief var 设置回调，由 shell 线程调用
 *
 * 只改开关并唤醒主线程：窗口的创建与销毁必须落在主线程（Qt 约束）。
 * 开关没有硬约束范围，所以用不带范围的 REGISTER_SHELL_VAR_NOTIFY 注册。
 *
 * @param entry 显示开关条目
 * @param value 新值，非 0 表示显示
 * @param context 被绑定的开关变量
 * @throws std::logic_error 上下文不是本模块的开关
 */
static void SetShowFromVar(const debug::VarEntry& entry, double value, void* context)
{
    (void)entry; // 回调只用 value
    if (context != &show_)
    {
        throw std::logic_error("invalid display context");
    }
    {
        std::lock_guard<std::mutex> lock(show_mutex_);
        show_ = value != 0.0;
    }
    show_condition_.notify_all();
}

/**
 * @brief 主线程回调：等新帧、显示、处理窗口事件
 *
 * 创建、显示、按键处理和销毁均在主线程执行，满足 Qt GUI 线程约束。
 * 显示关闭时销毁窗口并阻塞等待重新打开，不做像素处理。
 */
static void Display()
{
    if (!ShowWanted())
    {
        if (window_open_)
        {
            window_open_ = false;
            cv::destroyWindow(kWindowName);
            cv::waitKey(1);
        }
        // 等待重新打开；带超时只为及时响应停止请求（回调跑在主线程，卡住会拖住关闭流程）
        std::unique_lock<std::mutex> lock(show_mutex_);
        show_condition_.wait_for(lock, std::chrono::milliseconds(wait_ms_), [] { return show_; });

        return;
    }

    Message message;
    // 等新帧：轮询取帧会把主循环变成固定节拍，超时只用于泵 Qt 事件
    if (sub_camera_to.Wait(message, std::chrono::milliseconds(wait_ms_)))
    {
        if (!window_open_)
        {
            cv::namedWindow(kWindowName, cv::WINDOW_NORMAL);
            window_open_ = true;
        }
        cv::imshow(kWindowName, message.image);
    }

    if (window_open_)
    {
        const int key = cv::waitKey(1);
        if (key == 27 || key == 'q' || key == 'Q' ||
            cv::getWindowProperty(kWindowName, cv::WND_PROP_VISIBLE) < 1.0)
        {
            System_RequestStop();
        }
    }
}

/**
 * @brief 读取显示参数，注册主线程回调
 *
 * 显示跑在主线程回调里而不是独立线程（Qt 约束），所以本模块不注册 REGISTER_THREAD。
 * 主线程回调槽只有一个，这里注册即表示由本模块持有；回调常驻，显示开关由 var 控制。
 *
 * @return true 初始化成功
 * @throws std::exception 配置读取失败
 */
bool display_init()
{
    cmd::config::Config yaml;
    yaml.Load("thread/display/trd_display.yaml", {"show_window", "wait_ms"});

    show_    = yaml.GetBool("show_window"); // YAML 给启动初值，运行期由 display.show 接管
    wait_ms_ = yaml.GetInt("wait_ms", 1, 1000);

    System_SetMainLoop(Display);
    return true;
}

/**
 * @brief 注销主线程回调并销毁窗口，支持重复清理
 */
void Release()
{
    System_SetMainLoop(nullptr);

    if (window_open_)
    {
        window_open_ = false; // 窗口的创建和销毁都归主线程，这里由 Release 回调执行
        cv::destroyWindow(kWindowName);
        cv::waitKey(1);
    }
}

REGISTER_INIT(display_init, AppInit, High, HaltOnFail, Release, "display_init");

/**
 * @brief 显示开关：非 0 显示、0 不显示；无范围约束，由 var 机制提交新值
 */
REGISTER_SHELL_VAR_NOTIFY("display.show", show_, show_mutex_, SetShowFromVar, &show_);

} // namespace thread::display
