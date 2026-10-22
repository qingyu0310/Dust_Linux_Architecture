/**
 * @file trd_display.cpp
 * @author qingyu
 * @brief 显示线程 — 订阅上游图像流，在主线程回调里原样显示
 * @version 0.2
 * @date 2026-10-22
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "trd_display.hpp"
#include "framework/cmd/config/config.hpp"
#include "framework/init/startup/Init_entry.hpp"
#include "framework/init/startup/System_startup.hpp"
#include "framework/topic/vision_to/vision_to.hpp"
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

namespace thread::display
{

static constexpr char kWindowName[] = "Camera";

// 只订 vision_to：识别和标定都往这条通道发叠加好的帧，显示侧不做像素处理。
static topic::vision_to::Subscriber sub_vision_to {topic::vision_to::pub_vision_to_};

namespace
{

/**
 * @brief 从 vision_to 取一帧识别或标定发布的结果
 *
 * 本线程不生成也不改图像，叠加内容由发布方画好，显示只做转发。
 *
 * @param image 输出取到的图像
 * @return true 取到一帧；false 本次没有新帧
 */
bool TakeFrame(cv::Mat& image)
{
    topic::vision_to::Message vision;
    if (!sub_vision_to.Wait(vision, std::chrono::milliseconds(1)))
    {
        return false;
    }

    image = vision.source_image;
    return true;
}

} // namespace

/**
 * @brief 读取显示开关，主线程回调使用
 *
 * @return true 需要显示
 */
static bool ShowWanted()
{
    std::lock_guard<std::mutex> lock(show_.mutex);
    return show_.show;
}

/**
 * @brief 主线程回调：等上游图像、原样显示并处理窗口事件
 *
 * 创建、显示、按键处理和销毁均在主线程执行，满足 Qt GUI 线程约束。
 * 显示关闭时销毁窗口并阻塞等待重新打开，不做像素处理。
 */
static void Display()
{
    if (!ShowWanted())
    {
        if (window_.open)
        {
            window_.open = false;
            cv::destroyWindow(kWindowName);
            cv::waitKey(1);
        }
        // 等待重新打开；带超时只为及时响应停止请求（回调跑在主线程，卡住会拖住关闭流程）
        std::unique_lock<std::mutex> lock(show_.mutex);
        show_.condition.wait_for(lock, std::chrono::milliseconds(window_.wait_ms), [] { return show_.show; });

        return;
    }

    cv::Mat image;
    if (TakeFrame(image))
    {
        if (!window_.open)
        {
            cv::namedWindow(kWindowName, cv::WINDOW_NORMAL);
            window_.open = true;
        }

        // 原样显示：叠加内容由发布方画好，显示侧不做像素处理
        cv::imshow(kWindowName, image);
    }

    if (window_.open)
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
static bool display_init()
{
    cmd::config::Config yaml;
    yaml.Load("thread/display/trd_display.yaml", {"show_window", "wait_ms"});

    show_.show      = yaml.GetBool("show_window"); // YAML 给启动初值，运行期由 display.show 接管
    window_.wait_ms = yaml.GetInt("wait_ms", 1, 1000);

    System_SetMainLoop(Display);
    return true;
}

/**
 * @brief 注销主线程回调并销毁窗口，支持重复清理
 */
static void Release()
{
    System_SetMainLoop(nullptr);

    if (window_.open)
    {
        window_.open = false; // 窗口的创建和销毁都归主线程，这里由 Release 回调执行
        cv::destroyWindow(kWindowName);
        cv::waitKey(1);
    }
}

REGISTER_INIT(display_init, AppInit, High, HaltOnFail, Release, "display_init");

} // namespace thread::display
