/**
 * @file trd_vision.cpp
 * @author qingyu
 * @brief 识别线程 — 订阅最新相机帧、执行 0526 TensorRT 检测并发布结果
 * @version 0.3
 * @date 2026-10-22
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "trd_vision.hpp"
#include "framework/cmd/shell/log/log.hpp"
#include "framework/init/startup/Init_entry.hpp"
#include "framework/topic/vision_to/vision_to.hpp"
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>

namespace thread::vision
{

using namespace algorithm::vision;
using namespace topic::camera_to;
using namespace init;

/**
 * @brief 取最新帧同步推理；慢于相机时沿用单槽通道的丢旧帧语义
 *
 * @param context 本线程的统计状态
 * @return Continue=继续下一拍；通道关闭为 Fatal
 */
static Thread::TaskResult Task(void* context)
{
    auto& state = *static_cast<VisionState*>(context);

    if (!runtime_.detector) {
        throw std::runtime_error("vision detector is not initialized");
    }

    // 阻塞等待相机最新帧；相机通道关闭时退出识别线程
    Message message;
    if (!runtime_.camera_to.Wait(message)) {
        return Thread::TaskResult::Fatal;
    }

    // 对当前相机帧同步执行预处理、TensorRT 推理和后处理
    const auto processing_begin = std::chrono::steady_clock::now();
    ArmorDetectionResult result = runtime_.detector->Detect(message.image);

    #ifdef CONFIG_DUST_VISION_YOLO_REFINER
    {
        if (runtime_.refiner)
        {
            for (DetectedArmor& armor : result.armors)
            {
                const ArmorRefinementResult refinement = runtime_.refiner->Refine(message.image, armor.GetCorners());
                if (!refinement.Applied())
                {
                    continue;
                }

                const std::optional<DetectedArmor> refined = armor.WithCorners(refinement.final_corners, message.image.size());
                if (refined)
                {
                    armor = *refined;
                }
            }
        }
    }
    #endif

    const auto   now = std::chrono::steady_clock::now();
    const double processing_ms         = std::chrono::duration<double, std::milli>(now - processing_begin).count();
    const double age_ms                = std::chrono::duration<double, std::milli>(now - message.timestamp).count();
    state.report.Add(result, age_ms);

    // 位姿在这一帧内解完并直接描在发布帧上；原图不动，显示侧照旧只转发
    cv::Mat overlay = DrawPoses(message.image, result.armors, state.report);
    topic::vision_to::pub_vision_to_.Publish({overlay, message.timestamp, std::move(result.armors), processing_ms});

    // 日志每秒汇总一次，避免逐帧输出影响实时识别
    const double elapsed = std::chrono::duration<double>(now - state.last_report).count();
    if (elapsed >= 1.0)
    {
        LogReport(state.report, elapsed);
        state.report.Clear();
        state.last_report = now;
    }

    return Thread::TaskResult::Continue;
}

/**
 * @brief 一次加载 engine、校验固定张量并分配 CUDA/TensorRT 资源
 * @return true 检测器创建成功
 * @throws std::exception engine 读取、TensorRT 校验或 CUDA 资源分配失败
 */
static bool thread_init()
{
    ArmorDetector::Config detector_config;
    LoadVisionConfig(detector_config);
    runtime_.detector = std::make_unique<ArmorDetector>(detector_config);

#ifdef CONFIG_DUST_VISION_YOLO_REFINER
    ArmorRefiner::Config refiner_config;
    LoadRefinerConfig(refiner_config);

    runtime_.refiner = std::make_unique<ArmorRefiner>(refiner_config);
    const char* const refiner_state = "on";
#else
    const char* const refiner_state = "off";
#endif

    // 多目标跟踪：槽位与编号的绑定在 multi_target.hpp 的 kTargetLabels，参数已由 YAML 读入
    runtime_.multi = std::make_unique<MultiTargetTracker>(config_.tracker);

    DUST_LOG_INF("[vision] TensorRT device=%d engine=%s score=%.3f confidence=%.3f nms=%.3f refiner=%s input=%s/%s [1,3,%d,%d] output=%s FP32 [1,%d,%d]", detector_config.cuda_device, detector_config.engine_path.c_str(), detector_config.score_threshold, detector_config.confidence_threshold, detector_config.nms_threshold, refiner_state, runtime_.detector->GetInputName().c_str(), runtime_.detector->GetInputTypeName().c_str(), runtime_.detector->GetInputHeight(), runtime_.detector->GetInputWidth(), runtime_.detector->GetOutputName().c_str(), runtime_.detector->GetOutputRows(), runtime_.detector->GetOutputColumns());
    return true;
}

/**
 * @brief 启动识别工作线程
 * @return true 线程创建成功
 */
static bool thread_start()
{
    vision_state_.last_report = std::chrono::steady_clock::now();
    thread_.Start(Task, std::chrono::milliseconds(0), &vision_state_, "vision");
    return true;
}

/**
 * @brief 请求识别线程停止并关闭结果通道
 *
 * 关闭结果通道用于唤醒正在等待识别结果的显示侧；识别线程仍由 camera_to 关闭或
 * 当前 Detect 返回后观察停止标志退出。
 */
static void thread_stop()
{
    thread_.RequestStop();
    topic::vision_to::pub_vision_to_.Close();
}

/**
 * @brief 等待识别线程退出，之后才允许释放 TensorRT/CUDA 资源
 */
static void thread_join()
{
    thread_.Join();
}

/**
 * @brief 在线程退出并 join 后释放 TensorRT/CUDA 资源
 */
static void Release()
{
#ifdef CONFIG_DUST_VISION_YOLO_REFINER
{
    runtime_.refiner.reset();
}
#endif
    runtime_.multi.reset();
    runtime_.last_frame_time.reset();
    runtime_.detector.reset();
    topic::vision_to::pub_vision_to_.Clear();
}

REGISTER_INIT(thread_init, AppInit, High, HaltOnFail, Release, "vision_init");
REGISTER_THREAD(thread_start, AppThread, thread_stop, thread_join, "vision_start");

} // namespace thread::vision
