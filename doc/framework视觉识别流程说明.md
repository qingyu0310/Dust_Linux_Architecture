# `framework/algorithm/vision` 视觉识别流程说明

日期：2026-10-14。

面向读者：具备较好的 C++ 与 OpenCV 基础，但尚未系统学习目标检测和 YOLO。

本文只解释当前源码真实实现，不把尚未接入的传统灯条检测、角点精修或后续 PnP 写成已经运行的能力。

## 1. 先说结论：当前系统究竟怎么识别装甲板

当前实际运行的识别主链不是“OpenCV 找两根灯条，再裁剪数字做分类”，而是一个端到端的 `0526` YOLO 模型：模型直接从整张 BGR 图像中预测装甲板四角点、目标置信度、颜色和数字类别。

```text
海康相机读取 BGR 图像
→ 相机线程旋转 180° 回正
→ camera_to 单槽话题发布最新帧
→ trd_vision 取最新帧
→ ArmorDetector 做 640×640 预处理
→ TensorRT 执行 0526 模型
→ 解析 25200 个候选，每个候选 22 个输出值
→ objectness 门限
→ 四角点还原到原图坐标
→ NMS 去除重复框
→ 显式解码颜色、数字和大小类型
→ DetectedArmor 做最终几何与类型校验
→ vision_to 发布“同一张原图 + 识别结果”
→ display 绘制四边形、颜色、数字和置信度
```

这里最容易误解的两点是：

1. **数字不是 OpenCV OCR 识别出来的。** `0526` 模型内部已经同时学习了装甲板定位与数字类别，后处理只是在模型九路类别输出中取最大值并映射成工程枚举。
2. **当前主链没有先独立找灯条。** 模型直接回归装甲板四个关键点，这些点在训练语义上对应装甲板左右灯条的上下端点。`light_detector.*`、`armor_board.*` 和 `armor_refiner.*` 是另一条传统检测/精修能力，目前没有接入 `trd_vision.cpp` 的主链。

因此，当前系统“识别数字 + 灯条”的准确说法是：**YOLO 用整图特征同时判断装甲板位置、四个灯条端点、颜色和数字；OpenCV 负责图像容器、缩放、坐标变换、NMS 和显示，不负责单独读取数字字符。**

## 2. 目录分层与当前调用关系

`framework/algorithm/vision` 分成三层：

```text
framework/algorithm/vision/
├── core/       结果契约和纯几何类型，不负责模型推理
├── detect/     从图像产生检测结果
└── refine/     已有粗四点之后，可选地用灯条细化角点
```

### 2.1 当前主链真正调用的文件

```text
detect/armor_detector.hpp/.cpp
    ↓ 使用
detect/model_decode.hpp/.cpp
    ↓ 生成
core/detections.hpp/.cpp
    ↓ 依赖公共枚举和几何检查
core/armor_types.hpp
core/geometry_util.hpp
```

| 文件 | 当前职责 |
| --- | --- |
| `detect/armor_detector.*` | TensorRT engine 生命周期、OpenCV 预处理、GPU 推理、22 列输出解码和 NMS |
| `detect/model_decode.*` | 把模型原始颜色/数字下标转换成稳定的工程枚举，并检查数字与大小装甲是否自洽 |
| `core/detections.*` | 定义最终 `DetectedArmor`，校验四点、边界、颜色、尺寸和置信度 |
| `core/armor_types.hpp` | 定义 `TeamColor`、`ArmorSize`、`ArmorLabel` |
| `core/geometry_util.hpp` | 有限数、叉积、距离、四边形面积和凸性判断 |

线程层只负责组织调用：

```cpp
ArmorDetectionResult result = detector_->Detect(message.image);
topic::vision_to::pub_vision_to_.Publish(
    {message.image, message.timestamp, std::move(result.armors), processing_ms});
```

也就是说，算法库不知道相机、线程、窗口和话题；`trd_vision.cpp` 不知道 TensorRT 输出的每一列如何解析。两层通过 `ArmorDetector::Detect()` 和 `ArmorDetectionResult` 连接。

### 2.2 当前没有接入主链的文件

| 文件 | 已实现能力 | 当前状态 |
| --- | --- | --- |
| `detect/light_detector.*` | 用 OpenCV 从整图提取灯条 | 已编译，但 `trd_vision.cpp` 没有调用 |
| `core/light_bar.*` | 规范化单根灯条的上下端点、长度、宽度和倾角 | 只服务于传统灯条路线/精修 |
| `core/light_result.hpp` | 灯条结果、拒绝原因和统计 | 只服务于传统灯条路线/精修 |
| `core/armor_board.*` | 将两根灯条配成装甲板并计算几何代价 | 当前没有调用者 |
| `refine/armor_refiner.*` | 在 YOLO 粗四点附近重新寻找两根灯条以细化端点 | 已实现，但当前没有接到模型输出后面 |

因此不能因为这些文件存在，就认为运行时同时执行了两套算法。

## 3. 用 OpenCV 视角理解 YOLO

### 3.1 传统 OpenCV 与神经网络的差别

传统 OpenCV 通常由程序员明确规定特征：

```text
灰度大于 150
→ 找轮廓
→ 长宽比在某范围
→ 两根灯条高度接近
→ 中间区域可能是装甲板
```

YOLO 的规则不写在 `if` 里，而是存放在网络权重中。训练时给网络大量图像和标注，优化器不断调整卷积核参数，使输出接近标注。运行时 C++ 只做三件事：

1. 把图像变成模型要求的张量；
2. 执行训练好的计算图；
3. 把输出张量翻译成工程对象。

可以把它类比为：

| OpenCV 概念 | 当前模型中的对应物 |
| --- | --- |
| 手写卷积核/滤波规则 | 训练得到的大量卷积核权重 |
| `cv::Mat` 图像 | 输入张量 `[1,3,640,640]` |
| 轮廓候选集合 | 25200 行模型候选 |
| 手写阈值筛选 | objectness 阈值、类别 `argmax`、NMS |
| `RotatedRect` 或四点 | 模型直接回归的四个关键点 |

### 3.2 为什么仍然叫 YOLO

YOLO 的核心思想是“一次前向传播同时预测整张图中的目标”。它不先裁出每个装甲板，再逐个运行分类器，而是在多尺度特征图上一次产生大量候选。

当前固定输出有 25200 行。对于 640×640 输入，这正好符合常见 YOLOv5 三个尺度、每格三个候选的数量：

```text
3 × (80×80 + 40×40 + 20×20) = 25200
```

但当前 TensorRT engine 已经把候选坐标解码到 640×640 画布坐标，C++ 后处理不再手工展开 anchor 公式，只读取每行的四点和分类分支。

这个模型又不是原始的通用 YOLO 输出。普通 YOLO 常输出矩形中心、宽高和类别；`0526` 为装甲板任务增加了四个关键点，并把颜色和数字拆成两个分类分支。

## 4. 输入图像如何变成模型张量

入口要求是非空的 `CV_8UC3` BGR 图像。当前相机线程先对原始帧执行 180° 回正，再通过 `camera_to` 发布；模型看到的已经是方向正常的图像。

### 4.1 等比缩放，不拉伸

模型输入固定为 640×640。缩放系数是：

```text
scale = min(640 / 原图宽度, 640 / 原图高度)
```

例如原图为 1440×1080：

```text
scale = min(640/1440, 640/1080) = 0.4444...
缩放后大小 = 640×480
```

缩放图被贴到 640×640 黑色画布的左上角，下面剩余 160 行保持黑色。当前实现不是居中 letterbox，因此后处理只需要除以 `scale`，不需要再减去左右或上下 padding 偏移。

对应的 OpenCV 操作是：

```cpp
scratch_.canvas.setTo(cv::Scalar(0, 0, 0));
cv::resize(bgr_image, scratch_.canvas(cv::Rect(0, 0, resized_width, resized_height)), ...);
```

### 4.2 BGR 转 RGB、归一化和 HWC 转 NCHW

相机给出的内存排列可理解为：

```text
H × W × C，通道顺序 B、G、R，每个通道是 0..255 的 uint8
```

模型要求：

```text
N × C × H × W = 1 × 3 × 640 × 640
通道顺序 R、G、B
数值范围 0..1
```

当前使用：

```cpp
cv::dnn::blobFromImage(canvas, chw, 1.0 / 255.0, cv::Size(), cv::Scalar(), true, false, CV_32F);
```

这里：

- `1.0 / 255.0` 完成归一化；
- `swapRB=true` 把 BGR 换成 RGB；
- 输出是连续的 NCHW `CV_32F` 张量；
- 如果 engine 输入是 FP16，再将 FP32 张量转换成 FP16；如果 engine 输入是 FP32，直接复制。

### 4.3 TensorRT 在这里做什么

`.engine` 是 ONNX 模型针对当前 TensorRT/CUDA 环境编译后的计算图和权重，不需要把模型手工转换成 C++ 数组。

`ArmorDetector` 构造时只做一次：

1. 读取整个 engine；
2. 创建 TensorRT runtime、engine 和唯一 execution context；
3. 校验输入必须是 `[1,3,640,640]`，输出必须是 `[1,25200,22]` 或 `[25200,22]`；
4. 创建一条 CUDA stream；
5. 一次性分配主机锁页内存和 GPU 输入/输出内存。

每帧只做：

```text
主机输入 → cudaMemcpyAsync → GPU 输入
→ enqueueV3 执行网络
→ cudaMemcpyAsync → 主机输出
→ cudaStreamSynchronize 等待完成
```

当前是同步单 context 设计。它和单槽最新帧话题配合：推理慢于相机时，vision 下一次只拿最新图像，旧帧被覆盖，不形成不断增长的延迟队列。

## 5. 25200×22 输出到底表示什么

每一行都是一个候选，22 个 `float` 分成四组：

| 列范围 | 数量 | 含义 |
| --- | ---: | --- |
| `0..7` | 8 | 四个关键点的 `(x,y)`，坐标位于 640×640 输入画布 |
| `8` | 1 | objectness logit，即“这里存在一块装甲板”的原始分数 |
| `9..12` | 4 | 颜色分支：蓝、红、熄灭/灰、紫 |
| `13..21` | 9 | 数字/身份分支 |

### 5.1 四点顺序

按当前执行代码，模型原始八列与最终工程角点的关系是：

| 原始列 | 模型原始点 | 写入 `DetectedArmorGeometry::corners` |
| --- | --- | --- |
| `0,1` | 左上 | `corners[0]` 左上 TL |
| `2,3` | 左下 | `corners[3]` 左下 BL |
| `4,5` | 右下 | `corners[2]` 右下 BR |
| `6,7` | 右上 | `corners[1]` 右上 TR |

所以代码按 `0 → 6 → 4 → 2` 重排成统一的：

```text
左上 TL → 右上 TR → 右下 BR → 左下 BL
```

这个顺序直接决定后面的四边形凸性、面积、绘制和未来 PnP 点序，不能随意交换。

因为预处理只有缩放、没有居中偏移，坐标还原为：

```text
原图坐标 = 模型输出坐标 / scale
```

### 5.2 objectness：模型认为这里是不是装甲板

第 8 列是 logit，不是直接的 0..1 概率。代码先做 sigmoid：

```text
confidence = 1 / (1 + exp(-row[8]))
```

当前有两道门：

- `confidence < 0.7`：不进入 NMS；
- NMS 后 `confidence <= 0.8`：不形成最终结果。

当前 `DetectedArmor::confidence` 只保存这个 objectness，没有再乘颜色分支或数字分支的分数。这一点很重要：显示的 `0.92` 表示模型很确信“这里是装甲板”，不等于数字类别本身有 92% 的独立概率。

### 5.3 颜色如何识别

代码对 `row[9..12]` 做 `argmax`，即取四个数中最大的下标：

| 原始下标 | 模型颜色 | 当前处理 |
| ---: | --- | --- |
| 0 | 蓝 | 接受，映射为 `TeamColor::Blue` |
| 1 | 红 | 接受，映射为 `TeamColor::Red` |
| 2 | 熄灭/灰 | 拒绝 |
| 3 | 紫 | 拒绝 |

当前没有按敌方颜色过滤，红蓝都会发布；灰色和紫色候选计入 `rejected_color`。

`argmax` 本身没有“识别颜色”的能力。真正区分颜色的是网络训练得到的权重，C++ 只读取哪个输出最大。

### 5.4 数字和身份如何识别

代码对 `row[13..21]` 做 `argmax`，得到 `raw_label_id`：

| `raw_label_id` | 工程标签 | 模型尺寸映射 | 最终规则 |
| ---: | --- | --- | --- |
| 0 | 哨兵 `Sentry` | 小 | 接受 |
| 1 | 1 号 `One` | 大 | 接受 |
| 2 | 2 号 `Two` | 小 | 接受 |
| 3 | 3 号 `Three` | 小 | 接受 |
| 4 | 4 号 `Four` | 小 | 接受 |
| 5 | 5 号 `Five` | 小 | 接受 |
| 6 | 前哨站 `Outpost` | 小 | 接受 |
| 7 | 基地 `Base` | 小 | 被大小规则拒绝 |
| 8 | 基地 `Base` | 大 | 接受 |

这里没有执行以下传统流程：

```text
裁剪数字 ROI → 灰度化 → 二值化 → OCR/分类网络
```

数字是 `0526` 网络在整图特征中直接分类的。可以把网络理解成共享一个图像特征提取器，然后同时接出：

```text
四点回归头 + 目标存在性分支 + 颜色分类分支 + 数字分类分支
```

网络通常会综合数字纹理、灯条布局和装甲板整体外观。仅从运行时 C++ 无法断言它只看了数字区域；哪些像素最影响分类由训练数据和网络权重决定。

当前没有使用 `tiny_resnet.onnx`，也没有第二个数字分类器。数字错了时，首先要检查模型、训练域、预处理和类别顺序，而不是去调 `findContours()`。
