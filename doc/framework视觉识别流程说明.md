# `framework/algorithm/vision` 视觉识别流程说明

日期：2026-10-14。

面向读者：具备较好的 C++ 与 OpenCV 基础，但尚未系统学习目标检测和 YOLO。

本文只解释当前源码真实实现，不把尚未接入的传统全图灯条检测或后续 PnP 写成已经运行的能力；局部角点精修已接入为可选后处理。

## 1. 先说结论：当前系统究竟怎么识别装甲板

当前代码接入的识别主链不是“OpenCV 找两根灯条，再裁剪数字做分类”，而是一个端到端的 `0526` YOLO 模型：模型直接从整张 BGR 图像中预测装甲板四角点、目标置信度、颜色和数字类别。

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
2. **当前主链没有先独立找灯条。** 模型直接回归装甲板四个关键点，这些点在训练语义上对应装甲板左右灯条的上下端点。可选的 `ArmorRefiner` 会在模型粗检后用 OpenCV 局部提取灯条并修正角点；是否调用由视觉 YAML 的 `use_refiner` 决定。

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

### 2.2 可选精修文件

| 文件 | 已实现能力 | 当前状态 |
| --- | --- | --- |
| `core/light_bar.*` | 表示并规范化精修器提取到的单根灯条 | 精修器依赖，运行时启用精修时使用 |
| `core/light_result.hpp` | 保存精修器的灯条提取统计、结果与拒绝原因 | 精修器依赖，运行时启用精修时使用 |
| `refine/armor_refiner.*` | 在 YOLO 粗四点附近重新提取灯条并细化端点 | 已接到模型输出后面，由 YAML 开关控制 |

原先独立的全图 `LightDetector` 和两灯条配对器已移除：当前方案只保留 YOLO 检测及其可选局部精修所需代码。

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

### 3.3 模型如何“学会”数字和灯条

从当前输出契约可以确认，训练样本至少需要为每块装甲板提供与下面四类输出对应的监督信息：

```text
目标是否存在
四个灯条端点
颜色类别
数字/身份类别
```

训练过程可以从工程角度理解为：

```text
输入带标注的图像
→ 网络提取从边缘、纹理到高层目标语义的多级特征
→ 检测头产生四点、objectness、颜色和数字输出
→ 将输出与人工标注计算损失
→ 反向传播修改卷积权重
→ 大量样本反复迭代
```

灯条监督让关键点回归分支学会“装甲板左右发光边缘在哪里”；数字监督让身份分支学会“候选对应哪种数字或特殊目标”。两个任务共享前面的视觉特征，因此数字判断不一定只依赖数字笔画，也可能使用装甲板比例、灯条间距和上下文。

推理时不再修改权重，只执行已经训练好的函数：

```text
output = network(input, fixed_weights)
```

当前仓库拿到的是 ONNX/engine 和运行后处理，不包含完整训练代码、损失函数与训练集。因此本文能够确认输入输出和 C++ 后处理，不能仅凭 engine 断言其 backbone 的每个具体模块、训练增强策略或每项 loss 权重。

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

## 6. 后处理：从大量候选变成可信的 `DetectedArmor`

### 6.1 为什么会出现大量重复候选

YOLO 在不同尺度、不同网格和不同候选位置上同时预测。同一块真实装甲板附近往往有多行都给出较高分。如果把所有候选都画出来，会看到一块装甲板周围重叠着很多框。

当前先对每个通过 0.7 初筛的候选执行：

1. 将四点除以 `scale`，还原到原图坐标；
2. 求四点的 `min_x/max_x/min_y/max_y`；
3. 得到轴对齐外接矩形；
4. 保存四点、外接框、颜色下标、数字下标和 objectness。

### 6.2 NMS 如何去重

NMS 是 Non-Maximum Suppression，非极大值抑制。核心依据是两个外接矩形的 IoU：

```text
IoU = 交集面积 / 并集面积
```

处理思路是：

1. 先取置信度最高的候选；
2. 删除与它 IoU 过大的其他候选；
3. 在剩余候选里继续取最高分；
4. 直到没有候选。

当前代码调用：

```cpp
cv::dnn::NMSBoxes(boxes, confidences, 0.7f, 0.3f, kept_indices);
```

其中 `0.3` 是 IoU 门限。这里用的是四点的**轴对齐外接矩形**，不是旋转四边形 IoU。它实现简单、速度快，但两个倾斜目标挨得很近时可能比真实四边形重叠更严重。

### 6.3 为什么 NMS 后还要继续拒绝

NMS 只解决“重复候选”，不保证候选语义和几何一定有效。保留下来的候选还要依次通过：

```text
最终 objectness > 0.8
→ 颜色只能是红或蓝
→ 数字下标有效
→ 数字与大小类型自洽
→ 四点和外接框合法
```

`DetectedArmor::Create()` 是最后一道不变量边界，检查：

- 四点全部是有限数，不含 NaN/Inf；
- 点序按 TL、TR、BR、BL 构成严格凸四边形；
- 有向面积为正，不退化、不自交；
- 中心和外接框有效，宽高大于零；
- 四个点全部位于原图有效像素范围；
- 颜色不是 `Unknown`，尺寸不是 `Unknown`，标签不是 `NotArmor`；
- 置信度位于 `[0,1]`。

任何一项失败都返回 `std::nullopt`，不会把半合法对象送到下游。当前对越过图像边界的目标采取“整块拒绝”，没有把点强行钳制回边界。

### 6.4 结果对象保存什么

最终 `DetectedArmor` 包含：

```text
corners       TL、TR、BR、BL 四点，原图像素坐标
center        四点平均位置
bounding_box  四点的轴对齐外接矩形
color         Red / Blue
label         One..Five / Sentry / Outpost / Base
size          Small / Large
confidence    sigmoid(objectness)
raw_color_id  模型颜色分支原始下标
raw_label_id  模型数字分支原始下标
```

同时保留 raw ID 的意义是：如果显示结果整体错位，可以判断是模型输出错误，还是 C++ 映射表错误，而不是只看到转换后的枚举。

## 7. 从相机到显示的实时数据流

### 7.1 相机帧

相机线程读取一帧后执行：

```cpp
cv::flip(image, image, -1);
pub_camera_to_.Publish({std::move(image), timestamp});
```

`flip_code=-1` 同时翻转 x、y 两个方向，等价于旋转 180°。因此相机话题里的图像已经回正，显示和模型不需要各自再翻一次。

### 7.2 vision 线程

vision 线程持有自己的 `camera_to::Subscriber`，阻塞等待最新帧：

```text
Wait(camera message)
→ detector_->Detect(message.image)
→ 统计耗时和拒绝原因
→ 发布 vision_to
```

`camera_to` 是单槽最新值通道。假设相机 60 FPS，而识别只能跑 30 FPS：

- 相机不等待识别；
- 中间旧帧会被更新帧覆盖；
- vision 每次完成后拿当时最新的一帧；
- 延迟不会因为队列积压而不断增长。

这对于瞄准系统比“每帧都不能丢”更合理，因为控制更需要新鲜观测。

### 7.3 为什么 `vision_to` 要带原图

识别耗时期间，相机已经发布了更新的帧。如果 display 自己订阅相机，再把最新识别角点画到最新相机帧上，图像和坐标可能来自两个时刻。

因此 `vision_to::Message` 同时携带：

```text
source_image   模型实际处理的原图，cv::Mat 浅引用
timestamp      该相机帧时间戳
armors         该帧检测结果
processing_ms  本次 Detect 总耗时
```

`cv::Mat` 的浅拷贝只增加引用计数，不复制整张图。display 只有在确实存在目标、需要画线时才 `clone()`，然后绘制：

- 红色目标用红线；
- 蓝色目标用蓝线；
- 四条边连接 TL、TR、BR、BL；
- 文本显示颜色、数字/身份和 objectness。

## 8. 如何读运行日志

vision 每秒汇总一次，不逐帧刷日志。主要字段如下：

| 字段 | 含义 | 常见判断 |
| --- | --- | --- |
| `fps` | 实际完成识别的帧率 | 明显低于需求时先看三段耗时 |
| `taken` | vision 从相机话题取到的累计帧数 | 与识别工作量对应 |
| `dropped` | vision 因处理慢错过的旧帧数 | 增长不等于错误，说明吞吐低于相机 |
| `raw=25200` | 每帧固定原始候选行数 | 模型契约常量 |
| `score` | 通过 0.7 初筛的候选数 | 一直为 0 时检查画面、模型和预处理 |
| `nms` | NMS 后候选数 | `score` 很大而 `nms` 很小说明重复候选多 |
| `accepted` | 最终有效装甲板数 | 真正发布给 display 的数量 |
| `reject(conf/color/type/geometry)` | 四级拒绝计数 | 用来定位目标被哪一级过滤 |
| `max_obj` | 窗口内最大 objectness | 有目标但低于 0.7 时不会进入后续 |
| `ms(pre/infer/post/age)` | 平均预处理、推理、后处理和帧龄 | `age` 反映端到端新鲜度 |
| `target` | 一秒内最高置信目标 | 显示颜色/标签/置信度/四点 |

几个典型组合：

```text
max_obj 很低，score=0
→ 模型没有认为画面里存在装甲板；先查曝光、目标距离、预处理和模型域。

score>0，nms>0，accepted=0，reject_color 很高
→ 模型主要判成灰/紫，或颜色顺序与模型不一致。

score>0，reject_type 很高
→ 数字类别与大小映射不自洽，重点核对 raw_label_id。

reject_geometry 很高
→ 四点顺序、越界、非凸或退化；不要先降低置信度。

inference_ms 高，dropped 增长，但 age 不持续增长
→ 系统在主动丢旧帧，仍保持有界延迟，这是当前设计的预期行为。
```

## 9. 当前“数字 + 灯条”应怎样理解

### 9.1 模型主链里的灯条

模型没有输出 `LightBar` 对象，也没有输出灯条轮廓。它输出四个关键点：左灯条上端、左灯条下端、右灯条下端、右灯条上端，后处理将其整理成 TL、TR、BR、BL。

因此模型主链中的“灯条识别”实际是**灯条端点关键点回归**：

```text
输入整图
→ 网络找出装甲板整体
→ 回归左右灯条的四个端点
→ 四点构成装甲板二维几何
```

它不会像 `findContours()` 那样提供每根灯条的轮廓、面积、填充率或 `RotatedRect`。如果四点足够准，后续 PnP 可以直接使用；如果四点偏离真实发光灯条端点，才考虑启用 `ArmorRefiner`。

### 9.2 模型主链里的数字

数字与位置共享网络特征。网络对每个候选同时给出九个身份分数，C++ 取最大值下标。可以把它理解成：

```text
候选区域的深层特征
→ 九路分类输出
→ 哪一路最大，就认为是哪一类
```

与模板匹配不同，网络并不逐像素要求数字完全一致；它从训练数据中学习具有统计意义的笔画、亮暗关系和上下文。这使它能容忍一定旋转、缩放、模糊和曝光变化，但也意味着：训练数据没有覆盖的相机、装甲样式或曝光条件可能造成高置信度误判。

### 9.3 当前没有独立的数字置信度

虽然模型有九路数字输出，当前代码只用它们做 `argmax`，没有保存最大数字分数、第二名分数或二者差值。因此目前能看到：

- 装甲板 objectness；
- 最终数字类别；

但看不到：

- 数字分类本身有多确定；
- 第一名和第二名是否非常接近。

所以“框得很准但数字偶尔跳变”时，现有日志还不能完全区分是数字分支不确定还是映射错误。后续若要提高可观测性，应先保存数字分支最大值和 margin，而不是直接堆数字特判。

补充一点：当前颜色和数字分支直接对原始输出做 `argmax`，没有先做 softmax。只为选择最大下标时这是成立的，因为 softmax 不改变各分量的大小顺序；但不做 softmax就不能把最大原始值直接当作类别概率。

## 10. 精修器内部的 OpenCV 灯条提取

全图传统检测器与装甲板配对器已移除。保留的 `ArmorRefiner` 自己在 YOLO 四点附近裁剪 ROI，再用灰度阈值、外轮廓和 `minAreaRect()` 提取灯条；它不依赖全图 `LightDetector`，也不枚举灯条对来发现目标。详细流程见下一节。

## 11. `ArmorRefiner`：用 OpenCV 灯条细化 YOLO 四点

`ArmorRefiner` 设计为模型检测之后的可选后处理，而不是另一个完整检测器：

```text
YOLO 粗四点
→ 扩大四点附近的搜索 ROI
→ ROI 内灰度阈值 + 外轮廓 + minAreaRect
→ 过滤长度、长宽比和倾角
→ 找离粗左边最近的灯条
→ 找离粗右边最近的灯条
→ 端点总距离通过门限才替换四点
```

精修器只负责：

- 只在已知目标附近的小 ROI 工作；
- 不做形态学闭运算；
- 不检查轮廓填充率；
- 不按敌方颜色筛掉灯条；
- 目标是修正端点，不负责发现整图中的装甲板。

精修失败时不会删除模型目标，而是保持原始四点：

```text
final_corners = coarse_corners
```

只有找到两根灯条且端点距离和小于门限时，状态才是 `Applied`。当 `use_refiner: false` 或精修失败时，显示的是模型原始四点。

## 12. 模型检测与可选精修

| 对比项 | YOLO 检测 | OpenCV 局部精修 |
| --- | --- | --- |
| 输入 | 整张 BGR 图像 | YOLO 粗四点及原图 |
| 职责 | 发现目标，输出四点、颜色和数字 | 在局部 ROI 检查灯条并尝试修正四点 |
| 失败行为 | 后处理过滤无效候选 | 保留 YOLO 原始四点，不删除目标 |
| 当前接入状态 | 已接入 `trd_vision` | 由 `use_refiner` 决定是否运行 |

这两条路线不是必须二选一。合理组合可以是：

```text
YOLO 负责发现目标、颜色和数字
→ ArmorRefiner 只负责把粗四点吸附到真实灯条端点
```

但是否启用精修必须由实际角点误差和 PnP 结果证明，不能因为传统方法“看起来更精确”就默认叠加。

## 13. 建议的源码阅读顺序

按下面顺序读，最容易把 OpenCV 经验接到 YOLO：

1. [`core/detections.hpp`](../framework/algorithm/vision/core/detections.hpp)：先看最终结果长什么样；
2. [`detect/armor_detector.hpp`](../framework/algorithm/vision/detect/armor_detector.hpp)：看输入、输出、固定形状和阈值；
3. [`detect/armor_detector.cpp`](../framework/algorithm/vision/detect/armor_detector.cpp)：只先读 `Preprocess()`、`Infer()`、`Postprocess()`；
4. [`detect/model_decode.cpp`](../framework/algorithm/vision/detect/model_decode.cpp)：看颜色、数字和大小映射；
5. [`core/detections.cpp`](../framework/algorithm/vision/core/detections.cpp)：看最终不变量；
6. [`project/thread/vision/trd_vision.cpp`](../project/thread/vision/trd_vision.cpp)：看算法怎样接入实时线程；
7. [`framework/topic/vision_to/vision_to.hpp`](../framework/topic/vision_to/vision_to.hpp)：看同帧结果契约；
8. [`refine/armor_refiner.cpp`](../framework/algorithm/vision/refine/armor_refiner.cpp)：查看 YOLO 粗四点之后可选的 OpenCV 局部精修。

阅读 `armor_detector.cpp` 时可以先忽略 TensorRT 资源管理细节，把主干压缩成：

```cpp
double scale = Preprocess(image);
Infer();
ArmorDetectionResult result = Postprocess(scale, image.size());
```

先理解数据形状和坐标，再回头看 CUDA 内存、stream、context 和异常清理，会比从构造函数第一行硬啃更清楚。

## 14. 当前边界与下一步知识

当前代码已经描述并连接：

- 相机 BGR 帧回正；
- 最新帧通道；
- 0526 TensorRT 推理；
- 四点、颜色、数字和尺寸解码；
- NMS 与最终几何校验；
- 同帧结果发布与显示叠加；
- 分段耗时和逐级拒绝统计。

当前不包含：

- 相机畸变校正；
- 相机内参与装甲板物理尺寸；
- `solvePnP` 三维位姿；
- 跨帧跟踪、目标选择和预测；
- 独立数字 ROI/数字分类置信度；
- 已启用的灯条角点精修；
- 敌方颜色过滤。

如果下一步要从“画出装甲板”走向“计算目标在相机坐标系的位置”，最需要补的知识不是继续研究 YOLO 网络结构，而是：

```text
相机模型与畸变
→ 世界点/像素点对应
→ 装甲板物理四点模型
→ solvePnP
→ 坐标系变换
→ 跨帧状态估计
```

YOLO 负责给出可靠的二维观测；PnP、跟踪和预测才把二维识别结果变成可用于控制的状态。
