# HikRobot 相机模块与官方 SDK 接口说明

本文对应当前 `framework/modules/camera/hikrobot.hpp` 和 `hikrobot.cpp`，说明每个函数的职责、官方 SDK 调用以及图像资源的生命周期。

## 1. 模块在当前架构中的位置

- `framework/modules/camera/hikrobot.hpp`：相机配置和公共接口。
- `framework/modules/camera/hikrobot.cpp`：USB 相机枚举、参数设置、同步取帧、像素转换和资源释放。
- `framework/modules/CMakeLists.txt`：模块编译和海康 SDK 链接。
- `project/thread/camera/trd_camera.cpp`：组织采集线程、缓存最新图像和窗口显示。
- `project/thread/camera/trd_camera.yaml`：项目使用的曝光、增益、帧率和超时配置。

`HikRobot` 本身不创建线程，也不读取 YAML。调用方提供 `Config`，通过 `Init()` 启动设备，通过 `Read()` 获取图像，最后调用 `Stop()`。

当前项目采集线程负责取帧，主线程负责 OpenCV 窗口操作。停止时先结束并等待采集线程，再释放相机。

## 2. 配置与内部状态

### Config 配置

| 字段 | 默认值 | 含义 |
| --- | --- | --- |
| `exposure_ms` | `5.0` | 单帧曝光时间，单位 ms；写入 SDK 前乘以 1000，转换为 μs |
| `gain` | `0.0` | 相机增益，单位 dB；增大通常提高亮度，也放大噪声 |
| `frame_rate` | `30.0` | 请求采集帧率，单位 Hz；实际帧率受曝光、设备能力和带宽影响 |
| `vid_pid` | `2bdf:0001` | USB 厂商 ID 与产品 ID，十六进制；不是设备节点编号 |
| `serial_number` | 空字符串 | 指定相机序列号；为空时必须只有一个匹配设备 |
| `timeout_ms` | `100` | 单次 SDK 等待图像的最长时间，单位 ms，允许范围 1～1000 |

当前项目 YAML 的 `exposure_us: 3000` 会先转换为模块的 `3.0 ms`，随后模块写入 SDK 的 `ExposureTime = 3000 μs`。因此当前项目曝光是 3 ms，头文件的 5 ms 是未覆盖配置时的默认值。

曝光时间是传感器为一帧积累光线的时间，不是图像在窗口停留的时间。光圈控制进入镜头的光量，曝光控制积累多久；当前代码不控制镜头的机械光圈。

### 内部成员

| 成员 | 用途 |
| --- | --- |
| `handle_` | SDK 创建的设备句柄，`void*` 是不透明对象标识，不是 USB 文件描述符 |
| `opened_` | 记录设备是否已成功打开 |
| `grabbing_` | 记录是否已成功开始采集 |
| `runtime_` | 记录本对象是否持有 SDK 运行时引用 |
| `timeout_ms_` | 保存初始化配置中的取帧等待时间 |
| `frame_number_` | 保存最近一次成功读取的设备帧号 |

这些状态用于初始化失败后的回滚和正常退出时的分步清理。

## 3. 文件内部的辅助函数

### Check(int code, const char* operation)

统一检查 SDK 返回值。返回 `MV_OK` 时继续执行，否则把操作名称和十六进制错误码组合成 `std::runtime_error`，例如 `ExposureTime failed: 0x80000004`。

SDK 错误现使用 `HikRobotError`，保留错误码并标记是否可恢复。它负责抛异常，不负责输出日志。初始化阶段由 `Init()` 清理资源后重新抛出；运行阶段由调用方处理异常。

### AcquireRuntime()

获取一次 SDK 运行时引用。使用 `runtime_mutex` 保护共享计数 `runtime_users`：第一个对象调用 `MV_CC_Initialize()`，初始化成功后增加计数；后续对象只增加计数。

这样多个相机对象共享 SDK 初始化。初始化失败时计数不会增加。

### ReleaseRuntime()

释放一次运行时引用。先减少 `runtime_users`，仍有其他对象时直接返回成功；最后一个对象释放时调用 `MV_CC_Finalize()`。

调用方必须已经持有引用。这里的互斥锁只保护运行时引用计数，不会把整个相机模块变成可并发调用的接口。

### ParseId(const std::string& value)

将一段 USB ID 字符串按十六进制转换为整数。允许 1～4 位 `0-9`、`a-f`、`A-F`，不允许空串、`0x` 前缀或其他字符。

`vid_pid` 先按冒号拆开，再分别解析厂商 ID 和产品 ID。相同型号的多台相机可能具有相同 VID/PID，因此需要序列号进一步区分。

### BufferGuard::~BufferGuard()

为 SDK 返回的原始图像缓存提供作用域保护。保存设备句柄和 `MV_FRAME_OUT` 指针；只要指针未清空，析构时就调用 `MV_CC_FreeImageBuffer()`。

如果图像校验、内存分配或像素转换发生异常，栈展开仍会归还缓存。正常路径主动归还后把指针设为空，避免重复归还。析构中的 SDK 返回值没有进一步检查；正常路径的显式归还会检查返回值。

## 4. HikRobot 的每个函数

### HikRobot()

默认构造函数，只建立一个未初始化的对象。成员使用头文件中的默认值，不枚举、不打开相机，也不启动采集。

### ~HikRobot()

析构时调用 `Stop()`，作为资源释放的兜底。析构函数不处理 `Stop()` 的返回值；如果需要判断清理是否成功，应在正常退出流程中显式调用 `Stop()`。

### 被删除的复制构造与复制赋值

`HikRobot(const HikRobot&) = delete` 和 `operator=(const HikRobot&) = delete` 禁止复制对象，避免两个对象持有同一 SDK 句柄并重复关闭、销毁。

### ValidateConfig(const Config& config)

初始化前进行通用参数校验：

1. 曝光、增益、帧率必须是有限数值，不能是 NaN 或无穷大。
2. 曝光和帧率必须大于 0，增益不能小于 0。
3. 数值必须能安全转换为 SDK 使用的 `float`；曝光还要考虑乘以 1000 后的范围。
4. `timeout_ms` 必须在 1～1000 之间。
5. `vid_pid` 必须包含冒号，拆出的两个 ID 都必须通过 `ParseId()`。

这不是具体相机的能力查询。曝光、增益、帧率是否超出设备节点范围，仍由后续 SDK 写入操作判断。

### Configure(const Config& config)

在设备打开后、采集开始前配置相机。

| SDK 设置 | 当前值 | 作用 |
| --- | --- | --- |
| `AcquisitionMode` | `MV_ACQ_MODE_CONTINUOUS` | 连续采集 |
| `TriggerMode` | `MV_TRIGGER_MODE_OFF` | 关闭触发，设备自由运行 |
| `ExposureAuto` | `MV_EXPOSURE_AUTO_MODE_OFF` | 关闭自动曝光，使用手动曝光值 |
| `GainAuto` | `MV_GAIN_MODE_OFF` | 关闭自动增益 |
| `ExposureTime` | `exposure_ms × 1000` | 设置曝光，SDK 节点单位为 μs |
| `Gain` | `gain` | 设置增益 |
| `AcquisitionFrameRateEnable` | `true` | 启用帧率控制 |
| `AcquisitionFrameRate` | `frame_rate` | 设置请求帧率 |
| `MV_CC_SetImageNodeNum` | `3` | 设置 SDK 内部图像缓存节点数 |
| `MV_CC_SetGrabStrategy` | `LatestImagesOnly` | 优先取最新图像，清除输出列表中其余旧图像 |

3 个 SDK 缓存节点不等于项目层保存 3 帧。项目层当前只缓存最新一帧。

最新帧策略适合实时显示和视觉处理，帧号可能跳跃，不能据此认为每一帧都会交给业务代码。代码没有设置相机原始 `PixelFormat`，而是读取帧的实际格式，再转换为 BGR。

### Init(const Config& config)

完成从未初始化对象到可以取帧的全部步骤：

1. 检查本对象是否已经持有句柄或 SDK 引用，重复初始化会抛 `std::logic_error`。
2. 校验配置并解析 VID/PID。
3. 调用 `AcquireRuntime()`，记录本对象已持有 SDK 引用。
4. 调用 `MV_CC_EnumDevices(MV_USB_DEVICE, ...)` 枚举 USB 相机。
5. 按 USB VID/PID 和可选序列号筛选设备。没有匹配设备就报错；匹配多台也报错，要求指定序列号。
6. 调用 `MV_CC_CreateHandle()` 创建句柄。
7. 调用 `MV_CC_OpenDevice()` 打开设备，成功后设置 `opened_`。
8. 调用 `Configure()` 写入参数和采集策略。
9. 调用 `MV_CC_StartGrabbing()`，成功后设置 `grabbing_`。
10. 保存超时配置、清零最近帧号，通过公共宏 `DUST_LOG_OK` 输出初始化日志。

任何一步在资源获取阶段失败，都会进入 `catch (...)`，调用 `Stop()` 回收已获取的资源。回滚成功则重新抛出原始异常；回滚失败则抛出不可恢复的 `HikRobotError`。

初始化日志中的曝光、增益、帧率来自请求配置，没有通过 SDK 回读，也不是实际帧率测量结果。

### Read(cv::Mat& image, steady_clock::time_point& timestamp)

同步读取一帧并输出独立拥有图像内存的 BGR 图像。

执行顺序如下：

1. 清空输出图像并将时间戳置为默认值，避免失败时留下上一帧。
2. 检查是否已开始采集，否则抛 `std::logic_error`。
3. 调用 `MV_CC_GetImageBuffer()`，最多等待 `timeout_ms_`。
4. 返回 `MV_E_NODATA` 时返回 `false`；其他 SDK 错误通过 `Check()` 抛异常。
5. 取得缓存后立即建立 `BufferGuard`，记录主机 `steady_clock::now()`。
6. 检查原始指针、宽高、目标图像字节数和丢包计数。空指针、零尺寸、字节数溢出或丢包都会报错。
7. 创建 `CV_8UC3` 的 `cv::Mat`，分配本模块输出图像的内存。
8. 根据原始帧的像素格式调用 `MV_CC_ConvertPixelTypeEx()`，输出 `PixelType_Gvsp_BGR8_Packed`。
9. 检查转换后的字节数是否等于 `宽 × 高 × 3`。
10. 显式调用 `MV_CC_FreeImageBuffer()` 归还 SDK 缓存，清空保护对象的指针并检查归还结果。
11. 将 BGR 图像移交给输出参数，提交时间戳和帧号，返回 `true`。

| 结果 | 图像输出 | 含义 |
| --- | --- | --- |
| `true` | 有效 BGR 图像 | 成功完成取帧、转换和 SDK 缓存归还 |
| `false` | 空图像 | 本次等待没有获得图像，调用方可以继续尝试 |
| 抛异常 | 空图像 | 未启动采集、SDK 错误或图像无效 |

时间戳表示主机从 SDK 取出帧后的时刻，不是相机曝光开始或结束的硬件时间戳。`timeout_ms_` 只限制 SDK 等待，不包含后续像素转换和调用方显示耗时。

返回的 `cv::Mat` 使用自己分配的内存，因此 SDK 原始缓存归还后，输出图像仍然有效。`std::move` 移交图像对象，像素转换本身仍会写入新的目标内存，整个流程不是零拷贝。

### Stop() noexcept

按资源获取的逆序清理：

1. 已采集时调用 `MV_CC_StopGrabbing()`。
2. 已打开时调用 `MV_CC_CloseDevice()`。
3. 存在句柄时调用 `MV_CC_DestroyHandle()`。
4. 持有运行时引用时调用 `ReleaseRuntime()`，最后一个引用触发 SDK 反初始化。

每一步都清除对应状态；某一步失败仍继续执行后续清理。全部成功返回 `true`，至少一个 SDK 清理步骤失败返回 `false`。

可以重复调用，已清理对象再次调用通常直接返回成功。由于失败步骤也会清除本地状态，重复调用并不是对上次失败步骤的自动重试。

调用前必须等待读取结束，不能让 `Read()` 和 `Stop()` 同时操作同一设备。

### IsOpen() const noexcept

直接返回 `grabbing_`，表示本对象是否已经成功开始采集且尚未停止。它不会向 SDK 查询设备状态，也不能用来确认相机仍在线或图像仍在更新。

### GetFrameNumber() const noexcept

返回最近一次成功 `Read()` 的 `nFrameNum`。初始化后尚未成功读取时为 0，超时不会更新帧号。

帧号来自设备，不是本模块自行累计的读取次数。采用最新帧策略时可能跳号；设备重启或计数回绕也可能改变连续性。

## 5. 使用到的官方 SDK

### SDK 与本项目的区别

海康 MVS 提供相机管理软件及相机控制 SDK。这里使用 SDK 的 C 风格接口进行设备控制和图像采集，不需要通过 MVS 图形界面操作相机。

- `MV_CC_*`：海康官方 SDK 接口。
- `modules::camera::HikRobot`：本项目对 SDK 的封装。
- `cv::Mat`、`cv::imshow()`：OpenCV 接口。
- `DUST_LOG_*`：本项目公共日志宏，不属于海康 SDK。

本机 SDK 位于 `/home/qingyudd01/.local/opt/hikrobot/MvCamCtrlSDK`。当前控制库符号链接指向 `libMvCameraControl.so.4.8.2.1`；这是本机控制库文件版本，不等同于整个 MVS 软件套件的版本号。

| 文件 | 职责 |
| --- | --- |
| `include/MvCameraControl.h` | 相机控制、取帧、缓存归还、像素转换等函数声明及官方接口注释 |
| `include/CameraParams.h` | 设备信息、帧信息、像素转换参数等结构体及采集枚举 |
| `include/PixelType.h` | 原始与目标像素格式定义 |
| `include/MvErrorDefine.h` | SDK 返回错误码定义 |
| `lib/64/libMvCameraControl.so` | 相机控制接口的动态库实现 |
| `lib/64/libMvUsb3vTL.so` | USB3 Vision 传输层运行库 |
| `lib/64/libFormatConversion.so` | 格式转换相关运行库 |

当前模块的 CMake 导入控制库并配置头文件路径，还检查 USB 传输库是否存在。运行时也需要 SDK 的配套依赖，不能只保留一个控制库文件。

### 本模块调用的官方接口

除 `MV_E_NODATA` 的正常超时分支外，初始化和读帧阶段的 SDK 错误通过 `Check()` 转为异常；退出阶段汇总为 `Stop()` 的布尔结果。

| 接口 | 在哪里调用 | 用途及关键参数 |
| --- | --- | --- |
| `MV_CC_Initialize()` | `AcquireRuntime()` | 初始化 SDK 全局运行时，首个使用者调用 |
| `MV_CC_Finalize()` | `ReleaseRuntime()` | 释放 SDK 全局运行时，最后一个使用者调用 |
| `MV_CC_EnumDevices(MV_USB_DEVICE, &devices)` | `Init()` | 枚举 USB 设备，将设备数量和信息指针写入列表 |
| `MV_CC_CreateHandle(&handle_, selected)` | `Init()` | 根据枚举得到的设备信息创建 SDK 句柄 |
| `MV_CC_OpenDevice(handle_)` | `Init()` | 打开设备，建立控制连接 |
| `MV_CC_SetEnumValue(handle_, key, value)` | `Configure()` | 写枚举节点，如连续采集、关闭触发和自动曝光 |
| `MV_CC_SetFloatValue(handle_, key, value)` | `Configure()` | 写浮点节点，如曝光、增益和请求帧率；单位由节点决定 |
| `MV_CC_SetBoolValue(handle_, key, value)` | `Configure()` | 写布尔节点，当前用于开启帧率控制 |
| `MV_CC_SetImageNodeNum(handle_, 3)` | `Configure()` | 在开始采集前设置 SDK 内部缓存节点数 |
| `MV_CC_SetGrabStrategy(handle_, strategy)` | `Configure()` | 设置取帧策略，当前选取最新一帧 |
| `MV_CC_StartGrabbing(handle_)` | `Init()` | 启动图像采集；此后才可以取帧 |
| `MV_CC_GetImageBuffer(handle_, &raw, timeout)` | `Read()` | 在指定毫秒等待期限内取得 SDK 管理的图像缓存 |
| `MV_CC_ConvertPixelTypeEx(handle_, &conversion)` | `Read()` | 按实际原始像素格式转换，写入调用方提供的 BGR 内存 |
| `MV_CC_FreeImageBuffer(handle_, &raw)` | `Read()` / `BufferGuard` | 归还通过 `GetImageBuffer()` 获得的缓存，让 SDK 可以复用 |
| `MV_CC_StopGrabbing(handle_)` | `Stop()` | 停止采集 |
| `MV_CC_CloseDevice(handle_)` | `Stop()` | 关闭设备连接 |
| `MV_CC_DestroyHandle(handle_)` | `Stop()` | 销毁 SDK 设备句柄及其内部资源 |

`MV_CC_OpenDevice()` 的 C++ 声明提供默认访问参数，但官方说明 USB3 Vision 设备忽略访问模式和抢占参数，以控制权限打开；不能仅根据默认参数把本模块描述成 USB 独占模式。

枚举列表由 SDK 管理。官方接口说明提醒避免多线程同时枚举，因为 SDK 可能重分配列表内存；当前运行时引用锁并未覆盖整个枚举操作，相机初始化应按现有启动流程串行组织。

`MV_CC_CreateHandle()` 默认会生成 SDK 日志。`MvSdkLog` 中的 SDK 日志和控制台 `DUST_LOG_*` 是两套不同的日志来源。

## 6. 官方数据结构怎样使用

### MV_CC_DEVICE_INFO_LIST 与 MV_CC_DEVICE_INFO

`MV_CC_DEVICE_INFO_LIST` 的 `nDeviceNum` 表示枚举数量，`pDeviceInfo[i]` 指向对应设备信息。

每台设备通过 `nTLayerType` 判断传输类型；USB 设备的 `SpecialInfo.stUsb3VInfo` 中包含 `idVendor`、`idProduct`、`chSerialNumber`。本模块用这些字段匹配配置，而不是写死 `/dev/bus/usb/...` 路径。

序列号读取使用有长度上限的 `strnlen()`，避免 SDK 字符数组没有终止符时读出边界。

### MV_FRAME_OUT 与 MV_FRAME_OUT_INFO_EX

`MV_FRAME_OUT` 组合原始图像地址 `pBufAddr` 和帧信息 `stFrameInfo`。

| 帧信息字段 | 本模块的使用方式 |
| --- | --- |
| `nWidth` / `nHeight` | 检查非零，决定输出图像尺寸 |
| `enPixelType` | 告诉转换接口原始图像的真实格式 |
| `nFrameLen` | 提供原始图像数据长度 |
| `nFrameNum` | 成功读取后更新最近设备帧号 |
| `nLostPacket` | 非零时将当前图像判为不完整并抛异常 |

原始数据可能是 Bayer、Mono 或其他 SDK 支持的格式，不能直接假设它已经是 BGR 三通道。

### MV_CC_PIXEL_CONVERT_PARAM_EX

| 字段 | 当前赋值 | 含义 |
| --- | --- | --- |
| `nWidth` / `nHeight` | 原始帧宽高 | 转换尺寸 |
| `pSrcData` | `raw.pBufAddr` | SDK 原始像素地址 |
| `nSrcDataLen` | `info.nFrameLen` | 原始数据字节数 |
| `enSrcPixelType` | `info.enPixelType` | 原始像素格式 |
| `enDstPixelType` | `PixelType_Gvsp_BGR8_Packed` | 目标格式：每个像素 B、G、R 各 8 位，交错存储 |
| `pDstBuffer` | `bgr.data` | OpenCV 图像拥有的目标内存 |
| `nDstBufferSize` | `宽 × 高 × 3` | 目标缓冲区容量 |
| `nDstLen` | SDK 写入 | 实际转换输出长度，必须等于目标容量 |

`CV_8UC3` 表示每通道为 8 位无符号整数、每像素 3 通道。这里的颜色顺序由 SDK 目标格式确定为 BGR，符合 OpenCV 常用显示约定。

## 7. 完整生命周期与缓存所有权

```text
构造 HikRobot
    ↓
Init：校验 → 初始化 SDK → 枚举 → 创建句柄 → 打开设备
    ↓
Configure：设置曝光、增益、帧率、缓存和最新帧策略
    ↓
StartGrabbing：开始采集
    ↓
循环 Read：取得 SDK 缓存 → 转换到 cv::Mat → 归还 SDK 缓存
    ↓
结束采集线程并等待其退出
    ↓
Stop：停止采集 → 关闭设备 → 销毁句柄 → 释放 SDK 引用
```

图像内存分成两部分：

1. **SDK 原始缓存**：由 SDK 管理，通过 `GetImageBuffer()` 借用，通过 `FreeImageBuffer()` 归还。调用方不应对它执行 `delete` 或 `free`。
2. **输出 BGR 图像**：由 `cv::Mat` 分配和管理。转换完成后独立于 SDK 缓存，调用方可以继续显示或处理。

项目层保存 `cv::Mat` 时可以共享其引用计数管理的像素内存；这与借用已经归还的 SDK 原始缓存不同。

当前项目的 `timeout_ms: 100` 是单次取帧等待，而 `max_frame_gap_ms: 3000` 是项目层允许连续没有有效图像的最长时间。前者导致 `Read()` 返回 `false`，后者由采集线程判断并进入断线重连。

## 8. 返回码与定位错误

| 错误码 | 含义 |
| --- | --- |
| `MV_OK` | 成功 |
| `MV_E_HANDLE` | 无效句柄 |
| `MV_E_SUPPORT` | 接口或功能不支持 |
| `MV_E_CALLORDER` | 调用顺序错误 |
| `MV_E_PARAMETER` | 参数错误 |
| `MV_E_RESOURCE` | 资源申请失败 |
| `MV_E_NODATA` | 没有数据；当前取帧流程将其视为等待超时 |
| `MV_E_LOAD_LIBRARY` | 动态库加载失败 |
| `MV_E_DEV_OFFLINE` | 设备离线 |
| `MV_E_PARAMETER_RANGE` | 参数超出允许范围 |

错误信息前面的操作名称可以定位失败阶段。例如 `ExposureTime failed` 指向曝光节点写入，`MV_CC_GetImageBuffer failed` 指向取帧操作。错误码的准确含义以本机 `MvErrorDefine.h` 为准。

本模块没有内部自动重连或硬件曝光时间戳输出；项目采集线程已组织断线重连和帧率统计，详见 [断线重连机制](HikRobot断线重连机制.md)。USB 访问权限由操作系统决定，设置曝光不会解决设备节点权限问题。

## 9. 对照阅读入口

### 当前工程源码

- [HikRobot 公共接口](../framework/modules/camera/hikrobot.hpp)
- [HikRobot 实现](../framework/modules/camera/hikrobot.cpp)
- [模块构建配置](../framework/modules/CMakeLists.txt)
- [项目采集线程](../project/thread/camera/trd_camera.cpp)
- [项目相机参数](../project/thread/camera/trd_camera.yaml)

### 本机官方 SDK 头文件

以下链接对应本机安装的官方接口声明与注释，本文的接口解释以这些文件为依据：

- [MvCameraControl.h](/home/qingyudd01/.local/opt/hikrobot/MvCamCtrlSDK/include/MvCameraControl.h)
- [CameraParams.h](/home/qingyudd01/.local/opt/hikrobot/MvCamCtrlSDK/include/CameraParams.h)
- [PixelType.h](/home/qingyudd01/.local/opt/hikrobot/MvCamCtrlSDK/include/PixelType.h)
- [MvErrorDefine.h](/home/qingyudd01/.local/opt/hikrobot/MvCamCtrlSDK/include/MvErrorDefine.h)

## 10. 运行时通过 var 同步调整参数

```text
var list
var get camera.exposure_us
var set camera.exposure_us 5000
var set camera.gain_db 6
```

曝光单位为 μs，5000 表示 5 ms；增益单位为 dB。六个参数及范围存放在 `HikRobot::Parameters` 中，var 直接绑定类内成员。范围由 `MV_CC_GetFloatValue()` 读取真实设备节点，启动和重连后由类刷新。

普通 `REGISTER_SHELL_VAR` 和带范围版本直接写变量。通知版先在条目锁内检查动态范围并钳制候选值，随后释放条目锁并调用同步 setter。setter 返回成功后 var 才提交当前值；setter 抛异常时保留旧值。普通带范围 var 保持越界拒绝。

相机 setter 不在 shell 线程调用 SDK。回调把曝光或增益写入单请求槽并释放参数通知信号量，随后等待相机 Task 返回结果。相机 Task 每轮取帧前非阻塞检查信号量，在同一工作线程内调用 `SetExposure()` 或 `SetGain()`。模块 setter 根据当前设备范围再次校验，SDK 成功后更新当前值和重连配置。

`ok` 表示相机 Task 已经完成 SDK 设置；设备拒绝、离线或正在停止时返回命令错误并保留旧值。失败请求不在重连后补写。相机 Task 是运行期唯一 SDK 调用者，不再使用 `camera_access_` 让 shell 与采集线程争抢设备。

shell 等待设置结果期间不能处理下一条命令。停止流程会取消尚未执行的请求并唤醒 shell；已经开始的 SDK 设置完成后再退出。参数信号量只负责通知，不承担设备互斥，相机无调参请求时不会阻塞等待它。

成功配置由类持有，`Reopen()` 用它断线重连，不写回 YAML。再次启动仍读取 YAML。SDK 写入成功不保证紧接下一帧已使用新参数。

2026-10-13 实机验证通过：连续采集 1440×1080 BGR 图像时，曝光成功设置为 `50000 μs`、增益成功设置为 `6 dB`，命令返回 `ok`，查询值正确且未再出现 `device busy, retry`。参数提交期间 SIGTERM 能唤醒 shell，相机线程正常 Join。调参过程中拔线及重连仍待实机专项验证。
