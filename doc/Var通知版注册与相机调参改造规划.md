# Var 通知版注册与相机调参改造规划

> 历史说明：本文记录已经替换的“shell 直接调用 SDK、设备信号量互斥”方案，不再代表当前实现。当前方案及实施结果见 [相机运行时线程内调参规划](相机运行时线程内调参规划.md)。当前通知型 var 调用 setter 时不持条目锁，相机 Task 通过单请求槽和通知信号量执行 SDK；`camera_access_` 和 `VarEntry::access` 已删除。
>
> 类型说明（2026-10-13）：var 层已改为全类型——注册时由 `TypeMap<decltype(var_)>` 推导 `VarType`，`VarEntry::value/minimum/maximum` 是 `void*`，bool / 各宽度整数 / float / double 各自按类型解析与打印（`var set` 收 `true|false`，`var get` 不打多余小数）。因此本文第 1 节“不改变命令格式和 double 类型支持范围”以及文中把 Entry 字段写成 `double*` 的描述已过时。**通知型 setter 仍收 `double`**（shell 输入本就是文本），范围比较与钳制仍在 double 域，所以超过 2^53 的整数只适用于直接写入型注册。

## 1. 目标与现状

通知版 var 由 shell 线程直接调用注册的设置函数。普通版继续直接修改变量；命令按名称从哈希表定位条目。相机读帧、设置、重连和关闭使用同一个二值信号量互斥，不再每帧检查参数变化。

**本规划已实施。** `ApplyParameters()` 已删除，setter 不再重复校验范围，var 通知函数在 shell 线程同步执行。以下阶段保留实现设计和改动依据。

本次删除旧规划的 pending、revision、通知消费函数及采集线程参数探针。不增加设备线程、不自动写回 YAML、不改变命令格式和 double 类型支持范围。

## 2. 明确同步语义

```text
shell 收到 var set
    ↓
哈希查找条目
    ↓
解析有限 double
    ↓
普通版：持条目锁 → 校验范围 → 直接修改 → 返回 ok
通知版：调用注册函数 → 业务持设备信号量 → 校验范围
                         → 设置 SDK → 更新变量和重连配置 → 返回 ok
```

通知版不先改变量再尝试设备写入。由业务函数在设备操作成功后提交状态，失败时保留旧值，避免在失败后回滚覆盖其他状态。

通知版 callback 是同步设置入口，不再是仅置标志的短通知。为避免混淆，类型名采用 `VarSetter`，注册宏仍以 `NOTIFY` 表示有自定义变更处理。

信号量用于设备互斥，不是命令通知计数。shell 已经在执行命令，没有额外消费者需要唤醒。哈希表只负责定位，不承担线程同步。

## 3. 目标注册接口

```cpp
REGISTER_SHELL_VAR(name, value, mutex);
REGISTER_SHELL_VAR_RANGE(name, value, minimum, maximum, mutex);
REGISTER_SHELL_VAR_NOTIFY(name, value, mutex, setter, context);
REGISTER_SHELL_VAR_RANGE_NOTIFY(name, value, minimum, maximum, mutex, setter, context);
```

普通版参数与行为保持不变。通知版沿用条目变量及可选范围供 get/list 显示，额外注册设置函数和上下文。

```cpp
struct VarEntry;
using VarSetter = void (*)(const VarEntry& entry, double value, void* context);

struct VarEntry
{
    const char* name;
    double* value;
    double* minimum;
    double* maximum;
    std::mutex* mutex;
    VarSetter setter;       // 空指针：普通变量；非空：同步业务设置
    void* context;
};
```

业务 setter 不再只是接收“已写入通知”，而是接收候选值。它在自己的同步机制内调用 var 公共范围校验函数，随后执行设备设置和提交条目状态，失败时抛异常。var 捕获异常并返回错误，不能让相机错误逃出 shell Task 触发整个程序退出。

所有共享变量、范围、上下文及锁的生命周期必须覆盖注册条目。初版使用静态注册，不支持运行中增删条目。

## 4. 阶段一：扩展注册与哈希索引

### 修改文件

- `framework/cmd/shell/var.hpp`
- `framework/cmd/shell/var.cpp`
- `framework/cmd/shell/shell.cpp`

### 任务与改动

1. 在 Entry 末尾追加 setter/context；旧宏填空指针。
2. 新增两个通知版宏，继续生成 `shell_var` 链接段条目。
3. 在 `Var` 增加 `Init()`，遍历链接段构建 `std::unordered_map<std::string, const VarEntry*>`。
4. 在 shell 初始化函数中调用 `Var::Init()`，在启动线程前检测重复名称；重复注册抛初始化异常，不允许覆盖旧条目。
5. `Find()` 改用哈希表查找，平均查找复杂度 O(1)，不承诺最坏情况为常数。get/set 使用索引，list 保持链接段顺序输出。
6. 索引构建后只读，不在命令处理时注册新条目。

### 拟议代码

```cpp
// var.cpp 内部静态存储
static std::unordered_map<std::string, const VarEntry*> entries_;

void Var::Init()
{
    std::unordered_map<std::string, const VarEntry*> entries;
    for (auto* entry = __start_shell_var; entry != __stop_shell_var; ++entry)
    {
        if (!entries.emplace(entry->name, entry).second)
        {
            throw std::logic_error("duplicate shell variable name");
        }
    }
    entries_ = std::move(entries);
}

const VarEntry* Var::Find(const std::string& name)
{
    const auto found = entries_.find(name);
    return found == entries_.end() ? nullptr : found->second;
}
```

保持现有 Shell 命令分发，仅增加初始化调用；CMake 不需要新增 var 源文件。

## 5. 阶段二：改造命令写入，保留普通行为

### 修改文件

- `framework/cmd/shell/var.hpp`
- `framework/cmd/shell/var.cpp`

### 任务与代码改动

抽出 `Var::ValidateValue(const VarEntry&, double)`，集中检查有限数值和动态上下限。调用者须持有条目锁，校验失败抛 `std::invalid_argument`。

普通写入由 var 取得条目锁并调用该函数，随后直接写数值。通知版由 var 调用注册 setter；setter 先取得设备信号量再取得条目锁，调用同一 var 校验函数。这保证设备重连刷新范围与设置校验之间不会穿插，也避免锁顺序反转。

**范围规则仍由 var 维护，不复制到相机模块或每个回调中。** 通知版的校验调用放在业务同步范围内，而不是 shell 提前取得条目锁后调用 setter。

### 拟议代码

```cpp
void Var::ValidateValue(const VarEntry& entry, double value)
{
    if (!std::isfinite(value))
    {
        throw std::invalid_argument("finite number required");
    }
    if ((entry.minimum != nullptr && value < *entry.minimum) ||
        (entry.maximum != nullptr && value > *entry.maximum))
    {
        throw std::invalid_argument("value outside registered range");
    }
}

// Process() 中：沿用严格解析和参数数量检查，取得 entry 和 number 后。
try
{
    if (entry->setter != nullptr)
    {
        // 此处不持条目锁；setter 按设备锁 → 条目锁的顺序执行。
        entry->setter(*entry, number, entry->context);
    }
    else
    {
        std::lock_guard<std::mutex> lock(*entry->mutex);
        ValidateValue(*entry, number);
        *entry->value = number;
    }
    Log::SendCommandLine("ok");
}
catch (const std::exception& error)
{
    Log::SendCommandLine(error.what());
}
catch (...)
{
    Log::SendCommandLine("variable setter failed");
}
```

严格解析仍拒绝尾随字符、溢出和非法数字。`ok` 表示此次同步操作已完成，相机条目表示 SDK 写入成功，不保证硬件从哪一帧开始采用新值。

`get/list` 只取得条目锁读取显示，不取得设备信号量，不调用 setter。

## 6. 阶段三：设备二值信号量与模块精简

### 修改文件

- 新增 `framework/init/semaphore.hpp`：POSIX 二值信号量及作用域释放封装。
- `project/thread/camera/trd_camera.cpp`：增加一个相机操作信号量。
- `framework/modules/camera/hikrobot.hpp/.cpp`：精简 setter 及更新接口注释。

### 信号量设计

当前工程为 C++17，不使用 C++20 的 `std::binary_semaphore`。使用 POSIX `sem_init(..., 0, 1)`；`Acquire()` 调用 `sem_wait()`，遇到 EINTR 重试；`Release()` 调用 `sem_post()`。提供不可复制的 Guard，异常展开时也归还信号量。

封装构造和获取失败抛明确异常；Guard 析构不抛异常。信号量存活时间覆盖所有设备访问及线程 Join，不允许有人等待时销毁。不增加后台线程或通知任务队列。

一个相机的所有操作共享一个信号量。普通 var 不使用它；未来其他设备使用自己的实例，不把所有无关设备串行化。

### 锁顺序

```text
设备二值信号量 → parameter_mutex_
```

所有同时需要两者的代码保持此顺序。禁止持 `parameter_mutex_` 等待设备信号量。

相机 `Read()`、初始化、范围查询、setter、Stop 全部在设备信号量内执行。重连一秒等待不持信号量，窗口操作不取得设备信号量。

### 模块 setter 改动

保留设备打开检查、单位转换及 SDK 返回值检查，删除重复范围查询和比较：

```cpp
void HikRobot::SetExposure(double exposure_ms)
{
    if (!opened_)
    {
        throw std::logic_error("HikRobot is not open");
    }
    Check(MV_CC_SetFloatValue(handle_, "ExposureTime",
                             static_cast<float>(exposure_ms * 1000.0)),
          "ExposureTime");
}

void HikRobot::SetGain(double gain_db)
{
    if (!opened_)
    {
        throw std::logic_error("HikRobot is not open");
    }
    Check(MV_CC_SetFloatValue(handle_, "Gain", static_cast<float>(gain_db)), "Gain");
}
```

`GetExposureRange/GetGainRange` 保留，供初始化及重连刷新 var 范围。模块本身仍不创建线程，不包含 var 或 semaphore 的业务依赖。

## 7. 阶段四：注册同步设置函数并删除参数探针

### 修改文件

- `project/thread/camera/trd_camera.cpp`

### 拟议业务设置代码

下述 `Semaphore::Guard` 名称以阶段三封装为准；只展示同步责任和调用顺序。

```cpp
static init::Semaphore camera_access_{1};

static void SetExposureFromVar(const debug::VarEntry& entry, double value, void*)
{
    init::Semaphore::Guard device(camera_access_);
    if (thread_.StopRequested() || !camera_.IsOpen())
    {
        throw std::runtime_error("camera unavailable or stopping");
    }
    std::lock_guard<std::mutex> lock(*entry.mutex);
    debug::Var::ValidateValue(entry, value);
    camera_.SetExposure(value / 1000.0);
    params_.camera.exposure_ms = value / 1000.0;
    *entry.value = value;
}

static void SetGainFromVar(const debug::VarEntry& entry, double value, void*)
{
    init::Semaphore::Guard device(camera_access_);
    if (thread_.StopRequested() || !camera_.IsOpen())
    {
        throw std::runtime_error("camera unavailable or stopping");
    }
    std::lock_guard<std::mutex> lock(*entry.mutex);
    debug::Var::ValidateValue(entry, value);
    camera_.SetGain(value);
    params_.camera.gain = value;
    *entry.value = value;
}

REGISTER_SHELL_VAR_RANGE_NOTIFY("camera.exposure_us", exposure_us_,
    exposure_min_us_, exposure_max_us_, parameter_mutex_, SetExposureFromVar, nullptr);
REGISTER_SHELL_VAR_RANGE_NOTIFY("camera.gain_db", gain_db_,
    gain_min_db_, gain_max_db_, parameter_mutex_, SetGainFromVar, nullptr);
```

变量和重连配置仅在 SDK 成功后提交。SDK 拒绝、设备离线或正在退出时命令报错，旧值保持；不存储待恢复后应用的请求。断线期间用户需要等设备恢复后重新设置。

### 删除及保留

- 删除整个旧 `ApplyParameters()`，不添加 pending、revision 或替代的参数探针。
- 删除 Task 对 `ApplyParameters()` 的调用。
- 保留请求/显示数值及动态范围存储，成功提交后两者和配置一致。
- 不让 var 回调调用 Recover，不从 shell 中重启采集线程。

### Task 的 Read 改动

```cpp
bool received;
{
    init::Semaphore::Guard device(camera_access_);
    received = camera_.Read(image, timestamp);
}

// 后续保留图像提交、首帧日志、连续无帧判断及统计。
if (received)
{
    // 原成功读帧处理。
}
```

阶段六会将两个 shell setter 的许可获取替换为 200 ms 有限等待，阶段四的普通 Guard 展示基础互斥流程。

设备信号量只覆盖 Read；图像提交、帧率统计及主线程显示不持设备信号量。Read 抛异常时 Guard 先释放，随后 Thread 恢复 hook 才能获取信号量。

### 延迟与公平性

shell 调参可能等待 SDK 取帧及转换结束。当前 `timeout_ms` 为 100 ms，但它不限制转换或 SDK 设置耗时，POSIX 信号量也不保证 FIFO，不能宣称命令一定在 100 ms 内完成。

若实测采集线程反复取得信号量造成命令饥饿，应在锁封装层改用有顺序的竞争策略；不在采集循环重新添加参数探针。初版先做竞争压力验证，不凭假设承诺实时性。

## 8. 阶段五：初始化、范围刷新、重连及清理代码

### 修改文件

- `project/thread/camera/trd_camera.cpp`

### 范围刷新

把现有刷新函数改为调用者已持设备信号量的版本，避免重复获取同一个信号量。范围来自当前设备；变量同步为当前成功配置，不再限制或保留待应用请求。

```cpp
/** @brief 调用方已持设备信号量，刷新 var 范围及成功值 */
static void RefreshParameterRangesLocked()
{
    const auto exposure = camera_.GetExposureRange();
    const auto gain = camera_.GetGainRange();
    std::lock_guard<std::mutex> lock(parameter_mutex_);
    exposure_min_us_ = exposure.minimum * 1000.0;
    exposure_max_us_ = exposure.maximum * 1000.0;
    gain_min_db_ = gain.minimum;
    gain_max_db_ = gain.maximum;
    exposure_us_ = params_.camera.exposure_ms * 1000.0;
    gain_db_ = params_.camera.gain;
}
```

节点范围刷新失败不发布猜测范围。重新初始化因成功配置不被新设备接受而失败时，按现有 SDK 故障分类处理，不静默改变已成功配置。

### 首次初始化

`thread_init()` 保留 YAML 读取，设备初始化和范围查询改为同一保护块：

```cpp
{
    init::Semaphore::Guard device(camera_access_);
    camera_.Init(params_.camera);
    RefreshParameterRangesLocked();
}
System_SetMainLoop(Display);
return true;
```

范围在首次成功初始化时建立。初始化之前范围的默认零值不代表设备支持范围，通知 setter 通过设备可用检查拒绝设置。

### 重连

```cpp
static Thread::RecoveryAction Recover(
    const std::exception& error, Thread& worker, void*)
{
    const auto* fault = dynamic_cast<const HikRobotError*>(&error);
    if (fault == nullptr || !fault->IsRecoverable())
    {
        return Thread::RecoveryAction::Stop;
    }

    DUST_LOG_WRN("camera recovery: %s", error.what());
    {
        init::Semaphore::Guard device(camera_access_);
        if (!camera_.Stop())
        {
            throw std::runtime_error("camera cleanup failed before reconnect");
        }
    }
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        latest_frame_.release();
    }

    // 等待不持设备信号量，shell 可以取得它并立即报告设备不可用。
    while (!worker.WaitForStop(std::chrono::milliseconds(1000)))
    {
        init::Semaphore::Guard device(camera_access_);
        if (worker.StopRequested())
        {
            return Thread::RecoveryAction::Stop;
        }
        try
        {
            camera_.Init(params_.camera);
            RefreshParameterRangesLocked();
            DUST_LOG_OK("camera reconnected; waiting for first frame");
            return Thread::RecoveryAction::Restart;
        }
        catch (const HikRobotError& failure)
        {
            if (!failure.IsRecoverable())
            {
                throw;
            }
            if (!camera_.Stop())
            {
                throw std::runtime_error("camera cleanup failed during reconnect");
            }
            DUST_LOG_WRN("camera reconnect failed: %s", failure.what());
        }
    }
    return Thread::RecoveryAction::Stop;
}
```

每次尝试的配置读取、设备打开和范围刷新都在同一个信号量保护块内。失败后作用域退出归还信号量，再进入下一次等待。hook 仍在采集线程执行，Thread API 不变。

shell setter 捕获到设备错误时只返回命令失败，不直接触发相机 hook；采集线程随后通过 Read 错误或连续无帧判断进入恢复。

### 最终 Release

保留先通知所有线程停止、再 Join、再释放资源的启动框架顺序。相机 Release 的设备停止改为：

```cpp
System_SetMainLoop(nullptr);
bool stopped;
{
    init::Semaphore::Guard device(camera_access_);
    stopped = camera_.Stop();
}
// 保留帧缓存清理、主线程销毁窗口及 stopped 失败处理。
```

通知 setter 获得设备信号量后检查 `thread_.StopRequested()`，避免正常停止过程中的新设备写入。不能只根据 `IsOpen()` 判断相机物理在线，实际写入仍检查 SDK 返回结果。

信号量声明顺序要保证早于相机对象和线程对象，销毁晚于两者。析构兜底在所有访问线程已结束后执行。

## 9. 阶段六：Shell 线程调度与命令执行预算

### 当前真实执行链

当前 `shell.cpp` 的 Task 使用 `poll(stdin, Log::WakeFd(), -1)` 休眠；有输入时调用 `ProcessInput()`，回车处直接执行 `ProcessLine()`。输入处理结束后才清唤醒计数、驱动日志发送泵并检查停止标志。

`Thread::Start()` 当前没有设置 Linux 调度策略或运行优先级。线程沿用创建者的调度属性，常规启动通常是普通分时调度。注册中的 `PreThread` 和初始化优先级是启动顺序，不是运行时 CPU 优先级。

因此，SDK 设置放到 shell 后，shell 等设备信号量或执行 SDK 的期间不能处理下一条命令，也不能驱动其负责的日志泵。其他业务线程和主线程仍可以运行，但排队日志不能靠 shell 同时排空。

### 调度策略选择

初版保持普通 Linux 分时调度，不擅自设置 `SCHED_FIFO/SCHED_RR` 或更高 nice 优先级，不要求管理员权限。提高 shell 优先级不能中断 SDK 调用，也不能消除设备锁等待；高优先级 shell 还可能等待低优先级采集线程，POSIX 信号量没有优先级继承保证。

采用事件唤醒、有限锁等待和命令间日志泵推进作为调度约束，不用空转或 sleep 轮询参数。

### 修改文件及任务

- `framework/cmd/shell/shell.hpp/.cpp`：输入处理支持命令边界上的停止检查和日志泵推进。
- `framework/init/semaphore.hpp`：增加有限等待获取设备许可的接口，避免 shell 无限等锁。
- `project/thread/camera/trd_camera.cpp`：两个 setter 使用有等待上限的 Guard。

#### 1. Shell 每次执行命令前检查停止

保持回车处同步执行命令，不增加命令工作线程。`ProcessInput()` 按字节处理期间检查 shell 停止标志；检测到停止就不再执行该缓冲区中的后续命令。

可在 `ProcessInput()` 增加一个停止查询回调参数，Task 传入本 shell 线程的查询函数，不把 Thread 指针写死进公共 Shell 类。Ctrl-D 的全局退出行为保留。

```cpp
// 拟议新增接口，替代当前 ProcessInput 的签名。
using StopCheck = bool (*)();
void ProcessInput(const char* buffer, size_t size, StopCheck stopping);
```

输入处理每次调用 ProcessChar 前检查 stopping；回车提交路径同样遵守停止检查。不能只在处理完一整个 256 字节缓冲区后检查，因为其中可能包含多条耗时设置命令。

#### 2. 命令之间推进日志泵

一次命令完成后执行一次现有日志泵推进，再继续缓冲区中的下一条命令。保留 Task 原有事件循环中的 ConsumeWake/SendNextFrameIfIdle，不在 SDK 执行期间递归调泵，也不创建日志线程。

可以让 ProcessChar 返回是否已处理一个命令边界，ProcessInput 在该边界调用已注册的轻量服务回调，回调仅推进现有日志泵。编辑和输入回显逻辑保持不变。

```cpp
// 最终接口可将停止查询与命令间服务回调一并传入。
using Service = void (*)();
void ProcessInput(const char* buffer, size_t size,
                  StopCheck stopping, Service between_commands);
```

服务回调在命令已退出、设备和参数锁已经释放后调用。get/list 和其他命令也共享此命令间服务点。输入结束后仍回到原 poll 事件循环。

#### 3. Shell 设置的信号量等待上限

两个相机 setter 将阶段四的无限等待 Guard 改为有上限的获取。初版使用固定 200 ms 锁等待预算，不新增 YAML 项；超时返回“camera busy, retry”，不修改变量或重连配置，也不保留待执行命令。

```cpp
// 目标调用语义；具体构造签名在 semaphore.hpp 中实现。
init::Semaphore::Guard device(camera_access_, std::chrono::milliseconds(200));
```

封装使用 `sem_timedwait()` 并正确处理 EINTR 和超时；POSIX 接口使用绝对实时钟截止时间，应在文档中说明时钟调整对等待预算的影响，不能声称这是硬实时上限。获取失败的 Guard 不归还许可。

采集、重连及最终清理仍使用正常 Guard；shell 有限等待不会取消正在进行的 Read，也不会要求采集线程检查任何通知。

#### 4. SDK 执行时限与退出

200 ms 只限制等待设备许可，不限制已经取得许可后的 SDK 写节点耗时。不得在超时后强行释放另一线程的许可，也不得在 SDK 未返回时关闭其正在访问的设备。

退出顺序保持现有框架：向所有线程请求停止，唤醒 shell 的 poll，等待线程结束，再释放设备资源。shell 在锁等待或 SDK 中时，唤醒描述符不会中断该调用；正常退出需要等有限锁等待结束或 SDK 返回。

必须实测 SDK 设置及打开/关闭耗时。若 SDK 存在不可接受的长期阻塞，仅调整线程优先级或信号量不能解决；届时需要另行讨论执行线程隔离，这次不偷偷新增线程。

### 本阶段验收

1. 无输入、无日志、无停止请求时仍阻塞在 poll，不增加固定周期唤醒。
2. 批量输入多条设置命令时，每条之间可以驱动日志泵并检查停止。
3. 长时间持设备许可时，shell 设置在锁等待预算后失败，不无限挂住命令处理。
4. SDK 设置结束前不承诺日志泵持续运行；测量这段暂停及日志队列压力。
5. 主线程窗口事件不依赖 shell 执行完成，其他业务线程按原调度运行。
6. 停止过程中不继续执行缓冲区中的新命令，所有锁与线程能正常回收。

## 10. 阶段七：文档和验证

### 更新文件

- `doc/HikRobot相机模块与SDK接口说明.md`
- `doc/HikRobot断线重连机制.md`

删除“请求排队后采集线程应用”和“断线期间保留请求”的旧描述，改为 shell 同步设置、成功提交、断线拒绝、设备互斥及可能的命令等待。

### 软件验证

| 验证项 | 验收要求 |
| --- | --- |
| 普通两种宏 | 保持直接写变量，不调用 setter |
| 通知两种宏 | 使用正确 entry/context，每次合法命令执行对应 setter |
| 哈希索引 | 所有注册条目可查找，未知名称报错，重复名称初始化失败 |
| 数值解析 | 非法字符、溢出、NaN、无穷大和越界拒绝 |
| setter 成功/失败 | 成功更新状态，失败保留旧值并返回命令错误 |
| 回调异常 | 不逃出 shell Task，不错误触发 shell 所属线程退出 |
| 动态范围 | 重连刷新与校验受同一设备互斥保护 |
| 设备访问 | Read、设置、Init、Stop 无并发交叠 |
| 异常释放 | Read 或设置抛异常后信号量仍可再次取得 |
| 竞争压力 | 连续采集下设置能取得信号量，观测最长等待并检查饥饿 |
| 停止竞争 | 已停止时拒绝新设置，线程可回收，无锁顺序死锁 |

使用可控的设备调用替身制造写入失败、读取阻塞和重连竞争，验证同步语义；编译验证不能替代这些行为测试。

### 构建与实机

运行 `cmake --build build -j 4`，再在实际相机上验证曝光/增益变化、范围拒绝、重复设置、拔插恢复和设置过程中退出。记录 shell 命令等待情况，不将编译成功表述为实机验证通过。

## 11. 每个文件最终改了什么

| 文件 | 新增或修改 | 删除 |
| --- | --- | --- |
| `var.hpp` | setter 类型、Entry 字段、两种通知宏、Init/Find/ValidateValue 接口 | 无，保留旧宏参数 |
| `var.cpp` | 哈希索引、普通/回调写入分派、集中范围校验、回调错误响应 | get/set 的线性名称查找 |
| `shell.hpp/.cpp` | 建立 var 索引、停止查询和命令间日志泵服务接口 | 仅在整块输入结束后才处理服务的限制 |
| `init/semaphore.hpp` | POSIX 二值信号量、RAII Guard 和有上限获取 | 无 |
| `hikrobot.hpp/.cpp` | setter 设备状态检查、说明调用方校验责任 | setter 重复范围查询及比较 |
| `trd_camera.cpp` | 两个 shell 设置回调、通知注册、设备访问保护、重连刷新成功值 | ApplyParameters、Task 参数应用探针及不再适用的请求限制逻辑 |
| 两份现有相机文档 | shell 同步写入、互斥和命令结果语义 | 待应用队列、请求保留和逐帧应用描述 |

不修改 `Thread` 恢复接口，不增加 YAML 项，不改变普通 var 的直接写入行为。新增 semaphore 为头文件时无需增加 CMake 源文件列表。

## 12. 最终行为与限制

普通变量直接写入；通知变量通过注册函数同步设置，成功才提交状态。采集 Task 只取帧，没有参数变化探针。

设备信号量保证操作互斥，哈希索引保证平均查找效率，两者职责分离。shell 命令执行可能等待设备，日志泵和输入处理也可能受到此次同步等待影响，这是本方案的明确代价。

断线时调参报错而非保存请求；恢复使用最近成功配置。通知版设置函数是业务扩展点，其提交及异常规则必须遵守约定，不能仅依靠注册宏自动保证。

已实施并通过编译。软件测试通过普通及通知版注册、动态范围、回调失败保留旧值、重复名称拒绝、shell CRLF/命令间服务/停止、信号量超时及异常释放、互斥竞争。测试使用日志及设备替身，不代表实际硬件验证。实机调参和拔插恢复待验证。

## 13. 实施结果

设备互斥使用 `init::Semaphore`，构造初值固定为一。`Semaphore::Guard` 支持普通获取和限时获取，shell 两个 setter 使用 200 ms 预算。Shell::ProcessInput 提供 StopCheck 和 Service 回调，命令边界推进日志服务。Var::Init 在 shell 初始化时建立索引，空注册段也可运行。

## 14. 最新参数归属及通知事务调整

此节替代前文项目层保存参数及回调自行提交的旧实现。六个曝光、增益及范围成员已移入 HikRobot::Parameters，成功配置也由类持有。项目只注册类内成员和设备回调，不再保存参数镜像。

通知宏可在 context 后传入可选设备信号量指针。var 先限时取得设备许可，再取得条目锁，钳制有限输入、保存旧值、写入新值并回调；回调任何异常都会在同一保护区内回滚。回调不得重复获取这些锁。普通范围 var 仍拒绝越界输入。

Reopen 使用类保存的成功配置。初始化在类内刷新范围，设置成功更新类内配置；失败不修改成功配置。项目层原参数锁、六个静态 double 和范围刷新函数已删除。
