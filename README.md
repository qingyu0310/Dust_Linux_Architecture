# Linux 视觉基础骨架

原生 C++17 Linux 进程，采用 Dust 六层职责和 ELF 注册式启动，不依赖 ROS2。摄像头与识别模块尚未实现，对应目录保留为空。现有纯计算算法保留。

## 构建与运行

在项目根目录或其子目录执行：

```bash
dust build       # 增量编译
dust build -p    # 清空 build 后全量编译
dust run         # 运行 build/project/vision_app
```

入口脚本位于 `framework/cmd/build`。`.vscode/settings.json` 仅为本工作区的 VS Code 集成终端添加该目录到 PATH。打开本工程后新建终端即可使用；已有终端需关闭后重新创建。普通系统终端可在工程根目录执行 `./framework/cmd/build/dust build`，无需全局安装。

构建使用原生 CMake，默认 Debug，产物和真实 `compile_commands.json` 保存在根目录 `build`。`dust run` 直接运行现有程序，保留终端输入、Ctrl+C 和程序退出码。

Ctrl+C 或 SIGTERM 正常退出。每个业务线程在自己的子目录保存 YAML，由对应 `thread_init()` 使用 `cmd::config::Config` 类读取，没有总配置文件或全局参数结构。

当前 test 线程配置为 `project/thread/test/trd_test.yaml`，`heartbeat_ms` 是心跳间隔，`run_duration_ms` 为 0 时持续运行，正数时运行指定毫秒数。

`project/thread/CMakeLists.txt` 统一装配线程源码，并将 YAML 复制到 `build/project/thread/test/trd_test.yaml`。修改源码目录 YAML 后执行构建即可同步，无需修改 C++；修改运行目录 YAML 后，重启程序生效。程序按 `/proc/self/exe` 定位可执行文件旁的线程 YAML，不依赖工作目录。复制程序部署时，应同时保留旁边的 `thread/test/trd_test.yaml`。

## 分层

- `framework/algorithm`：现有计算库。
- `framework/drivers`、`modules`：设备与视觉能力预留。
- `framework/topic`：图像和结果契约，使用 OpenCV 数据类型。
- `framework/cmd`：DUST_LOG、调试台与 YAML 读取类。
- `framework/init`：ELF 注册表、阶段启动、线程停止等待与信号退出。
- `project`：业务线程及各线程自带 YAML，统一入口位于 framework/init，目前只运行心跳循环。

## 注册接口

```cpp
REGISTER_INIT(Initialize, AppInit, High, HaltOnFail, Release, "app_init");
REGISTER_THREAD(Start, AppThread, Stop, Join, "app_thread");
```

初始化注册参数依次是函数、阶段、等级、失败策略、清理函数和名称。线程注册参数是启动函数、阶段、停止通知函数、等待函数和名称。Linux 版本保留清理回调，参数数量与 Zephyr 原宏不同。

同阶段初始化按 High → Mid → Low 执行，同等级允许多个注册项；同等级初始化与同阶段线程启动按链接表遍历，不另设数字优先级。停机行为由失败策略决定。初始化返回失败或抛出异常时，本项清理函数仍会执行，必须能处理部分初始化。线程启动函数启动线程后立即返回；独立 `Task(void*, void*, void*)` 承载业务循环，`thread_start()` 仅调用 `thread_.Start(Task, nullptr, "test")`；任务异常由公共线程入口捕获并报告停止。Linux 基础版本使用默认调度策略，未移植 Zephyr 栈大小模板和实时优先级。

`System_Startup()` 一次启动，`System_WaitForStop()` 等待信号或业务停止请求，`System_Shutdown()` 先向所有已启动线程通知停止，再逆序等待，最后逆序释放资源。Shutdown 可重复调用；不支持同进程内重新启动。启动和关闭只由主线程执行。

注册段内收集固定大小指针，链接脚本 KEEP 保留，注册源直接进入可执行程序。不得直接将注册源放入未引用的普通静态库而假定其自动加载。框架线程封装的可中断等待适用于基础线程；未来设备阻塞操作还需后端专门处理。

## 基线与验证

原 ament CMake、ROS2 包清单、launch 和 clangd 配置已归档到 `temp/baseline_linux_skeleton`。MuJoCo 目录未修改，不参加根 CMake 构建。

已验证 Debug/Release 编译、注册段存在、初始化失败清理顺序和重复关闭、配置拒绝错误键及重复键、有限时长退出、SIGINT/SIGTERM 停止并等待线程。尚未验证摄像头、图像处理或目标板部署。

## DUST_LOG 与终端调试

日志位于 `framework/cmd/shell`，调用接口与子树一致：

```cpp
DUST_LOG_INF("普通日志 %d", value);
DUST_LOG_ERR("错误 %s", reason);
DUST_LOG_OK("初始化成功");
DUST_LOG_WRN("警告");
DUST_LOG_DBG("test", "调试数据 %.3f", value);
```

DBG 默认静默，在运行终端输入：

```text
log list
log on test
log mode test on
log off
```

模式 on 为 VOFA 纯文本，off 为颜色模式；实际颜色仅在 stderr 是终端时启用。一次性四类日志不受 DBG 选择影响，不使用 log.level 配置。

调试台使用独立 Task 和注册式启动。Linux 使用自有格式化文本与 8×128 字节优先级帧队列；不照搬 MCU 32 位参数槽。stderr 发送是同步后端，慢输出可能阻塞，队列压力下可丢弃 DBG。调试变量与 UART/DMA 暂未移植。

stdin 与 stderr 连接同一个终端时，显示 `dust> ` 提示符，由 shell 逐字接收并回显 ASCII 命令。后台日志输出后恢复当前输入和光标，输入 `log off` 时不会再被心跳拆开。支持退格、左右方向键、Home/End、Delete、Ctrl+U；长命令横向滚动，最长 255 字节，超长整行丢弃。退出时恢复原终端设置，Ctrl+C 仍有效。重定向或 `TERM=dumb` 时保持原输入方式，不输出提示符及输入重绘序列。[Linux termios 手册](https://man7.org/linux/man-pages/man3/termios.3.html)

平台启动先忽略 SIGPIPE。日志写入或刷新失败时保存首次错误并请求统一关闭，失效后不再反复写入该输出端。详见 [SIGPIPE 问题说明与解决方案](doc/SIGPIPE问题说明与解决方案.md)。

历史测试曾验证运行期断管后的线程清理，以及持续心跳中的输入恢复、按键编辑、终端属性恢复和纯文本重定向。当前工程已无 `tests` 目录，根 CMake 不再装配这些测试；历史结果见对应检查文档。

## 线程参数读取

`framework/cmd/config/config.hpp/.cpp` 提供 `cmd::config::Config` 类，封装文件定位、YAML 解析、字段集合及整数范围校验。各实例各自加载文件，不保存全局业务参数，不解析命令行。

线程中只声明本线程参数并调用：

```cpp
cmd::config::Config yaml;
yaml.Load("thread/test/trd_test.yaml", {"heartbeat_ms", "run_duration_ms"});
params_.heartbeat_ms = yaml.GetInt("heartbeat_ms", 1, 60000);
params_.run_duration_ms = yaml.GetInt("run_duration_ms", 0, 86400000);
```

YAML 仍放在对应线程目录，字段和允许范围由线程决定。
