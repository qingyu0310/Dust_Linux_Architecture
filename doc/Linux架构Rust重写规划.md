# Linux C++ 架构 Rust 重写规划

> 2026-11-22 · 状态：**规划完成，待拍板**
> 对象：**当前工作区**（`/home/rmconstruction/Desktop/qingyu`）里的 Linux C++ 架构，共 49 个源文件、约 9,076 行（`framework/` + `project/` + `scripts/`）。
> **目录结构照搬现有架构**：Rust 工程不再另起 `crates/` 树，`rust/` 下逐层镜像 `framework/<层>/…` 与 `project/thread/<线程>/…`，文件一一对应。不使用 `src/` 目录——每个 crate 用 `[lib] path = "lib.rs"` 平铺。
> 前置：Rust 工具链已装（rustc/cargo 1.100.0），`rust/` workspace 已存在。

---

## 0. 现状盘点（实测，不是推测）

### 0.1 层次与文件清单

| 层 | 文件（行数） | 职责 |
|---|---|---|
| **init** | `thread/thread.hpp`(570) · `thread/thread_fault.hpp`(79)+`.cpp`(64) · `thread/thread_registry.hpp`(119) | 受管线程外壳（周期/暂停/停止/统计/故障钩子）、线程注册表（16 项） |
| | `startup/Init_entry.hpp`(147)+`.cpp`(372) · `startup/System_startup.hpp`(46) · `startup/main.cpp`(25) · `startup/boot_timing.hpp`(58)+`.cpp`(51) · `signal_handler.hpp`(30)+`.cpp`(69) · `semaphore.hpp`(188) | 五阶段注册式启动、启动耗时槽、信号停机、POSIX 信号量封装 |
| **shell** | `shell.cpp`(432) · `shell.hpp`(62) · `shell_cmd.hpp`(60) · `token.hpp`(99) | stdin 行编辑（历史 16 条）、命令族分派、token 化 |
| | `log/log.hpp`(161) · `log/log.cpp`(101) · `log/process/log_process.hpp`(198)+`.cpp`(429) · `log/process/log_debug.hpp`(157) · `log/queue/log_policy.hpp`(121) · `log/queue/log_record.hpp`(51) · `log/queue/log_queue.hpp`(132) · `log/transport/log_transport.hpp`(46)+`.cpp`(75) · `log/editor/log_editor.hpp`(98)+`.cpp`(185) · `log/cmd/log_cmd.hpp`(25)+`.cpp`(113) | 四层日志：命令 / 加工 / 队列 / 发送；三档优先级、DBG 流式选择与分频、终端重绘 |
| | `var/var.hpp`(209)+`.cpp`(503) · `boot/boot_cmd.hpp`(31)+`.cpp`(109) · `thread/thread_cmd.cpp`(303) | `var list/get/set`、`boot` 启动耗时、`thread list/show/pause/resume/exit/reset` |
| **topic** | `channel.hpp`(235) · `ecat_rx/ecat_rx.hpp`(46) | 单槽最新消息通道 + 订阅者游标 + 丢帧计数；256 B PDO 双向契约 |
| **drivers** | `communication/ethercat/ecat.hpp`(912) | SOEM 主站：建链、PDO 映射、OP 请求、恢复、WKC/AL 状态监测 |
| **modules** | 空（只有 `CMakeLists.txt` 10 行 + Kconfig 4 行） | 骨架无设备模块 |
| **algorithm** | `filter/attitude/quaternion_attitude.hpp`(192) · `filter/kalman/ekf.hpp`(190) · `filter/kalman/kalman_filter.hpp`(107) · `math/angle.hpp`(62) | 纯头文件算法，当前骨架未被线程引用 |
| **cmd** | `config/config.hpp`(46)+`.cpp`(317) · `linker/dust_init.ld` · `build/{build.sh,dust,run.sh,README.md}` | YAML 参数读取、注册段链接脚本、构建/运行脚本 |
| **project** | `thread/ethercat/trd_ethercat.cpp`(394) · `thread/ros2/trd_ros2.cpp`(227) · `thread/test/trd_test.cpp`(81) | 三个业务线程：EtherCAT 周期 1 ms、ROS 2 桥接、自检 10 ms |
| **构建** | 根 `CMakeLists.txt`(75) · `framework/CMakeLists.txt`(75) · `project/CMakeLists.txt`(64) · `project/Kconfig`(61) · `scripts/kconfig.py`(351) · 各层 CMake/Kconfig | CMake + 自研 Kconfig 求值器；产物 `project/build/project/dust_app` |

### 0.2 不在重写范围

- `temp/dust-control-git/`（参照仓库 clone）、`doc/`、`temp/` 其它内容。
- `README.md`、`快速开始.md` 只在 P10 收尾时改写口径。
- Zephyr 子树 / youyeetoo 板卡（不在本工作区）。

### 0.3 关键机制（重写必须 1:1 保留）

1. **注册式生命周期**：`REGISTER_INIT` / `REGISTER_THREAD` 靠链接段 `.user_init` / `.user_thread` 收集；`REGISTER_SHELL_CMD` 用 `shell_cmd` 段；`REGISTER_SHELL_VAR` 用 `shell_var` 段；`REGISTER_INIT/THREAD` 同时生成 `.boot_timing` 槽。新增组件 = 自己文件里写一行，不改启动器。
2. **五阶段启动**：`PreInit → EarlyInit → MidInit → LateInit → AppInit`，同阶段按 `High → Mid → Low`；失败策略 `HaltOnFail`（请求进程停止）/ `ContinueOnFail`；初始化失败与退出都走 cleanup（可重复调用）。
3. **线程外壳**：`Start(entry, period, context, name, fault)`；`period=0` 表示任务自己阻塞（shell 用 poll、ros2 用 spin_some）；`TaskResult::{Continue,Retry,Exit}`；`Retry` 交 `RetryHook`，`Exit` 交 `DefaultHandle`；**停机路径下先判 `StopRequested`，不算故障**；`keep_alive` 线程拒绝 `Exit()`；收尾顺序 = 亡语 → 注销注册表 → 打日志。
4. **四层日志**：命令 / 加工 / 队列 / 发送，单向流水线；三档优先级 `Event(0) < Cmd(1) < Dbg(2)`；帧池 8 × 320 B；记录 512 B；DBG 64 条目、`divisor/quota` 分频、`epoch` 作废在途帧；输出到 stderr。
5. **topic 单槽通道**：槽深 1（只存最新）、全局序号、每订阅者独立游标、`Wait / WaitTimeout / TakeLatest / AlignToLatest`、`Taken/Dropped` 计数。
6. **EtherCAT 语义**：单实例单网口单线程（SOEM context 不可重入）；IOmap 4096 B；`expected_wkc = outputsWKC*2 + inputsWKC`；SM2/SM3 长度重写；切 OP 期间持续收发过程数据；写 ALCTL 返回 WKC=0 **立即重写**、超时用真实时钟；状态轮询 100 ms；发送间隔告警 20 ms；故障只打边沿日志。
7. **业务协议**：PDO 双向各 256 B；命令头 `0x6B`、15 B 有效 + CRC16；状态头 `0x6D`、20 B（CRC16 over 18 B）；CRC16 初值 `0xFFFF`、多项式 `0x8408`、无最终异或；ROS 命令 100 ms 未更新即发零速度。
8. **ROS 2 桥**：独立 `rclcpp::Context`、节点 `dust_ecat_bridge`、发布 `/ecat/rx_pdo`、订阅 `/ecat/tx_pdo`、`KeepLast(1)` + `best_effort` + `volatile`、256 B `UInt8MultiArray`、`spin_some(1ms)`。
9. **构建/运行**：根 CMake 先跑 `scripts/kconfig.py` 生成 `.config/autoconf.h/kconfig.cmake`；`build.sh` 输出 `project/build/project/dust_app`；`run.sh` source `/opt/ros/humble/setup.bash`、非 root 时 `sudo` 提权；SOEM 为系统依赖（`/usr/local/lib/libsoem.so`）。

---

## 1. 技术选型

| 项 | 选择 | 理由 | 状态 |
|---|---|---|---|
| 工程布局 | **镜像现有架构**：`rust/framework/{init,shell,cmd,drivers,topic,algorithm,modules}` + `rust/project/thread/{test,ethercat,ros2}` | 目录、文件、分层与 C++ 一一对应，迁移和对照都按路径走 | 定 |
| 无 `src/` | 每个 crate 用 `[lib] path = "lib.rs"`，文件平铺在层目录下 | 与 C++ 层内平铺结构一致；不需要 Cargo 的 `src/` 约定 | 定 |
| EtherCAT | **soem-rs 优先；缺 API 回退自建 `bindgen` + `cc` 绑系统 SOEM** | 当前 `ecat.hpp` 直接调 C 版 SOEM，改动面锁定在一个后端文件 | 待拍板 A |
| ROS 2 | **r2r 先试；不成熟则 ROS 2 节点保留独立进程，Rust 用 IPC 对接** | 当前只用 `rclcpp` 的标准 pub/sub + 独立 Context | 待拍板 B |
| 并发 | `std::thread` + 自研单槽通道（复刻 `topic::Channel`） | 与"类 = 1 线程 + 周期外壳"一一对应，不引 tokio | 定 |
| 注册机制 | **`inventory` crate**（链接段/ctor 收集） | 与现有 5 个链接段的"写一行即注册"同构 | 待拍板 C |
| 功能门禁 | **Cargo features 替换 Kconfig**（映射见 §1.1） | 保持"门禁决定存在"的装配语义 | 待拍板 D |
| 周期语义 | `thread::sleep_for`；`period == 0` 表示任务自阻塞 | 与现状完全一致（shell/ros2 就是 0） | 定 |
| 日志 | 自研 `log/` 模块，复刻四层与三档优先级 | 行为要 1:1，`tracing` 无法对齐 | 定 |
| 配置 | `serde` + `serde_yaml` | 替换 `yaml-cpp`，字段校验语义保持一致 | 待拍板 E |
| 错误处理 | 线程内 `TaskResult`；crate 边界 `thiserror`；热路径不 panic | 对齐现有异常→钩子模型 | 定 |
| 实时性 | 默认关；`--rt` 开 `mlockall` + `SCHED_FIFO` | 1 kHz PDO 需要，但现状 C++ 版也没开 | 待拍板 F |
| release | `panic = "abort"`、`lto = "thin"` | 与 C++ 版无 unwind 行为接近 | 定 |

### 1.1 Kconfig → Cargo features 映射

| C++ 符号 | Rust 落点 | 类型 |
|---|---|---|
| `DUST_CMD_SHELL` | crate `shell` feature `console` | 调试台底座 |
| `DUST_CMD_SHELL_LOG` | crate `shell` feature `log`（依赖 `console`） | 日志 + `log` 子命令 |
| `DUST_CMD_SHELL_VAR` | crate `shell` feature `var` | `var` 子命令 |
| `DUST_CMD_CONFIG` | crate `cmd` feature `config` | YAML 读取 |
| `DUST_COM_ETHERCAT` | crate `drivers` feature `ethercat` | SOEM 主站 |
| `DUST_TPC_ECAT_RX` | crate `topic` feature `ecat_rx` | 256 B PDO 契约 |
| `DUST_FLT_KALMAN` / `_EKF` / `_QUATERNION` / `DUST_MATH_ANGLE` | crate `algorithm` 同名小写 feature | 纯算法 |
| `TRD_TEST` | crate `project` feature `test` | 自检线程 |
| `TRD_ETHERCAT` | crate `project` feature `ethercat` | 周期线程 |
| `TRD_ROS2` | crate `project` feature `ros2`（隐含 `ethercat`） | 桥接线程 |

---

## 2. 目标工作区结构（镜像现有架构）

```text
rust/
├── Cargo.toml                          ← workspace（8 个成员 + 统一依赖/lints/profile）
├── framework/
│   ├── init/                           ← crate: init
│   │   ├── Cargo.toml                  ← [lib] path = "lib.rs"
│   │   ├── lib.rs
│   │   ├── error.rs
│   │   ├── semaphore.rs
│   │   ├── thread/
│   │   │   ├── thread.rs               ← thread.hpp
│   │   │   ├── fault.rs                ← thread_fault.hpp/.cpp
│   │   │   └── registry.rs             ← thread_registry.hpp
│   │   └── startup/
│   │       ├── boot.rs                 ← Init_entry.hpp + boot_timing.hpp/.cpp
│   │       ├── startup.rs              ← Init_entry.cpp + System_startup.hpp
│   │       └── signal.rs               ← signal_handler.hpp/.cpp
│   ├── shell/                          ← crate: shell
│   │   ├── Cargo.toml                  ← [lib] path = "lib.rs"
│   │   ├── lib.rs                      ← log.hpp 门面 + shell 线程装配
│   │   ├── shell.rs                    ← shell.cpp/.hpp（行编辑 + 分派）
│   │   ├── shell_cmd.rs                ← shell_cmd.hpp（注册宏）
│   │   ├── token.rs                    ← token.hpp
│   │   ├── var/var.rs                  ← var.hpp/.cpp
│   │   ├── boot/boot_cmd.rs            ← boot_cmd.hpp/.cpp
│   │   ├── thread/thread_cmd.rs        ← thread_cmd.cpp
│   │   └── log/
│   │       ├── process/process.rs      ← log_process.hpp/.cpp
│   │       ├── process/debug.rs        ← log_debug.hpp
│   │       ├── queue/policy.rs         ← log_policy.hpp
│   │       ├── queue/record.rs         ← log_record.hpp
│   │       ├── queue/queue.rs          ← log_queue.hpp
│   │       ├── transport/transport.rs  ← log_transport.hpp/.cpp
│   │       ├── editor/editor.rs        ← log_editor.hpp/.cpp
│   │       └── cmd/log_cmd.rs          ← log_cmd.hpp/.cpp
│   ├── cmd/                            ← crate: cmd
│   │   ├── Cargo.toml                  ← [lib] path = "lib.rs"
│   │   ├── lib.rs
│   │   └── config/config.rs            ← config.hpp/.cpp
│   ├── drivers/                        ← crate: drivers
│   │   ├── Cargo.toml                  ← [lib] path = "lib.rs"
│   │   ├── lib.rs
│   │   └── communication/ethercat/
│   │       ├── ecat.rs                 ← ecat.hpp（对外类型与泛型壳）
│   │       ├── slave.rs                ← SlaveInfo / SyncMode / Config
│   │       ├── pdo.rs                  ← 模板 IO → 泛型 trait
│   │       └── soem.rs                 ← 全部 ecx_* 调用（unsafe 收口）
│   ├── topic/                          ← crate: topic
│   │   ├── Cargo.toml                  ← [lib] path = "lib.rs"
│   │   ├── lib.rs
│   │   ├── channel.rs                  ← channel.hpp
│   │   └── ecat_rx/ecat_rx.rs          ← ecat_rx.hpp
│   ├── algorithm/                      ← crate: algorithm
│   │   ├── Cargo.toml                  ← [lib] path = "lib.rs"
│   │   ├── lib.rs
│   │   ├── filter/kalman/kalman_filter.rs
│   │   ├── filter/kalman/ekf.rs
│   │   ├── filter/attitude/quaternion_attitude.rs
│   │   └── math/angle.rs
│   └── modules/                        ← crate: modules（骨架为空，占位）
│       ├── Cargo.toml                  ← [lib] path = "lib.rs"
│       └── lib.rs
└── project/                            ← crate: project（bin: dust_rs）
    ├── Cargo.toml                      ← [[bin]] name="dust_rs" path="main.rs"
    ├── main.rs                         ← framework/init/startup/main.cpp 的 Rust 版
    ├── protocol.rs                     ← trd_ethercat.cpp 里的 CRC16/命令包/状态包
    └── thread/
        ├── test/trd_test.rs            ← trd_test.cpp
        ├── ethercat/trd_ethercat.rs    ← trd_ethercat.cpp
        └── ros2/trd_ros2.rs            ← trd_ros2.cpp
```

依赖方向与 C++ 一致：`project → drivers/shell/cmd/topic/algorithm`，`shell → init`，`drivers → shell(log)/init`，`topic`、`algorithm`、`modules` 相互独立。

### 2.1 与 C++ 文件的一一对应

| C++ | Rust |
|---|---|
| `framework/init/thread/thread.hpp` | `rust/framework/init/thread/thread.rs` |
| `framework/init/thread/thread_fault.*` | `rust/framework/init/thread/fault.rs` |
| `framework/init/thread/thread_registry.hpp` | `rust/framework/init/thread/registry.rs` |
| `framework/init/startup/Init_entry.*` + `boot_timing.*` | `rust/framework/init/startup/boot.rs` |
| `framework/init/startup/Init_entry.cpp`（启动/关闭流程） | `rust/framework/init/startup/startup.rs` |
| `framework/init/startup/main.cpp` | `rust/project/main.rs` |
| `framework/init/signal_handler.*` | `rust/framework/init/startup/signal.rs` |
| `framework/init/semaphore.hpp` | `rust/framework/init/semaphore.rs` |
| `framework/shell/shell.cpp/.hpp` | `rust/framework/shell/shell.rs` |
| `framework/shell/shell_cmd.hpp` | `rust/framework/shell/shell_cmd.rs` |
| `framework/shell/token.hpp` | `rust/framework/shell/token.rs` |
| `framework/shell/var/*` | `rust/framework/shell/var/var.rs` |
| `framework/shell/boot/*` | `rust/framework/shell/boot/boot_cmd.rs` |
| `framework/shell/thread/thread_cmd.cpp` | `rust/framework/shell/thread/thread_cmd.rs` |
| `framework/shell/log/log.hpp` + `log.cpp` | `rust/framework/shell/lib.rs` + 宏 |
| `framework/shell/log/process/*` | `rust/framework/shell/log/process/{process,debug}.rs` |
| `framework/shell/log/queue/*` | `rust/framework/shell/log/queue/{policy,record,queue}.rs` |
| `framework/shell/log/transport/*` | `rust/framework/shell/log/transport/transport.rs` |
| `framework/shell/log/editor/*` | `rust/framework/shell/log/editor/editor.rs` |
| `framework/shell/log/cmd/*` | `rust/framework/shell/log/cmd/log_cmd.rs` |
| `framework/topic/channel.hpp` | `rust/framework/topic/channel.rs` |
| `framework/topic/ecat_rx/ecat_rx.hpp` | `rust/framework/topic/ecat_rx/ecat_rx.rs` |
| `framework/drivers/communication/ethercat/ecat.hpp` | `rust/framework/drivers/communication/ethercat/{ecat,slave,pdo,soem}.rs` |
| `framework/cmd/config/*` | `rust/framework/cmd/config/config.rs` |
| `framework/algorithm/**` | `rust/framework/algorithm/**`（同名路径） |
| `framework/modules/*` | `rust/framework/modules/lib.rs`（空） |
| `project/thread/test/trd_test.cpp` | `rust/project/thread/test/trd_test.rs` |
| `project/thread/ethercat/trd_ethercat.cpp` | `rust/project/thread/ethercat/trd_ethercat.rs` |
| `project/thread/ros2/trd_ros2.cpp` | `rust/project/thread/ros2/trd_ros2.rs` |
| `scripts/kconfig.py` + 各 `Kconfig` | 删除，改 Cargo features |
| `framework/cmd/linker/dust_init.ld` | 删除（`inventory` 自带段） |
| `framework/cmd/build/{build.sh,run.sh,dust}` | 原文件改调 cargo（P10） |

---

## 3. 阶段与步表

> **通用纪律**：用户说"**开始第 N 步**"= 只做表格里那一步，改完停下报告，等"继续"。

---

### P0 现状冻结（3 步）

> **P0 拆成 3 步，每次只执行一步。**
> 用户说"开始第 N 步"= 只写那一个 md，写完停下报告。

| 步 | 文件 | 动作 | 依赖前置 | 连带 |
|---|---|---|---|---|
| 0-1 | `doc/Rust重写_现状清单.md` | 把 §0.1 的 49 个文件逐条展开：路径/行数/类与函数清单/被谁引用/是否重写（本规划 §0 是摘要，此文件是全量底账） | 无 | 无 |
| 0-2 | `doc/Rust重写_行为基线.md` | 跑 C++ 版记录基线：`dust build` 输出、启动日志顺序、`boot` 各阶段耗时、`thread list`、`log list`、连从站后的 `ecat` 日志与 WKC、ROS 2 两端消息 | 0-1 | 后续每阶段对照它验收 |
| 0-3 | `doc/Rust重写_依赖核实.md` | 逐条核实 `soem-rs` 是否覆盖 `ecat.hpp` 实际用到的 C 接口（§3.1）；核实 `r2r` 能否复刻 `trd_ros2.cpp`；结论写"可用/缺项/回退" | 0-1 | 决定拍板 A/B |

#### 3.1 `ecat.hpp` 实际用到的 SOEM 接口（0-3 逐条比对）

```text
ecx_init / ecx_close
ecx_config_init / ecx_readstate
ecx_config_map_group
ecx_configdc
ecx_send_processdata / ecx_receive_processdata
ecx_statecheck / ecx_writestate
ecx_FPRD / ecx_FPWR / ecx_FPWRw
数据结构：ecx_contextt{slavelist[], slavecount, grouplist[]{outputsWKC,inputsWKC}, port}
          slavelist[i]{configadr, state, ALstatuscode, Obytes, Ibytes, name, SM[2..3], outputs, inputs}
          ec_smt{SMlength}、ECT_REG_ALCTL/ALSTAT/ALSTATCODE/SM0/SM2/SM3、EC_STATE_*
```

**P0 编译状态**：不动代码，cargo 构建不受影响。

---

### P1 workspace 骨架（10 步，镜像七层 + project）

> **P1 拆成 10 步，每次只执行一步。**
> 用户说"开始第 N 步"= 只建那一个 crate（`Cargo.toml` + `lib.rs` + workspace members 一行），改完停下报告。
> **1-1 之后到 1-9 之间 `cargo build` 会报缺成员，这是设计内中间态；1-9 完成才恢复全绿。**

| 步 | 文件 | 动作 | 依赖前置 | 连带 |
|---|---|---|---|---|
| 1-1 | `rust/Cargo.toml` | members 改为 8 个路径成员（`framework/init`、`framework/shell`、`framework/cmd`、`framework/drivers`、`framework/topic`、`framework/algorithm`、`framework/modules`、`project`）；`[workspace.dependencies]`（thiserror、inventory、serde、serde_yaml、libc、nix）；`[workspace.lints]`；`release` profile | 无 | **编不过**：members 不存在 |
| 1-2 | `rust/framework/init/{Cargo.toml,lib.rs}` | crate `init`，`[lib] path="lib.rs"`；依赖 `thiserror`、`libc` | 1-1 | 缺其余成员 |
| 1-3 | `rust/framework/shell/{Cargo.toml,lib.rs}` | crate `shell`；依赖 `init`；features `console/log/var` | 1-2 | 缺其余成员 |
| 1-4 | `rust/framework/topic/{Cargo.toml,lib.rs}` | crate `topic`；feature `ecat_rx` | 1-2 | 缺其余成员 |
| 1-5 | `rust/framework/drivers/{Cargo.toml,lib.rs}` | crate `drivers`；依赖 `init`、`shell`；feature `ethercat` | 1-3 | 缺其余成员 |
| 1-6 | `rust/framework/cmd/{Cargo.toml,lib.rs}` | crate `cmd`；依赖 `serde_yaml`；feature `config` | 1-2 | 缺其余成员 |
| 1-7 | `rust/framework/algorithm/{Cargo.toml,lib.rs}` | crate `algorithm`；features `kalman/ekf/quaternion/angle` | 1-2 | 缺其余成员 |
| 1-8 | `rust/framework/modules/{Cargo.toml,lib.rs}` | crate `modules`（空骨架，与 C++ 同为空层） | 1-2 | 缺 app |
| 1-9 | `rust/project/{Cargo.toml,main.rs}` | crate `project`（bin `dust_rs`，`[[bin]] path="main.rs"`）；features `test/ethercat/ros2`；main 打一行 `dust_rs boot` | 1-3~1-8 | **本步恢复全绿** |
| 1-10 | `rust/Cargo.toml` | 核对 members 与磁盘一致，删占位 | 1-9 | 无 |

**P1 验收**：`cargo build --workspace` 通过；`cargo run -p project` 输出 `dust_rs boot`。

---

### P2 framework/init（10 步）

> **P2 拆成 10 步，每次只执行一步。**
> 用户说"开始第 N 步"= 只改那一个文件，改完停下报告。
> 每步必须保持 workspace 可编译，单测随文件带。

| 步 | 文件 | 动作 | 依赖前置 | 连带 |
|---|---|---|---|---|
| 2-1 | `rust/framework/init/error.rs` | `InitError`、`ThreadError`（`thiserror`） | 1-2 | 无 |
| 2-2 | `rust/framework/init/thread/period.rs` | `period: Option<Duration>`；`None`=任务自阻塞；`Delay` 用 `sleep_for` | 2-1 | 无 |
| 2-3 | `rust/framework/init/thread/stats.rs` | `ThreadStats{name,run_count,retry_count,exit_count,drop_count,run_ns,max_run_ns,last_run_ns,running,paused}`；`AtomicU64` 单一写者 | 2-1 | 无 |
| 2-4 | `rust/framework/init/thread/fault.rs` | `FaultPolicy{Isolate,Recover}`、`RecoveryAction{Restart,Stop}`、`RetryHook`/`ExitHook` 类型、`DefaultHandle`（可替换全局钩子）、`keep_alive` | 2-1 | 无 |
| 2-5 | `rust/framework/init/thread/thread.rs` | `Thread`：`start/request_stop/join/sleep/stop_requested/is_ready`、`TaskResult`、`Run` 循环（暂停等待→停止判断→调用任务→统计→Retry/Exit 处置→period>0 才 sleep） | 2-2~2-4 | 无 |
| 2-6 | `rust/framework/init/thread/registry.rs` | 定长 16 项：`add/remove(尾部填补)/at/count/find`；`Mutex` 保护 | 2-5 | 无 |
| 2-7 | `rust/framework/init/startup/boot.rs` | `InitStage/ThreadStage/InitLevel/InitFailPolicy`、`InitEntry/ThreadEntry`、`BootTimingSlot`、排序与遍历 | 2-6 | 无 |
| 2-8 | `rust/framework/init/startup/signal.rs` | `SIGPIPE` 忽略、`SIGINT/SIGTERM` 置原子标志（信号上下文不锁不分配） | 2-1 | 无 |
| 2-9 | `rust/framework/init/semaphore.rs` | POSIX 信号量封装：get/timed get/post、RAII 释放 | 2-1 | 无 |
| 2-10 | `rust/framework/init/startup/startup.rs` + `lib.rs` | `startup`：install signals → pre/early/mid/late/app init → 各阶段 thread 启动 → `wait_for_stop()` → `shutdown()`（逆序 stop/join + cleanup）；`lib.rs` 声明模块并导出 | 2-1~2-9 | 删占位后本步内改齐引用 |

**P2 验收**：单测覆盖线程周期、Retry→钩子、Exit→策略、注册表尾部填补、启动顺序与失败策略。

---

### P3 framework/shell/log（9 步）

> **P3 拆成 9 步，每次只执行一步。**
> 用户说"开始第 N 步"= 只改那一个文件，改完停下报告。
> 热路径禁分配：记录/帧/队列全部定长。

| 步 | 文件 | 动作 | 依赖前置 | 连带 |
|---|---|---|---|---|
| 3-1 | `rust/framework/shell/log/queue/policy.rs` | 通道表（INF/ERR/OK/WRN/DBG/CMD 的前缀、ANSI RGB、优先级、可选/可挤标志）+ 优先级表 | 1-3 | 无 |
| 3-2 | `rust/framework/shell/log/queue/record.rs` | `LogRecord{text[512], channel, epoch, ansi}` + 格式化入口 | 3-1 | 无 |
| 3-3 | `rust/framework/shell/log/queue/queue.rs` | `TxFrame{data[320], priority, epoch}`；8 帧插入/挤出/弹头、跳过过期 DBG、`format_record_to_frame` | 3-2 | 无 |
| 3-4 | `rust/framework/shell/log/process/debug.rs` | 64 条 DBG 注册表、`divisor/quota` 分频、`active/epoch`、`find_or_create/select/deselect/list` | 3-1 | 无 |
| 3-5 | `rust/framework/shell/log/transport/transport.rs` | 写 stderr、记录首个 errno、`supports_color` | 3-3 | 无 |
| 3-6 | `rust/framework/shell/log/editor/editor.rs` | 接管终端时的输入行快照/光标重绘、回车保留、退出恢复属性 | 3-5 | 无 |
| 3-7 | `rust/framework/shell/log/process/process.rs` | `info/error/ok/warning/debug/command_line`、`pass_divisor`、`send_next_frame_if_idle`、`wake_fd/consume_wake/flush`、`mark_shell_thread_ready` | 3-3~3-6 | 无 |
| 3-8 | `rust/framework/shell/log/cmd/log_cmd.rs` | `log list / on <name> [N] / off` 解析与分派 | 3-4/3-7 | 无 |
| 3-9 | `rust/framework/shell/lib.rs` | `log_inf!/log_err!/log_ok!/log_wrn!/log_dbg!` 宏 + `Log` 门面；声明 log 模块 | 3-1~3-8 | 无 |

**P3 验收**：单测覆盖"三档挤帧顺序、Event 不可挤、过期 DBG 跳过、DBG 默认静默、分频计数、命令行直发"。

---

### P4 framework/shell（7 步）

> **P4 拆成 7 步，每次只执行一步。**
> 用户说"开始第 N 步"= 只改那一个文件，改完停下报告。

| 步 | 文件 | 动作 | 依赖前置 | 连带 |
|---|---|---|---|---|
| 4-1 | `rust/framework/shell/token.rs` | `skip_space/next_token/SubCmdEntry/dispatch_sub_cmd`（原地切分、连续空格按一个） | 1-3 | 无 |
| 4-2 | `rust/framework/shell/shell_cmd.rs` | 命令族表：`inventory::collect!` + `register_shell_cmd!` 宏 + `process_line` + `print_help` | 4-1 | 无 |
| 4-3 | `rust/framework/shell/shell.rs` | 行编辑状态机：可见字符/退格/Ctrl-U/Ctrl-D/方向键/Delete/Home/End、历史 16 条与草稿、超 255 丢弃、CRLF 去重 | 4-2 | 无 |
| 4-4 | `rust/framework/shell/var/var.rs` | 变量注册（定宽类型 + 范围 + setter 通知）、`var list/get/set`、数值校验 | 4-1 / 3-7 | 无 |
| 4-5 | `rust/framework/shell/boot/boot_cmd.rs` | `boot`：输出每项 ns/ok 与总计 | 4-2 / 2-7 | 无 |
| 4-6 | `rust/framework/shell/thread/thread_cmd.rs` | `thread list/show/pause/resume/exit/reset`，对接 `registry` 与 `stats` | 4-2 / 2-6 | 无 |
| 4-7 | `rust/framework/shell/lib.rs` | shell 线程本体：`poll(stdin, log wake_fd)`、EOF 退化、`StopRequested → Exit`、命令间推进发送泵；导出 | 4-1~4-6 | 无 |

**P4 验收**：用 pty 单测驱动：输入 `h`/`var list`/`thread list`/`boot`/`log on x 2` 输出与 C++ 基线逐字对齐（除程序名）。

---

### P5 framework/topic（3 步）

> **P5 拆成 3 步，每次只执行一步。**
> 用户说"开始第 N 步"= 只改那一个文件，改完停下报告。

| 步 | 文件 | 动作 | 依赖前置 | 连带 |
|---|---|---|---|---|
| 5-1 | `rust/framework/topic/channel.rs` | `Channel<T>`：槽 + 序号 + `closed/valid` + `Mutex/Condvar`；`Subscriber`：`wait/wait_timeout/take_latest/align_to_latest/taken/dropped` | 1-4 | 无 |
| 5-2 | `rust/framework/topic/ecat_rx/ecat_rx.rs` | `ecat_rx::Message{data:[u8;256]}`、`ecat_tx::Message{data, received_at}` 与两个全局通道（feature `ecat_rx`） | 5-1 | 无 |
| 5-3 | `rust/framework/topic/lib.rs` | 声明模块并导出 | 5-1/5-2 | 无 |

**P5 验收**：单测覆盖"新消息唤醒、超时返回 false、关闭唤醒、`AlignToLatest`、覆盖丢帧计数"。

---

### P6 framework/drivers（EtherCAT，5 步）

> **P6 拆成 5 步，每次只执行一步。**
> 用户说"开始第 N 步"= 只改那一个文件，改完停下报告。
> **后端实现依赖 P0-3 结论（拍板 A）。**

| 步 | 文件 | 动作 | 依赖前置 | 连带 |
|---|---|---|---|---|
| 6-1 | `rust/framework/drivers/communication/ethercat/slave.rs` | `SlaveInfo{...}`、`SyncMode{FreeRun,SmSync,Dc}`、`Config{interface_name,sync_mode,rx_timeout_us}` | 1-5 | 无 |
| 6-2 | `rust/framework/drivers/communication/ethercat/ecat.rs` | 对外类型与错误枚举；`Ecat<Out,In>` 骨架（尺寸/偏移校验只比 `size_of`） | 6-1 | 无 |
| 6-3 | `rust/framework/drivers/communication/ethercat/pdo.rs` | `trait Pdo`（`Out`/`In` 关联类型 + 定长字节视图）；泛型 IO 语义 | 6-2 | 无 |
| 6-4 | `rust/framework/drivers/communication/ethercat/soem.rs` | soem-rs / bindgen 后端：Open（自动找有载波非回环非无线网口）、Discover、Enumerate、ConfigureSlaves（SM0/SM1 非零）、ConfigurePdo（4096 IOmap、SM2/SM3 重写、`expected_wkc`）、ConfigureDc、RequestState（WKC=0 立即重写、真实时钟 1000 ms、切 OP 期间持续收发）、RequestOperational、Recover、Read（WKC+AL 状态、100 ms 轮询、边沿日志）、Send（io_map 拷贝、发送间隔告警 20 ms）、Close（幂等） | 6-3 / 0-3 | 若走 bindgen：本步含 `build.rs` 链接系统 SOEM |
| 6-5 | `rust/framework/drivers/lib.rs` | 声明模块并导出（feature `ethercat`），删占位 | 6-4 | 无 |

**P6 验收**：真机进 OP；`send/read` 周期与 C++ 版一致；拔线/退 SAFE-OP 的日志与恢复路径与基线一致。

---

### P7 framework/cmd + framework/algorithm + framework/modules（7 步）

> **P7 拆成 7 步，每次只执行一步。**
> 用户说"开始第 N 步"= 只改那一个文件，改完停下报告。

| 步 | 文件 | 动作 | 依赖前置 | 连带 |
|---|---|---|---|---|
| 7-1 | `rust/framework/cmd/config/config.rs` | `Config::load(path, required, optional)`（校验字段集合与重复）、`has/get_int(min,max)/get_double(有限+范围)/get_bool/get_string/get_path`；根路径 = `DUST_CONFIG_ROOT` 或当前工作目录 | 1-6 | 无 |
| 7-2 | `rust/framework/cmd/lib.rs` | 声明 config 模块并导出（feature `config`） | 7-1 | 无 |
| 7-3 | `rust/framework/algorithm/math/angle.rs` | 角度卷绕/比较 | 1-7 | 无 |
| 7-4 | `rust/framework/algorithm/filter/kalman/kalman_filter.rs` | 线性卡尔曼（与 `kalman_filter.hpp` 同算法） | 7-3 | 无 |
| 7-5 | `rust/framework/algorithm/filter/kalman/ekf.rs` | 扩展卡尔曼 | 7-4 | 无 |
| 7-6 | `rust/framework/algorithm/filter/attitude/quaternion_attitude.rs` | 四元数姿态 | 7-3 | 无 |
| 7-7 | `rust/framework/algorithm/lib.rs` + `rust/framework/modules/lib.rs` | 算法模块导出（features）；modules 保持空骨架说明 | 7-3~7-6 | 无 |

> 说明：当前骨架线程未引用 algorithm；若拍板"不重写 algorithm"，7-3~7-7 跳过。

---

### P8 project（5 步）

> **P8 拆成 5 步，每次只执行一步。**
> 用户说"开始第 N 步"= 只改那一个文件，改完停下报告。
> Task 只写正常路径；错误一律 `TaskResult::Retry`；补救进钩子。

| 步 | 文件 | 动作 | 依赖前置 | 连带 |
|---|---|---|---|---|
| 8-1 | `rust/project/protocol.rs` | CRC16（0xFFFF/0x8408）、`read_u16`、`write_float`、命令包 15 B 组装、状态包 20 B 解析、`StatusPacket` | 1-9 | 无 |
| 8-2 | `rust/project/thread/test/trd_test.rs` | 自检线程：10 ms、`log_dbg!("test", ...)`、`Isolate`；`LateInit/High/HaltOnFail` + `LateThread` 注册等价物 | P2/P3 | 无 |
| 8-3 | `rust/project/thread/ethercat/trd_ethercat.rs` | 1 ms 周期：`is_ready` 检查、`update_command`（var 速度 → 命令包；ROS 命令 100 ms 内原样下发，超时置零；CRC）、`send/read`、状态包解析与 DBG、发布 `ecat_rx`；`init` 循环建链到成功；`on_retry`：先 `recover` 再 `close+init`；注册 `ecat.linear_x/linear_y` var；`Recover` 策略 | P2~P6 | 无 |
| 8-4 | `rust/project/thread/ros2/trd_ros2.rs` | 独立 Context、节点 `dust_ecat_bridge`、pub `/ecat/rx_pdo`、sub `/ecat/tx_pdo`（KeepLast(1)/best_effort/volatile）、`spin_some(1ms)`、`wait(1ms)`、长度错误发布失效标记、1 s 限频日志、`Isolate`、`ContinueOnFail` + `Release` | 8-3 / 拍板 B | 若拍板选独立进程：本步改为 IPC 客户端 |
| 8-5 | `rust/project/main.rs` | 装配：`init::startup::run()`（信号→阶段启动→等待停止→关闭）；features 把三个线程编进注册表 | P2 / 8-2~8-4 | 无 |

**P8 验收**：`cargo run -p project --features ethercat,ros2` 真机跑通；shell 命令集与基线一致；ROS 2 两端 256 B 互通。

---

### P9 影子对照（3 步）

> **P9 拆成 3 步，每次只执行一步。**
> 用户说"开始第 N 步"= 只改那一个文件，改完停下报告。

| 步 | 文件 | 动作 | 依赖前置 | 连带 |
|---|---|---|---|---|
| 9-1 | `scripts/shadow_run.sh` | 分时启动 `dust_app` 与 `dust_rs`，各落一份日志 + `thread list`/`boot` 快照 | P8 | 无 |
| 9-2 | `scripts/shadow_compare.py` | 对比：启动顺序与耗时、线程统计、PDO 周期与 WKC、shell 输出、ROS 2 收发 | 9-1 | 无 |
| 9-3 | `doc/Rust重写_验收报告.md` | 差异清单 + 结论 + 是否放行 P10 | 9-2 | 放行由用户拍板 |

---

### P10 收尾：切换构建入口、清理 C++（4 步）

> **P10 拆成 4 步，每次只执行一步。**
> 用户说"开始第 N 步"= 只改那一个文件，改完停下报告。
> **P9 放行前，C++ 树一行不删。**

| 步 | 文件 | 动作 | 依赖前置 | 连带 |
|---|---|---|---|---|
| 10-1 | `framework/cmd/build/build.sh` | 改调 `cargo build --release --manifest-path rust/Cargo.toml`，产物 `rust/target/release/dust_rs`；保留 `--clean` | P9 放行 | 旧 CMake 入口可选保留 `--legacy` |
| 10-2 | `framework/cmd/build/{run.sh,dust}` | 运行路径与 help 改 Rust 口径；保留 ROS 2 setup 与 sudo 提权逻辑 | 10-1 | 无 |
| 10-3 | （目录） | 删除 C++ 源：`framework/**` 的 `.cpp/.hpp` 与各 `CMakeLists.txt`/`Kconfig`、根 `CMakeLists.txt`、根 `Kconfig`、`scripts/kconfig.py`、`framework/cmd/linker/`、`project/`（`CMakeLists.txt`/`Kconfig`/`build/`）；保留 `doc/`、`temp/`、`rust/` | 10-2 | 删完必须 `cargo build` + 真机冒烟 |
| 10-4 | `README.md` / `快速开始.md` | 构建、目录、线程写法、门禁（features）全部改 Rust 口径 | 10-3 | 无 |

---

## 4. 语义对齐清单（重写时必须逐条照抄）

### 4.1 线程外壳

- `period == 0` → 外壳不 sleep，任务自己阻塞（shell 用 `poll`，ros2 用 `spin_some`）。
- `Retry` → `retry_hook`；钩子返回 `Restart` 继续，`Stop` 或未设钩子 → `DefaultHandle`。
- `Exit` → 先判 `StopRequested()`，是停机路径就不算故障；否则 `DefaultHandle`。
- `keep_alive == true` 时 `Exit()` 拒绝并打 WRN。
- 收尾：`exit_hook` → 注销注册表 → `thread %s exited` WRN。
- 暂停在拍间生效，暂停中收到停止要能醒来退出。
- 线程名 ≤ 15 字节（`pthread_setname_np` 限制）。

### 4.2 启动器

- 阶段顺序与同阶段 `High → Mid → Low`。
- `HaltOnFail` → `System_RequestStop(1)`；`ContinueOnFail` → 继续。
- 每项记 `BootTimingSlot{name, ns(u64), ok}`。
- 关闭：逆序先 `request_stop` 全部，再逆序 `join`，每一步检查 `failed` 并带错误码退出。
- 初始化 `cleanup` 必须支持"部分初始化"和"重复调用"。

### 4.3 日志

- 通道前缀：`[inf] ` / `[err] ` / `[ok] ` / `[wrn] ` / DBG 无前缀 / 命令无前缀。
- 颜色：黑 `0x000000`、红 `0xF50002`、绿 `0x00F700`、橙 `0xF6A753`、薄荷 `0x7BF7CD`。
- 帧池 8 × 320 B；记录 512 B；Event 永不被挤；插入按优先级稳定。
- DBG 条目上限 64；`divisor<=1` 短路放行；选择/取消递增 `epoch` 作废在途帧。
- 输出到 **stderr**；`stdout` 留给终端输入交互。

### 4.4 EtherCAT

- 单线程调用，SOEM context 非重入（Rust 用 `!Send` 包裹或注释锁死）。
- 自动选网口：有 `IFF_RUNNING`、非回环、非 `wl*`。
- `io_map` 4096 B；`expected_wkc = outputsWKC*2 + inputsWKC`。
- `Send`：长度/偏移校验 → memcpy → 发送间隔测量（>20 000 µs 报 ERR）→ `ecx_send_processdata`。
- `Read`：`recv` → WKC 校验 → 每 100 ms 或 WKC 异常时读 AL Status/Code → 故障边沿 ERR / 恢复边沿 OK。
- `RequestState`：写 ALCTL 失败立即重写（1 ms 间隔、1000 ms 真实时钟上限）；写完读回 ALCTL 作落地证据；随后持续 `send/recv` + `statecheck`。
- `Recover`：有错误位先 `SAFE-OP|ACK` → 请求 OP → 读回 AL Status 确认。
- `Close` 幂等，并清空 slaves/pdo/stats。

### 4.5 业务协议与线程

- PDO 结构双向各 256 B（Rust `const` 断言）。
- 命令包：`0x6B` + float x + float y + float z(=0) + CRC16（前 13 B）。
- 状态包：`0x6D` + 17 B + CRC16（前 18 B），共 20 B。
- ROS 命令：100 ms 内原样下发；过期/无效 → 零速度并打一次 WRN。
- ROS 桥：独立 Context；`UInt8MultiArray` 256 B；长度不符 → 发布失效标记。
- 自检线程：10 ms、DBG 名 `test`、`Isolate`。
- var：`ecat.linear_x`、`ecat.linear_y`。

### 4.6 构建/运行

- `build.sh`：根目录 → `project/build`，`--clean`、`--config`。
- `run.sh`：source `/opt/ros/humble/setup.bash`（关 nounset）；非 root 提权；无产物时提示先 build。
- `dust build|run|help` 分发语义不变。

---

## 5. 风险与坑

1. **soem-rs 覆盖率**：§3.1 的接口 + `ecx_contextt` 内部字段暴露程度决定能不能直接用。缺字段就自建 `bindgen`，`unsafe` 全部收敛到 `soem.rs`。
2. **SOEM 裸指针生命周期**：`ecx_contextt` 与创建线程绑定；Rust 用独占结构体 + `unsafe impl Send`（注释写清"仅创建线程使用"），禁止跨线程共享。
3. **WKC=0 ≠ 超时**：写失败必须立即重写，Rust 版不许"等一等"。
4. **日志热路径零分配**：`log_*!` 宏格式化进定长缓冲，任何 `String`/`Vec` 分配都是回归。
5. **注册表容量**：线程 16、DBG 64、帧 8，均定长；`inventory` 在启动期收集，不进热路径。
6. **features 组合**：`ros2` 隐含 `ethercat`；`log` 隐含 `console`；关闭 `console` 时 `project` 仍须可编（对应 C++ 无 shell 也能链）。
7. **ROS 2**：`r2r` 需要 colcon/ament；不成熟就改独立进程，Rust 侧只留 IPC 客户端。
8. **实时性**：默认不加实时属性；`--rt` 才 `mlockall` + `SCHED_FIFO`（需 `CAP_SYS_NICE`/root，`run.sh` 已有 sudo 逻辑）。
9. **编译/运行是用户动作**：所有 `cargo build`、真机验收由用户执行。
10. **P10 之前 C++ 只读**：禁止两套实现同时改功能，避免对照失效。
11. **同名路径不冲突**：`rust/framework/**` 与仓库根 `framework/**` 是两套树，Rust 侧不复制 C++ 源，也不在原目录生成 Cargo 文件（P10 前）。

---

## 6. 总验收清单

- [ ] P0 三份文档落地（现状清单 / 行为基线 / 依赖核实）
- [ ] `cargo build --workspace` 通过（默认 features 与全 features 两种组合）
- [ ] `cargo fmt --check`、`cargo clippy --workspace --all-targets -- -D warnings` 全绿
- [ ] 单测覆盖：线程外壳 / 注册表 / 启动顺序 / 三档日志 / 通道 / 协议 CRC
- [ ] shell 命令集（h/var/log/thread/boot）输出与基线逐字一致
- [ ] EtherCAT 进 OP，周期、WKC、故障边沿日志与基线一致
- [ ] ROS 2 双向 256 B PDO 互通
- [ ] 影子对照报告出具并经用户放行
- [ ] `build.sh` 切到 cargo，C++ 源清理完成，README 改口径

---

## 7. 待拍板

| # | 项 | 推荐 | 影响面 |
|---|---|---|---|
| A | EtherCAT：soem-rs vs 自建 bindgen | **先验 soem-rs，缺字段/接口即 bindgen** | P0-3 / P6-4 |
| B | ROS 2：r2r vs 独立进程 IPC | **r2r 先试，不成熟即退回独立进程** | P0-3 / P8-4 |
| C | 注册机制：inventory vs 显式注册表 | **inventory**（保持"写一行即注册"） | P2-7 / P4-2 / P4-4 |
| D | 门禁：Cargo features vs 运行期配置 | **Cargo features**（对齐 Kconfig 编译期语义） | P1 / 全阶段 |
| E | YAML 库 | **serde_yaml**（或 `serde_yaml_ng`） | P7-1 |
| F | 实时性 | **默认关，`--rt` 开启** | P6 / P8-5 |
| G | algorithm 是否重写 | **重写**（纯算法、低成本） | P7 |
| H | 线程/DBG/帧池容量 | **保持 16 / 64 / 8** | P2-6 / P3-3 / P3-4 |
| I | P10 后 Rust 树是否提为仓库根（`framework/**` 原位替换） | **先保留 `rust/` 一层，下一版再说** | P10 |
| J | C++ 树保留到何时 | **P9 放行后才删** | P9 / P10 |

> 执行前把这 10 条一句话确认（或"按推荐来"）即可开工。

---

## 8. 执行纪律

1. **一步一文件**：说"开始第 N 步"只做一步，做完停，等"继续"。
2. **中间态提前标注**：P1 的 1-1~1-8 编不过是设计内的，见「连带」栏。
3. **C++ 树只读**：P10 之前一行不删、不改功能。
4. **编译/真机由用户执行**：执行侧只改代码、给命令与判据。
5. **不许 git 回滚**：改回一律用编辑，不用 checkout/restore。
6. **失败不猜**：任何一步报错，贴真实输出停下，不编原因、不擅自"修到过"。
