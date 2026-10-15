# Kconfig 构建链路引入规划

日期：2026-10-15。

对象：当前 Linux 视觉工程的**构建期**功能选择链路 —— Kconfig 描述 → `.config` 生成 → CMake 装配 → 编译宏。

参考：`temp/Dust_Zephyr_Tree` 的 Kconfig 语义（业务层 `select` 架构能力、架构层默认 `n`）与它 `project/CMakeLists.txt` 的消费分工（CMake 只消费结果、不自己判断依赖）。

状态：**Kconfig 描述文件已在位，链路只接了半条。** 没有 Kconfig 前端、没有 `.config` 的生成路径、没有生成头；CMake 侧靠一个手写正则块把 `.config` 映射成 cache 变量。运行期那部分（YAML → typed `Config`）见 [视觉配置架构引入规划.md](视觉配置架构引入规划.md)，本规划不动它。

---

## 1. 依据来源与边界

**取自参考：**

- Kconfig 语义：`bool` + `default` + `select` + `depends on`，业务层门禁 `TRD_*`、架构层能力 `DUST_*`、架构层默认 `n` —— [参考 project/Kconfig](../temp/Dust_Zephyr_Tree/project/Kconfig)、[参考 framework/algorithm/Kconfig](../temp/Dust_Zephyr_Tree/framework/algorithm/Kconfig)。
- 分工：Kconfig 负责 option、默认值、依赖和 `select`；CMake 负责源文件裁剪、链接依赖和编译宏，不重复判断依赖 —— [参考 project/CMakeLists.txt](../temp/Dust_Zephyr_Tree/project/CMakeLists.txt)（`find_package(Zephyr)` 之后按 `CONFIG_*` 做 `add_subdirectory`）。
- 前端选型：Kconfig 语法要解析 `depends on`、`select`、`default` 并求解出确定值，必须有前端；本项目用 `kconfiglib`（Zephyr 的 `scripts/kconfig` 同样建立在它之上）。不自己写解析器。

**不取：** Zephyr 本体、设备树、`boards/*.conf` / `.overlay` / `board.cmake` 那套平台层（Linux 工程没有对应物）；`prj.conf` 里的 Zephyr 系统项（C++/FPU/堆栈）也不搬 —— 对应物是根 `CMakeLists.txt` 已有的 CMake 标准与编译选项。

**边界：** 本规划只动构建期 —— 三个 Kconfig 文件、一个生成脚本、根/层 `CMakeLists.txt`（+ 生成物落点）。算法、话题、线程运行参数、`trd_vision.yaml`、算法类里的 `Config` 一行不动；`DUST_CONFIG_ROOT` 那套运行时路径解析也不碰。

**前置决策（必须你先定）：** 本机现在**没有** kconfiglib（见 §2），而我不下载、不出仓库（这条已进记忆）。所以链路第一环从哪来（`pip install kconfiglib` / 由你放一份进仓库 / 换别的前端）由你定；不定这一项，阶段 1 无法开工，链路就只能停在现状。

---

## 2. 现状：实测数字

工具链（2026-10-15 本机实测）：

| 项 | 实测值 |
| --- | --- |
| CMake | 3.28.3 |
| Python | 3.12.3 |
| `import kconfiglib` | `ModuleNotFoundError: No module named 'kconfiglib'` |
| `conf` / `menuconfig` / `guiconfig` | PATH 中不存在 |
| 仓库内 `.config` / `defconfig` / `configs/` | 不存在 |

Kconfig 侧（[Kconfig](../Kconfig)、[framework/algorithm/Kconfig](../framework/algorithm/Kconfig)、[project/Kconfig](../project/Kconfig)）：共 **5** 个 `config` —— `PRJ_MAIN`、`TRD_VISION`、`TRD_DISPLAY`、`DUST_VISION`、`DUST_VISION_REFINER`；依赖关系 **3** 条：`TRD_VISION` → `select DUST_VISION`、`DUST_VISION_REFINER` → `depends on DUST_VISION`、`if PRJ_MAIN` 包住后两者。

CMake 侧（[CMakeLists.txt](../CMakeLists.txt)）：

- **13–45 行是一整块手写 Kconfig 解析**（33 行）：`file(STRINGS ...)` 读 `.config`，对硬编码的 4 个符号逐个 `MATCHES "^CONFIG_${符号}=y$"`，再走 `if/elseif` 把结果写回 cache。
- 硬编码符号表（第 20 行）：`TRD_VISION DUST_VISION DUST_VISION_REFINER TRD_DISPLAY` —— **4 个，比 Kconfig 里少 `PRJ_MAIN`**。
- 第 43–45 行再用 `set(CONFIG_*)` 从 `option()` 镜像出 `CONFIG_*` 供消费点使用；消费点 8 处：`CMakeLists.txt:51,55`、[framework/CMakeLists.txt:7,13,21](../framework/CMakeLists.txt#L7)、[project/CMakeLists.txt:7,9,16](../project/CMakeLists.txt#L7)、[project/thread/CMakeLists.txt:7,11](../project/thread/CMakeLists.txt#L7)、[project/thread/vision/trd_vision.cpp:13,277,329,337](../project/thread/vision/trd_vision.cpp#L13)。
- C++ 侧唯一的编译宏 `CONFIG_DUST_VISION_REFINER` 由**手写** `target_compile_definitions` 提供，共 2 处（[framework/CMakeLists.txt:22](../framework/CMakeLists.txt#L22)、[project/CMakeLists.txt:10](../project/CMakeLists.txt#L10)），没有任何生成头。

---

## 3. 病灶与缺口

| # | 病灶 | 怎么算出来的 | 现状量 | 目标量 |
| --- | --- | --- | --- | --- |
| 1 | **没有前端，`.config` 没有合法来源** | `python3 -c "import kconfiglib"` 失败、`which conf menuconfig` 为空、仓库内无 `.config`/`defconfig` → 只能手写 | 生成路径 0 条 | 1 条（`defconfig`/`menuconfig` → `.config`） |
| 2 | **手工解析只认 `=y`** | 第 23 行正则 `^CONFIG_${符号}=y$`：`=n`、`=m`、`# CONFIG_X is not set`、字符串、十六进制都不匹配；Kconfig 值类型共 5 类（bool/tristate/int/hex/string） | 类型覆盖 1/5 | 5/5 |
| 3 | **符号表硬编码，加开关要改多处** | 第 20 行的 `IN ITEMS` 是唯一入口名单，Kconfig 里改了符号它不会跟着变；`PRJ_MAIN` 就是这么漏掉的 | 识别 4/5；新增一个开关要动 **4** 处（Kconfig 定义、`IN ITEMS`、`set(CONFIG_*)` 映射分支、消费点） | 5/5；触点 **2** 处（Kconfig 定义、消费点） |
| 4 | **Kconfig 的依赖语义在 CMake 里没人执行** | 3 条依赖关系里，CMake 只手写复刻了 1 条（第 34–36 行的 `TRD_VISION AND NOT DUST_VISION` → `FATAL_ERROR`）；`DUST_VISION_REFINER depends on DUST_VISION` 与 `if PRJ_MAIN` 都没有对应代码 | 依赖执行覆盖 1/3 | 3/3（由前端求解 + 1 条兜底断言） |
| 5 | **符号有效消费率低** | 5 个符号里，真正决定编译/链接的是 3 个（`TRD_VISION`、`TRD_DISPLAY`、`DUST_VISION_REFINER`）；`DUST_VISION` 只被当断言用、`CONFIG_DUST_VISION` 在任何 `CMakeLists.txt` 中都不出现；`CONFIG_PRJ_MAIN` 全仓库无人读 | 有效 gate 3/5 = 60% | 5/5（或把不需要的符号删掉，不留死符号） |
| 6 | **一个开关两个真源** | `option(TRD_VISION ...)` 与 `set(CONFIG_TRD_VISION ...)` 手工同步（第 8–10、43–45 行）；`KCONFIG_CONFIG` 为空时走 option、非空时走 `.config`，两条路都改得动同一个量 | 真源 2 个 | 1 个（`.config`） |
| 7 | **编译宏与 Kconfig 结果无强制一致性** | `CONFIG_DUST_VISION_REFINER` 宏由手写 `target_compile_definitions` 给出，`.config` 里写 `n` 而选项默认 `ON` 时会出现"宏开着、配置写着关" | 手写宏 2 处 | 生成头 1 份 |

配置组合空间（用于验收枚举）：3 个业务/能力符号 → 2³ = **8** 种组合；按 `depends on` 排除 `REFINER=y ∧ DUST_VISION=n` 的 **2** 种，合法 profile **6** 种。

---

## 4. 方法学约束

- 不出仓库、不下载任何东西：kconfiglib 的获取方式等你拍（§1 前置决策），我不自己装。
- 只动本规划点名的文件；算法、话题、运行时 YAML 一行不碰。
- 一次只做他点名的那一步，做完报数字，不顺手把下一步也做了。
- 不顺手重排目录、不做格式化大清洗；生成物一律落在 `build/`（`.gitignore` 已忽略，无需改）。
- 不新开平行入口：Kconfig 文件继续是根 `Kconfig` + `framework/algorithm/Kconfig` + `project/Kconfig` 这三个，不加第二个总入口。

---

## 5. 目标链路

```text
configs/<profile>_defconfig                       ← 版本库内的具名配置（唯一真源）
        │  kconfiglib：载入 Kconfig 树，求解 default / depends on / select
        ▼
build/kconfig/.config                              ← 规范化结果（含 "# CONFIG_X is not set"）
        │  scripts/kconfig/gen_config.py（kconfiglib API）
        ├──► build/kconfig/autoconf.h              ← C/C++ 编译宏，替代手写 target_compile_definitions
        └──► build/kconfig/kconfig.cmake           ← set(CONFIG_* ...)，替代 33 行手写解析块
                        ▼
              CMake：if(CONFIG_TRD_*) 装配源文件与依赖（线程归 TRD_，算法能力归 DUST_）
```

各环节职责：

| 环节 | 负责 | 不负责 |
| --- | --- | --- |
| Kconfig 文件 | 符号、默认值、`select`、`depends on` | 源文件、链接库、`-D` 宏 |
| `defconfig` | 一次构建要打开哪些符号 | 运行参数（门限、路径、设备号） |
| `gen_config.py` | 把 `.config` 转成 CMake 片段与 C 头 | 判断依赖（依赖已由前端求解） |
| CMake | 按 `CONFIG_*` 裁剪源文件、链接、include 生成头 | 自己解析 `.config` 文本、自己复刻依赖 |

---

## 6. 分步骤

### 阶段 0：冻结基线（只读）

**做什么：** 把 §2 的数字和消费点固定成清单，作为改动前后的对照。

**落点：** 无（只读 `rg`）。

**验收：** 能一条命令列出全部 Kconfig 相关消费点，数量与 §2 一致（CMake 侧 8 处 + C++ 侧 4 处 `#ifdef`）。

**失败分叉：** 数字对不上 → 先只更新清单，不进阶段 1。

### 阶段 1：前端与 `.config` 生成

**做什么：** 确定 kconfiglib 来源（前置决策）→ 写 `configs/vision_defconfig`（当前默认组合：`TRD_VISION=y`、`TRD_DISPLAY=y`、`DUST_VISION_REFINER=y`）→ 用 `menuconfig`/`defconfig` 产出一份规范化 `.config`。

**落点：** `configs/vision_defconfig`（新增，进版本库）、`build/kconfig/.config`（生成物）。

**算法：** Kconfig 求解不是"读文本赋值"，而是按依赖图迭代：`select` 向上强制、`depends on` 向下门控、`default` 取值，最终每个符号得到确定值；被关掉的 bool 写成 `# CONFIG_X is not set`，也是合法结果（区别于"没这份配置"）。

**验收数字：** `.config` 里出现 **5** 个符号名；`CONFIG_DUST_VISION=y`（被 `select` 强制）、`CONFIG_DUST_VISION_REFINER=y`（`default y if DUST_VISION`）；在 `menuconfig` 里改任一开关，`.config` 的 diff 只动那一个符号。

**失败分叉：** 前端拿不到 → 本规划暂停在现状，明确记录"链路缺第一环"，不退回手搓解析器。

### 阶段 2：生成物接进 CMake（同一步完成两件，避免双真源漂移）

**做什么：** 写 `scripts/kconfig/gen_config.py`（kconfiglib API：载入 Kconfig 树 → `load_config(.config)` → `write_config()` 规范化 → 遍历 `kconf.syms`，`tri_value` 出 bool、`str_value` 出 int/hex/string，生成 `kconfig.cmake` 与 `autoconf.h`）；根 `CMakeLists.txt` 在配置阶段 `execute_process` 调它，然后 `include(build/kconfig/kconfig.cmake)` **替换掉 13–45 行的手写块**，并删除三个 `option()`；C++ 侧改为 include 生成头。

**落点：** `scripts/kconfig/gen_config.py`（新增）、`CMakeLists.txt`（删手工块与 option、加 `execute_process` + `include` + `include_directories`）、`framework/CMakeLists.txt:22`、`project/CMakeLists.txt:10`（删手写 `target_compile_definitions`）。

**验收数字：** 根 `CMakeLists.txt` 里 Kconfig 相关行数 **33 → ≤8**；符号识别 **4/5 → 5/5**；手写编译宏 **2 处 → 0**；新增一个开关的触点 **4 → 2**。

**失败分叉：** 生成期报错 → 保留 `-DKCONFIG_CONFIG=<path>` 直接喂现成 `.config` 的入口（现状入口不废），先让构建不断，再修生成器。

### 阶段 3：依赖与负例收口

**做什么：** `framework/CMakeLists.txt` 的视觉目标门禁从 `CONFIG_TRD_VISION` 改为 `CONFIG_DUST_VISION`（落实"线程归 `TRD_`、算法能力归 `DUST_`"）；根 CMake 保留一条兜底断言（手工改过的 `.config` 里出现 `REFINER=y ∧ DUST_VISION=n` 时 `FATAL_ERROR`）；配置结束打印 feature summary。

**落点：** `framework/CMakeLists.txt`、`CMakeLists.txt`。

**验收数字：** 3 条依赖关系全部可执行（1/3 → 3/3）；非法 `.config`（`REFINER=y` + `DUST_VISION=n`）在配置阶段就报错、不进入编译；`CONFIG_TRD_VISION=OFF` 时配置阶段不出现 CUDA/TensorRT 查找日志。

**失败分叉：** 改了门禁出现未定义注册符号/链接缺源 → 回退这一条改动（阶段 2 的生成链路不受影响）。

### 阶段 4：profile 化

**做什么：** 除主 `defconfig` 外加一份"关视觉"配置用于验收枚举；`.config` 由 profile 生成，不再手写。

**落点：** `configs/`（第二份 defconfig）。

**验收数字：** 6 种合法组合里至少验 3 种：`vision on + refiner off`、`vision on + refiner on`、`vision off + display on`；每种都能配置成功且源文件只被加入一次。

**失败分叉：** 某组合失败 → 只保留主 profile，把该组合记成已知限制，不硬凑。

### 阶段 5：文档收口

**做什么：** README 增加"配置构建"一段（Kconfig → `.config` → CMake 的入口命令）；把 [视觉配置架构引入规划.md](视觉配置架构引入规划.md) §5 的 CMake option 描述改为引用本规划，避免两份文档同时当规范。

**验收：** 按 README 的命令能从零配置出一份 `.config` 并构建；文档里不再出现 `-DTRD_VISION=OFF` 这种已删除的入口。

---

## 7. 顺序为什么不能换

1. **阶段 2 不能早于阶段 1**：手写解析块一删，`.config` 必须已经有合法来源，否则连现状都构建不了。
2. **阶段 2 的两件事必须同步**：只删手写解析、不改编译宏来源，`CONFIG_DUST_VISION_REFINER` 就会出现"`.config` 说关、宏说开"的第二真源，比现状更难查。
3. **阶段 3 不能早于阶段 2**：依赖断言要靠生成链路给出可信的 `CONFIG_*`；在手工解析下改门禁，等于让断言建在一个只认 `=y` 的解析器上。
4. **阶段 4 不能早于阶段 3**：profile 只是同一份 Kconfig 的不同取值；依赖语义没落地前做 profile，等于把非法组合固化进版本库。
5. **阶段 5 最后**：文档里的命令必须在链路稳定后才写，否则要改两遍。

---

## 8. 失败回退表

| 症状 | 先查 | 再查 | 最后才查 |
| --- | --- | --- | --- |
| 配置阶段报 `source` 找不到 | 根 `Kconfig` 里 `source "framework/algorithm/Kconfig"` 的相对路径基准（相对包含它的文件） | 从哪个目录调用生成脚本（工作目录） | 要不要改用绝对路径 |
| CMake 拿不到 `CONFIG_*`（变量为空） | `kconfig.cmake` 是否生成、路径是否对 | `execute_process` 是否在 `include` 之前、是否失败被忽略 | CMake 缓存里残留的旧 `CONFIG_*`（删 `build/CMakeCache.txt` 重配） |
| 改了 defconfig 但构建行为不变 | `.config` 是否真的重新生成（时间戳） | `include` 的是不是缓存路径 | 是否还有别的 `set(CONFIG_*)` 真源残留 |
| 关闭视觉仍链接 TensorRT | `framework_vision` 的门禁条件是 `TRD_` 还是 `DUST_` | 生成头/宏的来源是否唯一 | 遗留的 `target_compile_definitions` |
| 注册符号未定义 / 线程没跑 | 该线程源文件是否被条件加入 | 生成宏与源文件条件是否同一个符号 | 链接脚本 `--gc-sections` 与注册段保留 |
| 前端装不上（离线） | 是否允许把前端放进仓库 | 只用现成 `.config` + 保留手工解析 | 本规划暂停，不手搓解析器 |

---

## 9. 验收

| 验收项 | 目标 |
| --- | --- |
| 生成入口 | 一条命令从 `defconfig` 得到 `.config`；无手工编辑 |
| 值类型 | bool / int / hex / string 都能进 `kconfig.cmake` 与 `autoconf.h`（用一个 int 符号验证后即可移除） |
| 符号覆盖 | Kconfig 里每个符号都有唯一消费者，或删掉；无死符号 |
| 依赖 | 非法组合在配置阶段失败，且不依赖 CMake 手工复刻 |
| 双真源 | `option()` 与手写 `set(CONFIG_*)` 全部删除；`.config` 是唯一真源 |
| 构建边界 | `build/` 只有构建产物与生成物；运行期 YAML/模型仍走外部配置根（见 [视觉配置架构引入规划.md](视觉配置架构引入规划.md) §8.4） |

---

## 10. 明确不做

1. 不把 `temp/Dust_Zephyr_Tree` 或 Zephyr 树复制/引入本工程，也不引入设备树与板卡层。
2. 不自己写 Kconfig 语法解析器（那正是现状这块手工代码的放大版）。
3. 不把运行期参数（engine 路径、门限、CUDA 设备）放进 Kconfig；构建期只决定"有没有这段代码"。
4. 不改算法、话题、线程生命周期、`DUST_CONFIG_ROOT` 与 YAML 语义。
5. 不为 Kconfig 引入图形前端（`guiconfig` 需要额外 GUI 依赖），只用 `menuconfig`/`defconfig` 文本入口。
6. 不顺手重排 `doc/`、`framework/`、`project/` 的目录结构。

---

## 11. 完成定义

链路补齐的判据不是"能生成 `.config`"，而是：

- `.config` 有唯一来源（defconfig/profile），`.config` 是构建期唯一真源；
- Kconfig 里的 `select` / `depends on` 由前端执行，CMake 不再复刻依赖；
- 编译宏来自生成头，不再手写；
- 每个符号要么有唯一消费者、要么被删掉；
- 6 种合法组合里至少 3 种能从零配置成功构建；
- 现状的检测/显示行为与开关组合一一对应，没有"改了开关但程序没变"的情况。
