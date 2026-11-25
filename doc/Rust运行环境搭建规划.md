# Rust 运行环境搭建规划

> 2026-11-21 规划 · 2026-11-22 执行完成（P0~P3）· 状态：**P0~P3 已执行，P4 未做**
> 目标：在当前工作区（`/home/rmconstruction/Desktop/qingyu`）上装好 Rust 运行环境，覆盖**编译（build）、运行（run）、静态检查（fmt / clippy / check）**三件事。
> 前提：本文件只管"装环境 + 建骨架"，**不写任何业务 Rust 代码**。

---

## 0. 背景与目标

### 0.1 现状

- 当前工作区是 Linux C++17 骨架（`project/build` 为构建目录，可执行程序 `dust_app`），构建入口是仓库根 `build.sh` + CMake。
- 机器上**是否已有 Rust 工具链未知**，本规划 P0 第 1 步先探测，不假设。
- 本机到外网（GitHub / crates.io）**直连不稳定**，`cicd-pr-gate-upload` 记录过"代理实际监听 7897、全局配的是 7890 连不上"。Rust 安装与拉 crate 必须把这条考虑进去。

### 0.2 目标（逐条对应可验收）

| # | 目标 | 验收判据 |
|---|---|---|
| 1 | 编译 | `cargo build --release` 成功，产出二进制 |
| 2 | 运行 | `cargo run` 正常输出，退出码 0 |
| 3 | 静态检查 · 格式 | `cargo fmt --check` 退出码 0 |
| 4 | 静态检查 · 语义 | `cargo clippy --workspace --all-targets -- -D warnings` 退出码 0 |
| 5 | 静态检查 · 类型 | `cargo check --workspace` 退出码 0 |
| 6 | 可复现 | 工具链版本、镜像源、构建产物路径全部写死在本文档 |

### 0.3 明确不做（本规划范围外）

- 不写业务 Rust 代码、不引入第三方 crate。
- 不接 FFI（Rust ↔ C++ 互调）、不改 `dust_app` 的链接方式。
- 不动 `~/.cargo`、`~/.rustup` 之外的任何用户级配置；**不写 `~/.ssh/config`、不改全局 git 配置**（历史铁律）。
- 不与 `build.sh` / CMake 集成（P4 列为可选，待拍板）。

---

## 1. 决策与边界

### 1.1 已定（本文档写死，执行时不思考）

| 项 | 定值 |
|---|---|
| 工具链安装器 | `rustup`（官方 `rustup-init.sh`） |
| 频道 | `stable`（不钉具体小版本，装完把实际 `rustc -V` 回填本文档 §4） |
| profile | `default`（自带 `rustfmt` + `clippy`，无需额外装） |
| Rust 工程落点 | 仓库根 `rust/`（Cargo workspace） |
| 包管理 | `cargo`（不手动装 crate） |
| 静态检查工具 | `cargo fmt` / `cargo clippy` / `cargo check`（不引入 `cargo-audit`、`cargo-deny` 等额外工具） |
| 镜像源配置落点 | **仓库内** `rust/.cargo/config.toml`（不动 `~/.cargo/config.toml`） |
| 构建产物 | `rust/target/`，进 `.gitignore` |
| 编译执行者 | **本规划的编译/运行验收命令由用户执行**（本仓库既有规矩：编译是用户动作） |

### 1.2 待拍板（执行前需一句话确认，已给推荐值）

| # | 问题 | 推荐 | 备选 |
|---|---|---|---|
| A | Rust 工放置 `rust/` 还是 `project/rust/` | **`rust/`**（仓库根，与 C++ 构建树 `project/` 解耦，`target/` 不进 CMake 视野） | `project/rust/` |
| B | crate 名字 | **`dust_rs`**（与 C++ 可执行 `dust_app` 区分，不抢名） | `dust`、`app` |
| C | 镜像源 | **rsproxy.cn（sparse）**，失败再回退代理 7897 | 纯代理直连官方源 |
| D | clippy 是否默认 `-D warnings` | **默认不 deny**，只在验收/后续脚本里带 `-D warnings`；库里出现警告时不阻断日常 build | 直接 deny |
| E | P4 是否现在做（挂进 `build.sh`） | **现在不做**，Rust 工程独立 `cargo` 驱动 | 立即集成 |
| F | edition | **2024**（当前 stable 已支持） | 2021 |

---

## 2. 落点结构（执行完应有的样子）

```text
qingyu/
├── build.sh                     ← 不动（P4 才碰）
├── .gitignore                   ← 追加 rust/target/
├── doc/
│   └── Rust运行环境搭建规划.md   ← 本文件
└── rust/                        ← 新建
    ├── .cargo/
    │   └── config.toml          ← crates 镜像（仓库级，不动全局）
    ├── Cargo.toml               ← workspace 根
    ├── Cargo.lock               ← 首次 build 生成，进版本库
    ├── rustfmt.toml             ← 格式检查配置
    ├── clippy.toml              ← 语义检查配置
    └── app/
        ├── Cargo.toml
        └── src/
            └── main.rs          ← 验收用的最小 main
```

---

## 3. 阶段与步表

> **通用纪律（每个阶段都适用）**：用户说"**开始第 N 步**" = 只做表格里那一步（那一个文件），改完停下报告，等"继续"。不许一次做多步，不许跨步连带改。

---

### P0 工具链安装（机器级，不改仓库文件）

> **P0 拆成 3 步，每次只执行一步。**
> 用户说"开始第 1 步"= 只做下表第 1 行，做完停下报告，等"继续"。
> 不许一次做多步；P0 全部是**命令**，不产生仓库文件改动。

| 步 | 文件 | 动作 | 依赖前置 | 连带 |
|---|---|---|---|---|
| 0-1 | （无） | 探测现状：`command -v rustc cargo rustup`、`rustc -V`、`cargo -V`、`rustup show`、`ls ~/.cargo ~/.rustup`；同时确认可达源（见下） | 无 | 已有工具链 → 只补组件（跳 0-2 的下载，直接 0-3） |
| 0-2 | （无） | 用 rustup 安装 stable：下载 `rustup-init.sh` 并执行，`--default-toolchain stable --profile default -y` | 0-1 | 装完 `~/.cargo/env` 出现，需 `source` 生效 |
| 0-3 | （无） | 校验组件：`rustup component list --installed` 必须含 `rustfmt`、`clippy`；缺则 `rustup component add rustfmt clippy` | 0-2 | 无 |

**0-1 具体命令**

```bash
command -v rustc cargo rustup
rustc -V; cargo -V; rustup show
ls -la ~/.cargo ~/.rustup 2>/dev/null
```

**0-2 具体命令（镜像优先，失败回退代理）**

```bash
# 方案 A：rsproxy 镜像（推荐）
export RUSTUP_DIST_SERVER="https://rsproxy.cn"
export RUSTUP_UPDATE_ROOT="https://rsproxy.cn/rustup"
curl --proto '=https' --tlsv1.2 -sSf https://rsproxy.cn/rustup-init.sh -o /tmp/rustup-init.sh
sh /tmp/rustup-init.sh -y --default-toolchain stable --profile default
source "$HOME/.cargo/env"
```

```bash
# 方案 B：方案 A 失败时，改走本机代理（一次性环境变量，不改全局配置）
export https_proxy="http://127.0.0.1:7897"
export http_proxy="http://127.0.0.1:7897"
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs -o /tmp/rustup-init.sh
sh /tmp/rustup-init.sh -y --default-toolchain stable --profile default
source "$HOME/.cargo/env"
```

**P0 编译状态**：不涉及工程编译；本阶段结束时 `cargo --version` 可用即为通过。

**P0 验收**

```bash
rustc -V && cargo -V && rustup show
rustup component list --installed | grep -E 'clippy|rustfmt'
```

---

### P1 工作区 Rust 骨架

> **P1 拆成 4 步，每次只执行一步。**
> 用户说"开始第 N 步"= 只改表格里那一个文件，改完停下报告，等"继续"。
> 不许一次做多步。**第 1～3 步之间 cargo 命令会报错（workspace 成员缺失），这是设计内的中间态，不是出错；第 3 步做完才恢复可编译。**

| 步 | 文件 | 动作 | 依赖前置 | 连带 |
|---|---|---|---|---|
| 1-1 | `rust/Cargo.toml` | 新建 workspace 根：`[workspace] resolver="2" members=["app"]` + `[workspace.package] edition="2024"` | 无 | **此时 `cargo build` 报错**（`app` 不存在），第 1-3 步后恢复 |
| 1-2 | `rust/app/Cargo.toml` | 新建成员包：`name="dust_rs"`、`version="0.1.0"`、`edition.workspace=true` | 1-1 | **仍编不过**（无 `src/main.rs`） |
| 1-3 | `rust/app/src/main.rs` | 新建最小 `fn main() { println!("dust_rs ok"); }` | 1-2 | 本步起 `cargo build` 可编译 |
| 1-4 | `.gitignore` | 追加一行 `rust/target/` | 1-1 | 无（不影响编译） |

**1-1 全文**

```toml
[workspace]
resolver = "2"
members = ["app"]

[workspace.package]
edition = "2024"
version = "0.1.0"
license = "MIT"
```

**1-2 全文**

```toml
[package]
name = "dust_rs"
version.workspace = true
edition.workspace = true

[[bin]]
name = "dust_rs"
path = "src/main.rs"
```

**1-3 全文**

```rust
fn main() {
    println!("dust_rs ok");
}
```

**1-4 old→new**

```diff
  project/build/
+ rust/target/
```

**P1 编译状态**：
- 1-1 后：`cargo` 报 `failed to load manifest for workspace member app`（预期）。
- 1-2 后：报 `no targets specified`（预期）。
- 1-3 后：**恢复可编译**，`cargo build` 通过。
- 1-4 后：编译状态不变（仅版本控制）。

---

### P2 静态检查配置

> **P2 拆成 3 步，每次只执行一步。**
> 用户说"开始第 N 步"= 只改表格里那一个文件，改完停下报告，等"继续"。

| 步 | 文件 | 动作 | 依赖前置 | 连带 |
|---|---|---|---|---|
| 2-1 | `rust/rustfmt.toml` | 新建格式配置：`edition="2024"`、`max_width=100`、`tab_spaces=4` | 1-3 | 无 |
| 2-2 | `rust/clippy.toml` | 新建语义检查配置：`msrv` 留空不写，只写 `too-many-arguments-threshold = 8`（与 C++ 侧宽松习惯对齐） | 1-3 | 无 |
| 2-3 | `rust/.cargo/config.toml` | 新建 crate 源镜像（sparse rsproxy）+ `git-fetch-with-cli = true` | 0-2 | **写错会导致拉 crate 失败**，本步末尾立刻用 `cargo search` 或 `cargo metadata` 验证 |

**2-1 全文**

```toml
edition = "2024"
max_width = 100
tab_spaces = 4
```

**2-2 全文**

```toml
too-many-arguments-threshold = 8
```

**2-3 全文**

```toml
[source.crates-io]
replace-with = "rsproxy-sparse"

[source.rsproxy-sparse]
registry = "sparse+https://rsproxy.cn/index/"

[net]
git-fetch-with-cli = true
```

> 说明：镜像配置放在 **仓库内** `rust/.cargo/config.toml`，只作用于本工程的 cargo 调用，`~/.cargo/config.toml` 一律不动（不动用户环境）。若拍板 C 选"纯代理直连"，则本步改为不建此文件、运行时带 `https_proxy=http://127.0.0.1:7897`。

**P2 编译状态**：全程可编译，配置错误只影响格式化/依赖下载，不影响已有 `cargo build` 结果。

---

### P3 编译 / 运行 / 静态检查验收

> **P3 拆成 4 步，每次只执行一步。**
> 用户说"开始第 N 步"= 只执行表格里那一条命令组，做完停下报告，等"继续"。
> **本阶段不产生任何文件改动**（`Cargo.lock` 与 `target/` 除外，前者进版本库，后者被忽略）。

| 步 | 文件 | 动作 | 依赖前置 | 连带 |
|---|---|---|---|---|
| 3-1 | （无） | 编译：`cd rust && cargo build --release` | 1-3 | 生成 `target/release/dust_rs`、`Cargo.lock` |
| 3-2 | （无） | 运行：`cargo run --release`，期望输出 `dust_rs ok`、退出码 0；再直接跑二进制 `./target/release/dust_rs` | 3-1 | 无 |
| 3-3 | （无） | 格式检查：`cargo fmt --check`，退出码 0；同时 `cargo check --workspace` | 2-1 | 无 |
| 3-4 | （无） | 语义检查：`cargo clippy --workspace --all-targets -- -D warnings`，退出码 0 | 2-2 | 若报 warning，只改 `main.rs` 的问题，不许动配置放宽 |

**3-1 命令**

```bash
cd rust && cargo build --release
ls -l target/release/dust_rs
```

**3-2 命令**

```bash
cd rust && cargo run --release
./target/release/dust_rs
echo "exit=$?"
```

**3-3 命令**

```bash
cd rust && cargo fmt --check && cargo check --workspace
```

**3-4 命令**

```bash
cd rust && cargo clippy --workspace --all-targets -- -D warnings
```

**P3 编译状态**：本阶段只验证，不改代码；任何一步失败 → 停下报告实际输出，不许自行"修到过"。

---

### P4（可选，默认不执行）与现有构建链集成

> **本阶段默认不做**，只有用户拍板 E 选"立即集成"才执行。

| 步 | 文件 | 动作 | 依赖前置 | 连带 |
|---|---|---|---|---|
| 4-1 | `build.sh` | 在校验阶段后追加 `cargo build --release --manifest-path rust/Cargo.toml` | P3 全绿 | Rust 挂上后 C++ 构建失败会连带 Rust 也不跑，需要决定顺序（推荐 Rust 在前、失败即退出） |
| 4-2 | `doc/` 相关文档 | 文档补一句构建产物位置 | 4-1 | 无 |

---

## 4. 参数与版本表（装完回填）

| 参数 | 值 | 来源 |
|---|---|---|
| rustup 版本 | 未单独记录（`rustup show` 正常） | 执行记录 |
| rustc 版本 | 1.100.0 (0ef1bc386 2026-11-09) | `rustc -V` |
| cargo 版本 | 1.100.0 (3acc79c0c 2026-11-05) | `cargo -V` |
| channel | stable | 定死 |
| profile | default | 定死 |
| 公司/镜像源 | `https://rsproxy.cn`（sparse） | 待拍板 C |
| 工程根 | `rust/` | 待拍板 A |
| crate 名 | `dust_rs` | 待拍板 B |
| edition | 2024 | 待拍板 F |
| 产物 | `rust/target/release/dust_rs` | 定死 |
| 忽略路径 | `rust/target/` | 定死 |

---

## 5. 验收清单（逐条勾）

- [x] `rustc -V` / `cargo -V` / `rustup show` 三者均可执行
- [x] `rustup component list --installed` 含 `rustfmt` 与 `clippy`
- [x] `rust/Cargo.toml`、`rust/app/Cargo.toml`、`rust/app/src/main.rs` 三件齐全
- [x] `rust/.cargo/config.toml` 存在，`cargo build` 正常解析（`cargo metadata` 未单独跑）
- [x] `cargo build --release` 成功（warnings 0，`rust/target/release/dust_rs` 445008 B）
- [x] `cargo run --release` 输出 `dust_rs ok`，退出码 0
- [x] `cargo fmt --check` 退出码 0
- [x] `cargo check --workspace` 退出码 0
- [x] `cargo clippy --workspace --all-targets -- -D warnings` 退出码 0（manifest 的 `license` 未使用警告已删 `license` 字段消除）
- [x] `.gitignore` 含 `rust/target/`
- [ ] `git status` 里不出现 `rust/target/` 任何文件（未验证：执行侧 Bash 被权限阻断）
- [x] 本文档 §4 已回填实际版本号

---

## 6. 执行纪律（写给执行者）

1. **一步一个文件**："开始第 N 步" = 只做那一步，做完停，等"继续"。
2. **中间态不慌**：P1 第 1～3 步之间 cargo 报错是设计内的，按步表「连带」栏判断，不许自行"顺手补齐"。
3. **不动用户环境**：`~/.cargo/config.toml`、`~/.ssh/`、全局 git 配置、`/etc` 一律不写；镜像配置只放仓库内。
4. **编译/运行由用户执行**（本仓库既有规矩）；执行者只提供命令与判据。
5. **不许用 git 回滚**修自己的改动；改回用 Edit。
6. **不许顺手做 P4**：未拍板 E 之前，`build.sh` 一行都不碰。
7. 任何一步失败 → 贴真实报错停下，**不许编原因、不许猜着修**。

---

## 7. 待拍板汇总

| # | 项 | 推荐值 | 影响面 |
|---|---|---|---|
| A | 工程落点 | `rust/` | P1 全部文件路径 |
| B | crate 名 | `dust_rs` | P1-2 / P1-3 / 产物名 |
| C | 镜像 vs 代理 | 镜像（rsproxy） | P2-3 是否建、0-2 用方案 A/B |
| D | clippy 是否默认 deny | 否，只在验收命令带 `-D warnings` | P3-4 命令 |
| E | 是否现在集成 `build.sh` | 否 | P4 是否执行 |
| F | edition | 2024 | P1-1 / P2-1 |

> 执行前把这 6 条一句话确认（或直接说"按推荐来"）即可开工。
