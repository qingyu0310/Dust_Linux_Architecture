# 构建脚本

骨架工程只有一种构建形态，这三个脚本是 `cmake` 的薄封装。

| 脚本 | 作用 |
|---|---|
| `dust` | 子命令分发：`dust build` / `dust run` / `dust help` |
| `build.sh` | 配置并构建到 `<repo>/project/build` |
| `run.sh` | 运行 `project/build/project/dust_app` |

## 用法

```bash
# 直接调脚本（不需要改 PATH）
./framework/cmd/build/dust build
./framework/cmd/build/dust run

# 或者把目录加进 PATH，之后直接敲 dust
export PATH="$PWD/framework/cmd/build:$PATH"
dust build
dust run
```

`build.sh` 的参数：

| 参数 | 作用 |
|---|---|
| 无 | 增量构建 |
| `--clean` | 先删 `build/` 再全量重建 |
| `--config <file>` | 用指定 `.config` 覆盖 Kconfig 默认值（等价于 `-DKCONFIG_CONFIG=...`） |

`run.sh` 的参数：

| 参数 | 作用 |
|---|---|
| 无 | 直接运行 |
| `--build` | 先增量构建再运行 |

## 等价命令

```bash
cmake -S . -B build
cmake --build build --parallel
./project/build/project/dust_app
```
