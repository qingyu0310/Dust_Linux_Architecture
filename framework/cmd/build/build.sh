#!/usr/bin/env bash
# 配置并构建骨架工程。
#
# 用法：
#   ./framework/cmd/build/build.sh                # 增量构建到 project/build/
#   ./framework/cmd/build/build.sh --clean        # 删 project/build/ 后全量重建
#   ./framework/cmd/build/build.sh --config foo   # 用指定 .config 覆盖 Kconfig 默认值
#
# 退出码：0 成功；非 0 为 cmake 的失败码。

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
BUILD_DIR="${ROOT}/project/build"
CLEAN=0
KCONFIG_CONFIG=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --clean)   CLEAN=1; shift ;;
        --config)  KCONFIG_CONFIG="$2"; shift 2 ;;
        -h|--help)
            sed -n '2,10p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
            exit 0 ;;
        *)
            echo "未知参数: $1" >&2
            exit 2 ;;
    esac
done

if [[ "${CLEAN}" -eq 1 ]]; then
    rm -rf "${BUILD_DIR}"
fi

CMAKE_ARGS=(-S "${ROOT}" -B "${BUILD_DIR}")
if [[ -n "${KCONFIG_CONFIG}" ]]; then
    CMAKE_ARGS+=("-DKCONFIG_CONFIG=${KCONFIG_CONFIG}")
fi

cmake "${CMAKE_ARGS[@]}"
cmake --build "${BUILD_DIR}" --parallel "$(nproc)"

echo
echo "构建完成: ${BUILD_DIR}/project/dust_app"
