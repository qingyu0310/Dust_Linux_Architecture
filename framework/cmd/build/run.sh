#!/usr/bin/env bash
# 运行已构建的骨架程序。
#
# 用法：
#   ./framework/cmd/build/run.sh            # 直接运行
#   ./framework/cmd/build/run.sh --build    # 先增量构建再运行
#
# 说明：shell 是交互式终端。stdin 不是终端时（管道/重定向）不做行编辑重绘。

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
APP="${ROOT}/project/build/project/dust_app"

if [[ "${1:-}" == "--build" ]]; then
    "${ROOT}/framework/cmd/build/build.sh"
    shift
elif [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    sed -n '2,9p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
    exit 0
fi

if [[ ! -x "${APP}" ]]; then
    echo "找不到 ${APP}，先跑 build.sh" >&2
    exit 1
fi

exec "${APP}" "$@"
