#!/usr/bin/env bash
# 运行已构建的骨架程序。
#
# 用法：
#   ./framework/cmd/build/run.sh            # 直接运行
#   ./framework/cmd/build/run.sh --build    # 先增量构建再运行
#
# 说明：shell 是交互式终端。stdin 不是终端时（管道/重定向）不做行编辑重绘。
#       EtherCAT 主站要开 AF_PACKET 原始套接字，非 root 时自动用 sudo 提权。

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
APP="${ROOT}/project/build/project/dust_app"

# ROS 2 setup 脚本可能读取未定义变量，加载期间暂时关闭 nounset。
if [[ -f /opt/ros/humble/setup.bash ]]; then
    set +u
    source /opt/ros/humble/setup.bash
    set -u
fi

if [[ "${1:-}" == "--build" ]]; then
    "${ROOT}/framework/cmd/build/build.sh"
    shift
elif [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    sed -n '2,10p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
    exit 0
fi

if [[ ! -x "${APP}" ]]; then
    echo "找不到 ${APP}，先跑 build.sh" >&2
    exit 1
fi

# 提权后重新进入脚本加载 ROS 2，避免 sudo 清除动态库等运行环境。
if [[ "${EUID}" -ne 0 ]]; then
    exec sudo bash "${BASH_SOURCE[0]}" "$@"
fi

exec "${APP}" "$@"
