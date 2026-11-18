# ROS 2 外部进程控制接入

本文面向导航、规划等独立进程。外部进程通过 ROS 2 下发速度 PDO，主站通过 EtherCAT 转发给 MCU；MCU 回传 PDO 通过另一话题返回外部进程。

## 1. 接口约定

| 外部进程操作 | 话题 | 类型 | data 长度 |
| --- | --- | --- | --- |
| 发布速度命令 | `/ecat/tx_pdo` | `std_msgs/msg/UInt8MultiArray` | 必须为 256 字节 |
| 订阅 MCU 状态 | `/ecat/rx_pdo` | `std_msgs/msg/UInt8MultiArray` | 256 字节 |

话题名称中的 rx/tx 按主站视角命名。`layout` 留空，`data_offset=0`，主站仅使用 `data`。

双方使用 BestEffort、KeepLast(1)、Volatile。状态订阅端不能要求 Reliable，否则无法匹配当前 BestEffort 发布端。消息只保留最新值，允许跳帧。

当前主站不接收 Twist，不能向话题直接发送两个浮点数。发送端需要组装完整 PDO，包括帧头和 CRC。

## 2. 环境配置：先解决跨用户传输

本机为 ROS 2 Humble，主站经 sudo 以 root 运行。实测普通用户使用默认 Fast DDS 传输无法正常接收，改用 UDP 后通信成功。因此外部进程启动前配置 UDP；不依赖此前 `/tmp` 中的临时文件。

在外部进程运行用户下执行一次，创建持久配置：

```bash
mkdir -p "$HOME/.config/dust"
cat > "$HOME/.config/dust/fastdds_udp.xml" <<'XML'
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>udp_only</transport_id>
      <type>UDPv4</type>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="udp_only_participant" is_default_profile="true">
    <rtps>
      <userTransports>
        <transport_id>udp_only</transport_id>
      </userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
    </rtps>
  </participant>
</profiles>
XML
```

每次启动外部进程前，在同一个终端执行：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.config/dust/fastdds_udp.xml"
```

环境变量必须在创建 ROS Context/Node 之前设置。启动脚本、服务或其他工作空间也需要继承这些配置。主站当前使用域 0；修改域时需保证两端一致，注意 sudo 不一定保留 ROS 环境变量。

主站运行：

```bash
dust run
```

主站运行脚本已经自动加载 Humble 环境。外部进程仍需按上面的步骤加载自己的环境。

## 3. 速度命令格式

完整 PDO 为 256 字节，前 15 字节有效，后 241 字节填零。

| 字节偏移 | 类型 | 内容 |
| --- | --- | --- |
| 0 | uint8 | 固定帧头 `0x6B` |
| 1–4 | float32 | linear_x，m/s，向前为正 |
| 5–8 | float32 | linear_y，m/s，向左为正 |
| 9–12 | float32 | angular_z，rad/s，当前发送 0 |
| 13–14 | uint16 | 字节 0–12 的 CRC16，低字节在前 |
| 15–255 | 原始字节 | 填零 |

所有多字节字段为小端；float 为 IEEE 754 单精度。字段紧凑排列，不能补齐到四字节边界，也不能把浮点速度数值转成整数发送。

CRC：初值 `0xFFFF`、右移反射多项式 `0x8408`、无最终异或。CRC 不覆盖尾部填充。

发送端必须拒绝 NaN 和无穷值。MCU 当前没有速度范围限幅，发送端需按机器人允许范围约束速度。当前 MCU 不使用 angular_z，不能通过它控制旋转。

## 4. 发送频率、停止与命令来源

建议 50 Hz，即每 20 ms 发送一次，即使速度没有变化也继续发布。

- 第一次 ROS 命令到达前，主站使用现有 var 速度。
- ROS 消息到达后接管输出。长度为 256 的消息按字节原样下发，由 MCU 检查头、CRC 和浮点有限性。
- 主站从最近接收时刻起计算，命令年龄达到 100 ms 后组装合法零速度包。它不会自动回退到 var。
- 错误长度消息会发布失效标记，EtherCAT 线程取到后归零；最新值通道允许后续正确消息覆盖该标记。
- 再次收到完整 PDO 后恢复转发。
- 停止运动应主动持续发送合法零速度包；进程退出或停发后的 100 ms 归零是停更处理。

接收计时不等于导航目标更新时间。导航算法停更时，不能让独立发送循环无限刷新旧目标；外部进程应自行检测目标有效性，停更就发送零速度。

断开订阅不是立即停机通知，主站依靠上述时间判断停止。主站周期线程也必须持续运行，不能把 100 ms 理解为整个系统在任意故障下的硬实时保证。

## 5. MCU 回传状态格式

上行完整 256 字节原样发布；前 20 字节为状态包：

| 字节偏移 | 类型 | 字段 |
| --- | --- | --- |
| 0 | uint8 | header，固定 `0x6D` |
| 1 | uint8 | sequence，MCU 聚合序号，255 后回绕 |
| 2 | uint8 | game_type |
| 3–4 | uint16 | stage_remain_s |
| 5 | uint8 | game_progress |
| 6–7 | uint16 | current_hp |
| 8–9 | uint16 | maximum_hp |
| 10 | uint8 | robot_id |
| 11 | uint8 | chassis_power_enabled |
| 12–13 | uint16 | zone_state_and_rfid |
| 14–15 | uint16 | ally_control_degree |
| 16–17 | uint16 | enemy_control_degree |
| 18–19 | uint16 | 字节 0–17 的 CRC16 |
| 20–255 | 原始字节 | 填充 |

外部进程自行检查长度、帧头、CRC；主站为保留现场也会转发业务 CRC 错误包。SOEM Read 失败不发布旧 PDO。外部进程应记录本地接收时间，长时间没有新数据时把状态标为失效，不能无限使用最后一帧。

sequence 是 MCU 生成序号，不是命令应答编号；允许重复、跳号和回绕，不应用它确认某条速度命令已执行。

按当前提供的 MCU 实现，偏移 2–17 尚未接入裁判系统，全部为零。血量、电源使能等不能用于判断真实机器人状态。

## 6. Python 接入示例

将下面代码放在外部进程工程中。默认发送零速度；修改 `self.linear_x/y` 即可设置目标。此示例在同一个 executor 中执行目标更新与发布，若多线程修改目标，调用方需要同步。

```python
import math
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy,
)
from std_msgs.msg import UInt8MultiArray


def crc16(data):
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
    return crc


def make_command(vx, vy):
    if not (math.isfinite(vx) and math.isfinite(vy)):
        raise ValueError("速度必须为有限数")
    body = struct.pack("<Bfff", 0x6B, vx, vy, 0.0)
    packet = body + struct.pack("<H", crc16(body))
    message = UInt8MultiArray()
    message.data = list(packet.ljust(256, b"\x00"))
    return message


def parse_status(data):
    raw = bytes(data)
    if len(raw) != 256 or raw[0] != 0x6D:
        raise ValueError("状态长度或帧头错误")
    if struct.unpack_from("<H", raw, 18)[0] != crc16(raw[:18]):
        raise ValueError("状态 CRC 错误")
    values = struct.unpack_from("<BBHBHHBBHHH", raw, 1)
    keys = (
        "sequence", "game_type", "stage_remain_s", "game_progress",
        "current_hp", "maximum_hp", "robot_id", "chassis_power_enabled",
        "zone_state_and_rfid", "ally_control_degree", "enemy_control_degree",
    )
    return dict(zip(keys, values))


class EcatControl(Node):
    def __init__(self):
        super().__init__("navigation_ecat_control")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(
            UInt8MultiArray, "/ecat/tx_pdo", qos)
        self.subscription = self.create_subscription(
            UInt8MultiArray, "/ecat/rx_pdo", self.on_status, qos)
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.target_updated_at = 0.0
        self.last_status = None
        self.last_status_at = 0.0
        self.timer = self.create_timer(0.02, self.send_command)

    def set_target(self, vx, vy):
        # 导航每次产生新目标时调用，不能只刷新旧目标的时间戳。
        if not (math.isfinite(vx) and math.isfinite(vy)):
            raise ValueError("速度必须为有限数")
        # 提前验证 float32 可编码性；实际系统还需设置自己的速度范围。
        struct.pack("<ff", vx, vy)
        self.linear_x = vx
        self.linear_y = vy
        self.target_updated_at = time.monotonic()

    def send_command(self):
        fresh = time.monotonic() - self.target_updated_at < 0.1
        vx = self.linear_x if fresh else 0.0
        vy = self.linear_y if fresh else 0.0
        self.publisher.publish(make_command(vx, vy))

    def on_status(self, message):
        try:
            status = parse_status(message.data)
        except ValueError:
            self.last_status = None
            return
        self.last_status = status
        self.last_status_at = time.monotonic()


def main():
    rclpy.init()
    node = EcatControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 停发后的主站超时归零仍生效；不能依赖退出时单帧一定送达。
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
```

使用 Humble 环境下能导入 rclpy 的 Python 运行，例如 `/usr/bin/python3 your_control.py`。不要同时运行多个命令发布者；当前主站没有控制权仲裁，最后收到的消息会覆盖之前的命令。

## 7. 命令行联调

先按第 2 节加载环境和 UDP 配置。

持续发送 x=1 m/s、y=0、角速度=0：

```bash
ros2 topic pub -r 50 --qos-reliability best_effort \
/ecat/tx_pdo std_msgs/msg/UInt8MultiArray \
"{data: [107, 0, 0, 128, 63, 0, 0, 0, 0, 0, 0, 0, 0, 9, 249$(printf ', 0%.0s' {1..241})]}"
```

按 Ctrl+C 停发后，主站按 100 ms 超时归零。若需持续主动发送停止命令：

```bash
ros2 topic pub -r 50 --qos-reliability best_effort \
/ecat/tx_pdo std_msgs/msg/UInt8MultiArray \
"{data: [107, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 122, 197$(printf ', 0%.0s' {1..241})]}"
```

查看 MCU 原始状态：

```bash
ros2 topic echo /ecat/rx_pdo --qos-reliability best_effort
```

主站日志对照：

| 日志 | 含义 |
| --- | --- |
| `matched_publishers=0 received=0` | 主站尚未发现命令发布者，检查域、RMW、UDP 配置与进程环境 |
| `command received ... len=256` | ROS 订阅回调已经收到命令 |
| `ROS command selected` | EtherCAT 线程已取到并采用该命令 |
| `first_float=00 00 80 3f` | x 的 float32 位模式为 1.0 |
| `crc=0xf909 calculated=0xf909` | x=1 样例 CRC 匹配 |
| `expired or invalid` | 下行命令停更或失效，主站切零速度 |

收到 ROS 消息、采用 PDO 不等于 MCU 已执行速度；协议当前没有命令应答序号。

## 8. MCU 接管条件

按当前提供的 MCU 控制逻辑：SA=DOWN、SB=UP 且遥控持续在线时使用 EtherCAT 导航命令；SA=UP 时关闭输出；SB 其他档位使用遥控。MCU 不在 OP、命令校验失败、导航命令过期或遥控停更时不会执行该速度。

这些条件以实际烧录的固件为准。上行占位状态包不能证明遥控档位或真实底盘电源状态。
