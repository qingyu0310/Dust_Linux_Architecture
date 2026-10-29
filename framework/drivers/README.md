# framework/drivers —— 底层通讯/设备驱动层

> 靠近硬件与总线协议的实现层。`modules/` 用它，它不用任何人。

---

## 1. 放什么

打开设备、收发字节/帧、维护总线状态机、把硬件错误翻成返回值。

判据：**这段代码如果换一块板卡就要改，它属于 drivers。**

## 2. 不放什么

| 不放 | 归哪 |
|---|---|
| 目标选择、状态机编排、业务策略 | `project/thread/` |
| PDO 语义、设备能力契约（IMU/电机/遥控） | `framework/modules/` |
| 纯计算（PID、滤波、解算） | `framework/algorithm/` |
| 线程间消息契约 | `framework/topic/` |

## 3. 三类通讯形态

新增驱动先归类，再决定落哪个目录、用哪个符号前缀。

| 形态 | 特征 | 落点 | Kconfig 前缀 |
|---|---|---|---|
| 字节流 | 无序字节，有接收事件，需要缓冲 | `communication/stream/{uart,usb,rs485}/` | `DUST_COM_UART` / `DUST_COM_USB` / `DUST_COM_RS485` |
| 报文 | 定长帧 + ID，有丢帧语义 | `communication/can/` | `DUST_COM_CAN` |
| 周期过程数据 | 固定周期整帧交换，无队列无事件 | `communication/ethercat/` | `DUST_COM_ETHERCAT` |

设备类（GPIO / PWM）落 `device/{gpio,pwm}/`，前缀 `DUST_DEV_*`。

## 4. 新增一个驱动的四步

1. `Kconfig` 加 `config DUST_COM_<名字>`（`default n`）
2. 建目录 `communication/<名字>/`，写 `<名字>.hpp` + `<名字>.cpp`
3. `CMakeLists.txt` 加 `if(CONFIG_DUST_COM_<名字>)` 块，源文件挂 `framework_drivers`
4. 业务层在 `project/Kconfig` 里 `select DUST_COM_<名字>`

## 5. 不立抽象层

**没有第二个实现者之前，不写公共基类。**

只有一个实现时，抽象的形状靠猜；等第二个实现出现，两者的共同点才是真共同点。
判据与 `doc/shell下层形态与thread绝对调度规划.md` 的「何时才该拆」同源。

## 6. 日志

一律 `DUST_LOG_*`（`framework/shell/log/log.hpp`），不用 `printf`/`std::cout`。
