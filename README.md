# gimbal —— 麦轮小车云台 + 底盘控制器（Jetson Nano）

> 本文件由原 `README.md`、`CHASSIS_LINK.md`、`DEBUG_REPORT_2026-10-19.md` **合并精简**而成
> （2026-11-05）：工程概览与设计决策、底盘链路协议与排障、图传与无线、联调实测全部收在这一份里。
> 各目录的逐文件说明仍在 `include/gimbal/README.md`、`src/README.md` 等子文档中。

两轴云台（yaw + pitch）+ 底盘（STM32F407）的正式控制工程，跑在 Jetson Nano 上。

**云台**：ZDT X42S 闭环步进 ×2，UART（CH340，与底盘共用一个串口）或 **CAN 扩展帧** 两种后端；
**底盘**：F407 标准帧链路，**UART（`UnifiedSerial`，与云台共用一个串口）或 CAN（`CanChassisLink`）** 双后端可选。
两种总线可任意组合，见 §2.2。

| 项 | 值 |
|---|---|
| 平台 | Jetson Nano（L4T R32.6.1 / Ubuntu 20.04 / kernel 4.9.253-tegra） |
| 语言 / 构建 | **C++17**（g++ 8.4.0）+ CMake 3.16.3，单进程守护式，无 RTOS / ROS |
| 云台电机 | ZDT X42S 第二代闭环步进 ×2，**Emm5.0 固件** |
| 电机地址 | **yaw = 1**，**pitch = 2**（实机接线；早期文档写的 2/1 已作废） |
| 细分 / 脉冲 | 细分 **16** → **3200 脉冲/电机圈** |
| 传动比 | yaw **2:1**（`--gear 2.0`）；pitch **直连 1:1**（`--pitch-gear 1.0`） |
| 机械限位 | **yaw 有导电滑环，360° 无机械限位**；pitch 软限位 **−20° ~ +20°** |
| 云台总线 | UART 115200 8N1（CH340）/ SocketCAN（`can0`，ZDT 扩展帧） |
| 底盘总线 | USART2 115200 8N1（与云台共用一个 CH340）/ SocketCAN（`can0`，标准帧） |
| 控制入口 | **UDP 40001**（PC 客户端）/ stdin（本地调试） |
| 图传 | `mjpeg_server`：V4L2 取 MJPG → HTTP multipart，端口 **8080** |
| 水弹波箱 | 可选：fork 一个 Python worker 用 Jetson GPIO 直驱 MOS（§9） |
| 外部依赖 | **零第三方库** —— 仅 C++ 标准库 + Linux 系统调用（termios / SocketCAN / BSD socket） |

---

## 1. 系统拓扑

```
笔记本(PC)                            Jetson Nano                          STM32F407
┌──────────────┐   WiFi 或 网线      ┌──────────────┐   串口(115200) ┌────────────┐
│ gimbal_ctl   │ ──── UDP ────►    │ gimbal       │ ─── USART2 ──► │ uart_link  │
│ (C++17, X11) │  24B/64B          │ (C++17 守护) │  PD5(TX)/PD6(RX)│ (1kHz 控制)│
└──────────────┘                    └──────────────┘  └── 或 CAN ──► │ can_link   │
                                         │  CH340 → ZDT 步进 ×2（云台，与底盘共线）└────────────┘
                                         │  CANable → can0（底盘；ZDT 切 CAN 模式后也可上）
                                         │  GPIO 12 → MOS（水弹波箱，可选）
```

| 角色 | 设备 / 端口 | 说明 |
|---|---|---|
| 笔记本 ↔ Jetson（有线） | `enp8s0 192.168.2.1` ↔ `eth0 192.168.2.2` | `ssh Jetson`（免密） |
| 笔记本 ↔ Jetson（无线） | 笔记本 WiFi ↔ Jetson AP `MECAR-CTRL` / `10.42.0.1` | 车要走位时用这条 |
| Jetson ↔ F407 / ZDT（串口） | `/dev/ttyUSB0`（CH340）115200 8N1 | 底盘 + 云台**共用一条** CH340 |
| Jetson ↔ F407（CAN） | CANable 2.0 → `can0`，1 Mbps 标准帧 | 底盘 CAN 链路（§8.2） |
| 笔记本 ↔ F407（调试） | `/dev/ttyACM0`（DAP-LINK 虚拟串口） | USART1 调试打印，115200 |
| UDP 控制口 | `40001`（默认） | `gimbal_ctl` → `gimbal` |
| 图传 | `:8080/stream` | MJPEG |

---

## 2. 快速开始

### 2.1 构建与自检

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
ctest --test-dir build          # = ./build/gimbal --selftest，纯软件 13 段自检
```

### 2.2 典型后端组合

```bash
./build/gimbal --device /dev/ttyUSB0               # 默认：全串口，底盘 + 云台共用同一个 CH340
./build/gimbal --device /dev/ttyUSB0 --no-chassis  # 只控云台
./build/gimbal --chassis can0                      # 混合：云台 UART + 底盘 CAN（推荐先跑这个）
./build/gimbal --device can0                       # 全 CAN：两轴 ZDT 扩展帧 + 底盘标准帧同一条 can0
./build/gimbal --yaw-can can0                      # 仅 yaw 走 CAN，pitch + 底盘仍走串口
./build/gimbal --selftest                          # 自检后退出（不接硬件）
```

> CAN 后端需要先起 `can0`（CANable 2.0，SLCAN 固件）：
> `sudo slcand -o -s8 -t hw /dev/ttyACM1 can0 && sudo ip link set can0 up`
> （`ttyACM*` 编号随插拔变化，以 `ls /dev/ttyACM*` 为准）。

### 2.3 启动 / 停止 / 查后台

**前台（推荐）**——能看到状态、能用交互命令：

```bash
jetson@nano:~/gimbal$ ./build/gimbal --device /dev/ttyUSB0
```

**后台**——关终端也不退出：

```bash
nohup ./build/gimbal --device /dev/ttyUSB0 > /tmp/gimbal.log 2>&1 < /dev/null &
tail -f /tmp/gimbal.log        # 看日志
```

> 后台方式的 stdin 是空的，`s` / `q` 等交互命令用不了；想用就前台跑。

| 场景 | 命令 |
|---|---|
| 前台跑的 | 终端按 `q`（优雅退出，会先停手/停车）或 `Ctrl+C` |
| 后台跑的 | `kill <PID>`（PID 用 `pgrep -af 'build/gimbal'` 查） |
| 按名字杀 | `pkill -f 'build/gimbal'` |
| 卡死最后手段 | `kill -9 <PID>`（可能来不及收尾，慎用） |

查：`pgrep -af 'build/gimbal'` · `fuser -v /dev/ttyUSB0`（谁占着串口）· `ss -uln | grep 40001` · `tail -f /tmp/gimbal.log`

> ⚠️ **串口是独占的**：旧进程没退干净就起新的，新的会打不开 `/dev/ttyUSB0`。顺序永远是先关旧、再开新。
> ⚠️ 远程 shell 里**别用** `pkill -f build/gimbal` 之外的模糊匹配，`pkill -f gimbal` 会把自己的 shell 一起杀掉。

---

## 3. 目录结构

**31 个源文件（17 头 + 14 源）/ 7110 行**，另加 `CMakeLists.txt`、`README.md` 与 `src/device/fire_worker.py`。

```
gimbal/
├── CMakeLists.txt                   构建脚本：gimbal_core 静态库 + gimbal / mjpeg_server 两个可执行
├── README.md                        本文（概览 / 设计决策 / 链路协议 / 操作 / 排障 / 实测）
│
├── include/gimbal/                  ── 头文件：接口与线协议 ──
│   ├── protocol/                    ZDT（张大头）闭环步进电机线协议
│   │   ├── zdt_codes.hpp            功能码 / 返回码 / 状态位 / 固件类型（Emm5.0 vs X 差异）
│   │   └── zdt_frame.hpp            帧构造器声明 + 流式解析器（粘包/半包切帧）
│   ├── transport/                   与总线无关的收发层（换总线只换这里）
│   │   ├── itransport.hpp           ★ 抽象接口：12 ms 发送节流队列 + 收帧队列 + 统计
│   │   ├── can_transport.hpp        SocketCAN 实现（29-bit 扩展帧拆包重组，ZDT 云台用）
│   │   └── fake_transport.hpp       纯软件电机模拟（`--selftest` 用，无需硬件）
│   ├── device/
│   │   ├── zdt_motor.hpp            单台电机：使能 / 速度 / 位置 / 急停 / 遥测轮询
│   │   └── fire_control.hpp         水弹波箱：fork/exec Python worker，经管道发 `0`/`1`
│   ├── imu/                         IMU（ICM-42688-P）：只给 pitch 提供绝对仰角
│   │   ├── imu.hpp                  IImu 抽象 + ImuPitch 俯仰融合（互补滤波 + 创新门限）
│   │   └── imu_i2c.hpp              ICM-42688-P I²C 驱动接口（地址自动识别）
│   ├── gimbal/
│   │   ├── axis.hpp                 单轴控制律：角速度斜坡 + 死区 + 软限位 + 回中 + 角度估计
│   │   └── gimbal.hpp               双轴协调：统一时钟 + 两级看门狗 + 传输层 poll 去重
│   └── link/                        ── 通信链路（对上层 / 对下位机）──
│       ├── control_link.hpp         ★ UDP 线协议 v2：上行 24 B / 下行 64 B，小端编解码 + 校验和
│       ├── udp_control_link.hpp     UDP 收发实现：非阻塞 socket、按频率回发遥测、丢包统计
│       ├── chassis_link.hpp         ★ 底盘帧协议：命令 12 B / 反馈 40 B + IChassisLink 抽象
│       ├── unified_serial.hpp       ★ 单串口统一链路：底盘帧 + ZDT 帧共用一个 fd（RxDemux 分流）
│       └── can_chassis_link.hpp     ★ CAN 底盘链路：0x020/0x021/0x000 + 0x110~0x114 组拼装
│
└── src/                             ── 实现 ──
    ├── main.cpp                     入口：参数解析 / 100 Hz 主循环 / 本地 stdin / 自检 / 底盘仲裁
    ├── protocol/
    │   └── zdt_frame.cpp            帧构造与流式解析实现（校验 0x6B）
    ├── transport/
    │   ├── itransport.cpp           节流队列、收帧按地址路由、统计维护
    │   └── can_transport.cpp        SocketCAN 收发 + 扩展帧拆包重组
    ├── device/
    │   ├── zdt_motor.cpp            物理量 ↔ ZDT 帧、遥测解析、传动比/脉冲换算
    │   ├── fire_control.cpp         fork/exec + 管道写 `1\n`/`0\n`，析构写 `0\nq\n`
    │   └── fire_worker.py           水弹波箱节拍进程（BOARD 引脚 12，高 80 ms / 低 120 ms）
    ├── imu/
    │   ├── imu_pitch.cpp            俯仰融合：asin(u·â) + 陀螺积分 + 创新门限
    │   └── imu_i2c.cpp              寄存器配置/读取（不做软复位、写重试、地址自动识别）
    ├── gimbal/
    │   ├── axis.cpp                 单轴控制律（斜坡/死区/限位/回中）
    │   └── gimbal.cpp               双轴调度、看门狗、estop/home/enable 协调
    ├── link/
    │   ├── udp_control_link.cpp     UDP 收包解码、按频率回发遥测、RTT/丢包统计
    │   ├── unified_serial.cpp       单串口链路：底盘帧 + ZDT 帧共用一个 fd（节流 + RxDemux 分流）
    │   └── can_chassis_link.cpp     SocketCAN：标准帧收发 + 0x110~0x114 组规则 + 0x000 急停
    └── video/
        └── mjpeg_server.cpp         独立程序：V4L2 取 MJPG 帧 → HTTP multipart 转发（零转码）
```

行数分布（判断改动影响面用）：
`main.cpp` 2299 · `unified_serial.cpp` 356 · `mjpeg_server.cpp` 401 · `can_chassis_link.cpp` 278 ·
`zdt_motor.cpp` 243 · `zdt_frame.cpp` 200 · `can_transport.cpp` 179 · `udp_control_link.cpp` 175 ·
`axis.cpp` 171 · `imu_i2c.cpp` 187 · `gimbal.cpp` 111 · `imu_pitch.cpp` 81 · `itransport.cpp` 77 · `fire_control.cpp` 76 ·
头文件：`control_link.hpp` 347 · `fire_link.hpp` 303 · `fake_transport.hpp` 204 · `chassis_link.hpp` 194 · `unified_serial.hpp` 146 ·
`zdt_motor.hpp` 134 · `can_chassis_link.hpp` 121 · `zdt_codes.hpp` 104 · `zdt_frame.hpp` 97 · `axis.hpp` 100 ·
`itransport.hpp` 91 · `gimbal.hpp` 89 · `imu.hpp` 86 · `udp_control_link.hpp` 75 · `imu_i2c.hpp` 59 · `can_transport.hpp` 61 · `fire_control.hpp` 31。

> ★ = 协议/接口的关键文件，改动要三端（固件 ↔ Jetson ↔ PC 客户端）同步。

---

## 4. 分层设计

```
   main.cpp  ──  控制循环 100Hz + stdin 交互 + 底盘仲裁
        │
        ▼
   Gimbal    ──  两轴协调 / 看门狗 / 传输层 poll 去重
        │
        ▼
   Axis      ──  角速度斜坡 / 死区 / 软限位 / 回中
        │
        ▼
   ZdtMotor  ──  物理量 <-> ZDT 帧；遥测轮询；状态解析
        │
        ▼
   ITransport ── 节流队列 + 解析器 + 收帧路由
        ├─ UnifiedSerial (真机，底盘 + 云台共用一个串口)
        ├─ CanTransport  (真机，ZDT 扩展帧 / 仅 yaw 走 CAN)
        └─ FakeTransport (--selftest)

   底盘侧：
   IChassisLink ── setCommand() / state() / poll()
        ├─ UnifiedSerial  (UART：12 B 命令 / 40 B 反馈，与云台共用串口)
        └─ CanChassisLink (CAN：0x020/0x021/0x000 + 0x110~0x114 五帧反馈)
```

**换总线只换一个实现类**，上层一行不用改。

### 4.1 单拍数据流（100 Hz，dt = 10 ms）

```
操作端 UDP 包 ─┐
               ├─► Gimbal::setRate(yaw_dps, pitch_dps)
本地 stdin ────┘         │
                         ▼
                   Axis::setRate()  ── 限幅到 max_rate_dps，重置 since_cmd_s_
                         │
                         ▼
                   Axis::update(dt) ── ① 看门狗判断
                                        ② 斜坡逼近（rate_accel_dps2）
                                        ③ 死区吸收（rate_deadband_dps）
                                        ④ 软限位钳制 + 角度积分
                         │
                         ▼
        输出 °/s ──换算──► 电机 RPM（× gear ÷ 6）
                         │
                         ▼
                   ZdtMotor::setSpeedRpm() ──► ITransport::send()
                                                    │  12 ms 节流队列
                                                    ▼
                                              UART / CAN ──► 驱动板

收方向：UART 字节 ─► zdt::Parser 流式切帧 ─► 按 Addr 路由
        ─► ZdtMotor 解析（速度/角度/状态/电压/温度）─► Axis / Gimbal 查询
        ─► main 组装 ControlTelemetry ─► UDP 下行 50 Hz
```

`Gimbal` 只对**去重后**的传输层调 `poll()`（两轴共一根总线时只有一个），避免同一串口被 poll 两次。

### 4.2 传动比换算（最容易被忽略、但会全盘算错）

`--gear` 是 **电机转数 / 输出轴转数**（yaw 的 2:1 减速填 `2.0`；pitch 直连填 `1.0`）：

```
输出 °/s  ->  电机 RPM :  rpm = dps × gear / 6       （yaw 2:1 时 90°/s -> 30 RPM）
电机 RPM  ->  输出 °/s :  dps = rpm × 6 / gear
脉冲/输出圈              :  3200 × gear = 6400
```

自检里有断言：`90°/s @ gear=2.0 → 电机 30.0 RPM`。

---

## 5. 关键设计决策（都来自实测踩坑）

### 5.1 命令节流 12 ms —— 防 ZDT 粘包

ZDT 手册 §3.5.2：「每条命令发送完后，要延时几毫秒，防止命令之间粘包失效」

实测（两条命令间隔 → 两条都回复的成功率）：

| 间隔 | 成功率 |
|---|---|
| **0 ms** | **1/5** ❌ |
| ≥1 ms | 5/5 ✅ |

本工程取 **12 ms** 留足余量，实现在 `ITransport::nextTx()`（CAN 侧无粘包问题，取 2 ms）。

### 5.2 `urgent` 是插队，**不是清空队列**

踩过的坑：把 `urgent` 实现成 `txq_.clear()`，导致 `Gimbal::estop()` 里
`yaw_.estop()` 排入的「失能」命令，被紧跟着的 `pitch_.estop()` 的 clear 直接清掉
—— **yaw 永远没被失能**（自检里表现成 `yaw.en=1 pitch.en=0`）。一根总线挂多台电机时这是致命的。

正确语义：`push_front()` 插队到最前，绝不动别人的命令。

### 5.3 遥测轮询每项独立计时

不能用轮转指针 `next = (next+1) % N` —— 遇到长周期项会卡住等一整拍，
表现为「状态冻结几秒 → 猛跳一次」（实测回帧率从应有的 11.5/s 掉到 2.5/s）。

`ZdtMotor` 里每个遥测项各持一个 `next_ms`，且初始值错开 5 ms 避免同拍成簇。

### 5.4 看门狗分两级，且**只归零不失能**

| 级别 | 触发条件 | 动作 |
|---|---|---|
| 手感层 | `rate_hold_s`(0.12 s) 无新指令 | 角速度指数衰减到 0（τ = 0.05 s） |
| 安全层 | `command_timeout_s` 无指令 | 强制 hold，**保持使能** |

**为什么不失能**：云台一旦失能就松轴，受重力会垂下来 —— 比继续转更危险。
断流 0.9 s 实测状态字 `0x0F`（含 `Watchdog`），速度归零而 `Enabled` 位仍在，符合设计。

### 5.5 `Gimbal::enable()` 必须幂等 ⭐

操作端**每个控制包**都会带 `Enable` 标志（意思是"保持使能"）。
如果 `enable(true)` 每次都把 `enable_age_s_` 清零，`enable_delay_s`（上电空档）就永远走不完，
两轴会被永久压在 `hold()` 上 —— **表现是"发什么都是 0，但状态全绿"**。这个坑实地踩过一次，
最后靠独立 Python 实现交叉验证才排除协议问题。修法：`if (on == enabled_) return;`
**协议里「每包都带的保持位」必须做成幂等操作。**

### 5.6 显式 `Disable` 标志

最初只有 `Enable`，操作端**没法显式失能**（「不发送」≠「失能」）。
补上 `flags` 位 `0x40 Disable`，与 `Enable` 互斥且优先。

### 5.7 单串口统一链路 `UnifiedSerial`（三机挂一条 CH340）

一块 CH340 上挂三块 MCU：F407（底盘）+ ZDT yaw + ZDT pitch（云台）。

```
Jetson CH340 TX ──→ F407 USART2_RX + ZDT yaw RX + ZDT pitch RX
Jetson CH340 RX ←── F407 USART2_TX + ZDT yaw TX + ZDT pitch TX   （三机共线）
```

`UnifiedSerial` 同时实现 `ITransport`（云台）与 `IChassisLink`（底盘），独占一个 fd：

- **RX 按帧边界分流（`RxDemux`）**：不是把同一堆字节喂给两个解析器（那样底盘反馈帧里的字节可能
  被 ZDT 解析器误拆成假帧），而是按帧切流：
  `5A A5 81` + 36 B + CRC8 + `0D`（共 40 B）→ 底盘解码器；`Addr(1..32)` + `Func` + 负载 + `0x6B` → ZDT 帧队列；
  其余字节滚 1 字节重同步（与 F407 固件 `ulink_parse_rx` 同策略）。
- **发送**（同一 fd，整块写不交错）：底盘命令 12 B / **5 ms** 节流；ZDT 命令 **12 ms** 节流，
  并尽量排在**底盘反馈后的静默窗**（收完反馈 ≤6 ms 内发），避免 ZDT 应答与 F407 反馈在 RX 线上打架。
- **`poll()` 幂等**：主循环的 `chassis->poll()` 与 `Gimbal::step()` 内部的 poll 在同一拍会各调一次，
  用 1 ms 去重保证一拍只完整跑一次。
- **时效**：`state().age_s > chassis_timeout_s`(0.3 s) → `valid=0`；断线 0.5 s 自动重连。

> ⚠️ **已知限制：三机 TX 并线的抢线**。F407 反馈（100 Hz，每帧占线 3.5 ms）和 ZDT 应答都往
> CH340 的 RX 上发，同一时刻两个从机同时发字会冲突。静默窗调度能缓解（表现为客户端底盘偶发
> `OFFLINE`，不影响控制），物理根治需 485 收发器，或把 ZDT 遥测轮询周期放宽到 250 ms。

---

## 6. ⚠️ 单位换算是个坑（务必看懂）

**Emm 固件与 X 固件的位置模式单位完全不同**：

| | Emm5.0（本工程默认） | X |
|---|---|---|
| 位置命令 4 字节 | **脉冲数** | 0.1° 角度 |
| 速度单位 | RPM | 0.1 RPM |
| 位置回读 `0x36` | **0-65535 = 一圈 0-360°** | 角度/10 |
| 位置模式帧长 | 13 字节 | 不同（见手册 §5.3.10） |

所以 Emm 固件下：`脉冲数 = 角度/360 × (200 × 细分)`，`--pulses` 必须按**驱动板实际细分**设置，
默认 3200（200×16），**联调前请用 `42` 读驱动参数确认**（第 9 字节是 Microsteps）。

本工程默认走**速度模式**做控制，天然绕开这个换算问题；位置模式只实现了 Emm 分支。

**角度估计的诚实边界**：`0x36` 只有单圈值，`Axis::estAngleDeg()` 是靠转速积分出来的软件估计、会缓慢漂移。
真正的绝对保护应依赖驱动板的限位开关输入（状态位 `bit4/bit5`），或定期 `zeroHere()` 在机械零点重新对齐。

**用到的功能码**（`protocol/zdt_codes.hpp`）：

| 码 | 名称 | 作用 | 码 | 名称 | 作用 |
|---|---|---|---|---|---|
| `F3` | Enable | 使能 / 失能 | `36` | GetPos | 实时位置回读 |
| `F6` | Speed | 速度模式 | `3A` | GetStatus | 状态标志位 |
| `FD` | Position | 位置模式 | `39` | GetTemp | 驱动温度 |
| `FE 98` | EStop | 立即停止 | `24` | GetVbus | 母线电压 mV |
| `FF 66` | SyncMove | 多机同步触发 | `42` | GetConfig | 驱动参数（33 B） |
| `35` | GetSpeed | 实时转速回读 | `0A` | ZeroPos | 当前位置清零 |

`0x3A` 状态位：`bit0` 使能 · `bit1` 到位 · `bit2` 堵转 · `bit3` 堵转保护 · `bit4` 左限位 · `bit5` 右限位 · `bit7` 掉电。

---

## 7. 自检覆盖范围（`--selftest` / `ctest`）

`--selftest` 不需要任何硬件，共 **13 段**：

| 段 | 验证内容 |
|---|---|
| [1] 帧构造 | `mkEnable/mkSpeed/mkEStop/mkGet/mkPositionEmm` 逐字节比对期望 hex |
| [2] 解析器 | 混入垃圾字节后仍能正确切帧 + 校验 |
| [3] 角度↔脉冲 | 360°→3200、90°→800（用 FakeTransport 构造 ZdtMotor） |
| [4] UDP 线协议 | 上行 24 B / 下行 64 B 编解码往返、坏包/短包被拒 |
| [5] 端到端 | 一总线双电机 + 传动比 2:1：地址路由、斜坡、使能、遥测、看门狗（触发但**不失能**）、软限位、急停后失能 |
| [6] 底盘协议 | 命令帧编码向量、反馈帧解码向量、单字节破坏被拒、急停帧 cmd=0x02 且不带 EN |
| [7] 单总线帧分流 | `RxDemux` 半包/逐字节喂入、底盘帧 + ZDT 帧交错、垃圾字节、无坏帧 |
| [8] CAN 反馈组拼装 | `CanFbGroup`：0x110~0x114 顺序/乱序/缺帧/CRC 坏组/非反馈帧 |
| [9] 混合传输 | yaw 走 CAN、pitch 走 UART（两轴不同总线），命令/回读各自落在正确总线 |
| [10] pitch 会话状态机 | 纯软件（FakeTransport 模拟电机运动）：会话开始→抬升→定零→断流放下→锁死 |
| [11] 起停手感 | 指令流台阶：每帧 ≤1 RPM、单调不回头 |
| [12] IMU 俯仰融合 | 互补滤波 + 创新门限：静态精度、动态跟踪、线性加速度门限 |
| [13] 绝对仰角保持 | FakeTransport + 手构造 IMU 角度，验证底盘倾斜补偿 |

---

## 8. 底盘链路

**坐标约定（必须先记牢，与底盘库一致）**：`vx` = 车体前进、`vy` = 车体左移、`wz` = 俯视逆时针。
旧草案的「vx 右 / vy 前」已废弃，代码里只有这一套。

### 8.1 USART2 串口协议（`UnifiedSerial` 的底盘部分）

**命令帧（Jetson → F407，12 B，小端）**

| off | 0 | 1 | 2 | 3 | 4..5 | 6..7 | 8..9 | 10 | 11 |
|---|---|---|---|---|---|---|---|---|---|
| 字段 | `0xA5` | `0x5A` | cmd | seq | vx | vy | wz | flags | crc8 |
| 类型 | u8 | u8 | u8 | u8 | i16 | i16 | i16 | u8 | u8 |

- `vx/vy` 单位 **mm/s**，`wz` 单位 **0.01 rad/s**
- `crc8` = CRC-8/ATM（poly `0x07`，init `0`），覆盖第 0~10 字节
- 命令字：`0x01` 速度 / `0x02` 急停 / `0x03` 小陀螺 / `0x04` 清零
- flags：`0x01` = EN（允许驱动）

**反馈帧（F407 → Jetson，40 B，小端，100 Hz）**

| off | 字段 | 类型 | 单位 |
|---|---|---|---|
| 0..1 | `0x5A 0xA5` | — | 帧头 |
| 2 | 类型 `0x81` | u8 | — |
| 3 | seq（回显最后一条合法命令） | u8 | — |
| 4..7 / 8..11 | x / y（里程计） | i32 | mm |
| 12..13 | yaw（里程计） | i16 | 0.01° |
| 14..15 / 16..17 / 18..19 | 正解 vx / vy / wz | i16 | mm/s / 0.01 rad/s |
| 20..21 / 22..23 | gyro_yaw（陀螺积分）/ gyro_wz（陀螺实测） | i16 | 0.01° / 0.01 rad/s |
| 24..25 | status（位见下） | u16 | — |
| 26..27 / 28..29 | cmd_cnt（累计合法命令数）/ fb_seq（反馈帧序号） | u16 | — |
| 30..37 | 四轮实测转速 FL / FR / BL / BR | i16×4 | 0.01 RPM |
| 38 / 39 | crc8（覆盖 0~37）/ 帧尾 `0x0D` | u8 | — |

> **v2 变更**：30~37 新增四轮实测转速（0.01 RPM），麦轮方向检查靠它。

**status 状态位**

| 位 | 名称 | 含义 |
|---|---|---|
| `0x0001` | IMU_ONLINE | IMU 在线 |
| `0x0002` | LINK_OK | 500 ms 内收到过控制帧 |
| `0x0004` | WATCHDOG | 100 ms 无控制帧（已自动停车） |
| `0x0008` | ESTOP | 急停锁存中 |
| `0x0010` | DRIVER_ON | TB6612 STBY 高（输出使能） |
| `0x0020` | SPIN | 小陀螺模式 |
| `0x0040` | GYRO_ON | 陀螺外环非 OFF |
| `0x0080` | IMU_CAL | 零偏标定有效 |
| `0x0100` | CAN_ERR | CAN error-passive 预警（TEC/REC > 96，**仅 CAN 链路会出现**；Jetson 侧 `chassis_status::CanErr` 同名解码） |

**安全语义（三层）**：①Jetson 侧 UDP 指令断流 250 ms → 主动发 EN=0 停车帧；
②F407 侧控制帧 100 ms 未到 → 目标速度清零走斜坡减速，**但 STBY 保持高**（不松轴）；
③只有 `x` / `cx` 才拉低 STBY（电机自由）。

### 8.2 CAN 底盘链路（`can0`，11-bit 标准帧，1 Mbps）

`CanChassisLink`（`src/link/can_chassis_link.cpp`）用 SocketCAN 实现同一套协议：

| ID | 方向 | 内容 | 备注 |
|---|---|---|---|
| `0x000` | Jetson → F407 | 广播急停 | DLC 任意（含 0）；**Jetson 发、F407 只收不发**；ID 最小 = 仲裁最高优先级 |
| `0x020` | Jetson → F407 | `[cmd, seq, vx(i16), vy(i16), wz(i16)]`，100 Hz | 与串口命令帧字段语义一致（flags 取自 `0x021`） |
| `0x021` | Jetson → F407 | `[flags]`（bit0 = EN），10 Hz 保活 + 变化即发 | 只是标志帧，**不计入 `cmd_cnt`**、不喂看门狗 |
| `0x110~0x114` | F407 → Jetson | 40 B 反馈按 8 字节切 5 帧（字段同 §8.1），100 Hz | 接收端按组规则拼装 |

**组完整性规则**（`CanFbGroup`）：`0x110` 开新组 → `0x111~0x114` 按序填入 → 下一个 `0x110`
冲掉未满旧组 → 组超时 **5 ms** 丢弃 → `crc8` 失败整组丢弃（绝不输出半帧）。

**Jetson 侧参数**：命令 10 ms、flags 10 Hz、组超时 5 ms、反馈超时 100 ms（`state().valid=0`）、
socket 重开 1 s。起 `can0` 与验证：

```bash
sudo slcand -o -s8 -t hw /dev/ttyACM1 can0 && sudo ip link set can0 up   # 编号以 ls /dev/ttyACM* 为准
candump -td can0      # 应看到 0x110~0x114 每 10 ms 一组、组内连续
cansend can0 021#01   # EN 开；随后 020 速度才生效
cansend can0 000#     # 广播急停（DLC=0 也认）
```

**UART 与 CAN 的取舍**：只有一块 CH340 时用 UART（`UnifiedSerial`，底盘 + 云台共线）；
需要更短总线、多设备（ZDT 云台也上 CAN）时用 CAN（`--chassis can0` 或全 CAN `--device can0`）。
两者命令/反馈字段完全一致，上层 `IChassisLink` 不用改。协议细节见底盘仓库
`docs/README.md` **§15**（原 `CAN_CONTROL_PROTOCOL.md`，已合并）与 `UserLib/Comm/README.md`。

### 8.3 UDP 线协议（`gimbal_ctl` ↔ `gimbal`）

**所有字段小端，用显式字节偏移写编解码，不依赖结构体 padding** —— 两端必须逐字节一致。

**上行 24 B**：magic `0x4D454341` / seq / yaw_cdps / pitch_cdps / flags / chk（0~12 XOR）/
vx_mmps / vy_mmps / wz_mradps / chassis_flags / chk2（14~20 XOR）/ pad

**下行 64 B**：云台遥测（前 32 B）+ 底盘遥测（32~53：位姿 / 正解 / 陀螺 / status / valid）
+ 四轮转速（54~61，0.01 RPM）+ chk16b（62~63）

`flags`（上行）：`0x01` Enable · `0x02` Hold · `0x04` Home · `0x08` Zero · `0x10` EStop ·
`0x20` ClearStall · `0x40` Disable · `0x80` Fire（水弹扳机）。
`chassis_flags`：`0x01` EN / `0x02` EStop / `0x04` Clear。

> v1（上行 16 B / 下行 32 B）旧帧仍可解码，v1 的 16 字节前缀在 v2 里不变。

**为什么用 UDP**：控制包极小；**无队头阻塞**（丢包直接丢，等重传反而更糟）；
**无连接状态**（客户端崩溃 / 重连不需握手）。丢包容忍与时效判断交给两端看门狗。

---

## 8.5 pitch 轴整条链路（会话 / 零位 / 限位 / 手感 / IMU / 打滑）

**一句话**：daemon 启动先自检抬 **10°** → 客户端连上再抬 **40°** 并定为 0° → 方向键速率控制（软限位 ±20°）
→ 客户端结束放下 **40°**（回自检位）→ daemon 退出**放松 pitch 电机**（失能，yaw 保持使能）。

**0° 一旦建立就固定**：之后每次会话是"回到这个 0°"，而不是再抬 40°——否则零点每轮爬升 +10°，几轮后枪口贴到限位，表现成"向上抬不动、向下能动"。

```
gimbal_ctl ──UDP 40001──► main.cpp 100Hz ──► PitchSession 状态机 ──► Axis（斜坡/限位）
                                                                    └─► ZdtMotor ──CAN 扩展帧──► ZDT 驱动板 ──► 电机 ──► 枪口
                                                                                                        └─► IMU(ICM-42688-P, I²C) ──► 绝对仰角 / 打滑检测
```

### 8.5.1 会话状态机（`PitchSession`，`src/main.cpp`）

| 状态 | 行为 | 进入 | 离开 |
|---|---|---|---|
| `Locked` | 速率 0、保持使能；IMU 在线时先等零偏标定（约 2 s） | 启动 / 急停 / 放下完成 | 收到合法 UDP 包且就绪 → `Raising` |
| `Raising` | P 伺服抬到目标（≤60 °/s）；到点把该姿态定为 0°（`zeroHere`），IMU 同步清零 | 会话开始 | 到位（±1.5°）→ `Active`；**打滑 / 堵转 / 1 s 无位移 / 8 s 超时 → 中止并锁死** |
| `Active` | PC 速率直通（`--pitch-abs` 时为绝对仰角伺服） | 抬升到位 | 0.5 s（`--pitch-session-end`）无包 → `Lowering`；急停 → `Locked` + latch |
| `Lowering` | P 伺服放下到 `--pitch-lower`（默认 −40°），下限不越过会话起点 | 控制结束 | 到位 → `Locked`；控制端回来 → 回 `Active`；堵转/超时 → 停住抱死 |

- **启动自检**：daemon 起来先抬 `--startup-lift`（默认 **10°**）并保持；它完成前客户端会话会等（`startup_wait`）；
- **客户端会话**：同一进程内第一次会话是"抬 `--pitch-rise`（默认 **40°**）并定 0°"；之后每次会话"回到已建立的 0°"（`have_zero_`），零点不再漂；
- **急停**：`x` 立即停止 + 失能并 **latch**，需按 `g` 关再开（或本地 `en 1`）重新授权；
- **退出**：Ctrl-C / `q` 先放下（最多 6 s），然后**放松 pitch 电机（失能）**；yaw 保持使能；
- **重定零**：重启 daemon 后首次会话会重新抬 40° 定零。

### 8.5.2 零位、软限位与机械行程（三层，机械优先）

| 层 | 参数 | 默认 | 作用 |
|---|---|---|---|
| 启动自检 | `--startup-lift` | **10°** | daemon 启动后先抬这么多并保持（完成前客户端会话等待） |
| 抬升 | `--pitch-rise` | **40°** | 客户端会话首次从当前位抬多少并定为 0° |
| 抬升微调 | `--pitch-rise-extra` | 0° | 在 40° 基础上再加减 |
| 软限位 | `--pitch-limit` | **−20 20** | 0° 上下允许的操作范围 |
| 机械行程 | `--pitch-mech-travel` | **85°** | 相对会话起点最多能抬多少：抬升目标与软限位上界都被它夹住，贴近边缘会打警告 |
| 放下 | `--pitch-lower` | **−40°** | 断流/退出后放到的角度（夹在 `[会话起点, 0°]` 内） |

- 软限位到位后自动收进机械窗口：`hi = min(pitch_max, mech_travel − 抬升行程)`，`lo = max(−pitch_max, −抬升行程)`；
- 放下**没有** −5° 余量（过去会往机械下止点顶）；
- 抬升目标超出行程会被截断并打日志。

### 8.5.3 手感参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `--yaw-ramp` / `--pitch-ramp` | 600 °/s² | 角加速度（0→120 约 0.2 s） |
| `--hold-delay` / `--hold-tau` | 0.12 s / 0.05 s | 停手后衰减 |
| `--deadband` | 0.5 °/s | 速度死区 |
| `--soft-zone` / `--soft-ratio` | 0（关）/ 0.4 | 末段软化：速度很小且往 0 收时用更缓斜坡收尾（重负载停抖可试 8~12 / 0.3） |
| `--acc` | 200 | 驱动板加速度档位（每 RPM 2.8 ms；调小更柔） |
| `--tx-gap` | 8 ms | ZDT 命令最小间隔；速度帧按电机"合并 + 插队"，饱和时执行最新指令 |
| `--damp-k` | 0（关） | 主动阻尼：用 IMU 角速度压停车振铃（0.4~0.6 可试），只在用户没给指令时生效 |
| 客户端 `--tau` | 0.045 s | 方向键低通（越小越跟手、越大越丝滑） |

> 速度帧是**整数 RPM**：pitch 直连 1 RPM = 6°/s，最小非零速度就是 6°/s；台阶均匀（每帧 1 RPM）。
> 12 ms 节流下总线会被"速度帧 + 遥测轮询"挤满，所以速度帧必须合并+插队，否则会执行旧指令、起停顿挫。

### 8.5.4 IMU（ICM-42688-P，I²C）在 pitch 上的三个用途

1. **同步清零**：抬升到位时把 IMU 清零，让 IMU 的 0° 与编码器的 0° 一致；
2. **主动阻尼**（`--damp-k`）：压停车振铃；
3. **绝对仰角保持**（`--pitch-abs`）：方向键速率积分成目标仰角、用 IMU 闭环保持（底盘倾斜补偿）。
   该模式天然是"动编码器、让 IMU 不动"，所以**打滑检测在此模式下关闭**。

零偏：开机自动标定（2 s 窗口、σ<1.5 °/s，运动中自动重来），也可 `--imu-gyro-bias` 手动指定。
**鲁棒性**：模块 AD0 已接地（固定 0x68）；软件另有"3 s 无有效样本按零偏 0 放行"与"读失败每 20 次重探另一地址"两道兜底。

### 8.5.5 打滑检测（联轴器）

运动时比较**编码器增量**与 **IMU 增量**（0.5 s 窗口，est 至少动 2°），比值 > 1.6 判定打滑：

- `Raising` 阶段**中止并锁死**（日志：`疑似联轴器打滑（电机在转、枪口没跟上）`）；
- `Active` 阶段只打一次警告；
- `--pitch-abs` 绝对保持模式下不判断。

实测案例：电机 10 RPM（60 °/s）、编码器 65 °/s，而枪口 IMU 只有 25 °/s —— 联轴器持续打滑；
检测立即中止，避免把 est 推到目标就"上报成功"。

### 8.5.6 排障速查

| 现象 | 先查什么 |
|---|---|
| 向上抬不动、向下能动 | est 是否顶到 +20° 软限位（状态里 `[限位]`）；零点是否在爬升（必须同一 daemon 进程内复用 0°）；机械上止点 |
| 抬升"只抬一点点" | 打滑检测是否中止；电机实测 RPM 与 IMU 角速度是否一致 |
| 抬完抖一下 | 负载惯量引起的 3~5 Hz 振铃，试 `--soft-zone/--soft-ratio`、`--pitch-ramp`、`--damp-k`、`--acc` |
| 客户端按键没反应 | 会话是否卡在"等待 IMU 零偏标定"（IMU 读失败）；现有 3 s 无样本兜底会放行并打警告 |
| IMU 偶发读数全 0 | AD0 已接地（固定 0x68）；仍出现时看日志有无"地址自动切换"/"3 s 无有效样本" |
| 放下后零点越来越高 | 确认在同一 daemon 进程内测试；重启 daemon 会重新抬 40° 定零 |

## 9. 水弹波箱火力（可选）

gimbal 不直接驱动 MOS，而是 fork 一个 Python worker，用 `Jetson.GPIO` 的 BOARD 模式管子：

```
gimbal_ctl 按住 F ──UDP Fire 位(0x80)──► gimbal ──管道 "1\n"/"0\n"──► fire_worker.py ──BOARD 12──► MOS
```

- 触发条件：UDP 扳机位**最后一帧 0.3 s 内有效**，或本地 stdin `f 1|0`；
- 节拍：高 **80 ms** / 低 **120 ms**（环境变量 `FIRE_GRIP_MS` / `FIRE_GAP_MS` 可调），`FIRE_VERBOSE=1` 打时间戳；
- 停火：松键 / 0.3 s 超时 / 急停清本地请求 / 父进程退出（stdin EOF → worker 自杀）/ 析构写 `0\nq\n`；
- ⚠️ **没有连续导通 5 s 上限**（那是 F401 固件方案的保护，见 `fire/README.md`）。

---

## 10. 操作手册

### 10.1 Jetson 端命令行参数（默认值即当前实机配置）

| 参数 | 默认 | 说明 | 参数 | 默认 | 说明 |
|---|---|---|---|---|---|
| `--device` | `/dev/ttyUSB0` | 串口设备（或 `can0`） | `--acc` | 200 | 加速度档位 0–255 |
| `--baud` | 115200 | 波特率 | `--ctrl-port` | 40001 | UDP 控制口 |
| `--yaw-addr` | **1** | yaw 电机地址 | `--no-udp` | off | 不开 UDP，只用 stdin |
| `--pitch-addr` | **2** | pitch 电机地址 | `--hz` | 100 | 控制频率 |
| `--gear` | **2.0** | yaw 传动比 | `--can <iface>` | — | 两轴都走 SocketCAN |
| `--pitch-gear` | **1.0** | pitch 传动比（直连） | `--yaw-can <iface>` | — | 仅 yaw 走 CAN |
| `--pulses` | 3200 | 脉冲 / 电机圈 | `--chassis <serial\|canX>` | serial | 底盘后端 |
| `--yaw-limit a b` | 关闭 | 给 yaw 启软限位 | `--no-chassis` | off | 不接底盘 |
| `--pitch-limit a b` | **−20 20** | pitch 软限位 | `--fire-script` | `src/device/fire_worker.py` | 火力 worker 路径 |
| `--yaw-rate` | 180 | yaw 最大 °/s | `--fire-pin` | 12 | 火力 GPIO（BOARD 号） |
| `--pitch-rate` | 120 | pitch 最大 °/s | `--no-fire` | off | 不启动火力进程 |
| `--selftest` | off | 自检后退出 | `-h` / `--help` | — | 打印用法 |
| `--pitch-session-end` | 0.5 s | 断流多久算控制结束 | `--no-pitch-session` | off | 关闭 pitch 会话（速率直通） |
| `--startup-lift` | **10** | daemon 启动后先抬升 | `--pitch-rise-extra` | 0 | 在 40° 上再微调 |
| `--pitch-rise` | **40** | 客户端会话抬升并定 0° | | | |
| `--pitch-rise-rate` | 60 | 抬升/放下最大 °/s | `--pitch-servo-kp` | 2.0 | 抬升 P 增益 |
| `--pitch-servo-tol` | 1.5 | 到位容差 ° | `--pitch-raise-timeout` | 8 s | 过渡超时 |
| `--pitch-mech-travel` | **85** | 机械行程（相对会话起点） | `--pitch-lower` | **−40** | 放下目标角 |
| `--soft-zone` / `--soft-ratio` | 0 / 0.4 | 末段软化 | `--damp-k` | 0 | 主动阻尼 |
| `--tx-gap` | 8 ms | ZDT 命令间隔 | `--yaw-ramp` / `--pitch-ramp` | 600 | 角加速度 °/s² |
| `--hold-delay` / `--hold-tau` | 0.12 / 0.05 | 停手衰减 | `--deadband` | 0.5 | 死区 °/s |
| `--pitch-abs` | off | 绝对仰角保持 | `--abs-kp` | 3.0 | 绝对伺服 P 增益 |
| `--imu <dev>` | 空 | 启用 IMU（如 `/dev/i2c-1`） | `--imu-addr` | 0x68 | 地址（自动识别 0x68/0x69） |
| `--imu-gyro-axis <x\|y\|z>` | z | 俯仰转轴对应的陀螺轴 | `--imu-invert` | off | 方向翻转 |
| `--imu-offset` | 0° | 安装偏置 | `--imu-alpha` | 0.98 | 互补滤波系数 |
| `--imu-gyro-bias` | 0（自动标定） | 陀螺零偏 °/s | `--imu-log <文件>` | — | IMU 逐帧记录（调抖动/标定用） |
| `--imu-probe` | off | 只读 IMU 打印后退出 | | | |

**本地 stdin 命令**（一行一条）：

| 命令 | 作用 |
|---|---|
| `y <dps>` / `p <dps>` / `r <y> <p>` | 设 yaw / pitch / 两轴角速度（°/s） |
| `h` / `hold` | 停手（**保持使能**，抱死） |
| `home` / `zero` | 回中 / 把当前位置当作 0°（应在机械零点执行） |
| `en 0\|1` | 使能 / 失能 |
| `x` / `estop` | 急停（立即停止 + 失能） |
| `s` | 打印状态 |
| `q` | 退出 |
| `f 1\|0` | 开火 / 停火（调试用） |
| `c <vx> <vy> <wz>` / `cx` | 底盘速度指令（mm/s, mm/s, mrad/s）/ 底盘急停 |

### 10.2 笔记本客户端 `gimbal_ctl`

```bash
cd ~/gimbal_ctl && cmake -S . -B build && cmake --build build -j4
./build/gimbal_ctl --host 10.42.0.1        # 无线（Jetson AP）
./build/gimbal_ctl --host 192.168.2.2      # 有线
./build/gimbal_ctl --host 192.168.2.2 --no-video
```

| 键 | 作用 |
|---|---|
| `←/→` | 云台 yaw 左 / 右（按住转，松开停，`--gimbal-rate` 默认 120 °/s） |
| `↑/↓` | 云台 pitch 抬头 / 低头 |
| `W/S` `A/D` `Q/E` | 底盘前进/后退、左移/右移、左转/右转 |
| **空格** | 停手 / 继续（HOLD 开关，云台 + 底盘 + 发射一起停） |
| `g` | 云台使能 / 失能切换 |
| `j` | 底盘使能 / 失能切换（**底盘急停后按 `j` 恢复**） |
| `x` | 急停（云台 EStop + 底盘 STBY 拉低） |
| **`F`（按住）** | **水弹发射**（按住连发，松开停） |
| `c` | 底盘里程计清零 |
| `h` / `z` / `r` | 云台回中 / 当前位置归零 / 清空当前速率 |
| `v` | 视频开关 |
| `Esc` | 退出（退出前自动发一帧停手指令） |

> 云台改方向键控制后，**鼠标不再参与控制**（`--mode` / `--sens` / `--invert-y` 保留但已不生效）。
> 窗口文字是纯 ASCII（X11 核心字体只有 ASCII，写中文必乱码）。

### 10.3 图传（车载摄像头 → 笔记本）

Jetson 上用 `mjpeg_server`（C++17，V4L2 直出 MJPG，零转码）：

```bash
cd ~/gimbal
./build/mjpeg_server --size 640x480 --fps 30 --port 8080     # 默认就是这个
# 浏览器验证：http://<Jetson_IP>:8080/stream
```

`gimbal_ctl` 启动时自动拉 `http://<host>:8080/stream` 画在窗口里（`v` 键开关）。
带宽参考：640x480@30 实测约 **2 Mbps**；1280x720@30 约 8~15 Mbps。

### 10.4 无线连接（笔记本 ↔ Jetson AP）

**Jetson 侧起 AP**（`car-ap` 是 `autoconnect=no`，需手动拉起）：

```bash
sudo nmcli con up car-ap
cat /proc/net/dev | grep wlan0; sleep 3; cat /proc/net/dev | grep wlan0   # tx 字节要在涨（见 §11.6）
```

AP 参数：SSID `MECAR-CTRL` / 密码 `mecar12345` / 2.4 GHz ch6 / `10.42.0.1/24` / mDNS `nano.local`

**笔记本接入**：

```bash
# ① 连 AP（会切走校园网，暂时没外网，正常）
sudo nmcli device wifi connect MECAR-CTRL password mecar12345
ip -br addr show wlp14s0          # 应拿到 10.42.0.x

# ② 绕过 Meta(Clash) VPN：否则 10.42.0.1 会被塞进隧道
sudo ip rule add to 10.42.0.0/24 lookup main priority 8000

# ③ 验证：必须是 dev wlp14s0，不能是 dev Meta
ip route get 10.42.0.1
ping -c3 10.42.0.1
```

**笔记本 Wi-Fi 省电必须关**（否则 ping 的 RTT 会到 333~488 ms、抖动很大，控车是 100 Hz UDP 扛不住）：

```bash
nmcli con mod <MECAR-CTRL 配置名> 802-11-wireless.powersave 2   # 2 = disable
iw dev wlp14s0 get power_save                                   # 重连后应显示 off
```

> ⚠️ 这个 NetworkManager 版本取值是 `0=default`、`1=ignore`、`2=disable`、`3=enable`——
> 设成 `0`（default）**不会**关掉省电，之前踩过（`iw` 一直显示 on）。

**切回校园网 / 恢复上网**：

```bash
sudo ip rule del to 10.42.0.0/24 lookup main priority 8000   # 删临时绕行规则
sudo nmcli con up 黑大联通                                    # 或你平时连的 WiFi 名
ip -br addr show wlp14s0                                     # 应回到 10.230.x 之类
ping -c3 8.8.8.8                                             # 验证外网
sudo nmcli con down car-ap                                   # 可选：关 AP 省电（在 Jetson 上）
```

> `ip rule` 是**临时**的，每次重连 WiFi / 重启 VPN 后都要重新加。
> 开 AP 时 Jetson 自己连不了别的 Wi-Fi（网卡只能干一件事），有线调试链路不受影响。

---

## 11. 排障

### 11.1 串口被占用 / 串口怎么区分

`/dev/ttyUSB*` 同一时刻只能被一个进程打开，`gimbal` 在跑时再跑测试脚本会打不开：

```bash
fuser -v /dev/ttyUSB0     # 看谁占着
pkill -x gimbal           # 停掉（别用 pkill -f build/gimbal，远程 shell 里会误杀自己的 shell）
```

| 场景 | 设备 |
|---|---|
| F407 USART2（控制链路）+ 云台 ZDT（共用一个 CH340） | `/dev/ttyUSB0` |
| F407 USART1（调试打印） | `/dev/ttyACM0`（DAP-LINK 虚拟串口，笔记本上） |
| 底盘 CAN / 云台 CAN | `can0`（CANable 2.0；编号随插拔变化，以 `ls /dev/ttyACM*` 为准） |

> 2026-10-19 起底盘与云台合并到同一个 CH340（`UnifiedSerial`），不再需要第二路串口；
> 需要时也可 `--yaw-can can0` 把 yaw 单独放 CAN。

### 11.2 反馈帧全部解析失败 / 一帧都解不出来

先确认 **F407 固件版本**与 Jetson 侧一致：

| 版本 | 反馈帧长 | 特征 |
|---|---|---|
| v1（旧） | 32 B | 无四轮转速 |
| **v2（当前）** | **40 B** | 30~37 是四轮转速 |

帧长不匹配时表现为「CRC 错 / 同步错狂涨、一帧都解不出来」。重新烧录固件即可。

### 11.3 用 openocd/SWD 调试后电机还在转

`openocd ... -c "halt"` 会把 CPU 停住，但 **PWM 由定时器硬件输出，CPU 停了照样转**，而且此时看门狗也不工作。
调试完必须恢复运行：

```bash
openocd -f interface/cmsis-dap.cfg -f target/stm32f4x.cfg \
  -c "adapter speed 500" -c "init" -c "reset run" -c "shutdown"
```

或直接按板子上的 RESET。**别让板子停在 halt 状态去碰车。**

### 11.4 没给指令，轮子却按「自旋模式」转（陀螺外环积分残留）

**现象**：发送的 vx/vy/wz 全是 0，但轮速呈现 `FL −, FR −, BL +, BR +`（纯自旋），正解 wz ≈ 0.1 rad/s，
而**陀螺 wz ≈ 0**（车没转），架空时尤其明显。

**根因**：陀螺外环（`chassis_gyro.c`）的积分项被上一次自旋/打滑充满后没人清零——架空时轮子转、车体不转，
陀螺测不到转动 → 误差恒为 0 → 积分永远不泄 → 死锁。

**修复（F407 侧，2026-10-18）**：①`|wz_ref| < 0.02 且 |陀螺 wz| < 0.1` 时积分项按 τ=0.3 s 衰减到 0；
②新增 `chassis_gyro_reset_correction()`，`uart_link` 在停车/急停/看门狗时调用（只清修正量、不动 yaw）。

### 11.5 反馈帧率不是整 100 Hz

`chassis_debug` 每 250 ms 往 USART1 阻塞发一行（约 8.7 ms），会把个别 10 ms 节拍推后，
表现为 ~96 Hz、偶发 20.9 ms 间隔。**不丢帧、不影响控制**；想治就把它改成 IT 发送（最新固件已改）。

### 11.6 车载 AP「假活」：状态 activated 但不发信标

**现象**：`nmcli` 显示 `car-ap:activated`、`iw dev wlan0 info` 看着正常，但笔记本扫不到 SSID。

```bash
cat /proc/net/dev | grep wlan0; sleep 3; cat /proc/net/dev | grep wlan0
# tx 字节不增长 => 真的没在发
sudo nmcli con down car-ap && sudo nmcli con up car-ap    # 修复（Intel 8265 的 AP 模式偶发失效）
```

### 11.7 笔记本连上 AP 但 ping 不通 10.42.0.1

多半是 Meta(Clash) VPN 抢了路由：

```bash
ip route get 10.42.0.1     # 看到 dev Meta 就不对
sudo ip rule add to 10.42.0.0/24 lookup main priority 8000
```

### 11.8 无线控车延迟大 / 抖动

1. 先按 §10.4 确认笔记本 Wi-Fi 省电已关（`iw dev wlp14s0 get power_save` 应为 `off`）；
2. 客户端 `--tau` 默认 30 ms，想更跟手可试 `0.02`，抖了就回 `0.03~0.04`；
3. 有线调试用 `--host 192.168.2.2` 对比，排除是 WiFi 还是链路本身的问题。

### 11.9 台架测试工具（不是产品代码）

`~/stmproject/INTRA_SCHOOL_COMPETITION_27RM/tools/uart_link_test.py`（拷到 Jetson 上一般放 `/tmp/`）：

```bash
pkill -x gimbal                                                     # 串口只能一个进程占用
python3 -u /tmp/uart_link_test.py check -p /dev/ttyUSB0             # 全链路体检（约 10 s）
python3 -u /tmp/uart_link_test.py check --move 100 -p /dev/ttyUSB0  # 三轴方向（需架空）
python3 -u /tmp/uart_link_test.py soak --secs 30 -p /dev/ttyUSB0    # 长时间稳定性
python3 -u /tmp/uart_link_test.py wheels -p /dev/ttyUSB0            # 四轮方向（需架空）
```

> `-u` 必须带（否则输出被缓冲，看起来像卡死）。

**验证链路健康的三个数**：①Jetson `s` 里 yaw/pitch 的 frames 在涨（云台在回帧）；
②底盘 `valid=1`、`cmd_cnt` 在涨（命令到了 F407）；③客户端 `link ... rx=100`、`chassis ONLINE`。

---

## 12. 实测数据

### 12.1 2026-10-18 真机（UART 链路）

| 项 | 结果 |
|---|---|
| 反馈帧率 | 96.0 Hz（受 §11.5 影响） |
| 命令接收 | 99.8 Hz |
| 反馈丢帧 | 0（20 s soak，`fb_seq` 连续） |
| CRC 错 / 同步错 | 0 / 1（开播半帧） |
| 最大接收间隔 | 20.9 ms |
| 三轴方向 | vx/vy/wz 符号全部一致 |
| 急停 | ESTOP 位亮、DRIVER_ON 清除、恢复后正常 |
| 断流 1.2 s | WATCHDOG 亮、LINK_OK 灭、速度归零、STBY 保持 |
| 全栈 | 笔记本 UDP(vx=80 mm/s) → Jetson → F407 → 正解 71 mm/s |
| F407 阶跃响应 | 0→300 mm/s：**t50 = 34 ms，t90 = 64 ms**（轮速斜坡 1500 RPM/s + PID） |
| 图传 | 640x480@30，实测 ~21 fps / ~2 Mbps，HTTP multipart 正常 |

**延迟分解（"慢半拍"从哪来）**

| 环节 | 延迟 | 说明 |
|---|---|---|
| F407 轮速斜坡 + PID | ~64 ms (t90) | 1500 RPM/s 斜坡，防打滑，不建议再快 |
| Jetson 底盘指令周期 | 5 ms | `unified_serial.hpp` 的 `chassis_tx_period_s`（已从 10 ms 降到 5 ms） |
| Jetson 控制循环 | 10 ms | 100 Hz |
| UDP / 网络 | 0.4 ms（有线）/ 几 ms（WiFi） | |
| **客户端平滑 tau** | **30 ms**（默认，原 50 ms） | `--tau` 可调，越小越跟手、越易抖 |
| 遥测显示 | 20 ms | 50 Hz（原 20 Hz） |

### 12.2 CAN 链路实测（2026-10-21，CANable 2.0 ↔ F407 `can_link`）

| 项 | 结果 |
|---|---|
| 建立 | `sudo slcand -o -s8 -t hw /dev/ttyACM1 can0 && sudo ip link set can0 up` → `can0 UP` |
| 反馈组 | `0x110~0x114` 每 ~9.2 ms 一组（≈100 Hz），组内延迟 ~100 µs（远小于 2 ms 承诺） |
| 反馈解码 | 拼回 40 B，`valid=1`，status = `0x00D3`（IMU 在线 + 标定 + 驱动使能 + LINK_OK） |
| 组规则 | `CanFbGroup` 自检 [8] 全绿（顺序/乱序/缺帧/CRC 坏组/非反馈帧） |
| 云台 ZDT | 扩展帧在发（`0x000100/0x000200 35/36 6B` 遥测轮询），但电机仍 UART 模式 → 无应答（待切 CAN 模式 + 1 Mbps） |

### 12.3 2026-10-19 联调（单串口合并 + 无线 + F407 IMU 修复）

**一句话**：把底盘（F407 帧）和云台（ZDT 帧）合到同一个 CH340；Jetson 开 Wi-Fi 热点让笔记本无线控制；
客户端窗口英文化解决乱码；并配合修掉 F407 侧 IMU I2C 地址应答不稳导致的「电机一卡一卡」。
最终 F407 反馈回到 **100 Hz**、IMU `e=0 / CAL=1`、无线 UDP 链路通。

| 项 | 修复前 | 修复后 |
|---|---|---|
| F407 反馈帧率 | 57 Hz（p50 10.1 ms / p90 51.7 ms / max 67.7 ms） | **100 Hz**（2 s 200 帧，200/201 CRC 全对） |
| IMU 错误计数 `e=` | 149 万，持续 +280~520/s | **0** |
| 单次 I2C 读耗时 `us=` | 27 µs（地址 NACK） | 214 µs（正常） |
| 零偏标定 `CAL` | 0（无效） | **1** |
| 采样节拍 `hz=` | 499 | 499 |
| 无线 | — | 笔记本 10.42.0.169 ↔ Jetson 10.42.0.1，ping 通 |
| 共享总线 | — | 跑 gimbal 时 ZDT 应答偶与 F407 反馈抢线，客户端底盘偶发 `OFFLINE`（不影响控制） |

F407 侧根因与修法（完整证据链见底盘仓库 `docs/README.md` §17.7）：
IMU 约一半的读在**地址字节**就被 NACK（`us≈27 µs`，不是 3 ms 超时），推断 SA0 在 0x6A/0x6B 间漂移
→ 连续失败触发**阻塞式总线恢复**（每次 ~25 ms，每 100 mms 采样一次，把采样时间吃掉大半）；
改成非阻塞恢复 + 降低重试频率后，底盘反馈回到 **100 Hz**、IMU 错误计数 `e=0`、`CAL=1`。

### 12.4 2026-11-01 IMU 接入（pitch）

| 项 | 实测 |
|---|---|
| 芯片 / 总线 | ICM-42688-P，I²C 400 kHz（`/dev/i2c-1`，AD0 接地 → 0x68） |
| 采样 | 100 Hz 连续，10 s 999 帧，无零样本 |
| 零偏标定 | Z 轴 +0.38 ~ +0.56 °/s（σ≈0.1~1.1） |
| 安装姿态 | X 沿枪管、Z 为俯仰转轴，抬头 gz 为正（不需要 `--imu-invert`） |
| 融合静态精度 | ≤0.5° |
| 机构静止位 | ≈ −45°（IMU 实测）；自检抬 10° + 客户端抬 40° 后工作位 ≈ +5° 绝对仰角 |
| 联轴器打滑案例 | 电机 10 RPM / 编码器 65 °/s / IMU 25 °/s → 检测在 est≈17° 处中止 |

---

## 13. 代码位置

| 想改什么 | 看哪里 |
|---|---|
| 电机协议（帧/解析） | `include/gimbal/protocol/zdt_frame.hpp`、`src/protocol/zdt_frame.cpp` |
| 传输层（UART / CAN / 节流合并） | `include/gimbal/transport/`、`src/transport/` |
| 单轴控制律（斜坡/死区/限位/手感） | `include/gimbal/gimbal/axis.hpp`、`src/gimbal/axis.cpp` |
| 双轴调度 / 看门狗 | `src/gimbal/gimbal.cpp` |
| pitch 会话状态机 / 主动阻尼 / 打滑检测 | `src/main.cpp`（`PitchSession`、`applyDamping`） |
| IMU 驱动与融合 | `include/gimbal/imu/`、`src/imu/` |
| UDP 线协议 | `include/gimbal/link/control_link.hpp` |
| 底盘链路（UART / CAN） | `src/link/unified_serial.cpp`、`src/link/can_chassis_link.cpp` |
| 水弹开火 | `src/device/fire_control.cpp`、`src/device/fire_worker.py` |
| 自检 | `src/main.cpp` 的 `runSelftest()` |

---

## 14. 已知限制与待办

**限制**：
1. **pitch 半闭环**：角度来自 0x36 单圈累积（编码器），长途运行会缓慢漂；绝对参考靠 IMU，
   但 IMU 目前只做清零 / 阻尼 / 绝对保持，还没接进限位判定。
2. **联轴器打滑只能"发现"不能"修复"**：打滑时 `Raising` 会中止并锁死，需要人工检查联轴器顶丝。
3. **低速分辨率**：速度帧是整数 RPM，pitch 直连 1 RPM = 6°/s，微调精度受限于此。
4. **IMU 偶发 I²C 读失败**：AD0 已接地固定 0x68；软件有"3 s 无样本放行"与"地址重探"兜底。
5. **X 固件的位置模式未实现**；图传不做转码（只支持 MJPG）。
6. **绝对保持模式与打滑检测互斥**（该模式本来就"动编码器、让 IMU 不动"）。

**待办**：
- [ ] 用 IMU 做限位 / 零点校核（目前 est 与 IMU 只是并行记录）
- [ ] 弹道模块（需要实测水弹初速 v0）
- [ ] pitch 位置闭环（目标角伺服，替代现在的"速率 + 软限位"）
- [ ] 联轴器彻底解决后复测 est:IMU 比值应 ≈1.0

---

## 15. 修订记录

| 日期 | 变更 |
|---|---|
| 2026-11-07 | **生命周期调整**：daemon 启动自检抬 10°（`--startup-lift`）→ 客户端连上再抬 40° 定 0° → 客户端结束放下 40° → daemon 退出**放松 pitch 电机**；软限位改为 ±20°；补 IMU 鲁棒性兜底（AD0 接地 / 3 s 无样本放行 / 地址重探）与"按键没反应"排障条目 |
| 2026-11-07 | **pitch 轴整条链路文档化**：新增 §8.5（会话状态机 / 零位固定修零点爬升 / 机械行程与限位夹取 / 放下 −40° / IMU 接入 / 打滑检测 / 手感参数 / 排障速查）；参数表补齐 pitch 会话、手感、IMU 相关项；同步更新 `include/gimbal/imu/README.md`、`src/imu/README.md` |
| 2026-11-05 | **三份文档合并为本 README**（原 `CHASSIS_LINK.md` / `DEBUG_REPORT_2026-10-19.md` 已并入并删除）；顺带修正过期内容：`--dry` / `--chassis-dev` / `--chassis-baud` 等已移除参数、电机地址 yaw=1/pitch=2、pitch 直连 1:1 与限位、客户端按键 `g`/`j`/`F`、补水弹波箱与环境 GPIO 火力说明、补 `fire_control` 相关文件到目录树 |
