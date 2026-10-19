# gimbal —— 麦轮小车云台控制器

两轴云台（yaw + pitch）的正式控制工程。当前阶段**只做云台**，底盘仅预留接口。

| 项 | 值 |
|---|---|
| 平台 | Jetson Nano（L4T R32.6.1 / Ubuntu 20.04 / kernel 4.9.253-tegra） |
| 语言 | **C++17**（g++ 8.4.0） |
| 构建 | CMake 3.16.3 |
| 云台电机 | ZDT X42S 第二代闭环步进 ×2，**Emm5.0 固件** |
| 电机地址 | **yaw = 2**，**pitch = 1** |
| 细分 / 脉冲 | 细分 **16** → **3200 脉冲/电机圈** |
| 传动比 | **2:1**（电机转 2 圈 = 输出轴 1 圈） |
| 机械限位 | **yaw 有导电滑环，360° 无机械限位**；pitch 有软限位 |
| 总线 | UART 115200 8N1（CH340 `/dev/ttyUSB0`）；CAN 传输层已预留 |
| 控制入口 | **UDP 40001**（PC 鼠标客户端）/ stdin（本地调试） |

---

## 1. 快速开始

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4

./build/gimbal --selftest      # 纯软件自检，不需要任何硬件
./build/gimbal --dry           # 模拟电机跑交互
./build/gimbal                 # 接真机（/dev/ttyUSB0）
./build/gimbal --device /dev/ttyTHS1 --pulses 3200
```

**交互命令**（标准输入，一行一条）：

| 命令 | 作用 |
|---|---|
| `y <dps>` | 设 yaw 角速度（°/s，正 = 角度增大） |
| `p <dps>` | 设 pitch 角速度 |
| `r <y> <p>` | 同时设两轴 |
| `h` / `hold` | 停手（**保持使能**，抱死） |
| `home` | 回中 |
| `zero` | 把当前位置当作 0°（应在机械零点执行） |
| `en 0\|1` | 使能 / 失能 |
| `x` / `estop` | 急停（立即停止 + 失能） |
| `s` | 打印状态 |
| `q` | 退出 |

**常用参数**：

```
--yaw-addr 2 --pitch-addr 1        电机地址
--yaw-limit -180 180               软限位
--pitch-limit -45 45
--max-rate 180 120                 最大角速度 °/s
--pulses 3200                      脉冲/圈 = 200 × 细分  ⚠️ 必须按实际细分改
--acc 200                          加速度档位 0-255
--hz 100                           控制频率
--dry / --selftest
```

---

## 2. 目录结构

```
gimbal/                              21 个源文件 / 约 3300 行 + 2 份文档
├── CMakeLists.txt                   构建脚本：gimbal_core 静态库 + gimbal / mjpeg_server 两个可执行
├── README.md                        本文（概览 / 设计决策 / 踩坑）
├── CHASSIS_LINK.md                  底盘链路 + 图传 + 无线连接 的使用与排障手册
│
├── include/gimbal/                  ── 头文件：接口与线协议 ──
│   ├── protocol/                    ZDT（张大头）闭环步进电机线协议
│   │   ├── zdt_codes.hpp            功能码 / 返回码 / 状态位 / 固件类型（Emm5.0 vs X 差异）
│   │   └── zdt_frame.hpp            帧构造器声明 + 流式解析器（粘包/半包切帧）
│   ├── transport/                   与总线无关的收发层（换总线只换这里）
│   │   ├── itransport.hpp           ★ 抽象接口：12 ms 发送节流队列 + 收帧队列 + 统计 + 热插拔
│   │   ├── uart_transport.hpp       UART 实现（termios 115200，断线自动重连）
│   │   ├── can_transport.hpp        SocketCAN 实现（29-bit 扩展帧拆包重组，未上机验证）
│   │   └── fake_transport.hpp       纯软件电机模拟（--dry / --selftest 用，无需硬件）
│   ├── device/
│   │   └── zdt_motor.hpp            单台电机：使能 / 速度 / 位置 / 急停 / 遥测轮询
│   ├── gimbal/
│   │   ├── axis.hpp                 单轴控制律：角速度斜坡 + 死区 + 软限位 + 回中 + 角度估计
│   │   └── gimbal.hpp               双轴协调：统一时钟 + 两级看门狗 + 传输层 poll 去重
│   └── link/                        ── 通信链路（对上层 / 对下位机）──
│       ├── control_link.hpp         ★ UDP 线协议 v2：上行 24 B / 下行 64 B，小端编解码 + 校验和
│       ├── udp_control_link.hpp     UDP 收发实现：非阻塞 socket、按频率回发遥测、丢包统计
│       └── chassis_link.hpp         ★ 底盘链路：F407 USART2 协议（命令 12 B / 反馈 40 B）
│                                    + UartChassisLink（termios + 非阻塞 + 自动重连 + 100/200 Hz 节流）
│
└── src/                             ── 实现 ──
    ├── main.cpp                     入口：参数解析 / 100 Hz 主循环 / 本地 stdin 命令 / 自检 / 底盘转发
    ├── protocol/
    │   └── zdt_frame.cpp            帧构造与流式解析实现（校验 0x6B）
    ├── transport/
    │   ├── itransport.cpp           节流队列、收帧按地址路由、统计维护
    │   ├── uart_transport.cpp       termios 打开、poll 读写、断线重连
    │   └── can_transport.cpp        SocketCAN 收发 + 扩展帧拆包重组
    ├── device/
    │   └── zdt_motor.cpp            物理量 ↔ ZDT 帧、遥测解析、传动比/脉冲换算
    ├── gimbal/
    │   ├── axis.cpp                 单轴控制律（斜坡/死区/限位/回中）
    │   └── gimbal.cpp               双轴调度、看门狗、estop/home/enable 协调
    ├── link/
    │   └── udp_control_link.cpp     UDP 收包解码、按频率回发遥测、RTT/丢包统计
    └── video/
        └── mjpeg_server.cpp         ★ 图传：V4L2 取 MJPG 帧 → HTTP multipart 转发（零转码）
```

> 行数分布（判断改动影响面用）：
> `main.cpp` 752 · `chassis_link.hpp` 423 · `mjpeg_server.cpp` 401 · `control_link.hpp` 346 ·
> `zdt_motor.cpp` 221 · `axis.cpp` 148 · `udp_control_link.cpp` 175 · `can_transport.cpp` 172 ·
> `uart_transport.cpp` 167 · `zdt_frame.cpp` 200 · `fake_transport.hpp` 204 · `itransport.cpp` 77
>
> ★ = 协议/接口的关键文件，改动要三端（固件 ↔ Jetson ↔ PC 客户端）同步。

---

## 3. 分层设计

```
   main.cpp  ──  控制循环 100Hz + stdin 交互
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
        ├─ UartTransport   (当前)
        ├─ CanTransport    (预留)
        └─ FakeTransport   (自测)
```

**换总线只换一个实现类**，上层一行不用改。

---

## 3.5 传动比（**最容易被忽略、但会全盘算错**）

`--gear` 是 **电机转数 / 输出轴转数**（2:1 减速填 2.0）。它影响两处：

```
输出 °/s  ->  电机 RPM :  rpm = dps × gear / 6       （2:1 时 90°/s -> 30 RPM）
电机 RPM  ->  输出 °/s :  dps = rpm × 6 / gear
脉冲/输出圈              :  3200 × gear = 6400
```

自检里有断言：`90°/s @ gear=2.0 -> 电机 30.0 RPM`。

## 4. 关键设计决策（都来自实测踩坑）

### 4.1 命令节流 12ms —— 防 ZDT 粘包

ZDT 手册 §3.5.2：「每条命令发送完后，要延时几毫秒，防止命令之间粘包失效」

实测（两条命令间隔 → 两条都回复的成功率）：

| 间隔 | 成功率 |
|---|---|
| **0 ms** | **1/5** ❌ |
| ≥1 ms | 5/5 ✅ |

本工程取 **12ms** 留足余量。实现在 `ITransport::nextTx()`。

### 4.2 `urgent` 是插队，**不是清空队列**

踩过的坑：把 `urgent` 实现成 `txq_.clear()`，导致 `Gimbal::estop()` 里
`yaw_.estop()` 排入的「失能」命令，被紧跟着的 `pitch_.estop()` 的 clear 直接清掉
—— **yaw 永远没被失能**。一根总线挂多台电机时这是致命的。

正确语义：`push_front()` 插队到最前，绝不动别人的命令。

### 4.3 遥测轮询每项独立计时

不能用轮转指针 `next = (next+1) % N` —— 遇到长周期项会卡住等一整拍，
表现为「状态冻结几秒 → 猛跳一次」（实测回帧率从应有的 11.5/s 掉到 2.5/s）。

`ZdtMotor` 里每个遥测项各持一个 `next_ms`，且初始值错开 5ms 避免同拍成簇。

### 4.4 看门狗分两级，且**只归零不失能**

| 级别 | 触发条件 | 动作 |
|---|---|---|
| 手感层 | `rate_hold_s`(0.12s) 无新指令 | 角速度指数衰减到 0 |
| 安全层 | `command_timeout_s`(0.5s) 无指令 | 强制 hold，**保持使能** |

**为什么不失能**：云台一旦失能就松轴，受重力会垂下来 —— 比继续转更危险。

### 4.5 `Gimbal::enable()` 必须幂等 ⭐

操作端**每个控制包**都会带 `Enable` 标志（意思是"保持使能"）。
如果 `enable(true)` 每次都把 `enable_age_s_` 清零，`enable_delay_s`（上电空档）
就永远走不完，两轴会被永久压在 `hold()` 上 —— **表现是"发什么都是 0，但状态全绿"**。
这个坑实地踩过一次，排查花了很久。

### 4.6 UDP 操作端链路

线协议见 `include/gimbal/link/control_link.hpp`（16 字节上行 / 32 字节下行）。
两端**必须逐字节一致**，用显式字节偏移写编解码，不依赖结构体 padding。

`UdpControlLink` 是**单线程非阻塞**的：`poll()` 在控制循环里被调用，
不额外开线程 —— 控制循环本来就要跑 100Hz，顺手收包即可。

### 4.7 同步运动预留

ZDT 支持两种双轴同步（手册 §3.5.2）：
- **多机同步标志**：两轴各发带 `sync=01` 的命令（先缓存不动），再广播 `00 FF 66 6B` 触发同时运动
- **多电机命令**（X42S/Y42 专属）：`AA <总长> <cmd1> <cmd2> …` 一条发完

当前是速率控制，暂时用不到；做位置模式时再接入。

---

## 5. ⚠️ 单位换算是个坑（务必看懂）

**Emm 固件与 X 固件的位置模式单位完全不同**：

| | Emm5.0（本工程默认） | X |
|---|---|---|
| 位置命令 4 字节 | **脉冲数** | 0.1° 角度 |
| 速度单位 | RPM | 0.1 RPM |
| 位置回读 `0x36` | **0-65535 = 一圈 0-360°** | 角度/10 |
| 位置模式帧长 | 13 字节 | 不同（见手册 §5.3.10） |

所以 Emm 固件下：
```
脉冲数 = 角度/360 × (200 × 细分)
```
`--pulses` 必须按**驱动板实际细分**设置。默认填 3200（200×16），
**联调前请用 `42` 读驱动参数确认**（第 9 字节是 Microsteps）。

本工程默认走**速度模式**做鼠标控制，天然绕开这个换算问题。

---

## 6. 自检覆盖范围

`--selftest` 不需要任何硬件，验证：

1. **帧构造** —— 逐个字节比对期望 hex
2. **解析器** —— 混入垃圾字节后仍能正确切帧 + 校验
3. **单位换算** —— 角度 ↔ 脉冲
4. **端到端** —— 一总线挂两台电机：地址路由、斜坡、使能、遥测回读、
   看门狗（触发但**不失能**）、软限位、急停

---

## 7. 底盘链路（已实现，协议 v2）

`include/gimbal/link/chassis_link.hpp`：`IChassisLink` 抽象 + `UartChassisLink` 真实现
（CAN 版仍占位，见 §9）。

```cpp
struct ChassisCommand { double vx_mmps, vy_mmps, wz_mradps; bool enable, estop, clear_odom; };
struct ChassisState   { double vx/vy/wz, odom_x/y/yaw, gyro_yaw/wz, wheel_rpm[4];
                        uint16_t status; bool valid; double age_s; };
```

🚗 **坐标约定**：`vx` 前进、`vy` 左移、`wz` 逆时针（与底盘库一致，旧"右/前"草案已废弃）。

**USART2 协议（115200，双向 100 Hz）**：

```
命令帧 12 B：A5 5A | cmd | seq | vx(i16 mm/s) | vy | wz(i16 0.01rad/s) | flags | CRC8
反馈帧 40 B：5A A5 | 0x81 | seq | x(i32 mm) y(i32 mm) | yaw | 解算vx/vy/wz |
             陀螺yaw/wz | status(u16) | cmd_cnt | fb_seq | 四轮RPM(4×i16) | CRC8 | 0x0D
```

- 命令字：`01` 速度 / `02` 急停 / `03` 小陀螺 / `04` 清零；flags `0x01` = EN
- 看门狗：Jetson 250 ms 无 UDP → 发 EN=0；F407 100 ms 无帧 → 斜坡停车（**不失能**）
- 完整字段表、状态位、排障方法见 [`CHASSIS_LINK.md`](CHASSIS_LINK.md)

⚠️ 云台 ZDT 走 UART 扩展帧（CAN 模式），与底盘标准帧天然隔离，二者不冲突。

---

## 8. 下一步

- [x] **鼠标控制**：PC 端原生 X11 客户端 `../gimbal_ctl`（已完成）
- [ ] **云台联调**：接上第二个 CH340 + 两个 ZDT（现在唯一的 CH340 给了 F407），
      确认方向极性、pitch 软限位范围
- [ ] CAN 总线接入（KH-UCANFD 已装 `kcan` 驱动，`can0/can1` 可用）
- [ ] 双轴同步运动（多机同步标志 / 多电机命令）
- [ ] 位置模式（绝对角度指向）
- [x] 底盘链路协议落地（v2：命令 12 B / 反馈 40 B 含四轮转速，已真机验证）

---

## 9. 已知限制

1. **角度估计会漂移**：`0x36` 对 Emm 固件是单圈值，取不到多圈绝对值，
   所以 `estAngleDeg()` 是靠转速积分出来的软件估计。
   真正的绝对保护应依赖驱动板的限位开关输入（状态位 bit4/bit5）。
2. **CAN 传输层未验证**：帧封装/拆包/重组按手册写好了，但无硬件验证，
   联调前请先 `candump can0` 抓包核对。
3. **X 固件的位置模式未实现**：`mkPositionEmm()` 只覆盖 Emm；用 X 固件需按
   手册 §5.3.10 补一个帧构造器。
4. **`CanChassisLink` 仍是占位**：CAN 适配器在 Linux 下不可用（见 CHASSIS_LINK.md），
   UART 链路已验证；换总线时只需实现 `poll()`/`setCommand()`，上层不改。
5. **图传不做转码**：`mjpeg_server` 直接把摄像头输出的 MJPG 原样转发，
   不支持 YUYV-only 的摄像头；低光下自动曝光会拉长到 1/15 s，帧率掉到 ~16-20 fps。
