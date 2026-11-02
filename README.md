# gimbal —— 麦轮小车云台 + 底盘控制器

两轴云台（yaw + pitch）+ 底盘（STM32F407）的正式控制工程，跑在 Jetson Nano 上。

**云台**：ZDT X42S 闭环步进 ×2，UART（CH340，与底盘共用一个串口）或 **CAN 扩展帧** 两种后端；
**底盘**：F407 标准帧链路，**UART（`UnifiedSerial`，与云台共用一个串口）或 CAN（`CanChassisLink`）** 双后端可选。
两种总线可任意组合，见 §7。

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
| 云台总线 | UART 115200 8N1（CH340）/ SocketCAN（`can0`，ZDT 扩展帧） |
| 底盘总线 | USART2 115200 8N1（与云台共用一个 CH340）/ SocketCAN（`can0`，标准帧） |
| 控制入口 | **UDP 40001**（PC 客户端）/ stdin（本地调试） |
| 图传 | `mjpeg_server`：V4L2 取 MJPG → HTTP multipart，端口 **8080** |

---

## 1. 快速开始

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
ctest --test-dir build          # = ./build/gimbal --selftest，纯软件自检

# 典型后端组合（详见 §7）
./build/gimbal --device /dev/ttyUSB0            # 默认：全串口，底盘 + 云台共用同一个 CH340
./build/gimbal --device /dev/ttyUSB0 --no-chassis  # 只控云台
./build/gimbal --chassis can0                   # 混合：云台 UART + 底盘 CAN（推荐先跑这个）
./build/gimbal --device can0                    # 全 CAN：两轴 ZDT 扩展帧 + 底盘标准帧同一条 can0
./build/gimbal --yaw-can can0                   # 仅 yaw 走 CAN，pitch + 底盘仍走串口
./build/gimbal --selftest                       # 自检后退出（不接硬件）
```

> CAN 后端需要先起 `can0`（CANable 2.0，SLCAN 固件）：
> `sudo slcand -o -s8 -t hw /dev/ttyACM1 can0 && sudo ip link set can0 up`
> （`ttyACM*` 编号随插拔变化，以 `ls /dev/ttyACM*` 为准）。

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

**命令行参数**（`-h` 可打印）：

```
--device <路径>        串口设备 (默认 /dev/ttyUSB0)
--baud <n>             波特率 (默认 115200)
--yaw-addr <n>         yaw 电机地址 (默认 2)
--pitch-addr <n>       pitch 电机地址 (默认 1)
--gear <f>             传动比=电机转数/输出转数 (默认 2.0)
--pulses <n>           脉冲/电机圈 = 200×细分 (默认 3200)
--yaw-limit <a> <b>    给 yaw 启用软限位 (默认关闭)
--pitch-limit <a> <b>  pitch 软限位 (默认 -45 45)
--yaw-rate <dps>       yaw 最大角速度 (默认 180)
--pitch-rate <dps>     pitch 最大角速度 (默认 120)
--acc <0-255>          加速度档位 (默认 200)
--ctrl-port <n>        UDP 控制口 (默认 40001)
--no-udp               不开 UDP（只用本地 stdin）
--hz <n>               控制频率 (默认 100)
--can <iface>          SocketCAN 后端（如 can0；等价 --device can0，两轴同总线）
--yaw-can <iface>      仅 yaw 走 SocketCAN（如 can0），pitch 仍走 --device 串口
--chassis <serial|canX> 底盘后端：serial=与云台共用 --device（默认），can0=底盘走 CAN
--no-chassis           不接底盘（只控云台）
--selftest             自检后退出
```

> 已移除 `--dry`（模拟电机跑交互，FakeTransport 现仅用于 `--selftest`）与
> `--chassis-dev/--chassis-baud`（底盘不再单独占口，与云台共用 `--device`）。

### 启动 / 停止 / 查后台（jetson@nano:~/gimbal$）

**启动（前台，推荐）**——能看到状态、能用交互命令：

```bash
jetson@nano:~/gimbal$ ./build/gimbal --device /dev/ttyUSB0
```

**启动（后台）**——关终端也不退出：

```bash
jetson@nano:~/gimbal$ nohup ./build/gimbal --device /dev/ttyUSB0 > /tmp/gimbal.log 2>&1 < /dev/null &
jetson@nano:~/gimbal$ tail -f /tmp/gimbal.log        # 看日志
```

> 后台方式的 stdin 是空的，`s`/`q` 等交互命令用不了；想用就前台跑。

**停止**

| 场景 | 命令 |
|---|---|
| 前台跑的 | 在终端按 `q`（优雅退出，会先停手/停车）或 `Ctrl+C` |
| 后台跑的 | `kill <PID>`（PID 用 `pgrep -af 'build/gimbal'` 查） |
| 按名字杀 | `pkill -f 'build/gimbal'` |
| 卡死最后手段 | `kill -9 <PID>`（可能来不及收尾，慎用） |

> ⚠️ **串口是独占的**：旧进程没退干净就起新的，新的会打不开 `/dev/ttyUSB0`。顺序永远是先关旧、再开新。

**查后台**

| 目的 | 命令 |
|---|---|
| gimbal 在不在跑 | `pgrep -af 'build/gimbal'` |
| 谁占着串口 | `fuser -v /dev/ttyUSB0` 或 `lsof /dev/ttyUSB0` |
| 谁监听 UDP 40001 | `ss -uln \| grep 40001` |
| 看后台日志 | `tail -f /tmp/gimbal.log` |

**一条龙**：

```bash
jetson@nano:~/gimbal$ pgrep -af 'build/gimbal'              # 查
jetson@nano:~/gimbal$ kill $(pgrep -f 'build/gimbal')       # 关
jetson@nano:~/gimbal$ ./build/gimbal --device /dev/ttyUSB0   # 开
```

---

## 2. 目录结构

```
gimbal/                             25 个源文件 / 约 4900 行 + 3 份文档
├── CMakeLists.txt                   构建脚本：gimbal_core 静态库 + gimbal / mjpeg_server 两个可执行
├── README.md                        本文（概览 / 设计决策 / 踩坑）
├── CHASSIS_LINK.md                  底盘链路 + 图传 + 无线连接 的使用与排障手册
├── DEBUG_REPORT_2026-10-19.md       2026-10-19 联调记录（单串口合并 / 无线 / IMU 修复，历史留档）
│
├── include/gimbal/                  ── 头文件：接口与线协议 ──
│   ├── protocol/                    ZDT（张大头）闭环步进电机线协议
│   │   ├── zdt_codes.hpp            功能码 / 返回码 / 状态位 / 固件类型（Emm5.0 vs X 差异）
│   │   └── zdt_frame.hpp            帧构造器声明 + 流式解析器（粘包/半包切帧）
│   ├── transport/                   与总线无关的收发层（换总线只换这里）
│   │   ├── itransport.hpp           ★ 抽象接口：12 ms 发送节流队列 + 收帧队列 + 统计
│   │   ├── can_transport.hpp        SocketCAN 实现（29-bit 扩展帧拆包重组，ZDT 云台用）
│   │   └── fake_transport.hpp       纯软件电机模拟（--selftest 用，无需硬件）
│   ├── device/
│   │   └── zdt_motor.hpp            单台电机：使能 / 速度 / 位置 / 急停 / 遥测轮询
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
    │   └── zdt_motor.cpp            物理量 ↔ ZDT 帧、遥测解析、传动比/脉冲换算
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

> 行数分布（判断改动影响面用）：
> `main.cpp` 978 · `unified_serial.cpp` 356 · `can_chassis_link.cpp` 278 · `mjpeg_server.cpp` 401 ·
> `zdt_motor.cpp` 221 · `zdt_frame.cpp` 200 · `udp_control_link.cpp` 175 · `can_transport.cpp` 179 ·
> `axis.cpp` 148 · `gimbal.cpp` 111 · `itransport.cpp` 77。
> 头文件：`control_link.hpp` 346 · `chassis_link.hpp` 194 · `can_chassis_link.hpp` 121 ·
> `unified_serial.hpp` 146 · `itransport.hpp` 91 · `fake_transport.hpp` 204 · `udp_control_link.hpp` 75 ·
> `can_transport.hpp` 61 · `zdt_motor.hpp` 121 · `axis.hpp` 94 · `gimbal.hpp` 89 ·
> `zdt_codes.hpp` 104 · `zdt_frame.hpp` 97。
>
> ★ = 协议/接口的关键文件，改动要三端（固件 ↔ Jetson ↔ PC 客户端）同步。

---

## 3. 分层设计

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

### 3.5 传动比（**最容易被忽略、但会全盘算错**）

`--gear` 是 **电机转数 / 输出轴转数**（2:1 减速填 2.0）。它影响两处：

```
输出 °/s  ->  电机 RPM :  rpm = dps × gear / 6       （2:1 时 90°/s -> 30 RPM）
电机 RPM  ->  输出 °/s :  dps = rpm × 6 / gear
脉冲/输出圈              :  3200 × gear = 6400
```

自检里有断言：`90°/s @ gear=2.0 -> 电机 30.0 RPM`。

## 4. 关键设计决策（都来自实测踩坑）

### 4.1 命令节流 12 ms —— 防 ZDT 粘包

ZDT 手册 §3.5.2：「每条命令发送完后，要延时几毫秒，防止命令之间粘包失效」

实测（两条命令间隔 → 两条都回复的成功率）：

| 间隔 | 成功率 |
|---|---|
| **0 ms** | **1/5** ❌ |
| ≥1 ms | 5/5 ✅ |

本工程取 **12 ms** 留足余量。实现在 `ITransport::nextTx()`。

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

线协议见 `include/gimbal/link/control_link.hpp`（**v2：上行 24 字节 / 下行 64 字节**，
v1 的 16/32 字节旧帧仍可解码）。两端**必须逐字节一致**，用显式字节偏移写编解码，不依赖结构体 padding。

`UdpControlLink` 是**单线程非阻塞**的：`poll()` 在控制循环里被调用，
不额外开线程 —— 控制循环本来就要跑 100Hz，顺手收包即可。遥测按 50 Hz 回发。

### 4.7 CAN 底盘链路：组规则 + TX 队列

- **`0x110~0x114` 五帧不是原子的**：`CanFbGroup` 按「`0x110` 开组 → 按序填入 → 下一个
  `0x110` 冲掉未满组 → 组超时 5 ms 丢弃 → CRC8 失败整组丢弃」拼装，坏组绝不输出半帧。
- **F407 只有 3 个发送邮箱**：F407 侧用 20 帧深 TX 队列连续发（组内 ≤2 ms），
  Jetson 侧 10 ms 周期收到即收，组超时设 5 ms 有 2.5 倍余量。
- **`0x021` 是标志帧不是命令帧**：EN=0 立即停车，但 F407 侧不计入 `cmd_cnt`、不刷新看门狗。
- 详细帧表 / 语义 / 联调记录见底盘仓库 `docs/CAN_CONTROL_PROTOCOL.md`。

### 4.8 同步运动预留

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

本工程默认走**速度模式**做控制，天然绕开这个换算问题。

---

## 6. 自检覆盖范围

`--selftest` 不需要任何硬件，共 **9 段**：

| 段 | 验证内容 |
|---|---|
| [1] 帧构造 | `mkEnable/mkSpeed/mkEStop/mkGet/mkPositionEmm` 逐字节比对期望 hex |
| [2] 解析器 | 混入垃圾字节后仍能正确切帧 + 校验 |
| [3] 角度↔脉冲 | 360°→3200、90°→800（用 FakeTransport 构造 ZdtMotor） |
| [4] UDP 线协议 | 上行 24B / 下行 64B 编解码往返、坏包/短包被拒 |
| [5] 端到端 | 一总线双电机 + 传动比 2:1：地址路由、斜坡、使能、遥测、看门狗（触发但**不失能**）、软限位、急停后失能 |
| [6] 底盘协议 | 命令帧编码向量、反馈帧解码向量、单字节破坏被拒、急停帧 cmd=0x02 且不带 EN |
| [7] 单总线帧分流 | `RxDemux` 半包/逐字节喂入、底盘帧 + ZDT 帧交错、垃圾字节、无坏帧 |
| [8] CAN 反馈组拼装 | `CanFbGroup`：0x110~0x114 顺序/乱序/缺帧/CRC 坏组/非反馈帧 |
| [9] 混合传输 | yaw 走 CAN、pitch 走 UART（两轴不同总线），命令/回读各自落在正确总线 |

## 7. 底盘链路（已实现，UART + CAN 双后端）

`include/gimbal/link/chassis_link.hpp`：`IChassisLink` 抽象 + `ChassisCommand` / `ChassisState` +
帧编解码纯函数（与 F407 `UserLib/Comm/uart_link_config.h` **逐字节一致**）。

```cpp
struct ChassisCommand { double vx_mmps, vy_mmps, wz_mradps; bool enable, estop, clear_odom; };
struct ChassisState   { double vx/vy/wz, odom_x/y/yaw, gyro_yaw/wz, wheel_rpm[4];
                        uint16_t status, cmd_cnt, fb_seq; bool valid; double age_s; };
```

🚗 **坐标约定**：`vx` 前进、`vy` 左移、`wz` 逆时针（与底盘库一致，旧「右/前」草案已废弃）。

**UART 后端（`UnifiedSerial`，默认）**：云台 ZDT 帧与底盘 F407 帧共用一个 CH340 串口，
RX 按帧边界分流（`5A A5 81`+40B+CRC+`0D` → 底盘；`Addr∈1..32`+func+…+`6B` → ZDT），
底盘命令 5 ms 节流、反馈超时 0.3 s 置 `valid=false`。用于没有第二路串口、或不需要 CAN 的场合。

**CAN 后端（`CanChassisLink`，SocketCAN 标准帧）**：

```
0x000  广播急停（DLC 任意；Jetson 发、F407 只收）——最高优先级
0x020  速度命令 100 Hz：cmd seq vx(i16) vy(i16) wz(i16 0.01rad/s)
0x021  标志帧 10 Hz 保活 + flags 变化时立即发：flags(bit0 EN)
0x110~0x114  反馈 40 B 按 8 字节切 5 帧，100 Hz；组超时 5 ms、CRC 失败整组丢弃
```

命令帧/反馈帧字段与 UART 版完全一致（只是搬运方式不同）。打开逻辑：
`--device` 或 `--chassis` 选 `can0` 即走 SocketCAN；`CanFbGroup` 组拼装见 §4.7。

**后端组合与仲裁**：

| 想怎么接 | 参数 | 总线 |
|---|---|---|
| 全串口 | 默认（`--device /dev/ttyUSB0`） | 云台 ZDT + 底盘共用 1 个 CH340 |
| 混合 | `--chassis can0` | 云台 UART + 底盘 CAN |
| 全 CAN | `--device can0` | 云台 ZDT 扩展帧 + 底盘标准帧共用 `can0` |
| yaw CAN | `--yaw-can can0` | yaw 走 CAN，pitch + 底盘走串口 |
| 只控云台 | `--no-chassis` | 关底盘链路 |

- 命令字：`01` 速度 / `02` 急停 / `03` 小陀螺 / `04` 清零；flags `0x01` = EN
- 看门狗：Jetson 250 ms 无 UDP → 发 EN=0；F407 100 ms 无帧 → 斜坡停车（**不失能**）
- 完整字段表、状态位、排障方法见 [`CHASSIS_LINK.md`](CHASSIS_LINK.md)

⚠️ 云台 ZDT 切 CAN 模式后（面板/上位机切 + 1 Mbps），与底盘标准帧同总线混跑，
两者 ID 类型不同（11-bit vs 29-bit 扩展帧）天然隔离，不冲突。

---

## 8. 下一步

- [x] **PC 端控制客户端**：原生 X11 客户端 `../gimbal_ctl`（已完成；云台改方向键 + 底盘 WASD）
- [x] **底盘链路 v2**：命令 12 B / 反馈 40 B 含四轮转速，UART 真机验证 × UDP→Jetson→F407 全栈跑通
- [x] **CAN 底盘链路**：`CanChassisLink`（SocketCAN）+ `CanFbGroup` 组拼装，自检 [8] 全绿、实测 100 Hz 反馈
- [ ] **云台联调**：接上第二个 CH340 + 两个 ZDT（确认方向极性、pitch 软限位范围）或切 CAN 模式
- [ ] **云台 ZDT 切 CAN 模式 + 1 Mbps**：面板/上位机切换，切换后 `candump can0` 看到扩展帧应答
- [ ] **`--device can0` 全 CAN 端到端联调**：底盘标准帧 + ZDT 扩展帧同总线混跑
- [ ] 双轴同步运动（多机同步标志 / 多电机命令）
- [ ] 位置模式（绝对角度指向）

---

## 9. 已知限制

1. **角度估计会漂移**：`0x36` 对 Emm 固件是单圈值，取不到多圈绝对值，
   所以 `estAngleDeg()` 是靠转速积分出来的软件估计。
   真正的绝对保护应依赖驱动板的限位开关输入（状态位 bit4/bit5）。
2. **ZDT CAN 传输层未上机验证**：`can_transport.cpp` 帧封装/拆包/重组按手册写好了，
   但云台 ZDT 尚未切 CAN 模式，联调前请先 `candump can0` 抓包核对。
3. **X 固件的位置模式未实现**：`mkPositionEmm()` 只覆盖 Emm；用 X 固件需按
   手册 §5.3.10 补一个帧构造器。
4. **图传不做转码**：`mjpeg_server` 直接把摄像头输出的 MJPG 原样转发，
   不支持 YUYV-only 的摄像头；低光下自动曝光会拉长到 1/15 s，帧率掉到 ~16-20 fps。
5. **`can0` 编号随插拔变化**：`/dev/ttyACM*` 以实际枚举为准，`CanChassisLink` 打开失败时
   会按 `reconnect_s`（1 s）重试，但接口名写死 `can0` 时需要人工确认。

---

## 10. 详细文档索引

| 想看什么 | 去哪 |
|---|---|
| 头文件层（接口 / 线协议 / 分层） | [`include/gimbal/README.md`](include/gimbal/README.md) |
| 源码层（main.cpp 流程 / 各 cpp） | [`src/README.md`](src/README.md) |
| 底盘链路使用与排障（UART + CAN、图传、无线） | [`CHASSIS_LINK.md`](CHASSIS_LINK.md) |
| 2026-10-19 联调记录（单串口合并 / 无线 / IMU） | [`DEBUG_REPORT_2026-10-19.md`](DEBUG_REPORT_2026-10-19.md) |
| F407 固件与协议 | `~/stmproject/INTRA_SCHOOL_COMPETITION_27RM`（`UserLib/Comm/README.md`、`docs/CAN_CONTROL_PROTOCOL.md`） |
| PC 客户端 | `~/gimbal_ctl` |

### 修订记录

| 日期 | 说明 |
|---|---|
| 2026-10-18 | 重写上位机工程文档：五层架构、ZDT Emm5.0 协议要点、UDP v1 协议、底盘 UART 链路 v2 |
| 2026-10-19 | 单串口统一链路（`UnifiedSerial` 合并底盘 + 云台）、无线 AP、客户端英文化、UDP 协议升 v2（24 B/64 B 含底盘字段） |
| 2026-10-21 | 新增 CAN 底盘链路（`CanChassisLink` + `CanFbGroup`）、`--yaw-can` / `--no-udp` 参数、自检补 [8][9]；文档同步到当前代码（行数、目录树、双后端组合） |
