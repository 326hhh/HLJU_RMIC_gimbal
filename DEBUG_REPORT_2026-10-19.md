# 联调记录 2026-10-19：单串口合并 · 无线交互 · IMU 故障修复

> 本文记录 2026-10-19 当天的联调工作，供队友/后续接手者快速了解现状。
> 涉及两个仓库：
> - 本仓库（Jetson `~/gimbal`）：上位机侧改动
> - 底盘仓库 `~/stmproject/INTRA_SCHOOL_COMPETITION_27RM`：F407 侧改动（详见其 `docs/CHASSIS_LIBRARY.md` §17.8）
> - 客户端仓库（笔记本 `~/gimbal_ctl`）：窗口文字英文化

---

## 0. 一句话总结

把底盘（F407 帧）和云台（ZDT 帧）**合并到同一个 CH340 串口**（三块 MCU 挂一条总线），
Jetson 开 **Wi-Fi 热点**让笔记本无线控制，客户端窗口**改英文**解决乱码，
并配合修复了 F407 侧 **IMU I2C 地址应答不稳**导致的「电机一卡一卡」。
最终：F407 反馈回到 **100 Hz**、IMU `e=0 / CAL=1`、无线 UDP 链路通。

---

## 1. 单串口统一链路（`UnifiedSerial`）

### 1.1 背景

一块 CH340（USB-TTL）线上挂着三块 MCU：

```
Jetson CH340 TX ──→ F407 USART2_RX + 云台 ZDT yaw RX + 云台 ZDT pitch RX
Jetson CH340 RX ←── F407 USART2_TX + 云台 yaw TX + 云台 pitch TX   （三机共线）
```

旧代码是两个串口：云台 ZDT 走 `/dev/ttyUSB0`、底盘 F407 走 `/dev/ttyUSB1`。
只有一块 CH340 时底盘链路打不开，所以合并成一个口。

### 1.2 实现（新增文件）

- `include/gimbal/link/unified_serial.hpp`
- `src/link/unified_serial.cpp`

`UnifiedSerial` 同时实现两个接口，独占一个 fd：

| 接口 | 用途 | 说明 |
|---|---|---|
| `ITransport`（云台） | `send()/popFrameFor()/poll()` | 与旧 `UartTransport` 用法一致 |
| `IChassisLink`（底盘） | `setCommand()/state()/poll()` | 与旧 `UartChassisLink` 用法一致 |

**RX 按帧边界分流（`RxDemux`）**——不是把同一堆字节喂给两个解析器（那样底盘反馈帧
里的字节可能被 ZDT 解析器误拆成假帧），而是按帧切流：

- `5A A5 81` + 36B + CRC8 + `0D`（40 字节）→ 底盘解码器
- `Addr(1..32)` + `Func` + 负载 + `0x6B` → ZDT 帧队列
- 其余字节滚 1 字节重同步（与 F407 固件 `ulink_parse_rx` 同策略）

**发送**（同一 fd，按帧整块写，不交错）：

| 帧 | 节流 | 节奏 |
|---|---|---|
| 底盘命令帧（12B） | 5 ms | 200 Hz（主循环 100Hz 下实际 ~100Hz） |
| ZDT 命令帧 | 12 ms（防粘包） | 尽量排在**底盘反馈后的静默窗**（收完反馈 ≤6ms 内发），避免 ZDT 应答和 F407 反馈在 RX 线上打架 |

**poll() 幂等**：主循环里 `chassis->poll()` 与 `Gimbal::step()` 内部的 poll 在同一拍会各
调一次，用 1ms 去重保证一拍只完整跑一次。

**时效**：`state().age_s > chassis_timeout_s(0.3s)` 时 `valid=0`；断线 0.5s 自动重连。

### 1.3 目录与接口变化

| 项 | 变化 |
|---|---|
| `include/gimbal/transport/uart_transport.hpp` / `src/transport/uart_transport.cpp` | **删除**（被 UnifiedSerial 取代） |
| `include/gimbal/link/chassis_link.hpp` | 精简：只留帧常量/数据结构/编解码/`IChassisLink`；删 `UartChassisLink`、`CanChassisLink` |
| `include/gimbal/transport/fake_transport.hpp` | **保留**（只用于 `--selftest`） |
| `--dry` 模拟电机跑交互 | **移除** |
| `--chassis-dev` / `--chassis-baud` | **移除**，底盘与云台共用 `--device` |
| `--no-chassis` | 保留（只控云台） |

### 1.4 自检

`--selftest` 新增 `[7] 单总线帧分流`：半包不产出帧、底盘帧 + ZDT 帧 + 垃圾字节交错切分、
逐字节喂入一致、无坏帧。二进制已重新编译验证。

### 1.5 已知限制：三机 TX 并线的抢线

F407 反馈（100Hz，每帧发 3.5ms）和 ZDT 应答都往 CH340 的 RX 上发，同一时刻两个从机
同时发字会冲突。**静默窗调度**能缓解（ZDT 命令排在 F407 发完的缝里），但物理上根治需
485 收发器，或把 ZDT 遥测轮询周期放宽（见 §8）。

---

## 2. 无线交互（Jetson 当 AP）

### 2.1 现状

Jetson 内置 Intel 8265（`wlan0`），NetworkManager 里已有热点配置 `car-ap`：

| 项 | 值 |
|---|---|
| SSID | `MECAR-CTRL` |
| 密码（WPA2-PSK） | `mecar12345` |
| 频段/信道 | 2.4GHz / ch6 |
| Jetson IP | `10.42.0.1/24` |
| 地址分配 | `method=shared`（NetworkManager 自动 DHCP，笔记本拿到 10.42.0.x） |
| 自动连接 | **false（默认不自动拉起，见 §8 待办）** |

### 2.2 启用

```bash
sudo nmcli con up car-ap        # 拉起热点
ip -brief addr show wlan0       # 期望看到 10.42.0.1
iw dev wlan0 info               # 期望 type AP / ssid MECAR-CTRL
```

笔记本侧：

```bash
# 连热点（名字 MECAR-CTRL，密码 mecar12345），或图形界面点
nmcli dev wifi connect MECAR-CTRL password mecar12345
ping -c 2 10.42.0.1
```

**笔记本 Wi-Fi 省电（已处理）**：省电模式会让网卡在信标间隔休眠，实测连热点后
`ping` 的 RTT 到 **333~488ms**、抖动很大；控车是 100Hz UDP，必须关。

已把笔记本 `MECAR-CTRL` 配置的省电设为关闭：

```bash
nmcli con mod <MECAR-CTRL 配置名> 802-11-wireless.powersave 2   # 2 = disable
```

> ⚠️ 这个 NetworkManager 版本里取值是：`0=default`、`1=ignore`、`2=disable`、`3=enable`。
> 设成 `0`（default）**不会**关掉省电——之前踩过这个坑，`iw` 一直显示 on。

生效与验证：重新连接 `MECAR-CTRL` 后执行 `iw dev wlp14s0 get power_save`，应显示 `off`
（若仍为 on，临时强制：`sudo iw dev wlp14s0 set power_save off`）。
家里/校园网等其他配置不受影响。

### 2.3 客户端连接

- **无线**：`./build/gimbal_ctl --host 10.42.0.1 --no-video`
- **有线调试**：`./build/gimbal_ctl --host 192.168.2.2 --no-video`（Jetson eth0 = 192.168.2.2）

> 无线开 AP 时 Jetson 自己连不了别的 Wi-Fi（网卡只能干一件事），有线调试链路不受影响。

---

## 3. 客户端窗口英文化（笔记本 `~/gimbal_ctl`）

**乱码原因**：窗口用 `XLoadQueryFont("fixed")` + `XDrawString` 画字，这是只含 ASCII 的
X11 核心字体，中文字节必然乱码（方块/问号）。

**改动**：`src/main.cpp` 渲染段所有窗口文字改 ASCII（`yaw cmd/act/out`、`link ... lost`、
`chassis pos/wheel/act`、`ONLINE/OFFLINE` 等），终端输出保持中文。已重编译 `build/gimbal_ctl`。

---

## 4. F407 侧问题与修复（配合 IMU 联调）

- **现象**：无线遥控 W/S/A/D 时四轮直流减速电机**一卡一卡**；Jetson 抓包 F407 反馈只有
  **57Hz**（正常 100Hz，帧间隔 p90≈51.7ms）。
- **根因**：IMU（LSM6DSV16X）I2C **地址应答不稳**——约一半的读在地址字节就被 NACK
  （`us≈27µs`，不是 3ms 超时），推断 SA0 在 0x6A/0x6B 间漂移 → 连续失败触发**阻塞式总线
  恢复**（每次 ~25ms，每 100ms 一次）+ 部分读走满 3ms 超时 → 1kHz 控制环掉拍 30~40%。
- **修复**（F407 固件，已烧录）：
  1. `imu.c`：读失败换 0x6A/0x6B 另一地址重试一次并锁定 → `e` 150万 → **0**；
  2. `imu.c`：`hw_bus_unstick()` 的 `HAL_Delay(1)` 改 10µs DWT 忙等 → 25ms → **0.24ms**；
  3. `chassis_debug`：调试串口改非阻塞 `Transmit_IT` + 补 USART1 NVIC/IRQHandler；
  4. `uart_link.c`：TxCplt 回调转发清忙标志。
- **验证**：F407 反馈 **100Hz**（201 帧中 200 帧 CRC 全对）、IMU `CAL=1`、`e=0`。
- **完整证据链与给 IMU 同学的建议**：底盘仓库 `docs/CHASSIS_LIBRARY.md` **§17.8**。

---

## 5. 实测数据（本次）

| 项 | 修复前 | 修复后 |
|---|---|---|
| F407 反馈帧率 | 57 Hz（p50 10.1ms / p90 51.7ms / max 67.7ms） | **100 Hz**（2s 200 帧，200/201 CRC 全对） |
| IMU 错误计数 `e=` | 149 万，持续 +280~520/s | **0** |
| 单次 I2C 读耗时 `us=` | 27 µs（地址 NACK） | 214 µs（正常） |
| 零偏标定 `CAL` | 0（无效） | **1** |
| 采样节拍 `hz=` | 499 | 499 |
| 无线 | — | 笔记本 10.42.0.169 ↔ Jetson 10.42.0.1，ping 通 |
| 共享总线 | — | 跑 gimbal 时 ZDT 应答偶尔与 F407 反馈抢线，客户端底盘显示偶发 OFFLINE（不影响控制） |

---

## 6. 快速使用

```bash
# ---- Jetson 端 ----
cd ~/gimbal
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
./build/gimbal --selftest        # 7 段纯软件自检，全绿再上真机
./build/gimbal --device /dev/ttyUSB0

# ---- 笔记本端 ----
cd ~/gimbal_ctl
./build/gimbal_ctl --host 10.42.0.1 --no-video
```

**操作按键**（窗口里是英文）：鼠标=转云台；W/S/A/D=前后/平移；←/→=底盘转向；
空格=停手；e=云台使能；f=底盘使能；x=急停（云台失能+底盘 STBY 拉低，恢复按 e/f）；
c=底盘里程计清零；h=云台回中；z=云台当前位置归零；空格=停手；Esc/q=退出。

**Jetson 端本地命令**（stdin）：`s` 打印状态、`y/p/r` 设角速度、`h` 停手、`home` 回中、
`zero` 归零、`en 0|1` 使能、`x` 急停、`c <vx> <vy> <wz>` 底盘指令、`cx` 底盘急停、`q` 退出。

**验证链路健康的三个数**：
1. Jetson `s` 里 yaw/pitch 的 **frames 在涨**（云台在回帧）；
2. 底盘 `valid=1`、`cmd_cnt` 在涨（命令到了 F407）；
3. 客户端 `link ... rx=100`、`chassis ONLINE`。

---

## 7. 本次改动文件（Jetson 仓库）

| 文件 | 说明 |
|---|---|
| `include/gimbal/link/unified_serial.hpp` | **新增**：单串口统一链路（RxDemux 帧分流 + 静默窗调度） |
| `src/link/unified_serial.cpp` | **新增**：实现 |
| `include/gimbal/link/chassis_link.hpp` | 精简：删 UART/CAN 实现类，保留协议/接口 |
| `src/main.cpp` | 改用 UnifiedSerial；删 `--dry`/`--chassis-dev`/`--chassis-baud`；自检加 `[7]` |
| `include/gimbal/transport/uart_transport.hpp`、`src/transport/uart_transport.cpp` | **删除** |
| `CMakeLists.txt` | 源文件列表更新 |
| `include/gimbal/`、`src/` 下各目录 `README.md` | 新增 13 份逐文件详解文档 |

笔记本客户端仓库 `~/gimbal_ctl`：`src/main.cpp` 窗口文字英文化（重编译）。

---

## 8. 已知问题 / 待办

| # | 事项 | 说明 |
|---|---|---|
| 1 | **AP 开机自启** | `car-ap` 的 `connection.autoconnect=false`，重启后热点不会自己起来。需要的话执行 `sudo nmcli con mod car-ap connection.autoconnect yes` |
| 2 | ~~笔记本 Wi-Fi 省电~~ | **已完成（2026-10-19）**：`MECAR-CTRL` 配置已设 `802-11-wireless.powersave=2 (disable)`，重连后 `iw` 确认 off（取值映射见 §2.2） |
| 3 | 共享总线抢线 | ZDT 遥测轮询 100ms 周期偏密，可放宽到 250ms 减少与 F407 反馈的碰撞；根治需 485 |
| 4 | F407 `.ioc` | 已在 `.ioc` 里补上 `NVIC.USART1_IRQn=true:7:0`（等价于 CubeMX 勾选 USART1 global interrupt），重新生成代码会保留 `USART1_IRQHandler` 与 NVIC 使能 |
| 5 | IMU 硬件 SA0 | 软件已用「换地址重试」兜住，但根因（SA0 悬空/接触不良）待 IMU 负责同学检查 |
| 6 | 历史文档 | `CHASSIS_LINK.md` 与根 `README.md` 里仍有 `--dry`/`--chassis-dev` 等旧描述，尚未清理 |

---

## 9. 修订记录

| 日期 | 修订 |
|---|---|
| 2026-10-19 | 初版：单串口合并、无线 AP、客户端英文化、IMU 修复、实测数据、使用步骤 |
| 2026-10-19 晚 | 补充：笔记本 Wi-Fi 省电已关闭（`MECAR-CTRL` 配置 `802-11-wireless.powersave=2`），并记录 NM 取值映射坑（0=default 不生效） |
