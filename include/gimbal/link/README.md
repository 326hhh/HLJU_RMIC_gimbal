# gimbal/include/gimbal/link —— 通信链路层（PC 客户端 + F407 底盘）

本目录是云台工程的**两条对外链路**的协议与实现：

1. **对 PC 客户端**（UDP 40001）：云台角速度指令 + 遥测回发 —— `control_link.hpp` / `udp_control_link.hpp`
2. **对 F407 底盘**（USART2，115200）：底盘速度指令 + 反馈 —— `chassis_link.hpp`

两条链路互相独立、端口/帧格式完全不同；主循环里各走各的。

## 文件一览

| 文件 | 作用 |
|---|---|
| `control_link.hpp` | UDP 线协议定义（编解码纯函数 + 报文结构体，两端逐字节对齐） |
| `udp_control_link.hpp` | UDP 收发实现（非阻塞 socket + 遥测回发 + 丢包统计） |
| `chassis_link.hpp` | 底盘协议（12B 命令 / 40B 反馈）+ `IChassisLink` 抽象（UART 实现在 unified_serial） |
| `unified_serial.hpp` | ★ 单串口统一链路：底盘帧 + ZDT 帧共用一个 fd（`RxDemux` 帧分流） |

---

## 1. `control_link.hpp` —— UDP 线协议（PC ↔ Jetson）

**为什么用 UDP 而不是 TCP/WebSocket**：控制包极小（24B @ 100Hz ≈ 2.4KB/s），UDP 开销最低、
无队头阻塞、无握手；丢包容忍交给两端看门狗。

**⚠️ 两端必须逐字节一致。所有字段小端。** 编解码用显式字节偏移，不依赖 C 结构体
布局/padding —— 这是跨编译器、跨架构不出错的关键。

### 1.1 常量

| 常量 | 值 | 含义 |
|---|---|---|
| `CTRL_MAGIC` | `0x4D454341` | 魔数（"MAC" 的 ASCII 反转） |
| `CTRL_UP_LEN` | `24` | 上行（PC → Jetson）帧长 |
| `CTRL_DOWN_LEN` | `64` | 下行（Jetson → PC）帧长 |
| `CTRL_DEFAULT_PORT` | `40001` | 默认 UDP 端口 |
| `CTRL_TELEM_HZ` | `50.0` | 遥测回发频率 |

### 1.2 上行帧（PC → Jetson，24 字节，v2；v1 的 16 字节仍兼容）

| 偏移 | 大小 | 类型 | 字段 |
|---|---|---|---|
| 0 | 4 | u32 | `magic` = 0x4D454341 |
| 4 | 4 | u32 | `seq` 递增序号 |
| 8 | 2 | i16 | `yaw_cdps` yaw 角速度 0.01°/s |
| 10 | 2 | i16 | `pitch_cdps` pitch 角速度 |
| 12 | 1 | u8 | `flags` 云台 flags（`ctrl_flag`） |
| 13 | 1 | u8 | `chk` 前 13 字节逐字节 XOR（v1 兼容，位置不变） |
| 14 | 2 | i16 | `vx_mmps` 底盘 vx，mm/s，+ 前进 |
| 16 | 2 | i16 | `vy_mmps` 底盘 vy，mm/s，+ 左移 |
| 18 | 2 | i16 | `wz_mradps` 底盘 wz，0.001 rad/s，+ 逆时针 |
| 20 | 1 | u8 | `chassis_flags` 底盘 flags（`chs_flag`） |
| 21 | 1 | u8 | `chk2` 14~20 字节逐字节 XOR |
| 22 | 2 | u16 | `pad` 保留，填 0 |

### 1.3 下行帧（Jetson → PC，64 字节，前 32 字节与 v1 一致）

| 偏移 | 大小 | 类型 | 字段 |
|---|---|---|---|
| 0 | 4 | u32 | `magic` |
| 4 | 4 | u32 | `echo_seq` 回显最后收到的 seq（客户端算 RTT/丢包） |
| 8 | 2 | i16 | `yaw_cdps` 实际输出角速度 0.01°/s |
| 10 | 2 | i16 | `pitch_cdps` |
| 12 | 2 | i16 | `yaw_rpm` 电机实测转速 0.01 RPM |
| 14 | 2 | i16 | `pitch_rpm` |
| 16 | 4 | i32 | `yaw_cdeg` 输出角估计 0.01° |
| 20 | 4 | i32 | `pitch_cdeg` |
| 24 | 2 | u16 | `vbus_mv` 母线电压 |
| 26 | 1 | u8 | `temp_c` 驱动温度 |
| 27 | 1 | u8 | `status` `ctrl_status` |
| 28 | 2 | u16 | `rx_hz` 最近一秒接收频率 |
| 30 | 2 | u16 | `chk16` 前 30 字节按 u16 累加和 |
| 32 | 4 | i32 | `cs_x_mm` 底盘里程计 x（+ 前进） |
| 36 | 4 | i32 | `cs_y_mm` 底盘里程计 y（+ 左移） |
| 40 | 2 | i16 | `cs_yaw_cdeg` 底盘里程计 yaw 0.01° |
| 42 | 2 | i16 | `cs_vx_mmps` 底盘正解 vx |
| 44 | 2 | i16 | `cs_vy_mmps` 底盘正解 vy |
| 46 | 2 | i16 | `cs_wz_cs` 底盘正解 wz 0.01 rad/s |
| 48 | 2 | i16 | `cs_gyaw_cdeg` 底盘陀螺积分 yaw 0.01° |
| 50 | 2 | i16 | `cs_gwz_cs` 底盘陀螺 wz 0.01 rad/s |
| 52 | 1 | u8 | `cs_status` 底盘 status 低 8 位 |
| 53 | 1 | u8 | `cs_valid` 1 = 底盘反馈新鲜 |
| 54 | 2 | i16 | `cs_wheel_fl` 四轮实测转速 0.01 RPM（FL） |
| 56 | 2 | i16 | `cs_wheel_fr` |
| 58 | 2 | i16 | `cs_wheel_bl` |
| 60 | 2 | i16 | `cs_wheel_br` |
| 62 | 2 | u16 | `chk16b` 32~61 字节按 u16 累加和 |

### 1.4 flags / status 位

- **`ctrl_flag`（上行 offset 12）**：`Enable=0x01`、`Hold=0x02`、`Home=0x04`、
  `Zero=0x08`、`EStop=0x10`、`ClearStall=0x20`、`Disable=0x40`（与 Enable 互斥，Disable 优先）。
- **`chs_flag`（上行 offset 20）**：`Enable=0x01`、`EStop=0x02`、`Clear=0x04`。
- **`ctrl_status`（下行 offset 27）**：`Enabled=0x01`、`YawOk=0x02`、`PitchOk=0x04`、
  `Watchdog=0x08`、`YawLim=0x10`、`PitchLim=0x20`、`Stall=0x40`、`EStop=0x80`。

### 1.5 小端读写与换算工具（inline）

| 函数 | 作用 |
|---|---|
| `putU16/putU32/putI16/putI32` | 显式小端写 |
| `getU16/getU32/getI16/getI32` | 显式小端读 |
| `clampCdps(double dps)` | °/s → i16（0.01°/s）并夹取 |
| `clampI16(double v)` | 夹进 i16 整数范围并四舍五入（底盘字段用） |

### 1.6 `ControlCommand`（上行指令结构）

字段：`seq`、`yaw_dps`、`pitch_dps`、`flags`、`chassis_vx_mmps`、`chassis_vy_mmps`、
`chassis_wz_mradps`、`chassis_flags`；查询位 `enable()/hold()/home()/zero()/estop()`、
`chassisEnable()/chassisEstop()/chassisClear()`。

- `encode()`：按 1.2 表写 24 字节（含两段 XOR 校验）；
- `decode()`：**兼容 v1（16B）与 v2（24B）**，v1 时底盘字段按 0 处理 —— 旧客户端升级不会断。

### 1.7 `ControlTelemetry`（下行遥测结构）

字段：`echo_seq`、两轴角速度/转速/角度、`vbus_mv`、`temp_c`、`status`、`rx_hz`、
底盘 12 个字段（里程计/正解速度/陀螺/状态/四轮转速）。

- `encode()`：按 1.3 表写 64 字节（两段 u16 累加和校验）；
- `decode()`：长度 ≥64 才解，校验不过返回 false。

### 1.8 `ControlLinkStats`

`rx_ok`、`rx_bad`、`rx_lost`（按 seq 跳号估算）、`tx_telem`、`age_s`、`rx_hz`、`peer`。

---

## 2. `udp_control_link.hpp` —— UDP 收发实现（Jetson 侧）

单线程、非阻塞：`poll()` 在控制循环里被调用，不额外开线程 —— 控制循环本来就要跑 100Hz，
顺手收包即可。

```cpp
struct Config {
    uint16_t port        = 40001;
    int      telem_hz    = 50;
    double   peer_hold_s = 10.0;  // 多久没有上行就停止回发遥测
};
```

| 函数 | 行为 |
|---|---|
| `open()/close()` | 建 socket、`SO_REUSEADDR`、bind、非阻塞 |
| `poll()` | 收包 → 解码 → 记录 peer/seq/频率/时效；按 `telem_hz` 回发最新遥测；peer 超时停止回发 |
| `popCommand(ControlCommand&)` | 取出一条收到的指令（队列最多保留 8 条） |
| `setTelemetry(const ControlTelemetry&)` | 主循环每拍交最新遥测，poll 按频率自动回发 |
| `sendTelemetry()` | 立即回发一次（一般不直接用） |
| `stats()/hasPeer()/commandAgeS()` | 查询 |

**细节**：收包时对坏包计数（`rx_bad`）；按 seq 跳号累计丢包（`rx_lost`）；1 秒窗口统计
`rx_hz`；peer 以"最近一次发过包的客户端"为准，超过 `peer_hold_s` 没上行就不再回发。

---

## 3. `chassis_link.hpp` —— 底盘链路（F407 USART2）

与 F407 固件 `UserLib/Comm/uart_link_config.h` **逐字节一致**。

### 3.1 命令帧（Jetson → F407，12 字节，小端）

| 偏移 | 字段 | 类型 |
|---|---|---|
| 0-1 | `0xA5 0x5A` 帧头 | |
| 2 | `cmd` | u8 |
| 3 | `seq` | u8 |
| 4..5 | `vx_mm`（mm/s，+ 前进） | i16 |
| 6..7 | `vy_mm`（mm/s，+ 左移） | i16 |
| 8..9 | `wz_cs`（0.01 rad/s，逆时针为正） | i16 |
| 10 | `flags`（bit0 = EN） | u8 |
| 11 | `crc8` 覆盖 0..10 | u8 |

### 3.2 反馈帧（F407 → Jetson，40 字节，小端，100Hz）

| 偏移 | 字段 | 类型 |
|---|---|---|
| 0-1 | `0x5A 0xA5` 帧头 | |
| 2 | 类型码 `0x81` | u8 |
| 3 | seq 回显 | u8 |
| 4..7 / 8..11 | 里程计 x / y（mm） | i32 ×2 |
| 12..13 | 里程计 yaw（0.01°） | i16 |
| 14..15 / 16..17 / 18..19 | 正解 vx / vy / wz（mm/s、0.01 rad/s） | i16 ×3 |
| 20..21 / 22..23 | 陀螺 yaw / wz | i16 ×2 |
| 24..25 | status（`ULINK_ST_*`） | u16 |
| 26..27 / 28..29 | cmd_cnt / fb_seq | u16 ×2 |
| 30..37 | 四轮实测转速（0.01 RPM，FL FR BL BR） | i16 ×4 |
| 38 | crc8 覆盖 0..37 | u8 |
| 39 | 尾 `0x0D` | u8 |

> ⚠️ 文件顶部注释块里写的是 32 字节（旧版），**以本表 40 字节为准**（v2 加了四轮转速）。

### 3.3 常量与枚举

- 命令字 `chassis_cmd`：`Speed=0x01`、`EStop=0x02`、`Spin=0x03`、`Clear=0x04`；
- `chassis_flag`：`Enable=0x01`（F407 侧 `ULINK_FLAG_EN`）；
- `chassis_status`：`ImuOnline/LinkOk/Watchdog/EStop/DriverOn/Spin/GyroOn/ImuCal`；
- `chassisCrc8(data, len)`：CRC-8/ATM（poly=0x07, init=0x00，校验值 "123456789" = 0xF4）。

### 3.4 数据结构与编解码

- `struct ChassisCommand`：`vx_mmps`、`vy_mmps`、`wz_mradps`（注意是 **0.001 rad/s**）、
  `enable`、`estop`、`clear_odom`、`seq`；
- `struct ChassisState`：`valid`、正解 `vx/vy/wz`、里程计 `odom_x/y/yaw`、陀螺 `gyro_yaw/wz`、
  `status`、`cmd_cnt`、`fb_seq`、四轮转速、回显 `last_seq`、`age_s`；
- `chassisEncodeCmd(out[12], cmd)`：按 3.1 表编码（wz 除以 10 换成 0.01 rad/s）；
- `chassisDecodeFb(in[40], ChassisState&)`：校验帧头/类型/帧尾/CRC 后逐字段译码。

### 3.5 收发送实现（在 `link/unified_serial.hpp`）

`UartChassisLink` / `CanChassisLink` 已删除，底盘帧的收发并入单串口统一链路：
- `UnifiedSerial`：底盘命令 5ms 节流、反馈超时 0.3s 置 `valid=false`；
- `RxDemux`：`5A A5 81` + 40B + CRC8 + `0D` → 底盘解码器；`Addr∈1..32` + func + … + `6B` → ZDT 帧；
- 详见本节顶部「单串口链路」与 `unified_serial` 文档。

## 4. 链路状态位对照（Jetson ↔ F407）

| Jetson `chassis_status` | 值 | F407 `ULINK_ST_*` |
|---|---|---|
| `ImuOnline` | 0x0001 | `ULINK_ST_IMU_ONLINE` |
| `LinkOk` | 0x0002 | `ULINK_ST_LINK_OK` |
| `Watchdog` | 0x0004 | `ULINK_ST_WATCHDOG` |
| `EStop` | 0x0008 | `ULINK_ST_ESTOP` |
| `DriverOn` | 0x0010 | `ULINK_ST_DRIVER_ON` |
| `Spin` | 0x0020 | `ULINK_ST_SPIN` |
| `GyroOn` | 0x0040 | `ULINK_ST_GYRO_ON` |
| `ImuCal` | 0x0080 | `ULINK_ST_IMU_CAL` |

## 5. 单串口链路（已实现）

底盘帧与云台 ZDT 帧共用一个串口，实现在 `link/unified_serial.hpp`（原 UART 类已并入）：
- 一个 fd、一个写者：底盘帧按 5ms 节流、ZDT 帧按 12ms 节流，各自整帧写入；
- RX 按帧边界**分流**（`5A A5 81`+40B+CRC+`0D` → 底盘；`Addr∈1..32`+`func`+…+`6B` → ZDT），
  避免底盘反馈帧的字节被 ZDT 解析器误拆成假帧；
- ZDT 命令尽量安排在"刚收到底盘反馈后的静默窗"里发，避免三台从机 TX 抢线；
- UDP 线协议保持不变，客户端无需改。
