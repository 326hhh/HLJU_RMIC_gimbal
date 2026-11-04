# gimbal/src/link —— 链路实现（UDP 操作端 + 单串口统一链路 + CAN 底盘链路）

本目录三个实现文件：

- `udp_control_link.cpp`：`UdpControlLink` —— 与 PC 客户端之间的低延迟控制通道（Jetson 侧）。
- `unified_serial.cpp`：`UnifiedSerial` —— 单串口统一链路，底盘 F407 帧 + 云台 ZDT 帧共用一个 fd
  （`RxDemux` 按帧边界分流；实现 `ITransport` + `IChassisLink` 双接口）。
- `can_chassis_link.cpp`：`CanChassisLink` —— SocketCAN 底盘链路（标准帧 + `0x110~0x114` 组规则 + `0x000` 急停）。

> UDP 线协议定义（帧布局、校验、flags）在 `include/gimbal/link/control_link.hpp`；
> 底盘帧定义在 `include/gimbal/link/chassis_link.hpp`；两端必须逐字节一致。

**文件一览**

| 文件 | 内容 |
|---|---|
| `udp_control_link.cpp` | 非阻塞 UDP 收发、peer 管理、seq 丢包统计、遥测按频率回发 |
| `unified_serial.cpp` | 单串口统一链路：底盘帧 + ZDT 帧共用一个 fd（节流 + `RxDemux` 分流） |
| `can_chassis_link.cpp` | SocketCAN 底盘链路：标准帧收发 + 0x110~0x114 组规则 + 0x000 急停 |

---

## 1. `UdpControlLink` 函数一览

| 函数 | 行为 |
|---|---|
| `nowS()` | 单调时钟秒数 |
| `UdpControlLink(Config)` | 初始化计时基准 |
| `~UdpControlLink()` | `close()` |
| `name()` | `"udp:40001"` |
| `open()` | `socket(AF_INET, SOCK_DGRAM)` → `SO_REUSEADDR` → `bind(0.0.0.0:port)` → 非阻塞；失败打日志 |
| `close()` | 关 socket |
| `popCommand(out)` | 从 `cmds_` 队头取一条指令 |
| `sendTelemetry(t)` | 编码后 `sendto()` 发给当前 peer（没有 peer 时不发）；成功 `tx_telem++` |
| `poll()` | 主循环：收包解码 → peer/seq/频率/时效统计 → 按 `telem_hz` 回发遥测 → peer 超时处理 |

## 2. `poll()` 的细节

1. **收包**：`recvfrom()` 循环直到 `EAGAIN`（一次 poll 收完积压）。
   - 解码失败 → `rx_bad++`；
   - 记录 peer（IP:端口变化时打一行日志），供遥测回发；
   - seq 跳号估算丢包：`expect = last_seq_ + 1`，`c.seq > expect` 时 `rx_lost += 差值`；
   - `rx_ok++`、`hz_count++`、`last_rx_s_ = t`；
   - 指令队列最多留 8 条（上层不消费时防无限增长）。
2. **频率统计**：1 秒窗口 → `rx_hz = count / window`。
3. **时效**：`age_s = t - last_rx_s_`。
4. **遥测回发**：有 peer 且距上次回发 ≥ `1/telem_hz` → `sendTelemetry(last_telem_)`。
5. **peer 超时**：`age_s > peer_hold_s`（默认 10s）→ 停止回发（`have_peer_ = false`）。

## 3. 设计要点

- **单线程、非阻塞**：`poll()` 在控制循环里被调用，不额外开线程；
- **按频率回发**：主循环每拍只 `setTelemetry()` 更新，回发节流在链路里做，
  避免每收到一个包就立刻回发导致洪泛；
- **peer 语义**：以"最近一次发过包的客户端"为准，客户端换端口/重启自动跟随；
- **无连接状态**：客户端崩溃重启不需要握手，`seq` 断号由两端自行判断。

## 4. 与客户端的对应关系

| 本文件 | 客户端 `~/gimbal_ctl` |
|---|---|
| 上行帧 `ControlCommand::decode` | `ControlCommand::encode` |
| 下行帧 `ControlTelemetry::encode` | `ControlTelemetry::decode` |
| 丢包统计 `rx_lost`（按 seq 跳号） | RTT 统计（按 `echo_seq` 回显匹配） |

协议常量都来自 `control_link.hpp` 的同一份定义，改协议时**必须两端同步**。

---

## 5. `CanChassisLink` —— CAN 底盘链路（SocketCAN 标准帧）

对应头文件 `include/gimbal/link/can_chassis_link.hpp`。与 `UnifiedSerial` 的 UART 底盘链路
二选一（`--chassis can0` / `--device can0` 时启用），命令/反馈字段与 UART 版完全一致。

| 部分 | 行为 |
|---|---|
| `Config` | `iface`（默认 can0）、命令 10 ms、flags 10 Hz、组超时 5 ms、反馈超时 100 ms、socket 重开 1 s |
| `CanFbGroup` | `0x110` 开组 → `0x111~0x114` 按序填入 → 新 `0x110` 冲掉未满组 → 组超时丢弃 → CRC8/尾字节失败整组丢弃；返回 `1` 完整、`0` 未完成、`-1` 坏组 |
| `tryOpen()` | `socket(PF_CAN, SOCK_RAW)` → `SIOCGIFINDEX` → `bind` → `O_NONBLOCK` |
| `poll()` | 重连检查 → 收帧喂给 `CanFbGroup` → 完整组立即 `chassisDecodeFb` 更新 `state_` → 按周期发 `0x020`/`0x021` → 反馈超时置 `valid=false` |
| `setCommand()` | 缓存 `ChassisCommand`；EN 由命令帧 flags 控制；`estop` 置位后发 `0x000`（DLC=0），恢复需 EN=1 |
| `sendFlagsFrame()` | 10 Hz 保活 + `enable` 变化时**立即**发一次 |

帧表 / 组完整性规则 / 语义细节（`0x021` 不计 `cmd_cnt`、`CAN_ERR` 实时位）见底盘仓库
`docs/README.md` §15。

## 6. `UnifiedSerial` —— 单串口统一链路（UART）

对应头文件 `include/gimbal/link/unified_serial.hpp`。一个 CH340 上挂三块 MCU
（F407 + 两个 ZDT 驱动板），同一 fd 同时实现 `ITransport`（云台）与 `IChassisLink`（底盘）：

- **TX**：底盘帧按 5 ms 节流、ZDT 帧按 12 ms 节流，各自整帧写入；
- **RX**：`RxDemux` 按帧边界分流 —— `5A A5 81` + 40B + CRC8 + `0D` → 底盘解码器；
  `Addr∈1..32` + func + … + `6B` → ZDT 帧；避免底盘反馈字节被 ZDT 解析器误拆成假帧；
- **静默窗**：ZDT 命令尽量排在"刚收到底盘反馈后的静默窗"发送，避免三台从机 TX 抢线；
- 底盘反馈超时 0.3 s 置 `valid=false`；socket 断线自动重连。
