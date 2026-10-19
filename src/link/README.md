# gimbal/src/link —— UDP 链路实现

本目录只有 `udp_control_link.cpp`：实现 `include/gimbal/link/udp_control_link.hpp` 的
`UdpControlLink` —— 与 PC 客户端之间的低延迟控制通道（Jetson 侧）。

> 线协议定义（帧布局、校验、flags）在 `include/gimbal/link/control_link.hpp`，
> 两端必须逐字节一致；本文件只负责 socket 收发与状态维护。

## 文件一览

| 文件 | 内容 |
|---|---|
| `udp_control_link.cpp` | 非阻塞 UDP 收发、peer 管理、seq 丢包统计、遥测按频率回发 |

---

## 1. 函数一览

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
