# gimbal/src/transport —— 传输层实现

实现 `include/gimbal/transport/` 下的总线类（UART 实现在 `src/link/unified_serial.cpp`）。公共的队列/节流/收帧路由已经在
`itransport.cpp` 里写死，各总线实现只负责"字节怎么进、怎么出"。

## 文件一览

| 文件 | 对应头文件 | 内容 |
|---|---|---|
| `itransport.cpp` | `transport/itransport.hpp` | 发送队列、节流、收帧路由、统计 |
| `can_transport.cpp` | `transport/can_transport.hpp` | SocketCAN 收发、29-bit 扩展帧拆包重组 |
| （无） | `transport/fake_transport.hpp` | 纯头文件内联实现，见头文件文档 |

---

## 1. `itransport.cpp` —— 基类公共能力

| 函数 | 行为 |
|---|---|
| `nowUs()`（匿名命名空间） | 单调时钟微秒数（发送节流用） |
| `send(frame, urgent)` | 队列满（`txq_max_=64`）丢队头并计 `tx_dropped`；`urgent` 时 **`push_front` 插队**（不是清空——见头文件文档的踩坑说明） |
| `popFrame(out)` | 弹出任意一帧 |
| `popFrameFor(addr, out)` | 线性扫描 `rxq_`，匹配 `addr` 或 `ADDR_BROADCAST` 的帧弹出 |
| `feedRx(p, n)` | `rx_bytes += n` → `parser_.feed()` → 解析出的帧压入 `rxq_`；`rxq_ > 256` 时丢队头（下游不消费时防无限增长） |
| `txReady()` | `nowUs() - last_tx_us_ >= gap_us_` |
| `nextTx(out)` | 队列非空且到间隔 → 弹出队头、更新 `last_tx_us_`、返回 true |
| `onTxDone()` / `onTxFailed(why)` | `tx_frames++` / 记录错误并置 `connected=false`（派生类据此走重连） |

> 设计动机：节流放在基类，派生类（UART/CAN/模拟）不用各写一遍；`urgent` 插队保证
> 一根总线多电机时 `estop` 不会误杀别的电机的命令。

## 2. `uart_transport.cpp` ——（已删除，并入 `src/link/unified_serial.cpp`）

原 UART 实现被单串口统一链路取代：`UnifiedSerial` 同时实现 `ITransport` 与
`IChassisLink`，一个 fd 收/发两族帧（底盘帧 + ZDT 帧），细节见 `src/link/README.md`。

## 3. `can_transport.cpp` —— SocketCAN 实现（未上机验证）

### 3.1 工具常量

- `kCanMaxDlc = 8`：classic CAN 单帧最大数据字节数；
- `nowUs()` 由基类提供。

### 3.2 函数一览

| 函数 | 行为 |
|---|---|
| 构造 | `setGapUs(tx_gap_ms×1000)`（默认 2ms）、`connected=false` |
| `~CanTransport()` | `close()` |
| `name()` | `"can:can0"` |
| `open()` | `socket(PF_CAN, SOCK_RAW, CAN_RAW)` → `ioctl(SIOCGIFINDEX)` 取接口索引 → `bind(AF_CAN)`；失败记录错误 |
| `close()` | 关 fd、`connected=false` |
| `canSendPackets(addr, uartFrame)` | 帧首字节是 Addr，CAN 里 Addr 进 ID 不进数据：去掉首字节后按 8 字节 DLC 切包；`can_id = (addr<<8) \| packet` 并置 `CAN_EFF_FLAG`（手册强制 29-bit 扩展帧）；`write()` 每包一次 |
| `onCanFrame(canId, data, dlc)` | 按 `packet==0` 重置重组缓冲并记 func；累积数据；`expectedResponseLen(func)-1` 判定收齐（超出=丢包错位，清空重等）；收齐后**像素还原成 UART 帧**（补回 Addr）喂 `feedRx()` |
| `poll()` | 到点发一帧（`canSendPackets`），失败即返回；非阻塞读循环，按 `CAN_EFF_FLAG` 只收扩展帧 |

### 3.3 注意

- 该实现**没有上过真机**：KH-UCANFD 适配器在 Linux 下不可用（Jetson Nano t210 无设备树节点），
  联调前先 `candump can0` 抓包核对；
- 29-bit 扩展帧与底盘标准帧（11-bit）天然隔离，不会冲突；
- `enable_fd` 保持 false（ZDT 用 classic CAN）；
- 换总线只换实现类，上层（设备/云台）一行不改——这正是 `ITransport` 存在的意义。

## 4. `fake_transport.hpp`（纯头文件）

无对应 .cpp。全部实现内联，见 `include/gimbal/transport/README.md` §4。
仅 `--selftest` 使用（`--dry` 已移除，文件保留用于纯软件自测）。

---

## 5. 现状与后续

- 统一串口链路已落地（`src/link/unified_serial.cpp`），`uart_transport.cpp` 已删除；
- `--dry` 已移除，`fake_transport.hpp` 保留仅用于 `--selftest`；
- `can_transport.cpp` 暂时保留（未来可能换 CAN 总线）。
