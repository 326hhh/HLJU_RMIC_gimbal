# gimbal/include/gimbal/transport —— 与总线无关的收发层

传输层把「字节怎么进出总线」这件事收口：**换总线只换一个实现类**，设备层/云台层一行不改。

```
ITransport（抽象，职责与公共能力都在基类实现）
├── UnifiedSerial   当前真机使用（底盘 + 云台共用一个 CH340，实现在 link/unified_serial.hpp）
├── CanTransport    SocketCAN 骨架（29-bit 扩展帧拆包重组，未上机验证）
└── FakeTransport   纯软件模拟电机（--selftest，不需要硬件）
```

## 1. `itransport.hpp` —— 抽象接口 + 公共能力

### 1.1 `TransportStats`（统计快照）

| 字段 | 含义 |
|---|---|
| `tx_frames` | 成功发出的帧数 |
| `tx_dropped` | 队列满被丢掉的帧数 |
| `rx_frames` | 解析出的完整返回帧数 |
| `rx_bytes` | 累计收到的字节数 |
| `reconnects` | 自动重连次数 |
| `connected` | 当前是否已连接 |
| `last_error` | 最近一次错误描述 |

### 1.2 `class ITransport`（基类）

**纯虚接口**（派生类必须实现）：
| 函数 | 职责 |
|---|---|
| `bool open()` | 打开总线 |
| `void close()` | 关闭总线 |
| `bool isOpen() const` | 是否已打开 |
| `std::string name() const` | 可读名字（状态打印用） |
| `void poll()` | 控制循环里高频调用：重连、节流发送、收包解析 |

**基类已实现的公共能力**：
| 函数 | 行为 |
|---|---|
| `void send(const std::vector<uint8_t>& frame, bool urgent=false)` | 入队一帧。队列满（`txq_max_=64`）则丢队头、`tx_dropped++` |
| `bool popFrame(zdt::Frame&)` | 取任意一帧 |
| `bool popFrameFor(uint8_t addr, zdt::Frame&)` | 只取发给 `addr`（或广播）的帧——一总线多电机时的地址路由 |
| `void feedRx(const uint8_t* p, size_t n)` | **派生类在 `poll()` 里调用**：字节 → 解析器 → 收帧队列（`rxq_` 超过 256 截断） |
| `bool nextTx(std::vector<uint8_t>&)` | 从发送队列取一帧（**受最小间隔约束**，未到时间返回 false） |
| `void onTxDone()` / `void onTxFailed(const std::string&)` | 发送记账 / 失败上报（派生类据此触发重连） |
| `bool txReady() const` | 是否已到可发送时刻 |
| `void setGapUs(int64_t)` | 设置命令最小间隔（防粘包） |

**保护成员**：`txq_`（发送队列）、`parser_`（ZDT 解析器）、`rxq_`（收帧队列）、`stats_`、`txq_max_`（64）、`gap_us_`（默认 12000µs）、`last_tx_us_`。

**★ 关键语义：`urgent` 是"插队"，不是"清空队列"**
把 urgent 实现成 `txq_.clear()` 会导致：`Gimbal::estop()` 里 `yaw_.estop()` 排入的"失能"命令
被紧跟着的 `pitch_.estop()` 的 clear 直接清掉 —— yaw 永远没被失能。一根总线挂多台电机时这是致命的。
正确做法：`push_front()` 插到最前，绝不动别人的命令。

### 1.3 设计要点

- 发送节流由**基类**统一做（`nextTx` + `gap_us_`），派生类只负责"字节怎么发出去"。
- 收帧路由按地址：`popFrameFor` 匹配 `addr` 或 `ADDR_BROADCAST`。
- 解析器只认 `0x6B` 校验 + 期望长度，不依赖超时。

## 2. `uart_transport.hpp` ——（已删除，并入 `link/unified_serial.hpp`）

原 UART 实现（termios、热插拔重连、一次 poll 只发一帧）被单串口统一链路取代：
`UnifiedSerial` 同时实现 `ITransport` 与 `IChassisLink`，一个 fd 收/发两族帧。
细节见 `link/README.md` 与 `include/gimbal/link/README.md`。

## 3. `can_transport.hpp` —— SocketCAN 实现（未上机验证）

```cpp
struct Config {
    std::string iface     = "can0";
    int         tx_gap_ms = 2;      // CAN 无粘包问题，间隔可很小
    bool        enable_fd = false;  // 保持 false：ZDT 是 classic CAN
};
```

- `open()`：`socket(PF_CAN)` + `ioctl(SIOCGIFINDEX)` + `bind()`；
- `canSendPackets(addr, uartFrame)`：去掉首字节 Addr，按 8 字节 DLC 拆包，
  `can_id = (addr<<8) | packet` 并置 `CAN_EFF_FLAG`（29-bit 扩展帧，手册强制）；
- `onCanFrame(canId, data, dlc)`：`packet==0` 时重置重组缓冲，按
  `expectedResponseLen(func)-1` 判断收齐；收齐后**还原成 UART 风格帧**（补回 Addr）喂 `feedRx()` ——
  上层完全不感知总线类型；
- `poll()`：发送一帧 + 非阻塞收帧循环。

⚠️ 状态：帧封装/拆包/重组按手册写好了，但**无硬件验证**（KH-UCANFD 适配置器未接入）。
联调前先 `candump can0` 抓包核对。29-bit 扩展帧与底盘标准帧天然隔离，不会冲突。

## 4. `fake_transport.hpp` —— 纯软件模拟电机

**用途**：`--selftest`（`--dry` 已移除）。不需要任何硬件就能验证：
帧构造 → 解析器切帧 → 地址路由 → 角度/转速换算 → 斜坡与限位逻辑。

走的是和真实传输层**完全相同**的代码路径：
`上层 send()` → `txq_` → `poll()` 取出 → 生成响应 → `feedRx()` → `Parser`。
唯一被替换的只有"OS 串口/CAN 读写"这一层。

### 4.1 `struct Sim`（单台虚拟电机状态）

| 字段 | 含义 |
|---|---|
| `enabled` / `rpm` / `target_rpm` | 使能、当前转速、目标转速 |
| `accel_rpm_per_s` | 加速度（默认 4000 RPM/s） |
| `pos_deg` | 位置（单圈 0..360°） |
| `vbus_mv` / `temp_c` | 母线电压 / 温度 |
| `flags` | 状态位（使能/到位/限位） |
| `lim_left/lim_right` | 限位输入 |
| `tx_seen` / `get_seen` | 收到的命令条数 / 读取条数（自测断言用） |

### 4.2 成员与方法

| 函数 | 行为 |
|---|---|
| `addMotor(addr)` | 往总线挂一台虚拟电机（可挂多台，验证地址路由） |
| `sim(addr)` / `motors()` | 查询虚拟电机状态（自测断言用） |
| `open/close/isOpen/name` | 恒为打开；name = `"fake"` |
| `poll()` | 按真实时间 `dt` 推进所有模拟电机（`step()`：加速度斜率 + 位置积分 + 状态位），再把 `txq_` 里排队的命令全部取出 `handleTx()` |
| `handleTx(frame)` | 按功能码生成回应帧并 `feedRx()`：Enable/Speed/EStop/ZeroPos/ClearStall/GetSpeed/GetPos/GetStatus/GetVbus/GetTemp；**未知命令不回**（正好测超时路径） |

内部私有 `step(Sim&, dt)` 每拍更新一台电机；`degToRaw()` 把角度映射成 Emm 固件的
0-65535 单圈原始值。

---

## 5. 现状与后续

- 统一串口链路（一条 fd，同时实现 `ITransport` 与 `IChassisLink`）已落地：见 `link/unified_serial.hpp`；
- `uart_transport.hpp/.cpp` 已删除；`--dry` 已移除；`fake_transport.hpp` 保留（只用于 `--selftest`）；
- `can_transport.cpp` 暂时保留（未来可能换 CAN 总线，且与本次合并无关）。
