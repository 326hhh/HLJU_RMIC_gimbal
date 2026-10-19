# gimbal/include/gimbal/protocol —— ZDT 电机线协议

ZDT（张大头）X42S 第二代闭环步进电机协议的**常量表 + 帧构造器 + 流式解析器**。
本目录只定义协议本身，不依赖任何总线实现。

依据：《ZDT_X42S 第二代闭环步进电机用户手册 V1.0.5_260527》
§4.2（CAN 通讯）、§5.3（运动控制命令）、§5.5（读取系统参数命令）。

## 1. UART 帧格式（本工程当前使用）

```
主机 -> 电机:  Addr | 功能码 | 命令数据 | 校验码
电机 -> 主机:  Addr | 功能码 | 返回数据 | 校验码
校验码: 出厂默认固定 0x6B（可在驱动板里改成 XOR/CRC-8/Modbus/DMX512，改了就全错）
```

CAN 模式（`transport/can_transport`，未上机验证）：
`扩展帧 ID = (Addr << 8) | Packet`，数据段 = UART 帧去掉首字节 Addr。
固定 29-bit 扩展帧，与底盘标准帧天然隔离。

## 2. 文件一览

| 文件 | 作用 |
|---|---|
| `zdt_codes.hpp` | 功能码枚举、返回码、状态标志位、固件类型、运动模式 + 几个查询函数声明 |
| `zdt_frame.hpp` | 帧结构 `Frame`、8 个帧构造器 `mk*`、流式解析器 `Parser` |

---

## 3. `zdt_codes.hpp`

### 3.1 协议常量

| 常量 | 值 | 含义 |
|---|---|---|
| `CHECKSUM_FIXED` | `0x6B` | 出厂默认校验字节（所有帧末位） |
| `RPM_MAX` | `3000` | 速度命令上限（§5.3.7） |
| `ACC_MAX` | `255` | 加速度档位上限（00-FF） |
| `ADDR_BROADCAST` | `0x00` | 广播地址 |
| `RESP_LEN_MAX` | `64` | 本工程用到的最大返回帧（`0x42` 配置回读 33 字节） |

### 3.2 `enum class Func : uint8_t` —— 功能码

| 枚举 | 值 | 用途 |
|---|---|---|
| `Enable` | `0xF3` | 使能/失能：`Addr F3 AB <00/01> <sync> 6B` |
| `Speed` | `0xF6` | 速度模式：`Addr F6 <方向> <rpm 2B> <acc> <sync> 6B` |
| `Position` | `0xFD` | 位置模式（Emm 固件 13 字节，见 `mkPositionEmm`） |
| `EStop` | `0xFE` | 立即停止：`Addr FE 98 <sync> 6B`（**不是**失能！） |
| `SyncMove` | `0xFF` | 多机同步触发：广播 `00 FF 66 6B` |
| `MultiCmd` | `0xAA` | 多电机命令（X42S/Y42 专属，一条发多轴） |
| `CalEncoder` | `0x06` | 触发编码器校准 |
| `ZeroPos` | `0x0A` | 当前位置角度清零：`Addr 0A 6B` |
| `ClearStall` | `0x0E` | 解除堵转/过热/过流保护：`Addr 0E 6B` |
| `GetEnc` | `0x31` | 线性化校准后的编码器值（0-65535） |
| `GetVbus` | `0x24` | 母线电压 mV |
| `GetPhase` | `0x27` | 相电流 mA |
| `GetSpeed` | `0x35` | 实时转速 |
| `GetPos` | `0x36` | 实时位置 |
| `GetErr` | `0x37` | 位置误差 |
| `GetTemp` | `0x39` | 驱动温度 ℃ |
| `GetStatus` | `0x3A` | 状态标志位 |
| `GetConfig` | `0x42` | 驱动参数（33 字节，含细分值，用来确认 `--pulses`） |
| `GetSysSt` | `0x43` | 系统状态参数（31 字节） |

### 3.3 其他枚举

- **`enum class Ret`（返回码，§4.2.2）**
  `Ok=0x02`、`Ok12=0x12`、`Ok22=0x22`（三种"命令正确"变体）、
  `ParamErr=0xE2`（参数错误/低压警告/堵转保护）、`FmtErr=0xEE`（格式错误）、
  `Reached=0x9F`（到位/回零完成，主动上报）。
- **`enum class MoveMode`（位置模式，§5.3.12）**
  相对上次目标 `0x00` / 相对零绝对 `0x01` / 相对当前 `0x02`。
- **`enum class Firmware`**
  `Emm=0`（位置模式用脉冲数，转速单位 RPM）、`X=1`（位置用 0.1°，转速 0.1RPM）。
  两者位置帧长不同；本工程默认 Emm。
- **`namespace st`（0x3A 状态位，§5.5.15）**
  `Enabled=0x01`、`InPos=0x02`、`Stalled=0x04`、`StallProt=0x08`、
  `LimLeft=0x10`、`LimRight=0x20`、`PowerLoss=0x80`。

### 3.4 辅助函数（声明在头文件，定义在 `src/protocol/zdt_frame.cpp`）

| 函数 | 行为 |
|---|---|
| `const char* retName(uint8_t)` | 返回码 → 可读字符串（调试打印用） |
| `const char* funcName(uint8_t)` | 功能码 → 可读字符串（调试打印用） |
| `int expectedResponseLen(uint8_t func)` | **该功能码返回帧的总长度**（字节）；未知返回 `-1`。这是解析器切帧的唯一依据 |
| `bool isAckFunc(uint8_t func)` | 是否"控制类命令"（回帧只有返回码，长度 4） |

`expectedResponseLen` 的长度表：

| 功能码 | 返回帧长 | 说明 |
|---|---|---|
| Enable/Speed/Position/EStop/SyncMove/ZeroPos/ClearStall | 4 | `Addr func ret chk` |
| GetVbus / GetPhase / GetEnc / GetTemp | 5 | `Addr func 数据2B chk` |
| GetSpeed | 6 | `Addr 35 <符号> <rpm 2B> chk` |
| GetPos / GetErr | 8 | `Addr 36/37 <符号> <4B> chk` |
| GetStatus | 4 | `Addr 3A <flag> chk` |
| GetConfig | 33 | 含细分等参数 |
| GetSysSt | 31 | 系统状态参数 |

---

## 4. `zdt_frame.hpp`

### 4.1 `struct Frame` —— 一条已解析的返回帧

```cpp
struct Frame {
    uint8_t              addr;   // 地址（1..32）
    uint8_t              func;   // 功能码
    std::vector<uint8_t> data;   // = [返回码, 负载..., 校验码]，含末尾 0x6B
    bool   valid() const;        // data.size() >= 2
    uint8_t ret()  const;        // data[0] 返回码
    std::vector<uint8_t> payload() const;  // 去掉返回码与校验码后的负载
};
```

> 注意：`data` **包含**末尾校验码 `0x6B`，所以纯负载是 `data[1..size-2]`。

### 4.2 帧构造器（全部自动补 `0x6B` 校验）

| 构造器 | 帧布局 | 用途 |
|---|---|---|
| `mkEnable(addr, en, cache=false)` | `Addr F3 AB <en> <sync> 6B` | 使能/失能（6 字节） |
| `mkSpeed(addr, rpm, ccw, acc, cache=false)` | `Addr F6 <dir> <rpm 2B> <acc> <sync> 6B` | 速度模式（8 字节） |
| `mkPositionEmm(addr, pulses, rpm, acc, mode, ccw, cache=false)` | `Addr FD <dir> <rpm 2B> <acc> <脉冲 4B> <mode> <sync> 6B` | 位置模式（Emm，13 字节） |
| `mkEStop(addr, cache=false)` | `Addr FE 98 <sync> 6B` | 立即停止（5 字节） |
| `mkSyncTrigger()` | 广播 `00 FF 66 6B` | 触发多机同步 |
| `mkZeroPos(addr)` | `Addr 0A 6B` | 当前位置清零 |
| `mkClearStall(addr)` | `Addr 0E 6B` | 解除堵转保护 |
| `mkGet(addr, func)` | `Addr <func> 6B` | 读取类命令 |

参数要点：
- `rpm`：0..3000，超出自动夹到 `RPM_MAX`；
- `ccw`：`true` = 逆时针（逻辑正方向约定由设备层 `invert_dir` 决定）；
- `acc`：0 = 直接启停；非 0 时 `t = (256-acc) × 50µs` 每 1 RPM；
- `cache`：ZDT 的"多机同步"缓存位——`cache=true` 时命令先不执行，等 `mkSyncTrigger()` 广播一起动。本工程速率控制暂时用不到。

### 4.3 `class Parser` —— 流式解析器

```cpp
void     feed(const uint8_t* p, size_t n);   // 喂字节
void     feed(const std::vector<uint8_t>& v); // 便捷重载
bool     pop(Frame& out);                    // 取一帧
uint64_t goodFrames() const;                 // 成功切帧计数
uint64_t badFrames()  const;                 // 校验失败计数
size_t   pending()    const;                 // 缓冲中未消费字节
void     clear();                            // 清空缓冲
```

`feed()` 的切帧算法（**实测踩坑逻辑**，实现见 `src/protocol/zdt_frame.cpp`）：

1. 追加到内部 `buf_`；
2. 循环：若 `buf_.size() >= 3`，用 `expectedResponseLen(buf_[1])` 求期望帧长；
   - 未知功能码（返回 `-1`）→ 说明帧头没对齐，**丢掉 1 字节**重新对齐；
   - 字节不够 → 返回，等下次 `feed()`；
   - 帧末不是 `CHECKSUM_FIXED`（`0x6B`）→ `bad_++`，丢 1 字节重新对齐；
   - 否则切出一帧压入 `q_`，`good_++`，消费该帧长度继续。
3. 只有坏帧才计数；正常对齐丢弃的字节不计数（避免误报）。

设计动机：
- 串口上电/干扰会产生垃圾字节，必须能自恢复对齐；
- 用"地址 + 功能码 → 期望长度 + 末尾校验"切帧，比依赖超时更可靠；
- 切出的帧带完整原始字节，上层按功能码解释。

### 4.4 已知坑

1. **粘包**：ZDT 手册 §3.5.2 要求命令间隔 ≥几 ms（实测 0ms 时两条都回复的成功率 1/5，≥1ms 即 5/5），节流在 `ITransport`（默认 12ms）。
2. **校验码固定 0x6B**：驱动板改了校验方式，本工程所有帧构造/解析全部失效。
3. **Emm vs X 固件单位不同**：位置模式脉冲数换算依赖实际细分，`--pulses` 必须用 `GetConfig(0x42)` 的第 9 字节确认。
4. **0x36 位置回读是单圈值**（Emm 固件 0-65535 = 一圈 0-360°），拿不到多圈绝对值，多圈靠设备层积分估计（见 `gimbal/axis.hpp` 的诚实说明）。
