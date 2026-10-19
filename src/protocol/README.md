# gimbal/src/protocol —— ZDT 协议实现

本目录只有 `zdt_frame.cpp`：实现 `include/gimbal/protocol/zdt_codes.hpp` 声明的
查询函数，以及 `zdt_frame.hpp` 声明的帧构造器与流式解析器。

## 文件一览

| 文件 | 对应头文件 | 内容 |
|---|---|---|
| `zdt_frame.cpp` | `protocol/zdt_codes.hpp` + `protocol/zdt_frame.hpp` | 常量查询函数、8 个帧构造器、`Parser` 切帧算法 |

---

## 1. 匿名命名空间工具（文件内私有）

| 函数 | 行为 |
|---|---|
| `pushBE16(vector, u16)` | 按**大端**追加 2 字节（ZDT 帧数据段用大端） |
| `pushBE32(vector, u32)` | 按大端追加 4 字节（位置脉冲数） |

> 注意：ZDT 帧**数据段是 BE**，而底盘帧/UDP 帧是 LE —— 两套协议不要混。

## 2. 常量查询函数（对应 `zdt_codes.hpp` 声明）

| 函数 | 行为 |
|---|---|
| `retName(code)` | 返回码 → 字符串：`OK` / `OK(0x12)` / `OK(0x22)` / `PARAM_ERR` / `FORMAT_ERR` / `REACHED`，未知返回 `UNKNOWN` |
| `funcName(func)` | 功能码 → 字符串（`Enable`/`Speed`/`Position`/`EStop`/`SyncMove`/`MultiCmd`/`CalEncoder`/`ZeroPos`/`ClearStall`/`GetEnc`/`GetVbus`/`GetPhase`/`GetSpeed`/`GetPos`/`GetErr`/`GetTemp`/`GetStatus`/`GetConfig`/`GetSysStatus`），未知返回 `?` |
| `expectedResponseLen(func)` | 期望返回帧总长（switch 表，见头文件文档）；未知返回 `-1` |
| `isAckFunc(func)` | `expectedResponseLen(func) == 4` |

`expectedResponseLen` 是解析器的唯一长度依据：长度表改动会同时影响切帧正确性，
改功能码时务必同步。

## 3. 帧构造器（全部以 `0x6B` 结尾）

| 构造器 | 实现要点 |
|---|---|
| `mkEnable(addr, en, cache)` | 固定字节序列：`{addr, 0xF3, 0xAB, en?1:0, cache?1:0, 0x6B}` |
| `mkSpeed(addr, rpm, ccw, acc, cache)` | 先夹 `rpm` 到 `RPM_MAX`；`{addr, 0xF6, ccw?1:0, BE16(rpm), acc, cache?1:0, 0x6B}` |
| `mkPositionEmm(addr, pulses, rpm, acc, mode, ccw, cache)` | 先夹 `rpm`；`{addr, 0xFD, dir, BE16(rpm), acc, BE32(pulses), mode, cache?1:0, 0x6B}`（13 字节） |
| `mkEStop(addr, cache)` | `{addr, 0xFE, 0x98, cache?1:0, 0x6B}` |
| `mkSyncTrigger()` | 广播固定帧 `{0x00, 0xFF, 0x66, 0x6B}` |
| `mkZeroPos(addr)` | `{addr, 0x0A, 0x6B}` |
| `mkClearStall(addr)` | `{addr, 0x0E, 0x6B}` |
| `mkGet(addr, func)` | `{addr, (uint8_t)func, 0x6B}` |

`pulses` 用 `uint32_t` 写入（BE32），负数会在 `ZdtMotor::setPositionDeg` 里被取绝对值，
方向由 `ccw` 参数表达。

## 4. `Parser` 实现（切帧状态机）

### 4.1 `feed(p, n)`

1. 把字节追加到 `buf_`；
2. `while (buf_.size() >= 3)`：
   - `want = expectedResponseLen(buf_[1])`；
   - `want <= 0`（不认识的功能码）：`buf_.erase(begin)` 丢 1 字节重对齐，继续；
   - 字节不够 `want`：`return` 等下次 feed；
   - `buf_[want-1] != 0x6B`：`bad_++`，丢 1 字节重对齐，继续；
   - 合法帧：构造 `Frame{addr, func, data=buf_[2..want-1]}`（含返回码与校验码）压入 `q_`，
     `good_++`，`buf_.erase(0, want)` 消费该帧。

### 4.2 `pop(Frame&)`

从 `q_` 队头弹出一帧；空则返回 false。

### 4.3 为什么这样设计

- 串口上电/干扰会灌入垃圾字节：**丢 1 字节重对齐**能自恢复；
- 用「地址 + 功能码 → 期望长度 + 末尾校验」切帧，不依赖超时（超时在 100Hz 轮询下不可靠）；
- 只有坏帧计数（`bad_`），正常重新对齐的丢弃字节不计数，避免误报；
- `pending()` 暴露缓冲长度，联调时可观察是否有卡死半帧。
