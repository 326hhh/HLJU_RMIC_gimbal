# gimbal/src/device —— 电机设备层实现

本目录只有 `zdt_motor.cpp`：实现 `include/gimbal/device/zdt_motor.hpp` 的 `ZdtMotor`。

## 文件一览

| 文件 | 内容 |
|---|---|
| `zdt_motor.cpp` | 物理量 ↔ ZDT 帧、影子去重、遥测轮询、回帧解析 |

---

## 1. 函数一览

| 函数 | 行为 |
|---|---|
| `nowMs()` | 单调时钟毫秒数（遥测轮询计时用） |
| `ZdtMotor(transport, cfg)` | 绑定传输层；初始化 5 个遥测轮询项，初始 `next_ms` 按 5ms 错开 |
| `degToPulses(deg)` | `deg / 360 × pulses_per_rev` 四舍五入 |
| `pulsesToDeg(p)` | `p × 360 / pulses_per_rev`（`pulses_per_rev=0` 时返回 0） |
| `setEnabled(on)` | 影子有效且状态没变 → 不发；否则下发 `mkEnable` 并更新影子；失能时清速度影子 |
| `setSpeedRpm(signed_rpm)` | 限幅 `max_rpm`；绝对值 <0.5 视为 0；影子有效且变化 <0.5RPM → 不发；否则 `mkSpeed(addr, |rpm|, ccw, acc)`，其中 `ccw = (正值 != invert_dir)` |
| `setPositionDeg(deg, rpm, mode)` | 非 Emm 固件直接返回（X 未实现）；角度→脉冲；`ccw = (pulses>=0) != invert_dir`；下发 `mkPositionEmm`；位置命令改变运动状态 → 速度影子失效 |
| `estop()` | 下发 `mkEStop`（`urgent=true` 插队）；清速度影子 |
| `zeroPosition()` | 下发 `mkZeroPos`（当前位置归零） |
| `clearStall()` | 下发 `mkClearStall`（解除堵转/过热/过流保护） |
| `resyncCommands()` | 使能影子、速度影子都失效 → 下个周期强制重发 |
| `update()` | ① 到期的遥测项下发 `mkGet` 并按周期重排；② `popFrameFor(addr)` 收本机帧 → `handleFrame`；③ 更新 `age_s` |
| `handleFrame(f)` | 按功能码解释回帧并更新 `Status` |

## 2. `update()` 细节

```cpp
void ZdtMotor::update() {
    const double t = nowMs();
    // 1. 到期遥测：每项独立计时，不用轮转指针
    for (auto& p : polls_) {
        if (t >= p.next_ms) {
            transport_->send(zdt::mkGet(cfg_.addr, p.func));
            p.next_ms = t + p.period_ms;
        }
    }
    // 2. 收帧：一总线多电机时按地址过滤
    zdt::Frame f;
    while (transport_->popFrameFor(cfg_.addr, f)) handleFrame(f);
    // 3. 时效
    if (has_rx_tp_) st_.age_s = 距上次回帧秒数;
}
```

为什么每项独立计时：轮转指针 `next = (next+1) % N` 遇到长周期项（如温度 2000ms）会
让短周期项一起等一整拍，实测回帧率从 11.5/s 掉到 2.5/s —— 表现为"状态冻结几秒后猛跳"。
初始 `next_ms` 错开 5ms 也是为了避免所有遥测挤在同一拍。

## 3. `handleFrame()` 各功能码解析

| 功能码 | 处理 |
|---|---|
| Enable/Speed/Position/EStop/ZeroPos/ClearStall | `st_.last_ret = data[0]`；`0x02/0x12/0x22` → `acks_ok++`，否则 `acks_err++` |
| GetSpeed | `data[0]` 符号位、`data[1..2]` BE16 → `speed_rpm`（按 `invert_dir` 取反） |
| GetPos | `data[0]` 符号位、`data[1..4]` 4 字节 → `pos_raw`；Emm 固件按 `(raw & 0xFFFF) × 360 / 65536` 得单圈角度 |
| GetStatus | `data[0]` 的位 → `enabled/in_position/stalled/stall_prot/lim_left/lim_right/power_loss` |
| GetVbus | `data[0..1]` BE16 mV |
| GetTemp | `data[0]` 符号、`data[1]` 摄氏度 |
| 其他 | 忽略（`data.size()` 不够时也直接跳过，防越界） |

每次收到任意回帧都会：`st_.frames` 更新、记录 `last_rx_tp_`、`age_s = 0`、`valid = true`。

## 4. 关键坑

1. **影子去重**：`Axis::update()` 每拍（100Hz）都调 `setSpeedRpm()`，如果每次都发帧，
   总线会被同值命令刷爆；影子值只在"变了"才发（阈值 0.5RPM 防抖）。
2. **方向极性**：逻辑正方向与电机 CW 的关系由 `invert_dir` 统一表达；接线反了先改这个，
   不要动帧构造器。
3. **0x36 位置单圈**：`pos_deg` 是单圈 0..360°，多圈由 `Axis` 层积分估计。
4. **Emm 单位**：速度命令是电机 RPM、位置命令是脉冲数；X 固件单位不同（本工程未实现位置）。
5. **失能后速度影子失效**：重启电机后下发的速度地址要重发，`resyncCommands()` 负责。
