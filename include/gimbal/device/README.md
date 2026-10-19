# gimbal/include/gimbal/device —— 电机设备层

本目录只有 `zdt_motor.hpp`：把一台 ZDT 闭环步进电机抽象成"设备"，
负责 **物理量 ↔ 协议帧** 的翻译与遥测轮询。通信节流/收包由 `transport/` 负责，本层不碰 fd。

## 文件一览

| 文件 | 作用 |
|---|---|
| `zdt_motor.hpp` | `class ZdtMotor`：使能 / 速度 / 位置 / 急停 / 归零 / 状态回读 |

## `zdt_motor.hpp` —— `class ZdtMotor`

### 1. `Config`（电机参数）

| 字段 | 默认 | 含义 |
|---|---|---|
| `addr` | `1` | 电机地址（总线寻址，本工程 yaw=2、pitch=1） |
| `firmware` | `Emm` | 固件类型（Emm / X，位置模式单位不同） |
| `pulses_per_rev` | `3200` | 脉冲/电机圈 = 200 × 细分（16 细分） |
| `acc_default` | `200` | 加速度档位（0-255） |
| `invert_dir` | `false` | 逻辑正方向是否对应 CW（方向极性以实物为准） |
| `max_rpm` | `3000.0` | 速度命令限幅 |
| `poll_speed_ms` | `100` | 转速回读周期 |
| `poll_pos_ms` | `100` | 位置回读周期 |
| `poll_status_ms` | `200` | 状态位回读周期 |
| `poll_vbus_ms` | `1000` | 母线电压回读周期 |
| `poll_temp_ms` | `2000` | 温度回读周期 |

### 2. `Status`（电机状态快照）

| 字段 | 含义 |
|---|---|
| `valid` | 收到过有效回帧 |
| `enabled/in_position/stalled/stall_prot/lim_left/lim_right/power_loss` | 0x3A 状态位 |
| `speed_rpm` | 实测转速（带符号，按逻辑正方向） |
| `pos_deg` / `pos_raw` | 单圈位置（Emm：`raw × 360 / 65536`） |
| `vbus_mv` / `temp_c` | 母线电压 / 温度 |
| `last_ret` | 最近一次控制命令的返回码 |
| `acks_ok` / `acks_err` | 控制命令应答计数（按返回码判定） |
| `frames` | 收到的返回帧数 |
| `age_s` | 距上次收到回帧的秒数 |

### 3. 构造函数

`ZdtMotor(std::shared_ptr<ITransport> t, Config c)`：
构造时初始化 5 个遥测轮询项（`PollItem{func, period_ms, next_ms}`），
初始 `next_ms` **按 5ms 错开**，避免所有遥测挤在同一拍成簇发送。

### 4. 命令（物理量 → 帧）

| 函数 | 行为 |
|---|---|
| `setEnabled(bool on)` | 下发使能帧。**影子去重**：状态没变就不发；失能时清速度影子 |
| `setSpeedRpm(double signed_rpm)` | 限幅 `max_rpm`、绝对值 <0.5 视为 0；**影子去重**（变化 <0.5RPM 不发）；`ccw = (正负 != invert_dir)` |
| `setPositionDeg(double deg, uint16_t rpm, MoveMode mode)` | 仅 Emm 固件实现（X 固件需补帧构造器）；角度 → 脉冲（`deg/360 × pulses_per_rev`）；位置命令会改变运动状态，速度影子失效 |
| `estop()` | 下发急停帧（`urgent=true`，**插队**，见 transport 文档）；清速度影子 |
| `zeroPosition()` | 当前位置清零（机械零点时调用） |
| `clearStall()` | 解除堵转/过热/过流保护 |
| `resyncCommands()` | 清空"已下发"影子值，让下个周期强制重发（断线重连后调用） |

### 5. 周期

`void update()`：
1. 遍历 `polls_`，到期的项下发 `mkGet(addr, func)` 并重排下次时刻（每项独立计时，不用轮转指针）；
2. `popFrameFor(addr)` 收本机回帧 → `handleFrame()` 填 Status；
3. 维护 `age_s`。

> ⚠️ 为什么不用轮转指针：`next = (next+1) % N` 遇到长周期项会卡住等一整拍，
> 实测回帧率从 11.5/s 掉到 2.5/s（状态冻结几秒后猛跳一次）。

### 6. 收帧处理 `handleFrame()`

| 功能码 | 解析出的内容 |
|---|---|
| Enable/Speed/Position/EStop/ZeroPos/ClearStall | 返回码（`0x02/0x12/0x22` 记为 ok，否则 err） |
| GetSpeed | `data[0]` 符号、`data[1..2]` BE16 转速 → `speed_rpm`（按 `invert_dir` 取反） |
| GetPos | `data[0]` 符号、`data[1..4]` 4B 原始值 → 单圈角度 |
| GetStatus | `data[0]` 状态位 → 各标志 |
| GetVbus | BE16 mV |
| GetTemp | `data[0]` 符号、`data[1]` 摄氏度 |
| 其他 | 忽略 |

### 7. 查询

`status()`、`cfg()`、`addr()`、`transport()/transportPtr()`、`communicationOk(timeout_s)`、
`degToPulses()/pulsesToDeg()`、`setPulsesPerRev()`。

### 8. 关键坑

1. **速度单位**：ZDT 速度模式单位是电机 RPM（不是输出轴 °/s），传动比在 `Axis` 层换算。
2. **0x36 位置是单圈值**：多圈角度靠上层积分估计，会漂移；绝对保护靠限位开关。
3. **影子去重**：`setSpeedRpm` 每拍都会被 `Axis::update()` 调用，去重避免总线被同值命令刷爆。
4. **`invert_dir` 只管逻辑方向**，接线极性异常时先用它，不要改帧构造。
5. **遥测轮询占用总线**：每台电机约 26 帧/秒（speed 10 + pos 10 + status 5 + vbus 1 + temp 0.5），
   多用一根总线容量有限时优先降 `poll_*_ms`。
