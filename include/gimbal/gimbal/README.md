# gimbal/include/gimbal/gimbal —— 云台控制层（双轴）

把"操作端角速度指令"变成"两台电机的速度帧"的控制核心。

```
Axis（单轴控制律）  ←  yaw / pitch 各一个
    │ 斜坡、死区、软限位、回中、角度估计
    ▼
Gimbal（双轴协调）
    │ 统一时钟、两级看门狗、传输层 poll 去重
    ▼
ZdtMotor → ITransport
```

## 文件一览

| 文件 | 作用 |
|---|---|
| `axis.hpp` | `class Axis`：单轴速率控制律（斜坡/死区/限位/回中/角度估计） |
| `gimbal.hpp` | `class Gimbal`：双轴协调（看门狗/使能空档/传输层去重） |

---

## 1. `axis.hpp` —— `class Axis`

### 1.1 `Config`（单轴参数）

| 字段 | 默认 | 含义 |
|---|---|---|
| `name` | `"axis"` | 名字（打印用） |
| `min_deg` / `max_deg` | `-180 / 180` | 软限位（yaw 有导电滑环时关闭限位） |
| `max_rate_dps` | `180` | 最大角速度 °/s |
| `rate_accel_dps2` | `900` | 角加速度 °/s²（斜坡限幅，防机械冲击） |
| `rate_deadband_dps` | `0.5` | 死区（防抖） |
| `gear_ratio` | `1.0` | 传动比 = 电机转数 / 输出轴转数（2:1 减速填 2.0） |
| `rate_hold_s` | `0.12` | 停手后开始指数衰减的延时（手感层） |
| `rate_decay_tau` | `0.05` | 衰减时间常数（越小停得越快） |
| `use_soft_limits` | `true` | 用软件积分角度估计做软限位 |
| `use_limit_switches` | `true` | 联动驱动板限位开关（状态位 bit4/bit5） |

> ⚠️ 传动比影响两处：`电机 RPM = 输出 °/s × gear / 6`，以及角度估计的反算。
> 填错 = 全盘算错（自检里有 `90°/s @ gear=2 → 30 RPM` 的断言）。

### 1.2 操作端接口

| 函数 | 行为 |
|---|---|
| `setRate(double dps)` | 设定角速度指令：限幅到 `±max_rate_dps`，`since_cmd_s_ = 0` |
| `hold()` | 停止转动（保持使能、抱死）：`rate_cmd_ = 0` |
| `zeroHere()` | 把当前角度估计清零 + 电机零点对齐（**应在机械零点调用**） |
| `goHome(dps=60, tol=2°)` | 回中：进入 `homing_`，按 `dps` 转向 0°，误差进入 `tol` 后停 |
| `estop()` | 急停：速度清零 + 电机急停帧 + 失能 + 影子重置 |
| `enable(bool on)` | 使能/失能（失能时速度清零） |

### 1.3 `update(double dt)` —— 每拍控制律（顺序即优先级）

1. **`motor_->update()`**：遥测下发与收帧，**与 dt 无关，每拍都跑**；
2. dt 保护：`<=0` 直接返回；`>0.2` 夹到 0.2（调试断点后不要大步长跳变）；
3. **回中逻辑**：`homing_` 时按误差方向给 ±`home_rate_`，误差够小则退出回中；
4. **停手平滑衰减（手感层）**：`since_cmd_s_ += dt`，超过 `rate_hold_s` 后按
   `exp(-dt/tau)` 指数衰减 `rate_cmd_`，绝对值 <0.05 置 0；
5. **斜坡限幅**：`rate_act_` 以 `rate_accel_dps2 × dt` 为步长逼近 `rate_cmd_`；
6. **死区**：`|rate_act_| < deadband` 且指令为 0 → 归零；
7. **限位**：`clampToLimits()`（软限位 + 限位开关）；
8. **未使能**：`rate_act_ = 0`；
9. **下发**：`motor_->setSpeedRpm(rate_act_ × gear / 6)`（输出 °/s → 电机 RPM，360°/s = 60RPM）；
10. **角度估计**：优先用电机实测转速积分（`rpm × 6 / gear × dt`），无回帧（`age_s >= 0.5`）时
    退化为指令值；软限位开启时估计值硬夹在 `[min-5, max+5]`。

### 1.4 查询接口

`cfg()`、`cmdRateDps()`（指令）、`actRateDps()`（实际斜坡后）、`estAngleDeg()`、
`atSoftLimit()`、`limitSwitchActive()`、`homing()`、`targetDeg()`、`enabled()`、
`motor()/motorStatus()`。

### 1.5 诚实说明：角度估计会漂移

ZDT 的 `0x36` 回读对 Emm 固件是**单圈 0-360°**，拿不到多圈绝对值，所以 `est_angle_`
是转速积分出来的**软件估计**，长时间运行会缓慢漂移。真正的绝对保护应依赖驱动板限位开关
（状态位 bit4/bit5）；需要绝对角度时用 `zeroHere()` 在机械零点重新对齐。

---

## 2. `gimbal.hpp` —— `class Gimbal`

### 2.1 `Config`

| 字段 | 默认 | 含义 |
|---|---|---|
| `yaw` / `pitch` | `Axis::Config` | 两轴各自的参数 |
| `command_timeout_s` | `0.5` | 操作端失联判定（安全层看门狗） |
| `auto_enable` | `true` | 构造后自动使能 |
| `enable_delay_s` | `0.3` | 使能后空档期（避免上电抖动被立刻执行） |

### 2.2 构造函数

`Gimbal(yaw_motor, pitch_motor, Config)`：
- 构造两轴 `Axis`；
- `collectTransports()` 收集**去重后**的传输层（两轴共总线时只有一个）；
- `auto_enable` 时立即 `enable(true)`。

### 2.3 操作端接口

| 函数 | 行为 |
|---|---|
| `enable(bool on)` | 使能/失能。**幂等**（见坑）；状态没变直接返回 |
| `setRate(yaw, pitch)` | 两轴各设角速度；刷新命令时钟；清 `watchdog_` |
| `hold()` | 两轴停手；刷新命令时钟 |
| `home()` | 两轴回中；刷新命令时钟 |
| `estop()` | 两轴急停（立即停 + 失能）；刷新命令时钟 |

### 2.4 `step(double dt)` —— 一次控制拍

1. dt 保护（`<=0` 返回；`>0.2` 夹到 0.2）；
2. `since_cmd_s_ += dt`、`enable_age_s_ += dt`；
3. **先 poll 传输层**（把命令推到总线，节流在 transport 里）；
4. **安全层看门狗**：`enabled && since_cmd_s_ > command_timeout_s` → `watchdog_=true`，
   两轴 `hold()`。**只归零速度，不失能** —— 云台失能会松轴下垂，比继续转更危险；
5. 计算是否处于使能空档期（`enable_age_s_ < enable_delay_s`）；
6. 两轴 `update(in_delay ? 0 : dt)`；空档期内强制 `hold()`；
7. **再 poll 一次**收帧，把驱动板回帧填进状态。

### 2.5 查询

`yaw()/pitch()`（Axis 引用）、`watchdogTripped()`、`sinceCommandS()`、`enabled()`、
`cfg()`、`transports()`。

### 2.6 关键坑

1. **`enable()` 必须幂等 ★**：操作端每个控制包都带 `Enable` 标志（语义是"保持使能"）。
   如果每次都把 `enable_age_s_` 清零，`enable_delay_s` 就永远走不完，两轴被永久压在
   `hold()` 上 —— 表现为"发什么都是 0，但状态全绿"。这个坑实地踩过一次，排查花了很久。
2. **看门狗两级**：手感层在 `Axis`（`rate_hold_s` 后指数衰减），安全层在 `Gimbal`
   （`command_timeout_s` 后强制归零）。两层都**不失能**。
3. **传输层 poll 去重**：两轴挂同一总线时只 poll 一次，避免同一串口被 poll 两次。
