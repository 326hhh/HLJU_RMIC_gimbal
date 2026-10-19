# gimbal/src/gimbal —— 云台控制实现

实现 `include/gimbal/gimbal/` 下的两个类：`Axis`（单轴）与 `Gimbal`（双轴）。

## 文件一览

| 文件 | 实现 | 内容 |
|---|---|---|
| `axis.cpp` | `class Axis` | 单轴速率控制律：斜坡、死区、软限位、回中、角度估计 |
| `gimbal.cpp` | `class Gimbal` | 双轴调度、看门狗、使能空档、estop/home/enable 协调 |

---

## 1. `axis.cpp` —— 单轴控制

### 1.1 函数一览

| 函数 | 行为 |
|---|---|
| `Axis(motor, cfg)` | 保存电机与配置 |
| `limitSwitchActive()` | 读取电机状态 `lim_left || lim_right` |
| `clampToLimits(rate)`（私有） | 软限位：接近上限且还要正向 → 返回 0；接近下限且还要反向 → 0；限位开关：`lim_right` 时禁止正向、`lim_left` 时禁止反向（允许退出限位） |
| `setRate(dps)` | 限幅到 `±max_rate_dps`；`since_cmd_s_ = 0` |
| `hold()` | `rate_cmd_ = 0`；退出回中 |
| `zeroHere()` | 角度估计清零、清限位标志、电机 `zeroPosition()` |
| `goHome(dps=60, tol=2)` | 设置回中目标 0°、容差、速率，`homing_ = true` |
| `estop()` | 清指令/实际速度、退出回中、失能、电机急停 + 失能 + 影子重置 |
| `enable(on)` | 使能/失能；失能时清速度 |
| `update(dt)` | 每拍控制律（见下） |
| 查询（inline） | `cfg/cmdRateDps/actRateDps/estAngleDeg/atSoftLimit/limitSwitchActive/homing/targetDeg/enabled/motor/motorStatus` |

### 1.2 `update(dt)` 的控制律顺序

```cpp
1. motor_->update();               // 遥测下发 + 收帧，与 dt 无关，每拍都跑
2. dt 保护：<=0 返回；>0.2 夹到 0.2
3. 回中（homing_）：按误差方向给 ±home_rate_，|误差|<=tol 时退出
4. 停手衰减（手感层）：since_cmd_s_ += dt；
   if (since_cmd_s_ > rate_hold_s) rate_cmd_ *= exp(-dt/rate_decay_tau)；
   |rate_cmd_| < 0.05 置 0
5. 斜坡限幅：rate_act_ 以 rate_accel_dps2*dt 为步长逼近 rate_cmd_
6. 死区：|rate_act_| < deadband 且 cmd≈0 → 0
7. 限位：clampToLimits()
8. 未使能：rate_act_ = 0
9. 下发：motor_->setSpeedRpm(rate_act_ * gear / 6.0)     // °/s → 电机 RPM
10. 角度估计：实测转速（age<0.5s）积分 rpm*6/gear*dt；否则退化为指令值；
    软限位开启时估计值硬夹 [min-5, max+5]
```

### 1.3 注意

- `est_angle_` 是**转速积分估计**，会缓慢漂移（0x36 只有单圈值）；绝对保护靠限位开关。
- 停手衰减与斜坡配合：停手后先按 `rate_decay_tau` 指数衰减指令，再由斜坡限幅逼近，
  所以实际的停止曲线 ≈ 指数 × 加速度双重约束，手感柔和且不丢响应。
- `at_limit_`：本拍开始时有速度、被限位清零 → 置位，供操作端显示"顶到限位"。

---

## 2. `gimbal.cpp` —— 双轴协调

### 2.1 函数一览

| 函数 | 行为 |
|---|---|
| `Gimbal(yaw_motor, pitch_motor, cfg)` | 构造两轴 `Axis`；`collectTransports()`；`cfg.auto_enable` 时立即 `enable(true)` |
| `collectTransports()`（私有） | 收集两轴传输层指针，**按指针去重**（两轴共总线时只留一个） |
| `pollAllTransports()`（私有） | 对去重后的列表逐个 `poll()` |
| `enable(on)` | **幂等**：状态没变直接返回；变了才置标志、清 `enable_age_s_`、两轴 `enable()`、刷新命令时钟 |
| `setRate(yaw, pitch)` | 两轴 `setRate()`；刷新命令时钟；`watchdog_ = false` |
| `hold()` | 两轴 `hold()`；刷新命令时钟 |
| `home()` | 两轴 `goHome()`；刷新命令时钟 |
| `estop()` | 失能标志清零、两轴 `estop()`、刷新命令时钟 |
| `step(dt)` | 一次控制拍（见下） |
| 查询（inline） | `yaw()/pitch()/watchdogTripped()/sinceCommandS()/enabled()/cfg()/transports()` |

### 2.2 `step(dt)` 的顺序

```cpp
1. dt 保护（<=0 返回；>0.2 夹到 0.2）
2. since_cmd_s_ += dt；enable_age_s_ += dt
3. pollAllTransports()                 // 先发命令（节流在 transport 里）
4. 安全层看门狗：enabled && since_cmd_s_ > command_timeout_s
   → watchdog_ = true；两轴 hold()       // 只归零速度，不失能
5. in_delay = enabled && enable_age_s_ < enable_delay_s
6. yaw_.update(in_delay ? 0 : dt)；pitch_.update(in_delay ? 0 : dt)
   空档期内两轴 hold()
7. pollAllTransports()                 // 收帧，把回帧填进状态
```

### 2.3 关键坑

1. **`enable()` 幂等**：操作端每个控制包都带 `Enable` 标志。若每次清零 `enable_age_s_`，
   `enable_delay_s` 永远走不完 → 两轴被永久压在 `hold()` → "发什么都是 0 但状态全绿"。
2. **看门狗两级且都不失能**：`Axis` 层的停手衰减（手感）+ `Gimbal` 层的强制归零（安全）。
   云台一失能就松轴下垂，比继续转更危险。
3. **传输层去重**：两轴共总线时 `transports()` 只有一个元素，`step()` 里两次 poll 都只 poll
   一次 —— 也正好保证"先发后收"的时序。
4. **急停语义**：`Gimbal::estop()` 直接清 `enabled_`，不发使能帧（轴自己会发急停+失能帧）；
   下一帧操作端带 `Enable` 不会自动恢复使能（`enable()` 幂等，见 2.3.1）。
