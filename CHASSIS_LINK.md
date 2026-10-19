# 底盘链路（Jetson ↔ STM32F407 USART2）使用与排障手册

> 适用：`~/gimbal`（Jetson 侧 C++17 工程） + `~/gimbal_ctl`（笔记本侧 C++17 客户端）
> + `~/stmproject/INTRA_SCHOOL_COMPETITION_27RM`（F407 固件，C）
> 本文档只写「怎么用 / 怎么连 / 出问题怎么查」，协议细节以代码注释为准。

---

## 1. 系统拓扑

```
笔记本(PC)                         Jetson Nano                       STM32F407
┌──────────────┐   WiFi 或 网线   ┌──────────────┐   USB-TTL(115200)  ┌────────────┐
│ gimbal_ctl   │ ──── UDP ────►  │ gimbal       │ ─── USART2 ──────► │ uart_link  │
│ (C++17, X11) │  24B/56B        │ (C++17 守护) │  PD5(TX)/PD6(RX)   │ (1kHz 控制)│
└──────────────┘                  └──────────────┘                    └────────────┘
                                        │  USB-TTL → ZDT 步进 ×2（云台，暂未接）
                                        └─ 目前唯一 CH340 给了 F407
```

| 角色 | 设备/端口 | 说明 |
|---|---|---|
| 笔记本 ↔ Jetson（有线） | `enp8s0 192.168.2.1` ↔ `eth0 192.168.2.2` | `ssh Jetson` 免密 |
| 笔记本 ↔ Jetson（无线） | 笔记本 WiFi ↔ Jetson AP `MECAR-CTRL` / `10.42.0.1` | 车要走位时用这条 |
| Jetson ↔ F407 | `/dev/ttyUSB0`（CH340）115200 8N1 | 底盘控制链路 |
| 笔记本 ↔ F407 调试 | `/dev/ttyACM0`（DAP-LINK 虚拟串口）| USART1 调试打印，115200 |
| UDP 控制口 | `40001`（默认）| `gimbal_ctl` → `gimbal` |

---

## 2. 坐标系约定（必须记牢）

与底盘库一致（`docs/CHASSIS_LIBRARY.md` §4）：

| 量 | 正方向 |
|---|---|
| `vx` | 车体前进 |
| `vy` | 车体左移 |
| `wz` | 俯视逆时针 |

⚠️ 旧草案里的「vx 右 / vy 前」已废弃，代码里只有这一套。

---

## 3. USART2 二进制协议

### 3.1 命令帧（Jetson → F407，12 字节，小端）

| off | 0 | 1 | 2 | 3 | 4..5 | 6..7 | 8..9 | 10 | 11 |
|---|---|---|---|---|---|---|---|---|---|
| 字段 | `0xA5` | `0x5A` | cmd | seq | vx | vy | wz | flags | crc8 |
| 类型 | u8 | u8 | u8 | u8 | i16 | i16 | i16 | u8 | u8 |

- `vx/vy` 单位 **mm/s**，`wz` 单位 **0.01 rad/s**
- `crc8` = CRC-8/ATM（poly `0x07`, init `0`），覆盖第 0~10 字节
- 命令字：`0x01` 速度 / `0x02` 急停 / `0x03` 小陀螺 / `0x04` 清零
- flags：`0x01` = EN（允许驱动）

### 3.2 反馈帧（F407 → Jetson，40 字节，小端，100 Hz）

| off | 字段 | 类型 | 单位 |
|---|---|---|---|
| 0..1 | `0x5A 0xA5` | — | 帧头 |
| 2 | 类型 `0x81` | u8 | — |
| 3 | seq（回显最后一条合法命令） | u8 | — |
| 4..7 | x | i32 | mm |
| 8..11 | y | i32 | mm |
| 12..13 | yaw（里程计） | i16 | 0.01° |
| 14..15 | vx（正解） | i16 | mm/s |
| 16..17 | vy（正解） | i16 | mm/s |
| 18..19 | wz（正解） | i16 | 0.01 rad/s |
| 20..21 | gyro_yaw（陀螺积分） | i16 | 0.01° |
| 22..23 | gyro_wz（陀螺实测） | i16 | 0.01 rad/s |
| 24..25 | status | u16 | 位见下 |
| 26..27 | cmd_cnt（累计合法命令数） | u16 | — |
| 28..29 | fb_seq（反馈帧序号） | u16 | — |
| 30..31 | wheel FL 实测转速 | i16 | 0.01 RPM |
| 32..33 | wheel FR | i16 | 0.01 RPM |
| 34..35 | wheel BL | i16 | 0.01 RPM |
| 36..37 | wheel BR | i16 | 0.01 RPM |
| 38 | crc8（覆盖 0~37） | u8 | — |
| 39 | `0x0D` | u8 | 帧尾 |

> **v2 变更**：30~37 新增四轮实测转速（0.01 RPM）。麦轮方向检查靠它。

### 3.3 status 状态位

| 位 | 名称 | 含义 |
|---|---|---|
| `0x0001` | IMU_ONLINE | IMU 在线 |
| `0x0002` | LINK_OK | 500 ms 内收到过控制帧 |
| `0x0004` | WATCHDOG | 100 ms 无控制帧（已自动停车） |
| `0x0008` | ESTOP | 急停锁存中 |
| `0x0010` | DRIVER_ON | TB6612 STBY 高（输出使能） |
| `0x0020` | SPIN | 小陀螺模式 |
| `0x0040` | GYRO_ON | 陀螺外环非 OFF |
| `0x0080` | IMU_CAL | 零偏标定有效 |

### 3.4 安全语义（三层）

1. **Jetson 侧**：UDP 指令断流 250 ms → 主动发 EN=0 停车帧
2. **F407 侧**：控制帧 100 ms 未到 → 目标速度清零走斜坡减速，**但 STBY 保持高**（不松轴）
3. **急停**：只有 `x`/`cx` 才拉低 STBY（电机自由）

---

## 4. UDP 线协议（gimbal_ctl ↔ gimbal）

**上行 24 B**：magic `0x4D454341` / seq / yaw_cdps / pitch_cdps / flags / chk（0~12 XOR）/
vx_mmps / vy_mmps / wz_mradps / chassis_flags / chk2（14~20 XOR）/ pad

**下行 64 B**：云台遥测（前 32 B）+ 底盘遥测（32~53：位姿/正解/陀螺/status/valid）
+ 四轮转速（54~61，0.01 RPM）+ chk16b（62~63）

`chassis_flags`：`0x01` EN / `0x02` EStop / `0x04` Clear。

---

## 5. 无线连接（笔记本 ↔ 车载 AP）

### 5.1 Jetson 侧起 AP

```bash
# 注意：car-ap 是 autoconnect=no，需要手动拉起
sudo nmcli con up car-ap
# 验证（关键！见 §8.1 假活问题）
cat /proc/net/dev | grep wlan0; sleep 3; cat /proc/net/dev | grep wlan0
```

AP 参数：SSID `MECAR-CTRL` / 密码 `mecar12345` / 2.4G ch6 / `10.42.0.1/24` / mDNS `nano.local`

### 5.2 笔记本接入

```bash
# ① 连 AP（会切走校园网，暂时没外网，正常）
sudo nmcli device wifi connect MECAR-CTRL password mecar12345
ip -br addr show wlp14s0          # 应拿到 10.42.0.x

# ② 绕过 Meta(Clash) VPN：否则 10.42.0.1 会被塞进隧道
sudo ip rule add to 10.42.0.0/24 lookup main priority 8000

# ③ 验证：必须是 dev wlp14s0，不能是 dev Meta
ip route get 10.42.0.1
ping -c3 10.42.0.1
```

### 5.3 切回校园网 / 恢复上网

```bash
# 笔记本上
sudo ip rule del to 10.42.0.0/24 lookup main priority 8000   # 删掉临时绕行规则
sudo nmcli con up 黑大联通        # 或换成你平时连的 WiFi 名；也可 nmcli device wifi list 里挑
ip -br addr show wlp14s0          # 应回到 10.230.x 之类的校园网地址
ping -c3 8.8.8.8                  # 验证外网（VPN 生效的话走 Meta）

# 可选：顺手关掉车载 AP 省电（Jetson 上）
sudo nmcli con down car-ap
```

> `ip rule` 是临时的，**每次重连 WiFi / 重启 VPN 后都要重新加**。
> 如果要做持久化（NM dispatcher 或 systemd），需改系统文件，说一声再动。

---

## 5.4 图传（车载摄像头 → 笔记本）

Jetson 上用 `mjpeg_server`（C++17，V4L2 直出 MJPG，零转码）：

```bash
cd ~/gimbal
./build/mjpeg_server --size 640x480 --fps 30 --port 8080     # 默认就是这个
# 浏览器验证：http://<Jetson_IP>:8080/stream
```

`gimbal_ctl` 启动时会自动拉 `http://<host>:8080/stream` 并画在窗口里（`v` 键开关）。

带宽参考：640x480@30 实测约 **2 Mbps**（Jetson AP 够用）；1280x720@30 约 8~15 Mbps。

## 6. 运行

### 6.1 Jetson

```bash
cd ~/gimbal
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4

./build/gimbal --selftest                                  # 纯软件自检
./build/gimbal --dry --chassis-dev /dev/ttyUSB0            # 云台模拟 + 真底盘
./build/gimbal --device /dev/ttyUSB1 --chassis-dev /dev/ttyUSB0   # 云台串口就位后用这条
```

### 6.2 笔记本

```bash
cd ~/gimbal_ctl && ./build/gimbal_ctl --host 10.42.0.1      # 无线
# 或 ./build/gimbal_ctl --host 192.168.2.2                   # 有线
```

| 按键 | 作用 |
|---|---|
| 鼠标移动 | 云台 yaw/pitch |
| `W/S` | 底盘前进/后退 |
| `A/D` | 底盘左右平移 |
| `←/→` | 底盘左转/右转 |
| `空格` | 全部停手 |
| `e` / `f` | 云台 / 底盘使能 |
| `x` | 急停（云台 + 底盘） |
| `c` | 底盘里程计清零 |

### 6.3 F407 固件（烧录）

```bash
cd ~/stmproject/INTRA_SCHOOL_COMPETITION_27RM
cmake --build build/Debug --target INTRA_SCHOOL_COMPETITION_27RM
openocd -f interface/cmsis-dap.cfg -f target/stm32f4x.cfg \
        -c "adapter speed 500" \
        -c "program build/Debug/INTRA_SCHOOL_COMPETITION_27RM.elf verify reset exit"
```

> pyocd 0.13.1 没装 STM32 器件包，`-t stm32f407vgtx` 会报 target 不认识，用上面的 openocd。
> CubeMX 里必须勾 **USART2 global interrupt**（否则收不到数据）。

---

## 7. 台架测试工具（不是产品代码）

`~/stmproject/INTRA_SCHOOL_COMPETITION_27RM/tools/uart_link_test.py`（Jetson 上一般在 `/tmp/`）

```bash
pkill -x gimbal            # 串口同一时刻只能一个进程占用
python3 -u /tmp/uart_link_test.py check -p /dev/ttyUSB0          # 全链路体检（约10s）
python3 -u /tmp/uart_link_test.py check --move 100 -p /dev/ttyUSB0  # 三轴方向（需架空）
python3 -u /tmp/uart_link_test.py soak --secs 30 -p /dev/ttyUSB0    # 长时间稳定性
python3 -u /tmp/uart_link_test.py wheels -p /dev/ttyUSB0            # 四轮方向（需架空）
```

> `-u` 必须带（否则输出被缓冲，看起来像卡死）。

---

## 8. 排障

### 8.1 车载 AP「假活」：状态 activated 但不发信标

**现象**：`nmcli` 显示 `car-ap:activated`、`iw dev wlan0 info` 看着正常，但笔记本扫不到 SSID。

**判断**：
```bash
cat /proc/net/dev | grep wlan0; sleep 3; cat /proc/net/dev | grep wlan0
# tx 字节不增长 => 真的没在发
```

**修复**：
```bash
sudo nmcli con down car-ap && sudo nmcli con up car-ap
```
（Intel 8265 的 AP 模式偶发失效，冷启动最彻底；见 AI-COMPUTER-LOG §9）

### 8.2 笔记本连上 AP 但 ping 不通 10.42.0.1

多半是 Meta(Clash) VPN 抢了路由：
```bash
ip route get 10.42.0.1     # 看到 dev Meta 就不对
sudo ip rule add to 10.42.0.0/24 lookup main priority 8000
```

### 8.3 串口被占用

`/dev/ttyUSB*` 同一时刻只能被一个进程打开。`gimbal` 在跑时再跑测试脚本会打不开：
```bash
pkill -x gimbal
```
⚠️ 别用 `pkill -f build/gimbal` —— 在远程 shell 里会把自己的 shell 也匹配掉。

### 8.4 两个串口的区分

| 场景 | 设备 |
|---|---|
| F407 USART2（控制链路） | `/dev/ttyUSB0`（CH340） |
| F407 USART1（调试打印） | `/dev/ttyACM0`（DAP-LINK 虚拟串口，笔记本上） |
| 云台 ZDT 电机 | 需要**第二路**串口（`ttyUSB1` 或 Jetson 40-pin `ttyTHS1`） |

### 8.4b 反馈帧全部解析失败 / 收不到帧

先确认 **F407 固件版本**与 Jetson/Python 工具一致：

| 版本 | 反馈帧长 | 特征 |
|---|---|---|
| v1（旧） | 32 B | 无四轮转速 |
| **v2（当前）** | **40 B** | 30~37 是四轮转速 |

帧长不匹配时表现为「CRC 错/同步错狂涨、一帧都解不出来」。重新烧录即可。

### 8.4c 用 openocd/SWD 调试后电机还在转

`openocd ... -c "halt"` 会把 CPU 停住，但 **PWM 由定时器硬件输出，CPU 停了照样转**，
而且此时看门狗也不工作。所以调试完必须：

```bash
openocd -f interface/cmsis-dap.cfg -f target/stm32f4x.cfg \
  -c "adapter speed 500" -c "init" -c "reset run" -c "shutdown"
```
或直接按板子上的 RESET。**别让板子停在 halt 状态去碰车。**

### 8.4d 没给指令，轮子却按「自旋模式」转（IMU 外环积分残留）

**现象**：发送的 vx/vy/wz 全是 0，但轮速呈现 `FL −, FR −, BL +, BR +`（纯自旋），
正解 wz ≈ 0.1 rad/s，而**陀螺 wz ≈ 0**（车没转）。架空时尤其明显。

**根因**：陀螺外环（`chassis_gyro.c`）的积分项 `i_term` 被上一次自旋/打滑充满后没人清零：
架空时轮子转、车体不转，陀螺测不到转动 → 误差恒为 0 → 积分永远不泄 → 死锁。

**修复**（2026-10-18）：
1. `chassis_gyro.c`：`|wz_ref| < 0.02 且 |陀螺 wz| < 0.1` 时积分项按 τ=0.3 s 衰减到 0
2. 新增 `chassis_gyro_reset_correction()`：`uart_link` 在停车/急停/看门狗时调用，
   只清修正量、不动 yaw

### 8.5 反馈帧率不是整 100 Hz

`chassis_debug` 每 250 ms 往 USART1 阻塞发一行（约 8.7 ms），会把个别 10 ms 节拍推后，
表现为 ~96 Hz、偶发 20.9 ms 间隔。**不丢帧**，不影响控制；想治就把它改成 IT 发送。

---

## 9. 已实测数据（2026-10-18，真机）

| 项 | 结果 |
|---|---|
| 反馈帧率 | 96.0 Hz（受 §8.5 影响） |
| 命令接收 | 99.8 Hz |
| 反馈丢帧 | 0（20 s soak，`fb_seq` 连续） |
| CRC 错 / 同步错 | 0 / 1（开播半帧） |
| 最大接收间隔 | 20.9 ms |
| 三轴方向 | vx/vy/wz 符号全部一致 |
| 急停 | ESTOP 位亮、DRIVER_ON 清除、恢复后正常 |
| 断流 1.2 s | WATCHDOG 亮、LINK_OK 灭、速度归零、STBY 保持 |
| 全栈 | 笔记本 UDP(vx=80mm/s) → Jetson → F407 → 正解 71 mm/s |
| F407 阶跃响应 | 0→300 mm/s：**t50 = 34 ms，t90 = 64 ms**（轮速斜坡 1500 RPM/s + PID） |
| 图传 | 640x480@30，实测 ~21 fps / ~2 Mbps，HTTP multipart 正常 |

### 9.1 延迟分解（"慢半拍"从哪来）

| 环节 | 延迟 | 说明 |
|---|---|---|
| F407 轮速斜坡 + PID | ~64 ms (t90) | 1500 RPM/s 斜坡，防打滑，不建议再快 |
| Jetson 底盘指令周期 | 5 ms | `chassis_link.hpp` 的 `tx_period_s`（已从 10 ms 降到 5 ms） |
| Jetson 控制循环 | 10 ms | 100 Hz |
| UDP/网络 | 0.4 ms（有线）/ 几 ms（WiFi） | |
| **客户端平滑 tau** | **30 ms**（默认，原 50 ms） | `--tau` 可调，越小越跟手、越易抖 |
| 遥测显示 | 20 ms | 50 Hz（原 20 Hz） |

客户端里先用 `--tau 0.02` 试手感，抖了就回到 0.03~0.04。

---

## 10. 代码位置

| 内容 | 路径 |
|---|---|
| 底盘链路（Jetson） | `~/gimbal/include/gimbal/link/chassis_link.hpp` |
| UDP 协议 v2 | `~/gimbal/include/gimbal/link/control_link.hpp` |
| 主程序/接线逻辑 | `~/gimbal/src/main.cpp` |
| 笔记本客户端 | `~/gimbal_ctl/src/main.cpp` |
| F407 链路实现 | `~/stmproject/INTRA_SCHOOL_COMPETITION_27RM/UserLib/Comm/uart_link.{c,h,codec.h}` |
| F407 协议常量 | 同目录 `uart_link_config.h` |
| F407 台架工具 | 同仓库 `tools/uart_link_test.py`、`tools/codec_test.c` |
