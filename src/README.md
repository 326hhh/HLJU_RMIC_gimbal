# gimbal/src —— 实现层（全部 .cpp）

本目录是 `gimbal` 工程的全部实现，与 `../include/gimbal/` 一一对应。
另有一个独立的图传程序 `video/mjpeg_server.cpp`。

## 1. 目录结构

```
src/
├── main.cpp                 程序入口：参数解析 / 100Hz 主循环 / 本地 stdin / 自检 / 底盘转发
├── protocol/
│   └── zdt_frame.cpp        帧构造与流式解析实现（校验 0x6B）
├── transport/
│   ├── itransport.cpp       发送节流队列 / 收帧路由 / 统计
│   └── can_transport.cpp    SocketCAN 收发 + 扩展帧拆包重组（未上机验证）
├── device/
│   └── zdt_motor.cpp        物理量 ↔ ZDT 帧、遥测解析、传动比/脉冲换算
├── gimbal/
│   ├── axis.cpp             单轴控制律（斜坡/死区/限位/回中）
│   └── gimbal.cpp           双轴调度、看门狗、estop/home/enable 协调
├── link/
│   ├── udp_control_link.cpp UDP 收包解码、按频率回发遥测、RTT/丢包统计
│   └── unified_serial.cpp   单串口统一链路：底盘帧 + ZDT 帧共用一个 fd（RxDemux 分流）
└── video/
    └── mjpeg_server.cpp     独立程序：V4L2 取 MJPG 帧 → HTTP multipart 转发（零转码）
```

## 2. 编译与产物

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
```

| 产物 | 来源 | 说明 |
|---|---|---|
| `build/libgimbal_core.a` | protocol/transport/device/gimbal/link 下全部 .cpp | 静态库 |
| `build/gimbal` | `main.cpp` + gimbal_core | 主程序（云台 + 底盘转发 + UDP 控制） |
| `build/mjpeg_server` | `video/mjpeg_server.cpp` | 图传程序（独立进程） |

`ctest` 只有一项：`gimbal --selftest`（纯软件自检，不需要任何硬件）。

## 3. `main.cpp` —— 程序入口

### 3.1 `struct Options`（命令行参数默认值）

| 字段 | 默认 | 含义 |
|---|---|---|
| `device` / `baud` | `/dev/ttyUSB0` / 115200 | 单串口总线（底盘 + 云台共用一个 CH340） |
| `yaw_addr` / `pitch_addr` | 2 / 1 | 电机地址 |
| `selftest` | false | `--selftest`：自检后退出 |
| `loop_hz` | 100 | 控制循环频率 |
| `gear` | 2.0 | 传动比（电机转数/输出转数） |
| `pulses_per_motor_rev` | 3200 | 脉冲/电机圈 = 200×细分 |
| `yaw_min/max/rate`, `yaw_no_limit` | ±180 / 180 / 无限位 | yaw 参数（导电滑环默认无限位） |
| `pitch_min/max/rate` | ±45 / 120 | pitch 参数 |
| `acc` | 200 | 加速度档位 |
| `ctrl_port` | 40001 | UDP 控制端口 |
| `no_udp` | false | `--no-udp`：只用本地 stdin |
| `no_chassis` | false | `--no-chassis`：只控云台 |

### 3.2 函数一览

| 函数 | 行为 |
|---|---|
| `usage(argv[0])` | 打印帮助（所有命令行参数） |
| `CHECK(cond, msg)`（宏） | 自检断言：打印 `[ OK ]/[FAIL]`，失败计数加一 |
| `hex(vector)` | 字节流 → 十六进制字符串（自检比对用） |
| `runSelftest()` | 纯软件自检，共 6 段（见 3.3） |
| `printStatus(g, bus, link, chassis)` | 打印云台两轴状态、电机总线统计、操作端统计、底盘状态 |
| `buildTelemetry(g, echo_seq, rx_hz, chassis)` | 组装 `ControlTelemetry`（云台 + 底盘字段） |
| `runMain(o)` | 主流程：打开总线 → 建电机/云台/底盘/UDP → 主循环 → 退出清理 |
| `main(argc, argv)` | 解析参数，`--selftest` 走自检，否则 `runMain` |

### 3.3 `runSelftest()` 的 7 段

| 段 | 验证内容 |
|---|---|
| [1] 帧构造 | `mkEnable/mkSpeed/mkEStop/mkGet/mkPositionEmm` 逐字节比对期望 hex |
| [2] 解析器 | 混入垃圾字节后仍能正确切帧 + 校验 |
| [3] 角度↔脉冲 | 360°→3200、90°→800（用 FakeTransport 构造 ZdtMotor） |
| [4] UDP 线协议 | 上行 24B / 下行 64B 编解码往返、坏包/短包被拒 |
| [5] 端到端 | 一总线双电机 + 传动比 2:1：地址路由、斜坡、使能、遥测、看门狗（触发但**不失能**）、软限位、急停后失能 |
| [6] 底盘协议 | 命令帧编码向量、反馈帧解码向量、单字节破坏被拒、急停帧 cmd=0x02 且不带 EN |
| [7] 单总线帧分流 | `RxDemux` 半包/逐字节喂入、底盘帧 + ZDT 帧交错、垃圾字节、无坏帧 |

### 3.4 `runMain()` 主循环

```c
while (g_run) {
    1. 读 stdin（非阻塞，一行一条本地命令：y/p/r/h/home/zero/en/x/s/c/cx/q）
    2. link->poll();  popCommand() 循环：
         - estop → gimbal.estop()
         - Disable/Enable → gimbal.enable()
         - zero/home/hold → 对应动作
         - 否则 setRate(yaw, pitch)（远端接管本地设定）
         - 底盘字段 → chassis->setCommand(sc)
    3. local_has → gimbal.setRate()（本地优先）
    3b. 底盘仲裁：本地急停/本地命令 > UDP（0.25s 内） > EN=0 停车
    4. gimbal.step(dt_target)
    5. link->setTelemetry(buildTelemetry(...))
    6. 定频：按 dt_target 对齐下一拍
}
```

**退出流程**：`gimbal.hold()` + 30 拍减速（**保持使能抱死**）→ 底盘发 0.1s EN=0 停车帧 → 关串口。

**底盘仲裁优先级**（3b）：本地 `c` 命令 > 本地 `cx` 急停 > 250ms 内的 UDP 底盘指令 > 默认
`enable=false`（F407 侧 100ms 看门狗后斜坡停车，Jetson 这里主动发 EN=0 只是多一层保险）。

## 4. 子目录文档

| 目录 | 文档 | 内容 |
|---|---|---|
| `protocol/` | [`protocol/README.md`](protocol/README.md) | 帧构造与解析器实现 |
| `transport/` | [`transport/README.md`](transport/README.md) | 队列/串口/CAN 实现 |
| `device/` | [`device/README.md`](device/README.md) | 电机设备实现 |
| `gimbal/` | [`gimbal/README.md`](gimbal/README.md) | 单轴/双轴控制实现 |
| `link/` | [`link/README.md`](link/README.md) | UDP 链路实现 |
| `video/` | [`video/README.md`](video/README.md) | MJPEG 图传实现 |
