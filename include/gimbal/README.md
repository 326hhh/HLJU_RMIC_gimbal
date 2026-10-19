# gimbal/include/gimbal —— 头文件层（接口与线协议）

本目录是 `gimbal` 工程的**全部头文件**：对外接口、数据类型、线协议定义都在这里。
所有实现（`.cpp`）在 `../../src/` 下，一一对应。

> 读代码建议顺序：`protocol` → `transport` → `device` → `gimbal` → `link`。

## 1. 目录结构

```
include/gimbal/
├── protocol/             ZDT（张大头）闭环步进电机线协议
│   ├── zdt_codes.hpp     功能码/返回码/状态位/固件类型 等常量
│   └── zdt_frame.hpp     帧结构 Frame + 帧构造器 mk* + 流式解析器 Parser
├── transport/            与总线无关的收发层（换总线只换这里）
│   ├── itransport.hpp    ★ 传输层抽象接口（发送节流队列 / 收帧路由 / 统计）
│   ├── can_transport.hpp  SocketCAN 实现（29-bit 扩展帧拆包重组，未上机验证）
│   └── fake_transport.hpp  纯软件模拟电机（--selftest 用）
├── device/
│   └── zdt_motor.hpp     单台 ZDT 电机：使能/速度/位置/急停/遥测轮询
├── gimbal/
│   ├── axis.hpp          单轴控制律：斜坡 + 死区 + 软限位 + 回中 + 角度估计
│   └── gimbal.hpp        双轴协调：统一时钟 + 两级看门狗 + 传输层 poll 去重
└── link/                 ── 通信链路（对上位机 / 对下位机）──
    ├── control_link.hpp   ★ UDP 线协议（PC 客户端 ↔ Jetson）
    ├── udp_control_link.hpp  UDP 收发实现（非阻塞 socket）
    ├── chassis_link.hpp   ★ 底盘链路：F407 USART2 协议（命令 12B / 反馈 40B）
    └── unified_serial.hpp ★ 单串口统一链路：底盘 + 云台共用一个 CH340（ITransport + IChassisLink）
```

## 2. 分层与依赖方向

```
                    ┌────────────────────────────────────────────┐
                    │  main.cpp（运行时接线）                     │
                    └───────┬────────────────────────────────────┘
                            │
        ┌───────────────────┼───────────────────┐
        ▼                   ▼                   ▼
   link/udp_control    link/chassis        gimbal/gimbal
   （PC 指令/遥测）    （F407 底盘）        （双轴协调）
                                                │
                                                ▼
                                         gimbal/axis
                                                │
                                                ▼
                                         device/zdt_motor
                                                │
                                                ▼
                                   transport/itransport（抽象）
                                   ├── unified_serial（真机，底盘+云台共用一个串口）
                                   ├── can_transport（预留）
                                   └── fake_transport（自测）
                                                │
                                                ▼
                                         protocol/zdt_frame
                                       （ZDT 帧构造 + 解析）
```

依赖方向**只能向下**：上层不知道底层是 UART 还是 CAN，也不碰字节偏移。

## 3. 全局约定（所有文件共同遵守）

| 项 | 约定 |
|---|---|
| 坐标（云台） | 角速度：正 = 角度增大；yaw 正 = 逆时针，pitch 正 = 抬头 |
| 坐标（底盘） | `vx` 前进为正、`vy` 左移为正、`wz` 逆时针为正（与 F407 底盘库一致） |
| 云台速度单位 | 对外 `°/s`；帧内 `0.01°/s`（i16）；电机层 RPM（`RPM = °/s × gear / 6`） |
| 底盘速度单位 | 命令帧 `vx/vy = mm/s`（i16）、`wz = 0.01 rad/s`（i16） |
| 字节序 | 所有线协议**小端**，显式按字节偏移读写，不依赖结构体布局 |
| 帧校验 | 底盘帧 = CRC-8/ATM（poly 0x07）；ZDT 帧 = 出厂固定 0x6B；UDP 帧 = XOR / u16 累加和 |

## 4. 常改文件速查

| 想改什么 | 去哪 |
|---|---|
| ZDT 命令字/返回码/状态位 | `protocol/zdt_codes.hpp` |
| ZDT 帧格式与解析规则 | `protocol/zdt_frame.hpp`（实现 `src/protocol/zdt_frame.cpp`） |
| 命令最小间隔（防粘包）、重连周期 | `link/unified_serial.hpp` 的 `Config` |
| 电机地址、细分、遥测轮询周期 | `device/zdt_motor.hpp` 的 `Config` |
| 单轴手感（斜坡/死区/限位/停手衰减） | `gimbal/axis.hpp` 的 `Config` |
| 云台看门狗、使能空档 | `gimbal/gimbal.hpp` 的 `Config` |
| UDP 端口、遥测频率 | `link/control_link.hpp`（常量）/ `link/udp_control_link.hpp`（Config） |
| 底盘帧协议 | `link/chassis_link.hpp`（与 F407 `UserLib/Comm/uart_link_config.h` 逐字节对齐） |

## 5. 头文件 ↔ 实现文件映射

| 头文件 | 实现 | 说明 |
|---|---|---|
| `protocol/zdt_codes.hpp` | 无（纯常量/声明） | `retName/funcName/expectedResponseLen/isAckFunc` 实现在 `zdt_frame.cpp` |
| `protocol/zdt_frame.hpp` | `src/protocol/zdt_frame.cpp` | 帧构造 + 解析器 |
| `transport/itransport.hpp` | `src/transport/itransport.cpp` | 队列/统计/收帧路由 |
| `link/unified_serial.hpp` | `src/link/unified_serial.cpp` | 单串口统一链路（底盘 + 云台帧分流） |
| `transport/can_transport.hpp` | `src/transport/can_transport.cpp` | SocketCAN |
| `transport/fake_transport.hpp` | 无（纯头文件内联实现） | 软件模拟 |
| `device/zdt_motor.hpp` | `src/device/zdt_motor.cpp` | 电机设备层 |
| `gimbal/axis.hpp` | `src/gimbal/axis.cpp` | 单轴控制 |
| `gimbal/gimbal.hpp` | `src/gimbal/gimbal.cpp` | 双轴协调 |
| `link/control_link.hpp` | 无（内联编解码） | UDP 线协议 |
| `link/udp_control_link.hpp` | `src/link/udp_control_link.cpp` | UDP 收发 |
| `link/chassis_link.hpp` | 无（内联编解码 + 类内实现） | 底盘协议 |

## 6. 单串口链路（已实现）

一条 CH340 挂三块 MCU（F407 + 两个 ZDT 驱动板）：底盘帧与云台帧共用**同一个串口 fd**，
实现在 `link/unified_serial.hpp`（`RxDemux` 帧分流 + 静默窗调度）。
原 `uart_transport.hpp/.cpp` 已删除；`fake_transport.hpp` 保留，只用于 `--selftest`；
`--dry` 模拟电机跑交互已移除；底盘与云台共用 `--device`，不再有 `--chassis-dev`/`--chassis-baud`。
