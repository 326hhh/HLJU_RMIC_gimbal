// ===========================================================================
//  底盘链路：USART2 二进制协议（Jetson <-> STM32F407）
//
//  与下位机 UserLib/Comm/uart_link_config.h **逐字节一致**：
//    命令帧（Jetson -> F407）12 B
//    反馈帧（F407 -> Jetson）40 B（v2：末尾含四轮实测转速）
//    CRC-8/ATM(poly=0x07, init=0x00)，100 Hz 双向
//
//  ⚠️ 坐标约定跟随底盘库（docs/CHASSIS_LIBRARY.md §4）：
//      +x = 车体前进，+y = 车体左移，wz 逆时针为正。
//      **不是**旧 ChassisCommand 草案里的「vx 右 / vy 前」，那个已经废弃。
//
//  本文件只放：帧常量 / 数据结构 / 编解码纯函数 / IChassisLink 抽象 / 统计。
//  UART 收发送在 `link/unified_serial.hpp`（与云台 ZDT 帧共用一条串口，
//  一条 CH340 挂三块 MCU 时的单口链路）。
//
//  命令帧（12 B，小端）
//  ┌──────┬────┬─────┬─────┬─────┬──────┬──────┬────────┬───────┬──────┐
//  │ off  │ 0  │  1  │  2  │  3  │ 4..5 │ 6..7 │  8..9  │  10   │  11  │
//  │ 字段 │0xA5│0x5A │ cmd │ seq │  vx  │  vy  │  wz    │ flags │ crc8 │
//  │ 类型 │ u8 │ u8  │ u8  │ u8  │ i16  │ i16  │  i16   │  u8   │  u8  │
//  └──────┴────┴─────┴─────┴─────┴──────┴──────┴────────┴───────┴──────┘
//    vx/vy : mm/s，wz : 0.01 rad/s
//
//  反馈帧（40 B，小端）
//    0..1 0x5A 0xA5 | 2 类型0x81 | 3 seq回显
//    4..7  i32 x_mm     8..11 i32 y_mm     12..13 i16 yaw_cdeg
//    14..15 i16 vx_mm   16..17 i16 vy_mm   18..19 i16 wz_cs
//    20..21 i16 gyro_yaw_cdeg   22..23 i16 gyro_wz_cs
//    24..25 u16 status  26..27 u16 cmd_cnt  28..29 u16 fb_seq
//    30..37 i16 四轮实测转速（0.01 RPM）：FL FR BL BR
//    38 crc8（覆盖 0..37）  39 0x0D
// ===========================================================================
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>

#include "gimbal/link/control_link.hpp"   // 复用 putI16/getI32 等小端读写

namespace gimbal {

// ---- 帧常量（与 F407 侧 uart_link_config.h 对齐）--------------------------
constexpr uint8_t  CHASSIS_CMD_HEAD0 = 0xA5;
constexpr uint8_t  CHASSIS_CMD_HEAD1 = 0x5A;
constexpr uint8_t  CHASSIS_FB_HEAD0  = 0x5A;
constexpr uint8_t  CHASSIS_FB_HEAD1  = 0xA5;
constexpr uint8_t  CHASSIS_FB_TYPE   = 0x81;
constexpr uint8_t  CHASSIS_FB_TAIL   = 0x0D;
constexpr size_t   CHASSIS_CMD_LEN   = 12;
constexpr size_t   CHASSIS_FB_LEN    = 40;   // v2：30~37 为四轮实测转速

// ---- 命令字 --------------------------------------------------------------
namespace chassis_cmd {
constexpr uint8_t Speed = 0x01;
constexpr uint8_t EStop = 0x02;
constexpr uint8_t Spin  = 0x03;
constexpr uint8_t Clear = 0x04;
}  // namespace chassis_cmd

namespace chassis_flag {
constexpr uint8_t Enable = 0x01;
}  // namespace chassis_flag

// ---- 反馈 status 位（与 ULINK_ST_* 对齐）---------------------------------
namespace chassis_status {
constexpr uint16_t ImuOnline = 0x0001;
constexpr uint16_t LinkOk    = 0x0002;
constexpr uint16_t Watchdog  = 0x0004;
constexpr uint16_t EStop     = 0x0008;
constexpr uint16_t DriverOn  = 0x0010;
constexpr uint16_t Spin      = 0x0020;
constexpr uint16_t GyroOn    = 0x0040;
constexpr uint16_t ImuCal    = 0x0080;
}  // namespace chassis_status

// ---- CRC-8/ATM（校验值 "123456789" = 0xF4）-------------------------------
inline uint8_t chassisCrc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0x00;
    while (len-- > 0) {
        crc ^= *data++;
        for (int i = 0; i < 8; ++i)
            crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x07)
                               : static_cast<uint8_t>(crc << 1);
    }
    return crc;
}

// ---------------------------------------------------------------------------
// 底盘指令 / 状态
//
// ⚠️ 坐标约定：vx 前进为正、vy 左移为正、wz 逆时针为正（底盘库 §4）
// ---------------------------------------------------------------------------
struct ChassisCommand {
    double  vx_mmps    = 0.0;   ///< mm/s，+ = 前进
    double  vy_mmps    = 0.0;   ///< mm/s，+ = 左移
    double  wz_mradps  = 0.0;   ///< 0.001 rad/s，+ = 逆时针
    bool    enable     = false;
    bool    estop      = false;
    bool    clear_odom = false;
    uint8_t seq        = 0;
};

struct ChassisState {
    bool     valid          = false;
    double   vx_mmps        = 0.0;   ///< 正解速度（前进为正）
    double   vy_mmps        = 0.0;   ///< 左移为正
    double   wz_mradps      = 0.0;   ///< 正解角速度 0.001 rad/s
    double   odom_x_mm      = 0.0;
    double   odom_y_mm      = 0.0;
    double   odom_yaw_deg   = 0.0;
    double   gyro_yaw_deg   = 0.0;
    double   gyro_wz_mradps = 0.0;
    uint16_t status         = 0;
    uint16_t cmd_cnt        = 0;
    uint16_t fb_seq         = 0;
    double   wheel_rpm[4]   = {0, 0, 0, 0};  ///< 四轮实测转速 FL FR BL BR（RPM）
    uint8_t  last_seq       = 0;     ///< 回显的命令 seq
    double   age_s          = 999.0; ///< 距上一帧反馈的时间
};

// ---------------------------------------------------------------------------
// 编解码（与 F407 逐字节一致，selftest 里用 Python 生成的向量锁死）
// ---------------------------------------------------------------------------
inline void chassisEncodeCmd(uint8_t out[CHASSIS_CMD_LEN], const ChassisCommand& c) {
    out[0] = CHASSIS_CMD_HEAD0;
    out[1] = CHASSIS_CMD_HEAD1;
    out[2] = c.estop ? chassis_cmd::EStop
                     : (c.clear_odom ? chassis_cmd::Clear : chassis_cmd::Speed);
    out[3] = c.seq;
    putI16(out + 4, clampI16(c.vx_mmps));
    putI16(out + 6, clampI16(c.vy_mmps));
    putI16(out + 8, clampI16(c.wz_mradps / 10.0));   // 0.001 rad/s -> 0.01 rad/s
    out[10] = c.enable ? chassis_flag::Enable : 0;
    out[11] = chassisCrc8(out, CHASSIS_CMD_LEN - 1);
}

inline bool chassisDecodeFb(const uint8_t* in, ChassisState& st) {
    if (in[0] != CHASSIS_FB_HEAD0 || in[1] != CHASSIS_FB_HEAD1 ||
        in[2] != CHASSIS_FB_TYPE || in[CHASSIS_FB_LEN - 1] != CHASSIS_FB_TAIL)
        return false;
    if (chassisCrc8(in, CHASSIS_FB_LEN - 2) != in[CHASSIS_FB_LEN - 2]) return false;

    st.last_seq        = in[3];
    st.odom_x_mm       = getI32(in + 4);
    st.odom_y_mm       = getI32(in + 8);
    st.odom_yaw_deg    = getI16(in + 12) / 100.0;
    st.vx_mmps         = getI16(in + 14);
    st.vy_mmps         = getI16(in + 16);
    st.wz_mradps       = getI16(in + 18) * 10.0;
    st.gyro_yaw_deg    = getI16(in + 20) / 100.0;
    st.gyro_wz_mradps  = getI16(in + 22) * 10.0;
    st.status          = getU16(in + 24);
    st.cmd_cnt         = getU16(in + 26);
    st.fb_seq          = getU16(in + 28);
    for (int i = 0; i < 4; ++i) {
        st.wheel_rpm[i] = getI16(in + 30 + 2 * i) / 100.0;
    }
    st.valid           = true;
    return true;
}

// ---------------------------------------------------------------------------
// 链路抽象
// ---------------------------------------------------------------------------
class IChassisLink {
public:
    virtual ~IChassisLink() = default;

    virtual bool        open()  = 0;
    virtual void        close() = 0;
    virtual bool        isOpen() const = 0;
    virtual std::string name()  const = 0;

    virtual void               setCommand(const ChassisCommand& c) = 0;
    virtual const ChassisState& state() const = 0;
    virtual void               poll() = 0;
};

struct ChassisLinkStats {
    uint64_t    tx_frames  = 0;
    uint64_t    rx_frames  = 0;
    uint64_t    tx_dropped = 0;
    uint64_t    crc_err    = 0;
    uint64_t    sync_err   = 0;
    uint64_t    reconnects = 0;
    double      age_s      = 999.0;
    std::string last_error;
};

}  // namespace gimbal
