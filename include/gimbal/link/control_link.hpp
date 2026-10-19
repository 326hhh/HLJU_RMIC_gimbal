// ===========================================================================
//  操作端链路（UDP）—— 线协议定义
//
//  用途: 鼠标控制的 PC 客户端 <-> Jetson 之间的低延迟控制通道。
//
//  为什么用 UDP 而不是 TCP/WebSocket：
//    · 控制包极小（16 字节 @ 100Hz ≈ 1.6 KB/s），UDP 开销最低
//    · 无队头阻塞：丢一个包直接丢，不必等重传（等重传反而更糟）
//    · 无连接状态：客户端崩溃/重连不需要握手
//  丢包容忍与时效判断交给两端看门狗（安全层在 Gimbal 里）。
//
//  ⚠️ 两端必须逐字节一致。所有字段【小端】。用显式字节偏移写编解码，
//     **不依赖 C 结构体布局/padding** —— 这是跨编译器、跨架构不出错的关键。
//
//  ── 上行: PC -> Jetson，24 字节（v1 的 16 字节仍兼容：底盘字段按 0 处理）──
//    off  size  type   field
//    0    4     u32    magic        = 0x4D454341
//    4    4     u32    seq          递增序号（两端用来算丢包）
//    8    2     i16    yaw_cdps     yaw 角速度，单位 0.01°/s
//    10   2     i16    pitch_cdps   pitch 角速度，单位 0.01°/s
//    12   1     u8     flags        云台 flags，见 ctrl_flag
//    13   1     u8     chk          前 13 字节逐字节 XOR（v1 兼容，位置不变）
//    14   2     i16    vx_mmps      底盘 vx，mm/s，+ = 前进
//    16   2     i16    vy_mmps      底盘 vy，mm/s，+ = 左移
//    18   2     i16    wz_mradps    底盘 wz，0.001 rad/s，+ = 逆时针
//    20   1     u8     chassis_flags 见 chs_flag
//    21   1     u8     chk2         14~20 字节逐字节 XOR
//    22   2     u16    pad          保留，填 0
//
//  ── 下行: Jetson -> PC，56 字节（前 32 字节与 v1 完全一致）──────────
//    off  size  type   field
//    0    4     u32    magic
//    4    4     u32    echo_seq     回显最后收到的 seq（客户端算 RTT/丢包）
//    8    2     i16    yaw_cdps     实际输出角速度 0.01°/s
//    10   2     i16    pitch_cdps
//    12   2     i16    yaw_rpm      电机实测转速
//    14   2     i16    pitch_rpm
//    16   4     i32    yaw_cdeg     输出角估计，0.01°
//    20   4     i32    pitch_cdeg
//    24   2     u16    vbus_mv      母线电压
//    26   1     u8     temp_c       驱动温度
//    27   1     u8     status       见 ctrl_status
//    28   2     u16    rx_hz        最近一秒接收频率
//    30   2     u16    chk16        前 30 字节按 u16 累加和
//    32   4     i32    cs_x_mm      底盘里程计 x（+ 前进），mm
//    36   4     i32    cs_y_mm      底盘里程计 y（+ 左移），mm
//    40   2     i16    cs_yaw_cdeg  底盘里程计 yaw，0.01°
//    42   2     i16    cs_vx_mmps   底盘正解 vx，mm/s
//    44   2     i16    cs_vy_mmps   底盘正解 vy，mm/s
//    46   2     i16    cs_wz_cs     底盘正解 wz，0.01 rad/s
//    48   2     i16    cs_gyaw_cdeg 底盘陀螺积分 yaw，0.01°
//    50   2     i16    cs_gwz_cs    底盘陀螺 wz，0.01 rad/s
//    52   1     u8     cs_status    底盘 status 低 8 位
//    53   1     u8     cs_valid     1 = 底盘反馈新鲜
//    54   2     i16    cs_wheel_fl  四轮实测转速 0.01 RPM（FL FR BL BR）
//    56   2     i16    cs_wheel_fr
//    58   2     i16    cs_wheel_bl
//    60   2     i16    cs_wheel_br
//    62   2     u16    chk16b       32~61 字节按 u16 累加和
// ===========================================================================
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace gimbal {

constexpr uint32_t CTRL_MAGIC        = 0x4D454341u;
constexpr size_t   CTRL_UP_LEN       = 24;   // v2：v1 的 16 字节前缀不变
constexpr size_t   CTRL_DOWN_LEN     = 64;   // 前 32 字节与 v1 一致
constexpr uint16_t CTRL_DEFAULT_PORT = 40001;
constexpr double   CTRL_TELEM_HZ     = 50.0;   ///< 下行遥测频率（20→50，降视觉延迟）

// ---- flags（上行）---------------------------------------------------------
namespace ctrl_flag {
constexpr uint8_t Enable     = 0x01;
constexpr uint8_t Hold       = 0x02;
constexpr uint8_t Home       = 0x04;
constexpr uint8_t Zero       = 0x08;
constexpr uint8_t EStop      = 0x10;
constexpr uint8_t ClearStall = 0x20;
constexpr uint8_t Disable    = 0x40;   ///< 显式失能（与 Enable 互斥，Disable 优先）
}  // namespace ctrl_flag

// ---- status（下行）--------------------------------------------------------
// ---- 底盘 flags（上行 offset 20）-----------------------------------------
namespace chs_flag {
constexpr uint8_t Enable  = 0x01;   ///< 允许底盘驱动（等价 F407 侧 ULINK_FLAG_EN）
constexpr uint8_t EStop   = 0x02;   ///< 底盘急停（拉低 STBY）
constexpr uint8_t Clear   = 0x04;   ///< 里程计 + 陀螺 yaw 清零（单次）
}  // namespace chs_flag

namespace ctrl_status {
constexpr uint8_t Enabled  = 0x01;
constexpr uint8_t YawOk    = 0x02;
constexpr uint8_t PitchOk  = 0x04;
constexpr uint8_t Watchdog = 0x08;
constexpr uint8_t YawLim   = 0x10;
constexpr uint8_t PitchLim = 0x20;
constexpr uint8_t Stall    = 0x40;
constexpr uint8_t EStop    = 0x80;
}  // namespace ctrl_status

// ---------------------------------------------------------------------------
// 小端读写（显式字节操作，不受主机序与对齐影响）
// ---------------------------------------------------------------------------
inline void putU16(uint8_t* p, uint16_t v) {
    p[0] = static_cast<uint8_t>(v & 0xFF);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}
inline void putU32(uint8_t* p, uint32_t v) {
    p[0] = static_cast<uint8_t>(v & 0xFF);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
    p[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
    p[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
}
inline void putI16(uint8_t* p, int16_t v) { putU16(p, static_cast<uint16_t>(v)); }
inline void putI32(uint8_t* p, int32_t v) { putU32(p, static_cast<uint32_t>(v)); }

inline uint16_t getU16(const uint8_t* p) {
    return static_cast<uint16_t>(p[0]) | static_cast<uint16_t>(p[1] << 8);
}
inline uint32_t getU32(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}
inline int16_t getI16(const uint8_t* p) { return static_cast<int16_t>(getU16(p)); }
inline int32_t getI32(const uint8_t* p) { return static_cast<int32_t>(getU32(p)); }

/// 把 double 的 °/s 夹进 i16 的 0.01°/s 范围并编码
inline int16_t clampCdps(double dps) {
    const double v = dps * 100.0;
    if (v > 32767.0) return 32767;
    if (v < -32768.0) return -32768;
    return static_cast<int16_t>(v < 0 ? v - 0.5 : v + 0.5);
}

/// 把 double 夹进 i16 的整数范围并四舍五入（底盘字段用）
inline int16_t clampI16(double v) {
    if (v > 32767.0) return 32767;
    if (v < -32768.0) return -32768;
    return static_cast<int16_t>(v < 0 ? v - 0.5 : v + 0.5);
}

// ---------------------------------------------------------------------------
// 上行：操作端指令
// ---------------------------------------------------------------------------
struct ControlCommand {
    uint32_t seq       = 0;
    double   yaw_dps   = 0.0;   ///< 已换算成 °/s
    double   pitch_dps = 0.0;
    uint8_t  flags     = 0;

    // ---- 底盘（v2 新增；坐标约定：vx 前进、vy 左移、wz 逆时针）----
    double   chassis_vx_mmps   = 0.0;
    double   chassis_vy_mmps   = 0.0;
    double   chassis_wz_mradps = 0.0;
    uint8_t  chassis_flags     = 0;

    bool enable() const { return (flags & ctrl_flag::Enable) != 0; }
    bool hold()   const { return (flags & ctrl_flag::Hold) != 0; }
    bool home()   const { return (flags & ctrl_flag::Home) != 0; }
    bool zero()   const { return (flags & ctrl_flag::Zero) != 0; }
    bool estop()  const { return (flags & ctrl_flag::EStop) != 0; }

    bool chassisEnable() const { return (chassis_flags & chs_flag::Enable) != 0; }
    bool chassisEstop()  const { return (chassis_flags & chs_flag::EStop) != 0; }
    bool chassisClear()  const { return (chassis_flags & chs_flag::Clear) != 0; }

    std::vector<uint8_t> encode() const {
        std::vector<uint8_t> b(CTRL_UP_LEN, 0);
        putU32(&b[0], CTRL_MAGIC);
        putU32(&b[4], seq);
        putI16(&b[8], clampCdps(yaw_dps));
        putI16(&b[10], clampCdps(pitch_dps));
        b[12] = flags;
        uint8_t chk = 0;
        for (int i = 0; i < 13; ++i) chk ^= b[static_cast<size_t>(i)];
        b[13] = chk;

        putI16(&b[14], clampI16(chassis_vx_mmps));
        putI16(&b[16], clampI16(chassis_vy_mmps));
        putI16(&b[18], clampI16(chassis_wz_mradps));
        b[20] = chassis_flags;
        uint8_t chk2 = 0;
        for (int i = 14; i < 21; ++i) chk2 ^= b[static_cast<size_t>(i)];
        b[21] = chk2;
        return b;
    }

    static bool decode(const uint8_t* b, size_t n, ControlCommand& out) {
        /* v1（16 B）仍能解：底盘字段按 0 处理，保证旧客户端不会一升级就断 */
        const bool v2 = (n >= CTRL_UP_LEN);
        if (!v2 && n < 16) return false;
        if (getU32(b) != CTRL_MAGIC) return false;

        uint8_t chk = 0;
        for (int i = 0; i < 13; ++i) chk ^= b[static_cast<size_t>(i)];
        if (chk != b[13]) return false;
        if (v2) {
            uint8_t chk2 = 0;
            for (int i = 14; i < 21; ++i) chk2 ^= b[static_cast<size_t>(i)];
            if (chk2 != b[21]) return false;
        }

        out.seq       = getU32(b + 4);
        out.yaw_dps   = getI16(b + 8) / 100.0;
        out.pitch_dps = getI16(b + 10) / 100.0;
        out.flags     = b[12];
        if (v2) {
            out.chassis_vx_mmps   = getI16(b + 14);
            out.chassis_vy_mmps   = getI16(b + 16);
            out.chassis_wz_mradps = getI16(b + 18);
            out.chassis_flags     = b[20];
        } else {
            out.chassis_vx_mmps   = 0.0;
            out.chassis_vy_mmps   = 0.0;
            out.chassis_wz_mradps = 0.0;
            out.chassis_flags     = 0;
        }
        return true;
    }
};

// ---------------------------------------------------------------------------
// 下行：云台遥测
// ---------------------------------------------------------------------------
struct ControlTelemetry {
    uint32_t echo_seq  = 0;
    double   yaw_dps   = 0.0;
    double   pitch_dps = 0.0;
    double   yaw_rpm   = 0.0;
    double   pitch_rpm = 0.0;
    double   yaw_deg   = 0.0;
    double   pitch_deg = 0.0;
    int      vbus_mv   = 0;
    int      temp_c    = 0;
    uint8_t  status    = 0;
    int      rx_hz     = 0;

    // ---- 底盘（v2 新增；坐标约定：vx 前进、vy 左移、wz 逆时针）----
    double   chassis_x_mm      = 0.0;
    double   chassis_y_mm      = 0.0;
    double   chassis_yaw_deg   = 0.0;
    double   chassis_vx_mmps   = 0.0;
    double   chassis_vy_mmps   = 0.0;
    double   chassis_wz_mradps = 0.0;
    double   chassis_gyro_yaw_deg  = 0.0;
    double   chassis_gyro_wz_mradps = 0.0;
    uint8_t  chassis_status    = 0;     ///< 底盘 status 低 8 位
    bool     chassis_valid     = false;
    double   chassis_wheel_rpm[4] = {0, 0, 0, 0};  ///< 四轮实测转速 FL FR BL BR

    std::vector<uint8_t> encode() const {
        std::vector<uint8_t> b(CTRL_DOWN_LEN, 0);
        putU32(&b[0], CTRL_MAGIC);
        putU32(&b[4], echo_seq);
        putI16(&b[8], clampCdps(yaw_dps));
        putI16(&b[10], clampCdps(pitch_dps));
        putI16(&b[12], clampCdps(yaw_rpm));      // 单位 0.01 RPM
        putI16(&b[14], clampCdps(pitch_rpm));
        putI32(&b[16], static_cast<int32_t>(yaw_deg * 100.0));
        putI32(&b[20], static_cast<int32_t>(pitch_deg * 100.0));
        putU16(&b[24], static_cast<uint16_t>(vbus_mv < 0 ? 0 : vbus_mv));
        b[26] = static_cast<uint8_t>(temp_c & 0xFF);
        b[27] = status;
        putU16(&b[28], static_cast<uint16_t>(rx_hz < 0 ? 0 : rx_hz));
        uint16_t sum = 0;
        for (int i = 0; i < 30; i += 2) sum = static_cast<uint16_t>(sum + getU16(&b[static_cast<size_t>(i)]));
        putU16(&b[30], sum);

        // ---- 底盘扩展段 ----
        putI32(&b[32], static_cast<int32_t>(chassis_x_mm));
        putI32(&b[36], static_cast<int32_t>(chassis_y_mm));
        putI16(&b[40], clampCdps(chassis_yaw_deg));
        putI16(&b[42], clampI16(chassis_vx_mmps));
        putI16(&b[44], clampI16(chassis_vy_mmps));
        putI16(&b[46], clampI16(chassis_wz_mradps / 10.0));
        putI16(&b[48], clampCdps(chassis_gyro_yaw_deg));
        putI16(&b[50], clampI16(chassis_gyro_wz_mradps / 10.0));
        b[52] = chassis_status;
        b[53] = chassis_valid ? 1 : 0;
        for (int i = 0; i < 4; ++i) {
            putI16(&b[54 + 2 * i], clampCdps(chassis_wheel_rpm[i]));
        }
        uint16_t sum2 = 0;
        for (int i = 32; i < 62; i += 2) sum2 = static_cast<uint16_t>(sum2 + getU16(&b[static_cast<size_t>(i)]));
        putU16(&b[62], sum2);
        return b;
    }

    static bool decode(const uint8_t* b, size_t n, ControlTelemetry& out) {
        if (n < CTRL_DOWN_LEN) return false;
        if (getU32(b) != CTRL_MAGIC) return false;
        uint16_t sum = 0;
        for (int i = 0; i < 30; i += 2) sum = static_cast<uint16_t>(sum + getU16(&b[static_cast<size_t>(i)]));
        if (sum != getU16(b + 30)) return false;
        uint16_t sum2 = 0;
        for (int i = 32; i < 62; i += 2) sum2 = static_cast<uint16_t>(sum2 + getU16(&b[static_cast<size_t>(i)]));
        if (sum2 != getU16(b + 62)) return false;

        out.echo_seq  = getU32(b + 4);
        out.yaw_dps   = getI16(b + 8) / 100.0;
        out.pitch_dps = getI16(b + 10) / 100.0;
        out.yaw_rpm   = getI16(b + 12) / 100.0;   // 0.01 RPM -> RPM
        out.pitch_rpm = getI16(b + 14) / 100.0;
        out.yaw_deg   = getI32(b + 16) / 100.0;
        out.pitch_deg = getI32(b + 20) / 100.0;
        out.vbus_mv   = getU16(b + 24);
        out.temp_c    = static_cast<int8_t>(b[26]);
        out.status    = b[27];
        out.rx_hz     = getU16(b + 28);

        // ---- 底盘扩展段 ----
        out.chassis_x_mm            = getI32(b + 32);
        out.chassis_y_mm            = getI32(b + 36);
        out.chassis_yaw_deg         = getI16(b + 40) / 100.0;
        out.chassis_vx_mmps         = getI16(b + 42);
        out.chassis_vy_mmps         = getI16(b + 44);
        out.chassis_wz_mradps       = getI16(b + 46) * 10.0;
        out.chassis_gyro_yaw_deg    = getI16(b + 48) / 100.0;
        out.chassis_gyro_wz_mradps  = getI16(b + 50) * 10.0;
        out.chassis_status          = b[52];
        out.chassis_valid           = (b[53] != 0);
        for (int i = 0; i < 4; ++i) {
            out.chassis_wheel_rpm[i] = getI16(b + 54 + 2 * i) / 100.0;
        }
        return true;
    }
};

// ---------------------------------------------------------------------------
// 链路统计
// ---------------------------------------------------------------------------
struct ControlLinkStats {
    uint64_t    rx_ok    = 0;
    uint64_t    rx_bad   = 0;
    uint64_t    rx_lost  = 0;    ///< 按 seq 跳号估算
    uint64_t    tx_telem = 0;
    double      age_s    = 999.0;
    int         rx_hz    = 0;
    std::string peer;
};

}  // namespace gimbal
