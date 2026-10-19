// ===========================================================================
//  ZDT（张大头）闭环步进电机协议 —— 常量表
//
//  依据《ZDT_X42S 第二代闭环步进电机用户手册 V1.0.5_260527》
//     §4.2  CAN 通讯（扩展帧）
//     §5.3  运动控制命令
//     §5.5  读取系统参数命令
//
//  UART 帧格式（本工程当前使用）:
//      主机 -> 电机:  Addr | 功能码 | 命令数据 | 校验码
//      电机 -> 主机:  Addr | 功能码 | 返回数据 | 校验码
//      校验码出厂默认固定 0x6B（可改为 XOR / CRC-8 / Modbus / DMX512）
//
//  CAN 帧格式（未来切换，见 can_transport）:
//      扩展帧 ID = (Addr << 8) | Packet
//      数据      = 功能码 + 命令数据 + 校验码
//      —— 固定使用 29-bit 扩展帧，与标准帧天然隔离
// ===========================================================================
#pragma once

#include <cstdint>

namespace gimbal {
namespace zdt {

// ---- 协议常量 -------------------------------------------------------------
constexpr uint8_t  CHECKSUM_FIXED = 0x6B;   // 出厂默认校验字节
constexpr uint16_t RPM_MAX        = 3000;   // §5.3.7 速度上限 0000-0BB8
constexpr uint8_t  ACC_MAX        = 255;    // 加速度档位 00-FF
constexpr uint8_t  ADDR_BROADCAST = 0x00;
constexpr int      RESP_LEN_MAX   = 64;     // 本工程用到的最大返回帧（0x42 = 33 字节）

// ---- 功能码 ---------------------------------------------------------------
enum class Func : uint8_t {
    // 运动控制
    Enable     = 0xF3,   // 使能: Addr F3 AB <00/01> <sync> 6B
    Speed      = 0xF6,   // 速度模式(Emm/X): Addr F6 <dir> <rpm2> <acc> <sync> 6B
    Position   = 0xFD,   // 位置模式: 见 zdt_frame.hpp 的 mkPositionEmm()
    EStop      = 0xFE,   // 立即停止: Addr FE 98 <sync> 6B
    SyncMove   = 0xFF,   // 触发多机同步: Addr FF 66 6B
    MultiCmd   = 0xAA,   // 多电机命令（X42S/Y42，一条广播发多轴）
    // 触发动作
    CalEncoder = 0x06,   // 触发编码器校准
    ZeroPos    = 0x0A,   // 将当前位置角度清零
    ClearStall = 0x0E,   // 解除堵转/过热/过流保护
    // 读取
    GetEnc     = 0x31,   // 线性化校准后的编码器值 (0-65535)
    GetVbus    = 0x24,   // 母线电压 mV
    GetPhase   = 0x27,   // 相电流 mA
    GetSpeed   = 0x35,   // 实时转速
    GetPos     = 0x36,   // 实时位置
    GetErr     = 0x37,   // 位置误差
    GetTemp    = 0x39,   // 驱动温度 ℃
    GetStatus  = 0x3A,   // 状态标志位
    GetConfig  = 0x42,   // 驱动参数（33 字节，含细分值）
    GetSysSt   = 0x43,   // 系统状态参数（31 字节）
};

// ---- 电机返回码（§4.2.2）-------------------------------------------------
enum class Ret : uint8_t {
    Ok       = 0x02,   // 命令接收正确
    Ok12     = 0x12,   // 变体正确码
    Ok22     = 0x22,   // 变体正确码
    ParamErr = 0xE2,   // 参数错误（范围/条件不满足，如低压警告、堵转保护）
    FmtErr   = 0xEE,   // 命令格式错误
    Reached  = 0x9F,   // 动作执行完成（主动返回：到位/回零完成）
};

// ---- 位置模式运动模式（Emm §5.3.12）--------------------------------------
enum class MoveMode : uint8_t {
    RelativeToLastTarget = 0x00,
    AbsoluteFromZero     = 0x01,
    RelativeToCurrent    = 0x02,
};

// ---- 固件类型 -------------------------------------------------------------
//  Emm5.0: 位置模式用【脉冲数】，转速单位 RPM
//  X:      位置模式用【0.1°】角度，转速单位 0.1RPM
//  两者位置模式帧长不同，且手册明确 Emm 刚性强、适合小车；本工程默认 Emm。
enum class Firmware : uint8_t { Emm = 0, X = 1 };

// ---- 状态标志位（§5.5.15，0x3A 返回值）----------------------------------
namespace st {
constexpr uint8_t Enabled   = 0x01;   // bit0 使能
constexpr uint8_t InPos     = 0x02;   // bit1 位置到达
constexpr uint8_t Stalled   = 0x04;   // bit2 堵转
constexpr uint8_t StallProt = 0x08;   // bit3 堵转保护触发
constexpr uint8_t LimLeft   = 0x10;   // bit4 左限位输入为高
constexpr uint8_t LimRight  = 0x20;   // bit5 右限位输入为高
constexpr uint8_t PowerLoss = 0x80;   // bit7 掉电标志
}  // namespace st

// ---- 辅助函数 -------------------------------------------------------------
const char* retName(uint8_t code);
const char* funcName(uint8_t func);

/// 该功能码的返回帧总长度；未知功能码返回 -1
int expectedResponseLen(uint8_t func);

/// 是否为控制类命令（回帧只有 返回码，长度 4）
bool isAckFunc(uint8_t func);

}  // namespace zdt
}  // namespace gimbal
