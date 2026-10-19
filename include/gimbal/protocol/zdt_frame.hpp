// ===========================================================================
//  ZDT 帧构造器 + 流式解析器
// ===========================================================================
#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

#include "gimbal/protocol/zdt_codes.hpp"

namespace gimbal {
namespace zdt {

/// 一条完整的电机返回帧
struct Frame {
    uint8_t              addr = 0;
    uint8_t              func = 0;
    std::vector<uint8_t> data;  ///< 含末尾校验码

    bool   valid() const { return data.size() >= 2; }
    uint8_t ret()  const { return valid() ? data[0] : 0; }
    /// 去掉 返回码 与 校验码 之后的负载
    std::vector<uint8_t> payload() const {
        if (data.size() < 3) return {};
        return std::vector<uint8_t>(data.begin() + 1, data.end() - 1);
    }
};

// ---------------------------------------------------------------------------
// 帧构造（全部自动补 0x6B 校验码）
// ---------------------------------------------------------------------------

/// 使能: Addr F3 AB <00/01> <sync> 6B
std::vector<uint8_t> mkEnable(uint8_t addr, bool en, bool cache = false);

/// 速度模式: Addr F6 <00/01 CW/CCW> <rpm 2B> <acc> <sync> 6B
/// @param rpm  0..3000
/// @param ccw  true = CCW，false = CW
/// @param acc  0 = 直接启停；非 0 时 t=(256-acc)*50us 每 1 RPM
std::vector<uint8_t> mkSpeed(uint8_t addr, uint16_t rpm, bool ccw,
                             uint8_t acc, bool cache = false);

/// 位置模式（Emm 固件 §5.3.12，13 字节）
///   Addr FD <dir> <rpm 2B> <acc> <脉冲数 4B> <运动模式> <sync> 6B
/// @param pulses 脉冲数 = 角度/360 * (200 * 细分)
std::vector<uint8_t> mkPositionEmm(uint8_t addr, int32_t pulses, uint16_t rpm,
                                   uint8_t acc, MoveMode mode, bool ccw,
                                   bool cache = false);

/// 立即停止: Addr FE 98 <sync> 6B
std::vector<uint8_t> mkEStop(uint8_t addr, bool cache = false);

/// 触发多机同步运动（广播）: 00 FF 66 6B
std::vector<uint8_t> mkSyncTrigger();

/// 将当前位置角度清零: Addr 0A 6B
std::vector<uint8_t> mkZeroPos(uint8_t addr);

/// 解除堵转/过热/过流保护: Addr 0E 6B
std::vector<uint8_t> mkClearStall(uint8_t addr);

/// 读取类命令: Addr <func> 6B
std::vector<uint8_t> mkGet(uint8_t addr, Func f);

// ---------------------------------------------------------------------------
// 流式解析器
//
//  逐字节喂入，按「地址 + 功能码 -> 期望长度 + 末尾校验」切帧。
//  设计要点（联调实测教训）:
//    · 地址不匹配就丢掉一个字节重新对齐（串口上电/干扰会产生垃圾字节）
//    · 末尾必须是 0x6B，否则判为坏帧并重新对齐
//    · 只有坏帧才计数，正常丢字节不计数（避免误报）
// ---------------------------------------------------------------------------
class Parser {
public:
    void feed(const uint8_t* p, size_t n);
    void feed(const std::vector<uint8_t>& v) { feed(v.data(), v.size()); }

    /// 取出一条已解析的帧；无则返回 false
    bool pop(Frame& out);

    uint64_t goodFrames() const { return good_; }
    uint64_t badFrames()  const { return bad_; }
    size_t   pending()    const { return buf_.size(); }
    void     clear() { buf_.clear(); }

private:
    std::vector<uint8_t> buf_;
    std::deque<Frame>    q_;
    uint64_t             good_ = 0;
    uint64_t             bad_  = 0;
};

}  // namespace zdt
}  // namespace gimbal
