#include "gimbal/protocol/zdt_frame.hpp"

#include <algorithm>

namespace gimbal {
namespace zdt {

// ---------------------------------------------------------------------------
// 常量表
// ---------------------------------------------------------------------------
namespace {

void pushBE16(std::vector<uint8_t>& o, uint16_t v) {
    o.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
    o.push_back(static_cast<uint8_t>(v & 0xFF));
}

void pushBE32(std::vector<uint8_t>& o, uint32_t v) {
    o.push_back(static_cast<uint8_t>((v >> 24) & 0xFF));
    o.push_back(static_cast<uint8_t>((v >> 16) & 0xFF));
    o.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
    o.push_back(static_cast<uint8_t>(v & 0xFF));
}

}  // namespace

const char* retName(uint8_t code) {
    switch (static_cast<Ret>(code)) {
        case Ret::Ok:       return "OK";
        case Ret::Ok12:     return "OK(0x12)";
        case Ret::Ok22:     return "OK(0x22)";
        case Ret::ParamErr: return "PARAM_ERR";
        case Ret::FmtErr:   return "FORMAT_ERR";
        case Ret::Reached:  return "REACHED";
    }
    return "UNKNOWN";
}

const char* funcName(uint8_t func) {
    switch (static_cast<Func>(func)) {
        case Func::Enable:     return "Enable";
        case Func::Speed:      return "Speed";
        case Func::Position:   return "Position";
        case Func::EStop:      return "EStop";
        case Func::SyncMove:   return "SyncMove";
        case Func::MultiCmd:   return "MultiCmd";
        case Func::CalEncoder: return "CalEncoder";
        case Func::ZeroPos:    return "ZeroPos";
        case Func::ClearStall: return "ClearStall";
        case Func::GetEnc:     return "GetEnc";
        case Func::GetVbus:    return "GetVbus";
        case Func::GetPhase:   return "GetPhase";
        case Func::GetSpeed:   return "GetSpeed";
        case Func::GetPos:     return "GetPos";
        case Func::GetErr:     return "GetErr";
        case Func::GetTemp:    return "GetTemp";
        case Func::GetStatus:  return "GetStatus";
        case Func::GetConfig:  return "GetConfig";
        case Func::GetSysSt:   return "GetSysStatus";
    }
    return "?";
}

int expectedResponseLen(uint8_t func) {
    switch (static_cast<Func>(func)) {
        // 控制类：Addr + func + 返回码 + 校验
        case Func::Enable:
        case Func::Speed:
        case Func::Position:
        case Func::EStop:
        case Func::SyncMove:
        case Func::ZeroPos:
        case Func::ClearStall:  return 4;
        // 读取类（Manual §5.5）
        case Func::GetVbus:     return 5;   // Addr 24 <mV 2B> 6B
        case Func::GetPhase:    return 5;   // Addr 27 <mA 2B> 6B
        case Func::GetEnc:      return 5;   // Addr 31 <0-65535 2B> 6B
        case Func::GetTemp:     return 5;   // Addr 39 <符号> <℃> 6B
        case Func::GetSpeed:    return 6;   // Addr 35 <符号> <rpm 2B> 6B
        case Func::GetPos:      return 8;   // Addr 36 <符号> <pos 4B> 6B
        case Func::GetErr:      return 8;   // Addr 37 <符号> <err 4B> 6B
        case Func::GetStatus:   return 4;   // Addr 3A <flag> 6B
        case Func::GetConfig:   return 33;
        case Func::GetSysSt:    return 31;
        default:                return -1;
    }
}

bool isAckFunc(uint8_t func) {
    const int n = expectedResponseLen(func);
    return n == 4;
}

// ---------------------------------------------------------------------------
// 帧构造
// ---------------------------------------------------------------------------
std::vector<uint8_t> mkEnable(uint8_t addr, bool en, bool cache) {
    return {addr, static_cast<uint8_t>(Func::Enable), 0xAB,
            static_cast<uint8_t>(en ? 1 : 0),
            static_cast<uint8_t>(cache ? 1 : 0), CHECKSUM_FIXED};
}

std::vector<uint8_t> mkSpeed(uint8_t addr, uint16_t rpm, bool ccw,
                             uint8_t acc, bool cache) {
    if (rpm > RPM_MAX) rpm = RPM_MAX;
    std::vector<uint8_t> o;
    o.reserve(8);
    o.push_back(addr);
    o.push_back(static_cast<uint8_t>(Func::Speed));
    o.push_back(static_cast<uint8_t>(ccw ? 1 : 0));
    pushBE16(o, rpm);
    o.push_back(acc);
    o.push_back(static_cast<uint8_t>(cache ? 1 : 0));
    o.push_back(CHECKSUM_FIXED);
    return o;
}

std::vector<uint8_t> mkPositionEmm(uint8_t addr, int32_t pulses, uint16_t rpm,
                                   uint8_t acc, MoveMode mode, bool ccw,
                                   bool cache) {
    if (rpm > RPM_MAX) rpm = RPM_MAX;
    std::vector<uint8_t> o;
    o.reserve(13);
    o.push_back(addr);
    o.push_back(static_cast<uint8_t>(Func::Position));
    o.push_back(static_cast<uint8_t>(ccw ? 1 : 0));
    pushBE16(o, rpm);
    o.push_back(acc);
    pushBE32(o, static_cast<uint32_t>(pulses));
    o.push_back(static_cast<uint8_t>(mode));
    o.push_back(static_cast<uint8_t>(cache ? 1 : 0));
    o.push_back(CHECKSUM_FIXED);
    return o;
}

std::vector<uint8_t> mkEStop(uint8_t addr, bool cache) {
    return {addr, static_cast<uint8_t>(Func::EStop), 0x98,
            static_cast<uint8_t>(cache ? 1 : 0), CHECKSUM_FIXED};
}

std::vector<uint8_t> mkSyncTrigger() {
    return {ADDR_BROADCAST, static_cast<uint8_t>(Func::SyncMove), 0x66,
            CHECKSUM_FIXED};
}

std::vector<uint8_t> mkZeroPos(uint8_t addr) {
    return {addr, static_cast<uint8_t>(Func::ZeroPos), CHECKSUM_FIXED};
}

std::vector<uint8_t> mkClearStall(uint8_t addr) {
    return {addr, static_cast<uint8_t>(Func::ClearStall), CHECKSUM_FIXED};
}

std::vector<uint8_t> mkGet(uint8_t addr, Func f) {
    return {addr, static_cast<uint8_t>(f), CHECKSUM_FIXED};
}

// ---------------------------------------------------------------------------
// 解析器
// ---------------------------------------------------------------------------
void Parser::feed(const uint8_t* p, size_t n) {
    if (!p || n == 0) return;
    buf_.insert(buf_.end(), p, p + n);

    while (buf_.size() >= 3) {
        const int want = expectedResponseLen(buf_[1]);
        if (want <= 0) {
            // 功能码不认识：很可能帧头没对齐，丢掉一个字节重来
            buf_.erase(buf_.begin());
            continue;
        }
        if (buf_.size() < static_cast<size_t>(want)) return;  // 等更多字节

        if (buf_[static_cast<size_t>(want) - 1] != CHECKSUM_FIXED) {
            // 校验不对：坏帧，重新对齐
            ++bad_;
            buf_.erase(buf_.begin());
            continue;
        }

        Frame f;
        f.addr = buf_[0];
        f.func = buf_[1];
        // data = 返回码 ... 校验码（不含 addr/func）
        f.data.assign(buf_.begin() + 2, buf_.begin() + want);
        q_.push_back(std::move(f));
        ++good_;
        buf_.erase(buf_.begin(), buf_.begin() + want);
    }
}

bool Parser::pop(Frame& out) {
    if (q_.empty()) return false;
    out = std::move(q_.front());
    q_.pop_front();
    return true;
}

}  // namespace zdt
}  // namespace gimbal
