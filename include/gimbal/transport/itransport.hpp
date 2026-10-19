// ===========================================================================
//  传输层抽象
//
//  设计目标：让上层（ZdtMotor / Axis / Gimbal）完全不关心底下是
//            UART 还是 CAN —— 换总线只换一个实现类。
//
//    UartTransport : 当前使用（CH340 / ttyTHS1，115200 8N1）
//    CanTransport  : 骨架，等 KH-UCANFD 硬件到货后填（扩展帧 (Addr<<8)|Packet）
//    FakeTransport : 纯软件模拟电机，用于 --dry 自测（不需要任何硬件）
//
//  公共职责（在基类实现，派生类只需 open/close/writeRaw/readRaw）：
//    · 发送节流队列（ZDT 要求命令间隔 ≥1ms，实测 0ms 时成功率 1/5，本工程用 12ms）
//    · 接收字节 -> Parser -> 按地址可路由的帧队列
//    · 统计计数
// ===========================================================================
#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "gimbal/protocol/zdt_frame.hpp"

namespace gimbal {

struct TransportStats {
    uint64_t tx_frames   = 0;  ///< 成功发出的帧
    uint64_t tx_dropped  = 0;  ///< 队列满被丢弃的帧
    uint64_t rx_frames   = 0;  ///< 解析出的完整帧
    uint64_t rx_bytes    = 0;
    uint64_t reconnects  = 0;  ///< 自动重连次数
    bool     connected   = false;
    std::string last_error;
};

class ITransport {
public:
    virtual ~ITransport() = default;

    // ---- 派生类必须实现 ----
    virtual bool        open()  = 0;
    virtual void        close() = 0;
    virtual bool        isOpen() const = 0;
    virtual std::string name()  const = 0;

    /// 在控制循环里按固定频率调用：负责重连、节流发送、收包解析
    virtual void poll() = 0;

    // ---- 基类提供的公共能力 ----
    /// 入队一帧。urgent=true 时清空队列插队（急停用）
    void send(const std::vector<uint8_t>& frame, bool urgent = false);

    /// 取任意一帧
    bool popFrame(zdt::Frame& out);
    /// 只取发给指定地址的帧（一总线多电机时用）
    bool popFrameFor(uint8_t addr, zdt::Frame& out);

    const TransportStats& stats() const { return stats_; }
    size_t queueLen() const { return txq_.size(); }

protected:
    // ---- 派生类在 poll() 里调用 ----
    /// 把收到的原始字节喂给解析器
    void feedRx(const uint8_t* p, size_t n);
    /// 从发送队列取一帧待发（受最小间隔约束）；无则返回 false
    bool nextTx(std::vector<uint8_t>& out);
    /// 发送完成后记账
    void onTxDone();
    /// 发送失败（派生类据此触发重连）
    void onTxFailed(const std::string& why);

    /// 是否已到可以发送的时刻（子类也可自行判断）
    bool txReady() const;

    void setGapUs(int64_t us) { gap_us_ = us; }

    std::deque<std::vector<uint8_t>> txq_;
    zdt::Parser                      parser_;
    std::deque<zdt::Frame>           rxq_;
    TransportStats                   stats_;

    size_t  txq_max_ = 64;
    int64_t gap_us_  = 12000;   ///< 命令最小间隔（ZDT 防粘包）
    int64_t last_tx_us_ = 0;
};

}  // namespace gimbal
