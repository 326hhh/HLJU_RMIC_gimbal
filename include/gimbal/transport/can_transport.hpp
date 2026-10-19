// ===========================================================================
//  CAN 传输层（ZDT 电机走 CAN 总线）
//
//  ⚠️ 状态: 骨架已按手册 §4.2 实现帧封装/拆包/重组，但**无硬件验证**
//           （KH-UCANFD 适配器当前未接入 gimbal 总线）。
//           接线与联调前请先跑 candump 抓包核对。
//
//  协议（手册 §4.2）:
//      扩展帧 ID = (Addr << 8) | Packet
//      数据      = 功能码 + 命令数据 + 校验码
//      · 固定使用【29-bit 扩展帧】——与标准帧天然隔离，不会和底盘标准帧冲突
//      · Packet 从 0 计数；命令 >8 字节需拆包，Packet 递增
//      · 校验码默认固定 0x6B
//      · CAN_Baud 默认 500000；Response 默认 Receive（每条控制命令都回 ACK）
//
//  与 UART 的差异: UART 帧首字节是 Addr，CAN 里 Addr 被搬到帧 ID 里了，
//                  所以 CAN 数据段 = UART 帧去掉第 0 字节。
// ===========================================================================
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "gimbal/transport/itransport.hpp"

namespace gimbal {

class CanTransport : public ITransport {
public:
    struct Config {
        std::string iface     = "can0";
        int         tx_gap_ms = 2;     ///< CAN 无粘包问题，间隔可很小
        bool        enable_fd = false; ///< 保持 false：ZDT 是 classic CAN
    };

    explicit CanTransport(Config c);
    ~CanTransport() override;

    bool        open()  override;
    void        close() override;
    bool        isOpen() const override;
    std::string name() const override;
    void        poll() override;

private:
    bool canSendPackets(uint8_t addr, const std::vector<uint8_t>& uartFrame);
    void onCanFrame(uint32_t canId, const uint8_t* data, uint8_t dlc);

    Config cfg_;
    int    fd_ = -1;

    /// 重组缓冲：addr -> 已收到的数据
    struct Reasm {
        uint8_t              func = 0;
        std::vector<uint8_t> buf;
    };
    Reasm reasm_;
};

}  // namespace gimbal
