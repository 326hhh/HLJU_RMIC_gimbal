// ===========================================================================
//  UART 传输层
//
//  设备:  CH340 USB-TTL (/dev/ttyUSB0) 或 Jetson 板载 /dev/ttyTHS1
//  参数:  115200 8N1，无硬件流控，raw 模式
//
//  关键设计（都来自实测教训）:
//    · 命令最小间隔 12ms —— ZDT 手册 §3.5.2 明确要求「每条命令发送完后
//      要延时几毫秒，防止命令之间粘包失效」；实测间隔 0ms 时两条命令
//      都回复的成功率只有 1/5，≥1ms 即 5/5。这里取 12ms 留足余量。
//    · 热插拔自动重连 —— USB-TTL 掉线重插是常态（实测遇到过一次），
//      打不开就每 2 秒重试，读写出错立刻丢弃句柄交给重连逻辑。
// ===========================================================================
#pragma once

#include <cstdint>
#include <string>

#include "gimbal/transport/itransport.hpp"

namespace gimbal {

class UartTransport : public ITransport {
public:
    struct Config {
        std::string device       = "/dev/ttyUSB0";
        int         baud         = 115200;
        int         tx_gap_ms    = 12;    ///< 命令最小间隔
        int         reconnect_ms = 2000;  ///< 重连重试周期
    };

    explicit UartTransport(Config c);
    ~UartTransport() override;

    bool        open()  override;
    void        close() override;
    bool        isOpen() const override;
    std::string name() const override;
    void        poll() override;

private:
    bool tryOpen();
    void drop(const std::string& why);

    Config  cfg_;
    int     fd_ = -1;
    int64_t next_reconnect_us_ = 0;
    uint8_t rbuf_[1024];
};

}  // namespace gimbal
