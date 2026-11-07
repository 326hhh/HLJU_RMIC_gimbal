// ===========================================================================
//  Icm42688I2c —— ICM-42688-P over /dev/i2c-N (i2c-dev)
//
//  只用 bank0 寄存器；数据一次性从 0x1D 起读 14 字节
//  （温度 2 + 加速度 6 + 陀螺 6），避免两次事务之间被打断。
//
//  ⚠️ 该芯片 SPI/I²C 都支持；Jetson 侧 spidev 未启用，所以走 I²C。
//     I²C 400 kHz 下一次读 ≈0.3 ms，100~200 Hz 采样完全够。
// ===========================================================================
#pragma once

#include <cstdint>
#include <cstdio>
#include <string>

#include "gimbal/imu/imu.hpp"

namespace gimbal {

class Icm42688I2c : public IImu {
public:
    struct Config {
        std::string dev = "/dev/i2c-1";   ///< Jetson 40pin I²C 是 i2c-1
        int         addr = 0x68;          ///< AP_AD0=0 -> 0x68，=1 -> 0x69
    };

    explicit Icm42688I2c(Config c) : cfg_(std::move(c)) {}
    ~Icm42688I2c() override { close(); }

    bool        open() override;
    void        close() override;
    bool        isOpen() const override { return fd_ >= 0; }
    std::string name() const override {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "icm42688:%s@0x%02X", cfg_.dev.c_str(),
                      cfg_.addr);
        return buf;
    }
    /// 实际探测到的 I²C 地址（AD0 悬空时会在 0x68/0x69 之间跳，open() 自动识别）
    int addr() const { return cfg_.addr; }

    bool read(ImuSample* out) override;

    /// 诊断：读 WHO_AM_I（ICM-42688-P 应为 0x47），失败返回 -1
    int whoAmI();
    /// 最近一次失败原因
    const std::string& lastError() const { return err_; }

private:
    bool writeReg(uint8_t reg, uint8_t val);
    bool readRegs(uint8_t reg, uint8_t* buf, size_t n);

    Config      cfg_;
    int         fd_ = -1;
    std::string err_;
    int         probe_fail_ = 0;   ///< 连续读失败计数（地址重探用）
};

}  // namespace gimbal
