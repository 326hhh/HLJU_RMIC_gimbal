#include "gimbal/imu/imu_i2c.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>   // i2c_msg / I2C_M_RD 在这里，i2c-dev.h 只在部分发行版带
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <thread>

namespace gimbal {

namespace {

// ---- ICM-42688-P 寄存器（bank 0）------------------------------------------
constexpr uint8_t kRegDeviceConfig = 0x11;  ///< bit0 = 软复位
constexpr uint8_t kRegDataStart    = 0x1D;  ///< TEMP_DATA1 … GYRO_DATA_Z0（14 B）
constexpr uint8_t kRegPwrMgmt0     = 0x4E;
constexpr uint8_t kRegGyroConfig0  = 0x4F;
constexpr uint8_t kRegAccelConfig0 = 0x50;
constexpr uint8_t kRegWhoAmI       = 0x75;
constexpr uint8_t kRegBankSel      = 0x76;

constexpr uint8_t kWhoAmICm42688 = 0x47;

// 量程与换算：±8 g -> 4096 LSB/g；±2000 °/s -> 16.4 LSB/(°/s)
constexpr double kAccelLsbPerG  = 4096.0;
constexpr double kGyroLsbPerDps = 16.4;

double nowS() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

}  // namespace

bool Icm42688I2c::open() {
    close();
    fd_ = ::open(cfg_.dev.c_str(), O_RDWR);
    if (fd_ < 0) {
        err_ = std::string("open ") + cfg_.dev + ": " + std::strerror(errno);
        return false;
    }

    // 地址自动识别：模块的 AD0/AP_AD0 若悬空，地址会在 0x68 与 0x69 之间跳。
    // 先试传入的地址，再试另一个；找到后把实际地址写回 cfg_，后续读写都用它。
    int cand[3] = {cfg_.addr, 0x68, 0x69};
    int found = -1;
    for (int k = 0; k < 3; ++k) {
        cfg_.addr = cand[k];
        if (whoAmI() == kWhoAmICm42688) {
            found = cand[k];
            break;
        }
    }
    if (found < 0) {
        err_ = "WHO_AM_I != 0x47（检查接线/地址：SCL→pin5, SDA→pin3, 3V3→pin1；"
               "AD0 悬空会导致地址在 0x68/0x69 间跳）";
        ::close(fd_);
        fd_ = -1;
        return false;
    }
    cfg_.addr = found;

    // ⚠️ 不做软复位：ICM-42688-P 复位后会对 I²C 请求 NAK 一段时间（实测复位后
    //    紧接写 PWR_MGMT0 直接 Remote I/O error，还会把总线挂住，后续读全失败）。
    //    上电默认寄存器是确定的，直接把要用的配好 + 回读校验即可。
    if (!writeReg(kRegBankSel, 0x00)) return false;

    // 加速度 + 陀螺都进低噪声模式
    if (!writeReg(kRegPwrMgmt0, 0x0F)) return false;
    // 陀螺 ±2000 °/s @ 200 Hz；加速度 ±8 g @ 200 Hz
    if (!writeReg(kRegGyroConfig0, 0x00 | 0x04)) return false;   ///< FS=±2000, ODR=200Hz
    if (!writeReg(kRegAccelConfig0, 0x20 | 0x04)) return false;  ///< FS=±8g,   ODR=200Hz
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    // 回读校验：没配上就明确报错，别让上层拿到"看起来正常"的假数据
    uint8_t rb = 0;
    if (!readRegs(kRegPwrMgmt0, &rb, 1) || rb != 0x0F) {
        err_ = "PWR_MGMT0 回读不符（传感器未进入低噪声模式）";
        return false;
    }
    if (!readRegs(kRegAccelConfig0, &rb, 1) || (rb & 0xE0) != 0x20) {
        err_ = "ACCEL_CONFIG0 回读不符（量程没设成 ±8g）";
        return false;
    }
    return true;
}

void Icm42688I2c::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

int Icm42688I2c::whoAmI() {
    uint8_t v = 0;
    if (!readRegs(kRegWhoAmI, &v, 1)) return -1;
    return v;
}

bool Icm42688I2c::writeReg(uint8_t reg, uint8_t val) {
    if (fd_ < 0) return false;
    uint8_t buf[2] = {reg, val};
    // 长线 I2C 偶发 NAK，重试几次（写配置是幂等的，重试安全）
    for (int attempt = 0; attempt < 3; ++attempt) {
        struct i2c_msg msg{};
        msg.addr  = static_cast<uint16_t>(cfg_.addr);
        msg.flags = 0;
        msg.len   = 2;
        msg.buf   = buf;
        struct i2c_rdwr_ioctl_data io{};
        io.msgs   = &msg;
        io.nmsgs  = 1;
        if (::ioctl(fd_, I2C_RDWR, &io) >= 0) return true;
        err_ = std::string("i2c write 0x") + std::to_string(reg) + ": " +
               std::strerror(errno);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return false;
}

bool Icm42688I2c::readRegs(uint8_t reg, uint8_t* buf, size_t n) {
    if (fd_ < 0) return false;
    uint8_t          regbuf[1] = {reg};
    struct i2c_msg   msgs[2]{};
    msgs[0].addr  = static_cast<uint16_t>(cfg_.addr);
    msgs[0].flags = 0;
    msgs[0].len   = 1;
    msgs[0].buf   = regbuf;
    msgs[1].addr  = static_cast<uint16_t>(cfg_.addr);
    msgs[1].flags = I2C_M_RD;
    msgs[1].len   = static_cast<uint16_t>(n);
    msgs[1].buf   = buf;
    for (int attempt = 0; attempt < 3; ++attempt) {
        struct i2c_rdwr_ioctl_data io{};
        io.msgs  = msgs;
        io.nmsgs = 2;
        if (::ioctl(fd_, I2C_RDWR, &io) >= 0) return true;
        err_ = std::string("i2c read 0x") + std::to_string(reg) + ": " +
               std::strerror(errno);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return false;
}

bool Icm42688I2c::read(ImuSample* out) {
    if (!out || fd_ < 0) return false;
    uint8_t b[14];
    if (!readRegs(kRegDataStart, b, sizeof(b))) return false;

    auto s16 = [](uint8_t hi, uint8_t lo) {
        return static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | lo);
    };

    out->valid = true;
    out->t_s   = nowS();
    out->ax    = s16(b[2], b[3]) / kAccelLsbPerG;
    out->ay    = s16(b[4], b[5]) / kAccelLsbPerG;
    out->az    = s16(b[6], b[7]) / kAccelLsbPerG;
    out->gx    = s16(b[8], b[9]) / kGyroLsbPerDps;
    out->gy    = s16(b[10], b[11]) / kGyroLsbPerDps;
    out->gz    = s16(b[12], b[13]) / kGyroLsbPerDps;
    return true;
}

}  // namespace gimbal
