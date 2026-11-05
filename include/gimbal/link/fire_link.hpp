// ===========================================================================
//  开火链路：Jetson <-> STM32F401RCT6（水弹波箱，MOS 驱动）
//
//  硬件：Jetson CH340(/dev/ttyUSB0) <-> F401 USART1(PA9 TX / PA10 RX)
//        115200 8N1；F401 PA12 = FIRE，高电平 -> MOS 导通 -> 发射
//
//  与 F401 固件（stmproject/biu）**逐字节一致**，CRC-8/ATM(poly 0x07, init 0)。
//
//  命令帧（Jetson -> F401，6 B）
//  ┌──────┬─────┬──────┬──────┬──────┬───────┬──────┐
//  │ off  │  0  │  1   │  2   │  3   │   4   │  5   │
//  │ 字段 │ 0xA5│ 0x5A │ 0x10 │ seq  │ flags │ crc8 │
//  │ 类型 │ u8  │ u8   │ u8   │ u8   │  u8   │  u8  │
//  └──────┴─────┴──────┴──────┴──────┴───────┴──────┘
//      flags bit0 = FIRE（1=请求开火，0=停火）
//      crc8 覆盖 0..4
//
//  反馈帧（F401 -> Jetson，7 B）
//  ┌──────┬─────┬──────┬──────┬───────┬───────┬──────┬──────┐
//  │ off  │  0  │  1   │  2   │  3    │   4   │  5   │  6   │
//  │ 字段 │ 0x5A│ 0xA5 │ 0x81 │ seq回显│ state │ crc8 │ 0x0D │
//  └──────┴─────┴──────┴──────┴───────┴───────┴──────┴──────┘
//      state bit0 = FIRE_ON（PA12 当前高电平）
//      state bit1 = TIMEOUT（>100ms 未收到合法命令，固件已强制关断）
//      state bit2 = OVERTIME（连续导通超过固件保护时限，已强制关断）
//
//  安全语义（三层，逐层兜底）：
//    1. 客户端：按住 F 才置 Fire 位，松手/断流立即清零
//    2. Jetson ：UDP Fire 位只在 0.3 s 内有效，退出时显式发停火帧
//    3. F401   ：100 ms 收不到合法命令 -> 硬件层强制 PA12 低；
//                连续导通超过 FIRE_MAX_ON_MS -> 强制关断并锁存 OVERTIME
// ===========================================================================
#pragma once

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "gimbal/link/chassis_link.hpp"   // 复用 chassisCrc8 与 put/get 小端读写

namespace gimbal {

// ---- 帧常量（与 F401 侧 fire_link.h 对齐）--------------------------------
constexpr uint8_t  FIRE_CMD_HEAD0 = 0xA5;
constexpr uint8_t  FIRE_CMD_HEAD1 = 0x5A;
constexpr uint8_t  FIRE_FB_HEAD0  = 0x5A;
constexpr uint8_t  FIRE_FB_HEAD1  = 0xA5;
constexpr uint8_t  FIRE_FB_TYPE   = 0x81;
constexpr uint8_t  FIRE_FB_TAIL   = 0x0D;
constexpr uint8_t  FIRE_CMD_WORD  = 0x10;   ///< 开火命令字
constexpr size_t   FIRE_CMD_LEN   = 6;
constexpr size_t   FIRE_FB_LEN    = 7;

namespace fire_flag {
constexpr uint8_t On = 0x01;      ///< bit0：请求开火
}  // namespace fire_flag

namespace fire_status {
constexpr uint8_t FireOn   = 0x01;   ///< PA12 当前为高
constexpr uint8_t Timeout  = 0x02;   ///< F401 侧链路看门狗已触发
constexpr uint8_t Overtime = 0x04;   ///< F401 侧连续导通保护已触发（锁存）
}  // namespace fire_status

// ---------------------------------------------------------------------------
// 编解码（两端逐字节一致）
// ---------------------------------------------------------------------------
inline size_t fireEncodeCmd(uint8_t out[FIRE_CMD_LEN], bool fire_on, uint8_t seq) {
    out[0] = FIRE_CMD_HEAD0;
    out[1] = FIRE_CMD_HEAD1;
    out[2] = FIRE_CMD_WORD;
    out[3] = seq;
    out[4] = fire_on ? fire_flag::On : 0;
    out[5] = chassisCrc8(out, FIRE_CMD_LEN - 1);
    return FIRE_CMD_LEN;
}

inline bool fireDecodeFb(const uint8_t* in, uint8_t& seq, uint8_t& state) {
    if (in[0] != FIRE_FB_HEAD0 || in[1] != FIRE_FB_HEAD1 ||
        in[2] != FIRE_FB_TYPE || in[FIRE_FB_LEN - 1] != FIRE_FB_TAIL)
        return false;
    if (chassisCrc8(in, FIRE_FB_LEN - 2) != in[FIRE_FB_LEN - 2]) return false;
    seq   = in[3];
    state = in[4];
    return true;
}

// ---------------------------------------------------------------------------
// 链路统计
// ---------------------------------------------------------------------------
struct FireLinkStats {
    uint64_t    tx_frames  = 0;
    uint64_t    rx_frames  = 0;
    uint64_t    tx_dropped = 0;
    uint64_t    crc_err    = 0;
    uint64_t    sync_err   = 0;
    uint64_t    reconnects = 0;
    double      age_s      = 999.0;   ///< 距上一帧反馈
    std::string last_error;
};

// ---------------------------------------------------------------------------
// UART 实现：一键开火 / 一键停火，断线自动重连，反馈新鲜度看门狗
// ---------------------------------------------------------------------------
class UartFireLink {
public:
    struct Config {
        std::string device      = "/dev/ttyUSB0";
        int         baud        = 115200;
        double      tx_period_s = 0.010;  ///< 100 Hz 下发命令（与 UDP 控制同频）
        double      timeout_s   = 0.10;   ///< 反馈超时 -> state().valid 置 0（F401 自己的看门狗更硬）
        double      reconnect_s = 0.5;    ///< 断线后重开周期
    };

    explicit UartFireLink(Config c) : cfg_(std::move(c)) {}

    ~UartFireLink() { close(); }

    bool open() {
        close();
        fd_ = ::open(cfg_.device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) {
            stats_.last_error = std::strerror(errno);
            return false;
        }
        termios tio{};
        if (tcgetattr(fd_, &tio) != 0) {
            stats_.last_error = std::strerror(errno);
            ::close(fd_);
            fd_ = -1;
            return false;
        }
        cfmakeraw(&tio);
        tio.c_cflag |= (CLOCAL | CREAD);
        tio.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);
        tio.c_cflag = static_cast<tcflag_t>((tio.c_cflag & ~CSIZE) | CS8);
        tio.c_cc[VMIN]  = 0;
        tio.c_cc[VTIME] = 0;
        const speed_t sp = (cfg_.baud == 921600) ? B921600
                         : (cfg_.baud == 460800) ? B460800
                         : (cfg_.baud == 230400) ? B230400
                         : (cfg_.baud == 57600)  ? B57600
                         : B115200;
        cfsetispeed(&tio, sp);
        cfsetospeed(&tio, sp);
        if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
            stats_.last_error = std::strerror(errno);
            ::close(fd_);
            fd_ = -1;
            return false;
        }
        tcflush(fd_, TCIOFLUSH);
        fd_ok_ = true;
        rx_buf_.clear();
        /* 上电先把扳机按成「停火」，别让 F401 停在未知状态 */
        fire_ = false;
        return true;
    }

    void close() {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
        fd_ok_ = false;
        rx_buf_.clear();
    }

    bool isOpen() const { return fd_ok_ && fd_ >= 0; }

    std::string name() const { return "fire-uart:" + cfg_.device; }

    /// 请求开火 / 停火（幂等；实际由 poll() 按节流发出去）
    void setFire(bool on) { fire_ = on; }

    bool fireRequested() const { return fire_; }

    const FireLinkStats& stats() const { return stats_; }

    /// 最近一帧反馈的 state 位；超时后返回 Timeout
    uint8_t state() const {
        if (state_.age_s > cfg_.timeout_s) return fire_status::Timeout;
        return state_.state;
    }

    /// 反馈是否新鲜（F401 在线）
    bool linkOk() const { return fd_ok_ && state_.age_s <= cfg_.timeout_s; }

    /// PA12 实际是否高（来自反馈帧，不是我们的请求）
    bool fireOn() const { return (state_.state & fire_status::FireOn) != 0; }

    /// 往回灌一次反馈超时/断线（供上层展示用）
    double ageS() const { return state_.age_s; }

    void poll() {
        const double now = nowS();

        if (fd_ < 0) {
            /* 断线自动重连：限速，别把 CPU 打满 */
            if (now - last_open_try_s_ > cfg_.reconnect_s) {
                last_open_try_s_ = now;
                if (open()) {
                    ++stats_.reconnects;
                    last_tx_s_ = now;
                }
            }
            state_.age_s = 999.0;
            return;
        }

        /* ---- 收：非阻塞读到空为止 ---- */
        uint8_t buf[256];
        for (;;) {
            const ssize_t n = ::read(fd_, buf, sizeof(buf));
            if (n > 0) {
                appendRx(buf, static_cast<size_t>(n));
                parseRx(now);
            } else {
                break;
            }
        }

        /* ---- 发：按 tx_period_s 节流（命令帧只 6 B，100 Hz 才 600 B/s）---- */
        if (now - last_tx_s_ >= cfg_.tx_period_s) {
            last_tx_s_ = now;
            sendCommand();
        }

        state_.age_s = now - last_rx_s_;
    }

private:
    struct FbState {
        uint8_t state = 0;
        uint8_t seq   = 0;
        double  age_s = 999.0;
    };

    static double nowS() {
        return std::chrono::duration<double>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }

    void appendRx(const uint8_t* d, size_t n) { rx_buf_.insert(rx_buf_.end(), d, d + n); }

    /// 从累积缓冲切帧：丢噪声、校验 CRC、更新反馈状态
    void parseRx(double now) {
        size_t pos = 0;
        while (rx_buf_.size() - pos >= FIRE_FB_LEN) {
            if (rx_buf_[pos] != FIRE_FB_HEAD0 || rx_buf_[pos + 1] != FIRE_FB_HEAD1) {
                ++pos;
                ++stats_.sync_err;
                continue;
            }
            uint8_t seq = 0, st = 0;
            if (!fireDecodeFb(&rx_buf_[pos], seq, st)) {
                ++pos;
                ++stats_.crc_err;
                continue;
            }
            state_.state = st;
            state_.seq   = seq;
            state_.age_s = 0.0;
            last_rx_s_   = now;
            ++stats_.rx_frames;
            pos += FIRE_FB_LEN;
        }
        rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + static_cast<long>(pos));
        if (rx_buf_.size() > 4096) rx_buf_.clear();
    }

    void sendCommand() {
        uint8_t frame[FIRE_CMD_LEN];
        fireEncodeCmd(frame, fire_, tx_seq_++);
        const ssize_t n = ::write(fd_, frame, sizeof(frame));
        if (n == static_cast<ssize_t>(sizeof(frame))) {
            ++stats_.tx_frames;
        } else {
            ++stats_.tx_dropped;
        }
    }

    Config               cfg_;
    int                  fd_ = -1;
    bool                 fd_ok_ = false;
    bool                 fire_ = false;
    FbState              state_{};
    FireLinkStats        stats_{};
    std::vector<uint8_t> rx_buf_;
    uint8_t              tx_seq_ = 0;
    double               last_tx_s_      = 0.0;
    double               last_rx_s_      = -1e9;
    double               last_open_try_s_ = -1e9;
};

}  // namespace gimbal
