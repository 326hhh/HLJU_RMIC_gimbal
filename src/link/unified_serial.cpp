/**
  ******************************************************************************
  * @file    unified_serial.cpp
  * @brief   单串口统一链路实现（底盘帧 + ZDT 帧共用一个 fd）
  ******************************************************************************
  */
#include "gimbal/link/unified_serial.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <iostream>

namespace gimbal {

// ===========================================================================
//  小工具
// ===========================================================================
namespace {
int64_t nowUs() {
    using namespace std::chrono;
    return duration_cast<microseconds>(
               steady_clock::now().time_since_epoch())
        .count();
}

int baudToSpeed(int baud) {
    switch (baud) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 500000: return B500000;
        case 921600: return B921600;
        default:     return B115200;
    }
}
}  // namespace

int64_t UnifiedSerial::nowUs() { return gimbal::nowUs(); }

// ===========================================================================
//  RxDemux —— 帧边界分流
// ===========================================================================
void RxDemux::feed(const uint8_t* p, size_t n) {
    if (!p || n == 0) return;
    rx_.insert(rx_.end(), p, p + n);

    size_t pos       = 0;
    const size_t total = rx_.size();
    while (pos < total) {
        /* ---- 1) 底盘反馈帧：5A A5 81 + 36B + CRC8 + 0D ---- */
        if (rx_[pos] == CHASSIS_FB_HEAD0) {
            if (pos + 2 >= total) break;                // 等第 3 字节（需要 pos+1、pos+2）
            if (rx_[pos + 1] == CHASSIS_FB_HEAD1) {
                if (rx_[pos + 2] != CHASSIS_FB_TYPE) {  // 帧头不对，滚 1 字节
                    ++pos;
                    continue;
                }
                if (pos + CHASSIS_FB_LEN > total) break; // 等满 40 字节
                ChassisState st;
                if (chassisDecodeFb(&rx_[pos], st)) {
                    chassis_.push_back(st);
                    ++chassis_ok_;
                } else {
                    ++chassis_bad_;
                }
                pos += CHASSIS_FB_LEN;
                continue;
            }
            ++pos;                                      // 0x5A 但不是帧头
            continue;
        }

        /* ---- 2) ZDT 返回帧：Addr(1..32) + Func + 负载 + 6B ---- */
        const uint8_t addr = rx_[pos];
        if (addr >= kZdtAddrMin && addr <= kZdtAddrMax) {
            if (pos + 2 > total) break;                 // 等 func
            const int want = zdt::expectedResponseLen(rx_[pos + 1]);
            if (want < 3) {                             // 未知功能码
                ++pos;
                continue;
            }
            if (pos + static_cast<size_t>(want) > total) break;   // 等满
            if (rx_[pos + static_cast<size_t>(want) - 1] != zdt::CHECKSUM_FIXED) {
                ++zdt_bad_;                             // 校验不对，滚 1 字节
                ++pos;
                continue;
            }
            zdt::Frame f;
            f.addr = addr;
            f.func = rx_[pos + 1];
            f.data.assign(rx_.begin() + static_cast<long>(pos) + 2,
                          rx_.begin() + static_cast<long>(pos + want));
            zdt_.push_back(std::move(f));
            ++zdt_ok_;
            pos += static_cast<size_t>(want);
            continue;
        }

        ++pos;                                          // 噪声字节
    }
    rx_.erase(rx_.begin(), rx_.begin() + static_cast<long>(pos));

    /* 数据源异常时防无限增长 */
    if (rx_.size() > 4096) rx_.clear();
}

bool RxDemux::popChassis(ChassisState& s) {
    if (chassis_.empty()) return false;
    s = chassis_.front();
    chassis_.pop_front();
    return true;
}

bool RxDemux::popZdt(zdt::Frame& f) {
    if (zdt_.empty()) return false;
    f = std::move(zdt_.front());
    zdt_.pop_front();
    return true;
}

void RxDemux::clear() {
    rx_.clear();
    chassis_.clear();
    zdt_.clear();
}

// ===========================================================================
//  UnifiedSerial
// ===========================================================================
UnifiedSerial::UnifiedSerial(Config c) : cfg_(std::move(c)) {
    setGapUs(static_cast<int64_t>(cfg_.zdt_gap_ms) * 1000);
    stats_.connected = false;
    chst_.last_error.clear();
}

UnifiedSerial::~UnifiedSerial() { close(); }

bool UnifiedSerial::isOpen() const { return fd_ >= 0; }

std::string UnifiedSerial::name() const {
    return "unified:" + cfg_.device + "@" + std::to_string(cfg_.baud);
}

bool UnifiedSerial::tryOpen() {
    const int fd = ::open(cfg_.device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        stats_.last_error = std::string("open 失败: ") + std::strerror(errno);
        chst_.last_error  = stats_.last_error;
        return false;
    }

    struct termios tty;
    std::memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) {
        stats_.last_error = std::string("tcgetattr: ") + std::strerror(errno);
        chst_.last_error  = stats_.last_error;
        ::close(fd);
        return false;
    }

    cfmakeraw(&tty);                      // 8N1、无回显、无行缓冲
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;              // 无硬件流控
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    const int spd = baudToSpeed(cfg_.baud);
    cfsetispeed(&tty, spd);
    cfsetospeed(&tty, spd);

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        stats_.last_error = std::string("tcsetattr: ") + std::strerror(errno);
        chst_.last_error  = stats_.last_error;
        ::close(fd);
        return false;
    }
    tcflush(fd, TCIOFLUSH);

    fd_ = fd;
    stats_.connected = true;
    stats_.last_error.clear();
    chst_.last_error.clear();
    demux_.clear();
    return true;
}

bool UnifiedSerial::open() {
    if (fd_ >= 0) return true;
    if (tryOpen()) {
        std::cerr << "[link] 已打开 " << cfg_.device << " @ " << cfg_.baud
                  << "（底盘 + 云台共用）\n";
        return true;
    }
    return false;
}

void UnifiedSerial::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    stats_.connected = false;
}

void UnifiedSerial::drop(const std::string& why) {
    stats_.last_error = why;
    chst_.last_error  = why;
    close();
    ++stats_.reconnects;
    last_open_us_ = nowUs();
    std::cerr << "[link] 连接丢失 (" << why << ")，" << cfg_.reconnect_s
              << "s 后重连\n";
}

bool UnifiedSerial::writeAll(const std::vector<uint8_t>& out) {
    size_t off = 0;
    while (off < out.size()) {
        const ssize_t n = ::write(fd_, out.data() + off, out.size() - off);
        if (n > 0) {
            off += static_cast<size_t>(n);
        } else if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            continue;   // 内核缓冲满，稍等
        } else {
            drop(std::string("write: ") + std::strerror(errno));
            return false;
        }
    }
    return true;
}

void UnifiedSerial::sendChassis() {
    if (fd_ < 0) return;

    ChassisCommand c = cmd_;
    if (clear_pending_) {
        c.clear_odom   = true;
        clear_pending_ = false;
        c.estop        = false;
        c.enable       = false;   // CLEAR 帧不动电机，EN 位无意义
    }
    if (c.estop) {                // 急停帧不带速度
        c.vx_mmps   = 0.0;
        c.vy_mmps   = 0.0;
        c.wz_mradps = 0.0;
    }
    c.seq = tx_seq_++;

    uint8_t frame[CHASSIS_CMD_LEN];
    chassisEncodeCmd(frame, c);

    const ssize_t n = ::write(fd_, frame, sizeof(frame));
    if (n == static_cast<ssize_t>(sizeof(frame))) {
        ++chst_.tx_frames;
    } else {
        chst_.tx_dropped++;
    }
}

bool UnifiedSerial::zdtWindowOpen(int64_t now_us) const {
    if (!cfg_.zdt_after_fb) return true;
    const int64_t since = now_us - last_chassis_rx_us_;
    if (since > 100000) return true;  // 100ms 没收到底盘帧 -> 不限制
    return since <= 6000;             // 刚收完一帧：F407 已发完，约 6.5ms 静默窗
}

void UnifiedSerial::poll() {
    const int64_t now = nowUs();
    /* 同一控制拍内去重：main 的 chassis->poll() 与 Gimbal::step() 里的 poll()
     * 会各调一次，这里保证一拍只完整跑一次（读+发+时效）。 */
    if (now - last_poll_us_ < 1000) return;
    last_poll_us_ = now;

    // ---- 断线自动重连（限速 reconnect_s 一次）----
    if (fd_ < 0) {
        if (now - last_open_us_ < static_cast<int64_t>(cfg_.reconnect_s * 1e6)) return;
        last_open_us_ = now;
        if (!tryOpen()) return;
        ++stats_.reconnects;
    }

    // ---- 收：非阻塞读到空为止，整块交给分流器 ----
    for (;;) {
        const ssize_t n = ::read(fd_, rbuf_, sizeof(rbuf_));
        if (n > 0) {
            stats_.rx_bytes += static_cast<uint64_t>(n);
            demux_.feed(rbuf_, static_cast<size_t>(n));
            continue;
        }
        if (n == 0) break;
        if (errno == EAGAIN || errno == EWOULDBLOCK) break;
        drop(std::string("read: ") + std::strerror(errno));
        return;
    }

    // ---- 消费分流结果：底盘状态 / ZDT 帧队列 ----
    ChassisState st;
    while (demux_.popChassis(st)) {
        state_               = st;
        state_.valid         = true;
        state_.age_s         = 0.0;
        last_chassis_rx_us_  = now;
    }
    zdt::Frame f;
    while (demux_.popZdt(f)) {
        ++stats_.rx_frames;
        rxq_.push_back(std::move(f));
    }
    while (rxq_.size() > 256) rxq_.pop_front();

    state_.age_s = static_cast<double>(now - last_chassis_rx_us_) / 1e6;
    if (state_.age_s > cfg_.chassis_timeout_s) state_.valid = false;

    // ---- 发：ZDT 帧（12ms 节流；尽量落在底盘反馈后的静默窗）----
    if (zdtWindowOpen(now)) {
        std::vector<uint8_t> out;
        if (nextTx(out)) {
            if (!writeAll(out)) return;
            onTxDone();
        }
    }

    // ---- 发：底盘命令（固定 5ms 节流）----
    if (now - last_chassis_tx_us_ >=
        static_cast<int64_t>(cfg_.chassis_tx_period_s * 1e6)) {
        last_chassis_tx_us_ = now;
        sendChassis();
    }
}

// ===========================================================================
//  IChassisLink
// ===========================================================================
void UnifiedSerial::setCommand(const ChassisCommand& c) {
    cmd_           = c;
    clear_pending_ = clear_pending_ || c.clear_odom;
}

const ChassisState& UnifiedSerial::state() const { return state_; }

}  // namespace gimbal
