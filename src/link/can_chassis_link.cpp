// ===========================================================================
//  CAN 底盘链路实现（SocketCAN 标准帧）
// ===========================================================================
#include "gimbal/link/can_chassis_link.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <iostream>

namespace gimbal {

namespace {
double nowSImpl() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

int16_t clampI16v(double v) {
    if (v > 32767.0) return 32767;
    if (v < -32768.0) return -32768;
    return static_cast<int16_t>(v < 0 ? v - 0.5 : v + 0.5);
}

void putI16LE(uint8_t* p, int16_t v) {
    p[0] = static_cast<uint8_t>(static_cast<uint16_t>(v) & 0xFFu);
    p[1] = static_cast<uint8_t>((static_cast<uint16_t>(v) >> 8) & 0xFFu);
}
}  // namespace

double CanFbGroup::nowS() { return nowSImpl(); }

int CanFbGroup::feed(uint32_t id, const uint8_t* data, uint8_t dlc,
                     uint8_t out[kFrameLen]) {
    if (data == nullptr || dlc != kChunk) return 0;

    const int idx = static_cast<int>(id - base_);
    if (idx < 0 || idx > 4) return 0;

    if (idx == 0) {
        // 新组：上一个没拼满的组直接丢弃（规则 3）
        active_ = true;
        got_    = 0;
        std::memset(buf_, 0, sizeof(buf_));
        t_start_ = nowS();
    } else if (!active_) {
        return 0;   // 没有开组，孤帧忽略
    }

    std::memcpy(buf_ + static_cast<size_t>(idx) * kChunk, data, kChunk);
    got_ |= static_cast<uint8_t>(1u << idx);
    if (got_ != 0x1Fu) return 0;   // 还没拼满

    active_ = false;               // 组结束
    // 规则 5：CRC8 + 尾字节 0x0D
    if (chassisCrc8(buf_, CHASSIS_FB_LEN - 2) != buf_[CHASSIS_FB_LEN - 2] ||
        buf_[CHASSIS_FB_LEN - 1] != CHASSIS_FB_TAIL) {
        return -1;
    }
    std::memcpy(out, buf_, kFrameLen);
    return 1;
}

// ---------------------------------------------------------------------------
CanChassisLink::CanChassisLink(Config c) : cfg_(std::move(c)) {}

CanChassisLink::~CanChassisLink() { close(); }

double CanChassisLink::nowS() { return nowSImpl(); }

std::string CanChassisLink::name() const { return "can:" + cfg_.iface; }

bool CanChassisLink::isOpen() const { return fd_ >= 0; }

bool CanChassisLink::tryOpen() {
    const int fd = ::socket(PF_CAN, SOCK_RAW | SOCK_CLOEXEC, CAN_RAW);
    if (fd < 0) {
        st_.last_error = std::string("socket: ") + std::strerror(errno);
        return false;
    }

    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, cfg_.iface.c_str(), IFNAMSIZ - 1);
    if (::ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
        st_.last_error = "找不到接口 " + cfg_.iface + ": " + std::strerror(errno);
        ::close(fd);
        return false;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        st_.last_error = std::string("bind: ") + std::strerror(errno);
        ::close(fd);
        return false;
    }

    // 非阻塞
    const int fl = ::fcntl(fd, F_GETFL, 0);
    ::fcntl(fd, F_SETFL, fl | O_NONBLOCK);

    fd_         = fd;
    opened_     = true;
    last_cmd_s_ = 0.0;
    flags_      = 0;
    group_      = CanFbGroup(cfg_.id_fb_base);
    group_.setTimeoutS(cfg_.group_timeout_s);
    st_.last_error.clear();
    return true;
}

bool CanChassisLink::open() {
    if (fd_ >= 0) return true;
    if (tryOpen()) {
        std::cerr << "[chassis-can] 已打开 " << cfg_.iface << "（标准帧 0x"
                  << std::hex << cfg_.id_speed << "/0x" << cfg_.id_flags << "/0x"
                  << cfg_.id_fb_base << std::dec << "~0x" << std::hex
                  << (cfg_.id_fb_base + 4) << std::dec << "）\n";
        return true;
    }
    return false;
}

void CanChassisLink::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    opened_       = false;
    group_.drop();
}

void CanChassisLink::drop(const std::string& why) {
    st_.last_error = why;
    close();
    last_open_try_s_ = nowS();
    std::cerr << "[chassis-can] 连接丢失 (" << why << ")，" << cfg_.reconnect_s
              << "s 后重连\n";
}

bool CanChassisLink::sendFrame(uint32_t id, const uint8_t* data, uint8_t dlc) {
    if (fd_ < 0) return false;
    struct can_frame cf;
    std::memset(&cf, 0, sizeof(cf));
    cf.can_id  = id;                       // 标准帧（无 EFF 标志）
    cf.can_dlc = dlc;
    if (data != nullptr && dlc > 0) std::memcpy(cf.data, data, dlc);
    const ssize_t n = ::write(fd_, &cf, sizeof(cf));
    if (n == static_cast<ssize_t>(sizeof(cf))) {
        ++st_.tx_frames;
        return true;
    }
    ++st_.tx_dropped;
    st_.last_error = std::string("write: ") + std::strerror(errno);
    return false;
}

void CanChassisLink::sendSpeedFrame(uint8_t cmd) {
    uint8_t d[8] = {0};
    const bool zero = (cmd == 0x02) || (cmd == 0x04);   // 急停 / 清零不带速度
    d[0] = cmd;
    d[1] = seq_++;
    putI16LE(d + 2, clampI16v(zero ? 0.0 : cmd_.vx_mmps));
    putI16LE(d + 4, clampI16v(zero ? 0.0 : cmd_.vy_mmps));
    putI16LE(d + 6, clampI16v(zero ? 0.0 : cmd_.wz_mradps / 10.0));
    sendFrame(cfg_.id_speed, d, 8);
}

void CanChassisLink::sendFlagsFrame() {
    uint8_t d[8] = {0};
    d[0] = flags_;
    sendFrame(cfg_.id_flags, d, 8);
}

void CanChassisLink::poll() {
    const double now = nowS();

    // 断线重连
    if (fd_ < 0) {
        if (now - last_open_try_s_ < cfg_.reconnect_s) return;
        last_open_try_s_ = now;
        if (!tryOpen()) return;
    }

    // 过期未拼满的组（规则 4）
    if (group_.pending() && now - group_.startS() > group_.timeoutS()) {
        group_.drop();
    }

    // ---- 收 ----
    for (;;) {
        struct can_frame cf;
        const ssize_t n = ::read(fd_, &cf, sizeof(cf));
        if (n == static_cast<ssize_t>(sizeof(cf))) {
            if (cf.can_id & CAN_EFF_FLAG) continue;   // ZDT 扩展帧，不属于底盘
            const uint32_t id = cf.can_id & CAN_SFF_MASK;
            if (id < cfg_.id_fb_base || id > cfg_.id_fb_base + 4) continue;

            if (group_.pending() && nowS() - group_.startS() > group_.timeoutS()) {
                group_.drop();
            }
            uint8_t fb[CHASSIS_FB_LEN];
            const int r = group_.feed(id, cf.data, cf.can_dlc, fb);
            if (r > 0) {
                ChassisState st;
                if (chassisDecodeFb(fb, st)) {
                    state_        = st;
                    state_.valid  = true;
                    state_.age_s  = 0.0;
                    last_fb_s_    = nowS();
                    ++st_.rx_frames;
                } else {
                    ++st_.crc_err;
                }
            } else if (r < 0) {
                ++st_.crc_err;
            }
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
        if (n < 0) {
            drop(std::string("read: ") + std::strerror(errno));
            return;
        }
        break;
    }

    state_.age_s = nowS() - last_fb_s_;
    if (state_.age_s > cfg_.fb_timeout_s) state_.valid = false;

    // ---- 发 ----
    // 急停边沿：0x000 广播（DLC=0），F407 锁存
    if (cmd_.estop && !estop_active_) {
        estop_active_ = true;
        sendFrame(cfg_.id_estop, nullptr, 0);
    } else if (!cmd_.estop) {
        estop_active_ = false;
    }

    if (clear_pending_) {
        clear_pending_ = false;
        sendSpeedFrame(0x04);          // 里程计 + 陀螺 yaw 清零
    }

    /* 容差 0.9x：100Hz 循环下严格 >= 10ms 会因浮点/调度抖动隔一拍才发 */
    if (now - last_cmd_s_ >= cfg_.cmd_period_s * 0.9) {
        last_cmd_s_ = now;
        sendSpeedFrame(estop_active_ ? 0x02 : 0x01);
    }

    // 0x021：flags 变化立即发，否则 10Hz 保活
    const uint8_t flags = cmd_.enable ? 0x01u : 0x00u;
    if (flags != flags_ || now - last_flags_s_ >= cfg_.flags_period_s * 0.9) {
        flags_       = flags;
        sendFlagsFrame();
        last_flags_s_ = now;
    }
}

void CanChassisLink::setCommand(const ChassisCommand& c) {
    cmd_           = c;
    clear_pending_ = clear_pending_ || c.clear_odom;
}

const ChassisState& CanChassisLink::state() const { return state_; }

}  // namespace gimbal
