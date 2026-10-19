#include "gimbal/transport/can_transport.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <algorithm>
#include <cstring>
#include <iostream>

namespace gimbal {

namespace {
constexpr uint8_t kCanMaxDlc = 8;
}

CanTransport::CanTransport(Config c) : cfg_(std::move(c)) {
    setGapUs(static_cast<int64_t>(cfg_.tx_gap_ms) * 1000);
    stats_.connected = false;
}

CanTransport::~CanTransport() { close(); }

std::string CanTransport::name() const { return "can:" + cfg_.iface; }

bool CanTransport::open() {
    if (fd_ >= 0) return true;

    fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd_ < 0) {
        stats_.last_error = std::string("socket: ") + std::strerror(errno);
        return false;
    }

    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, cfg_.iface.c_str(), IFNAMSIZ - 1);
    if (ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
        stats_.last_error = "找不到接口 " + cfg_.iface + ": " + std::strerror(errno);
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        stats_.last_error = std::string("bind: ") + std::strerror(errno);
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    stats_.connected = true;
    stats_.last_error.clear();
    std::cerr << "[can] 已绑定 " << cfg_.iface << "\n";
    return true;
}

void CanTransport::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    stats_.connected = false;
}

// ---------------------------------------------------------------------------
// 发送：把 UART 风格的帧转成 CAN 扩展帧
//   uartFrame = [addr, func, payload..., chk]
//   CAN data  = [func, payload..., chk]        (addr 进 ID)
//   ID        = (addr << 8) | packet
// ---------------------------------------------------------------------------
bool CanTransport::canSendPackets(uint8_t addr, const std::vector<uint8_t>& uartFrame) {
    if (uartFrame.size() < 3) return false;

    // 去掉首字节 addr
    std::vector<uint8_t> body(uartFrame.begin() + 1, uartFrame.end());

    uint8_t packet = 0;
    size_t  off    = 0;
    while (off < body.size()) {
        const size_t n = std::min<size_t>(kCanMaxDlc, body.size() - off);

        struct can_frame cf;
        std::memset(&cf, 0, sizeof(cf));
        cf.can_id  = static_cast<canid_t>((static_cast<uint32_t>(addr) << 8) | packet);
        cf.can_id |= CAN_EFF_FLAG;   // 29-bit 扩展帧（手册强制）
        cf.can_dlc = static_cast<__u8>(n);
        std::memcpy(cf.data, body.data() + off, n);

        if (::write(fd_, &cf, sizeof(cf)) != static_cast<ssize_t>(sizeof(cf))) {
            stats_.last_error = std::string("can write: ") + std::strerror(errno);
            stats_.connected  = false;
            return false;
        }

        off += n;
        ++packet;
    }
    return true;
}

void CanTransport::onCanFrame(uint32_t canId, const uint8_t* data, uint8_t dlc) {
    const uint8_t  addr   = static_cast<uint8_t>((canId >> 8) & 0xFF);
    const uint8_t  packet = static_cast<uint8_t>(canId & 0xFF);
    (void)addr;

    if (packet == 0) {
        reasm_.buf.clear();
        reasm_.func = dlc > 0 ? data[0] : 0;
    }
    reasm_.buf.insert(reasm_.buf.end(), data, data + dlc);

    // 已知功能码 -> 期望的 CAN 数据段长度 = UART 帧长 - 1（去掉 addr）
    const int want = zdt::expectedResponseLen(reasm_.func);
    if (want <= 0) return;
    if (reasm_.buf.size() < static_cast<size_t>(want - 1)) return;
    if (reasm_.buf.size() > static_cast<size_t>(want - 1)) {
        // 超出：说明丢包错位，丢弃重新等
        reasm_.buf.clear();
        return;
    }

    // 还原成 UART 风格帧喂给解析器，上层无需区分总线类型
    std::vector<uint8_t> uartFrame;
    uartFrame.reserve(reasm_.buf.size() + 1);
    uartFrame.push_back(static_cast<uint8_t>((canId >> 8) & 0xFF));   // 还原 Addr
    uartFrame.insert(uartFrame.end(), reasm_.buf.begin(), reasm_.buf.end());

    feedRx(uartFrame.data(), uartFrame.size());
    reasm_.buf.clear();
}

void CanTransport::poll() {
    if (fd_ < 0) return;

    // ---- 发送 ----
    std::vector<uint8_t> out;
    if (nextTx(out)) {
        if (out.size() >= 2) {
            if (canSendPackets(out[0], out)) onTxDone();
            if (!stats_.connected) return;
        }
    }

    // ---- 接收 ----
    for (;;) {
        struct can_frame cf;
        const ssize_t n = ::read(fd_, &cf, sizeof(cf));
        if (n == static_cast<ssize_t>(sizeof(cf))) {
            if (cf.can_id & CAN_EFF_FLAG) {
                onCanFrame(cf.can_id & CAN_EFF_MASK, cf.data, cf.can_dlc);
            }
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
        if (n < 0) {
            stats_.last_error = std::string("can read: ") + std::strerror(errno);
            break;
        }
        break;
    }
}

}  // namespace gimbal
