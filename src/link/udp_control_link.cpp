#include "gimbal/link/udp_control_link.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <iostream>

namespace gimbal {

double UdpControlLink::nowS() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

UdpControlLink::UdpControlLink(Config c) : cfg_(std::move(c)) {
    last_rx_s_ = nowS();
    hz_window_s_ = last_rx_s_;
}

UdpControlLink::~UdpControlLink() { close(); }

std::string UdpControlLink::name() const {
    return "udp:" + std::to_string(cfg_.port);
}

bool UdpControlLink::open() {
    if (fd_ >= 0) return true;

    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) {
        std::cerr << "[ctrl] socket: " << std::strerror(errno) << "\n";
        return false;
    }

    int opt = 1;
    ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port        = htons(cfg_.port);

    if (::bind(fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::cerr << "[ctrl] bind " << cfg_.port << ": " << std::strerror(errno) << "\n";
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    // 非阻塞
    const int fl = ::fcntl(fd_, F_GETFL, 0);
    ::fcntl(fd_, F_SETFL, fl | O_NONBLOCK);

    std::cerr << "[ctrl] UDP 控制口已监听 0.0.0.0:" << cfg_.port << "\n";
    return true;
}

void UdpControlLink::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool UdpControlLink::popCommand(ControlCommand& out) {
    if (cmds_.empty()) return false;
    out = cmds_.front();
    cmds_.pop_front();
    return true;
}

void UdpControlLink::sendTelemetry(const ControlTelemetry& t) {
    if (fd_ < 0 || !have_peer_) return;

    const std::vector<uint8_t> b = t.encode();

    struct sockaddr_in dst;
    std::memset(&dst, 0, sizeof(dst));
    dst.sin_family = AF_INET;
    dst.sin_port   = htons(peer_port_);
    if (::inet_pton(AF_INET, peer_ip_.c_str(), &dst.sin_addr) != 1) return;

    const ssize_t n = ::sendto(fd_, b.data(), b.size(), 0,
                               reinterpret_cast<struct sockaddr*>(&dst), sizeof(dst));
    if (n == static_cast<ssize_t>(b.size())) {
        ++st_.tx_telem;
    }
    last_telem_s_ = nowS();
}

void UdpControlLink::poll() {
    if (fd_ < 0) return;
    const double t = nowS();

    // ---- 1. 收包（一次 poll 把积压的都收完）----
    for (;;) {
        uint8_t                buf[512];
        struct sockaddr_in     src;
        socklen_t              slen = sizeof(src);

        const ssize_t n = ::recvfrom(fd_, buf, sizeof(buf), 0,
                                     reinterpret_cast<struct sockaddr*>(&src), &slen);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) break;
            break;   // 其它错误：忽略，下轮再试
        }

        ControlCommand c;
        if (!ControlCommand::decode(buf, static_cast<size_t>(n), c)) {
            ++st_.rx_bad;
            continue;
        }

        // 记录 peer（回发遥测用）
        char ipbuf[INET_ADDRSTRLEN] = {0};
        ::inet_ntop(AF_INET, &src.sin_addr, ipbuf, sizeof(ipbuf));
        const std::string ip = ipbuf;
        const uint16_t    pt = ntohs(src.sin_port);
        if (!have_peer_ || ip != peer_ip_ || pt != peer_port_) {
            std::cerr << "[ctrl] 操作端: " << ip << ":" << pt << "\n";
            peer_ip_   = ip;
            peer_port_ = pt;
            have_peer_ = true;
        }

        // 丢包统计（seq 跳号）
        if (have_seq_) {
            const uint32_t expect = last_seq_ + 1;
            if (c.seq > expect) {
                st_.rx_lost += static_cast<uint64_t>(c.seq - expect);
            }
        }
        last_seq_  = c.seq;
        have_seq_  = true;

        ++st_.rx_ok;
        ++hz_count_;
        last_rx_s_ = t;

        // 只保留最近几条，避免上层不消费时堆积
        if (cmds_.size() > 8) cmds_.pop_front();
        cmds_.push_back(c);
    }

    // ---- 2. 频率统计（1 秒窗口）----
    if (t - hz_window_s_ >= 1.0) {
        st_.rx_hz    = static_cast<int>(hz_count_ / (t - hz_window_s_) + 0.5);
        hz_count_    = 0;
        hz_window_s_ = t;
    }
    st_.age_s = t - last_rx_s_;
    if (!st_.peer.empty() || have_peer_) {
        st_.peer = peer_ip_ + ":" + std::to_string(peer_port_);
    }

    // ---- 3. 按频率回发遥测 ----
    if (have_peer_ && cfg_.telem_hz > 0 &&
        (t - last_telem_s_) >= 1.0 / static_cast<double>(cfg_.telem_hz)) {
        sendTelemetry(last_telem_);
    }

    // ---- 4. peer 超时：超过 peer_hold_s 没上行就别再回了 ----
    if (have_peer_ && st_.age_s > cfg_.peer_hold_s) {
        have_peer_ = false;
    }
}

}  // namespace gimbal
