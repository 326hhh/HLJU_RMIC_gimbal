#include "gimbal/transport/uart_transport.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <iostream>

namespace gimbal {

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

UartTransport::UartTransport(Config c) : cfg_(std::move(c)) {
    setGapUs(static_cast<int64_t>(cfg_.tx_gap_ms) * 1000);
    stats_.connected = false;
}

UartTransport::~UartTransport() { close(); }

bool UartTransport::isOpen() const { return fd_ >= 0; }

std::string UartTransport::name() const {
    return "uart:" + cfg_.device + "@" + std::to_string(cfg_.baud);
}

bool UartTransport::tryOpen() {
    const int fd = ::open(cfg_.device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        stats_.last_error = std::string("open 失败: ") + std::strerror(errno);
        return false;
    }

    struct termios tty;
    std::memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) {
        stats_.last_error = std::string("tcgetattr: ") + std::strerror(errno);
        ::close(fd);
        return false;
    }

    cfmakeraw(&tty);                       // 8N1、无回显、无行缓冲
    tty.c_cflag &= ~PARENB;                // 无校验
    tty.c_cflag &= ~CSTOPB;                // 1 停止位
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;               // 无硬件流控
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cc[VMIN]  = 0;                   // 非阻塞读
    tty.c_cc[VTIME] = 0;

    const int spd = baudToSpeed(cfg_.baud);
    cfsetispeed(&tty, spd);
    cfsetospeed(&tty, spd);

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        stats_.last_error = std::string("tcsetattr: ") + std::strerror(errno);
        ::close(fd);
        return false;
    }
    tcflush(fd, TCIOFLUSH);

    fd_ = fd;
    stats_.connected  = true;
    stats_.last_error.clear();
    return true;
}

bool UartTransport::open() {
    if (fd_ >= 0) return true;
    if (tryOpen()) {
        std::cerr << "[uart] 已打开 " << cfg_.device << " @ " << cfg_.baud << "\n";
        return true;
    }
    return false;
}

void UartTransport::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    stats_.connected = false;
}

void UartTransport::drop(const std::string& why) {
    stats_.last_error = why;
    close();
    ++stats_.reconnects;
    next_reconnect_us_ = nowUs() + static_cast<int64_t>(cfg_.reconnect_ms) * 1000;
    std::cerr << "[uart] 连接丢失 (" << why << ")，将在 " << cfg_.reconnect_ms
              << "ms 后重连\n";
}

void UartTransport::poll() {
    const int64_t t = nowUs();

    // ---- 重连 ----
    if (fd_ < 0) {
        if (t < next_reconnect_us_) return;
        if (!tryOpen()) {
            next_reconnect_us_ = t + static_cast<int64_t>(cfg_.reconnect_ms) * 1000;
            return;
        }
        std::cerr << "[uart] 已重新连接 " << cfg_.device << "\n";
    }

    // ---- 发送：一次 poll 最多发一条，受最小间隔约束 ----
    std::vector<uint8_t> out;
    if (nextTx(out)) {
        size_t off = 0;
        bool   ok  = true;
        while (off < out.size()) {
            const ssize_t n = ::write(fd_, out.data() + off, out.size() - off);
            if (n > 0) {
                off += static_cast<size_t>(n);
            } else if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                continue;  // 内核缓冲满，忙等一小会
            } else {
                drop(std::string("write: ") + std::strerror(errno));
                ok = false;
                break;
            }
        }
        if (ok) onTxDone();
        if (fd_ < 0) return;
    }

    // ---- 接收：非阻塞读到空为止 ----
    for (;;) {
        const ssize_t n = ::read(fd_, rbuf_, sizeof(rbuf_));
        if (n > 0) {
            feedRx(rbuf_, static_cast<size_t>(n));
            continue;
        }
        if (n == 0) break;
        if (errno == EAGAIN || errno == EWOULDBLOCK) break;
        drop(std::string("read: ") + std::strerror(errno));
        return;
    }
}

}  // namespace gimbal
