// ===========================================================================
//  mjpeg_server —— 车载摄像头 MJPEG 图传（C++17，V4L2 直出，零转码）
//
//  为什么零转码：UVC 摄像头本身就能输出 MJPG（压缩帧），
//  V4L2 取到的每个 buffer 就是一张完整 JPEG（FFD8...FFD9），
//  直接原样转发即可，CPU 几乎不花（Jetson 上实测 < 3%）。
//
//  协议：HTTP multipart/x-mixed-replace，与 ~/gimbal_ctl 的 MjpegView 对齐。
//        MjpegView 是按 JPEG 的 SOI(FFD8)/EOI(FFD9) 切帧的，不依赖 boundary 文本，
//        所以这里 boundary 只是给浏览器（http://<ip>:8080/stream）看。
//
//  用法：
//      ./mjpeg_server                        # 640x480@30，端口 8080
//      ./mjpeg_server --size 800x600 --fps 20 --port 8080
//      ./mjpeg_server --device /dev/video0
//
//  带宽参考（MJPG，动态场景）：
//      640x480@30 ≈ 3~6 Mbps   800x600@20 ≈ 3~5 Mbps   1280x720@30 ≈ 8~15 Mbps
//      Jetson AP（2.4G）实测能到 20+ Mbps，建议 640x480 或 800x600 起。
// ===========================================================================
#include <arpa/inet.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

// ---------------------------------------------------------------------------
static volatile sig_atomic_t g_run = 1;
static void onSignal(int) { g_run = 0; }

static double nowS() {
    return std::chrono::duration<double>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

// ---------------------------------------------------------------------------
// V4L2 MJPG 采集
// ---------------------------------------------------------------------------
class V4l2Capture {
public:
    bool open(const std::string& dev, int w, int h, int fps) {
        fd_ = ::open(dev.c_str(), O_RDWR | O_NONBLOCK);
        if (fd_ < 0) {
            std::fprintf(stderr, "[v4l2] 打开 %s 失败: %s\n", dev.c_str(),
                         std::strerror(errno));
            return false;
        }

        v4l2_capability cap{};
        if (::ioctl(fd_, VIDIOC_QUERYCAP, &cap) < 0) {
            std::fprintf(stderr, "[v4l2] VIDIOC_QUERYCAP 失败\n");
            return false;
        }

        v4l2_format fmt{};
        fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width       = static_cast<uint32_t>(w);
        fmt.fmt.pix.height      = static_cast<uint32_t>(h);
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
        fmt.fmt.pix.field       = V4L2_FIELD_ANY;
        if (::ioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
            std::fprintf(stderr, "[v4l2] VIDIOC_S_FMT(MJPG) 失败: %s\n",
                         std::strerror(errno));
            return false;
        }
        if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_MJPEG) {
            std::fprintf(stderr, "[v4l2] 摄像头不接受 MJPG，实际给的是 0x%08X\n",
                         fmt.fmt.pix.pixelformat);
            return false;
        }
        w_ = static_cast<int>(fmt.fmt.pix.width);
        h_ = static_cast<int>(fmt.fmt.pix.height);

        v4l2_streamparm parm{};
        parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (::ioctl(fd_, VIDIOC_G_PARM, &parm) == 0) {
            parm.parm.capture.timeperframe.numerator   = 1;
            parm.parm.capture.timeperframe.denominator =
                static_cast<uint32_t>(fps);
            ::ioctl(fd_, VIDIOC_S_PARM, &parm);
        }

        v4l2_requestbuffers req{};
        req.count  = kBufCount;
        req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;
        if (::ioctl(fd_, VIDIOC_REQBUFS, &req) < 0) {
            std::fprintf(stderr, "[v4l2] VIDIOC_REQBUFS 失败\n");
            return false;
        }
        if (req.count < 2) {
            std::fprintf(stderr, "[v4l2] 缓冲不足\n");
            return false;
        }

        bufs_.resize(req.count);
        for (uint32_t i = 0; i < req.count; ++i) {
            v4l2_buffer b{};
            b.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            b.memory = V4L2_MEMORY_MMAP;
            b.index  = i;
            if (::ioctl(fd_, VIDIOC_QUERYBUF, &b) < 0) {
                std::fprintf(stderr, "[v4l2] VIDIOC_QUERYBUF 失败\n");
                return false;
            }
            bufs_[i].len = b.length;
            bufs_[i].ptr = ::mmap(nullptr, b.length, PROT_READ | PROT_WRITE,
                                  MAP_SHARED, fd_, static_cast<off_t>(b.m.offset));
            if (bufs_[i].ptr == MAP_FAILED) {
                std::fprintf(stderr, "[v4l2] mmap 失败\n");
                return false;
            }
            if (::ioctl(fd_, VIDIOC_QBUF, &b) < 0) {
                std::fprintf(stderr, "[v4l2] VIDIOC_QBUF 失败\n");
                return false;
            }
        }

        v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (::ioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
            std::fprintf(stderr, "[v4l2] VIDIOC_STREAMON 失败: %s\n",
                         std::strerror(errno));
            return false;
        }
        streaming_ = true;
        std::fprintf(stderr, "[v4l2] %s 打开成功 %dx%d MJPG\n", dev.c_str(), w_, h_);
        return true;
    }

    /// 取一帧（非阻塞）。成功返回 true，数据在 data/len 里，处理完必须 requeue()
    bool dequeue(const uint8_t** data, size_t* len, int* idx) {
        v4l2_buffer b{};
        b.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        b.memory = V4L2_MEMORY_MMAP;
        if (::ioctl(fd_, VIDIOC_DQBUF, &b) < 0) {
            if (errno == EAGAIN) return false;
            std::fprintf(stderr, "[v4l2] VIDIOC_DQBUF 失败: %s\n",
                         std::strerror(errno));
            return false;
        }
        *data = static_cast<const uint8_t*>(bufs_[b.index].ptr);
        *len  = b.bytesused;
        *idx  = static_cast<int>(b.index);
        return true;
    }

    void requeue(int idx) {
        v4l2_buffer b{};
        b.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        b.memory = V4L2_MEMORY_MMAP;
        b.index  = static_cast<uint32_t>(idx);
        ::ioctl(fd_, VIDIOC_QBUF, &b);
    }

    int width()  const { return w_; }
    int height() const { return h_; }

    void close() {
        if (streaming_) {
            v4l2_buf_type t = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            ::ioctl(fd_, VIDIOC_STREAMOFF, &t);
            streaming_ = false;
        }
        for (auto& b : bufs_) {
            if (b.ptr && b.ptr != MAP_FAILED) ::munmap(b.ptr, b.len);
        }
        bufs_.clear();
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }

    ~V4l2Capture() { close(); }

private:
    static constexpr uint32_t kBufCount = 4;
    struct Buf {
        void*  ptr = nullptr;
        size_t len = 0;
    };
    int               fd_ = -1;
    int               w_ = 0;
    int               h_ = 0;
    bool              streaming_ = false;
    std::vector<Buf>  bufs_;
};

// ---------------------------------------------------------------------------
// 极简 MJPEG HTTP 服务器（多客户端，慢客户端直接踢掉）
// ---------------------------------------------------------------------------
class MjpegServer {
public:
    static constexpr int kMaxClients = 4;

    bool listen(uint16_t port) {
        lfd_ = ::socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        if (lfd_ < 0) return false;
        int one = 1;
        ::setsockopt(lfd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

        sockaddr_in addr{};
        addr.sin_family      = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port        = htons(port);
        if (::bind(lfd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
            std::fprintf(stderr, "[http] bind :%u 失败: %s\n", port,
                         std::strerror(errno));
            return false;
        }
        if (::listen(lfd_, kMaxClients) < 0) {
            std::fprintf(stderr, "[http] listen 失败\n");
            return false;
        }
        std::fprintf(stderr, "[http] 监听 0.0.0.0:%u（浏览器开 http://<ip>:%u/stream）\n",
                     port, port);
        return true;
    }

    /// 接受新连接（非阻塞），返回当前客户端数
    void acceptClients() {
        for (;;) {
            sockaddr_in peer{};
            socklen_t   plen = sizeof(peer);
            int c = ::accept4(lfd_, reinterpret_cast<sockaddr*>(&peer), &plen,
                              SOCK_NONBLOCK | SOCK_CLOEXEC);
            if (c < 0) break;
            if (clients_.size() >= kMaxClients) {
                ::close(c);
                continue;
            }
            int one = 1;
            ::setsockopt(c, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
            // 读掉请求行（不解析路径，一律给流）
            char tmp[512];
            ::recv(c, tmp, sizeof(tmp), MSG_DONTWAIT);

            static const char kHdr[] =
                "HTTP/1.1 200 OK\r\n"
                "Connection: close\r\n"
                "Cache-Control: no-cache, private\r\n"
                "Pragma: no-cache\r\n"
                "Content-Type: multipart/x-mixed-replace; "
                "boundary=--ulinkframe\r\n"
                "\r\n";
            if (::send(c, kHdr, sizeof(kHdr) - 1, MSG_DONTWAIT | MSG_NOSIGNAL) < 0) {
                ::close(c);
                continue;
            }
            clients_.push_back(c);
            std::fprintf(stderr, "[http] 客户端接入 %s:%u（共 %zu）\n",
                         ::inet_ntoa(peer.sin_addr), ntohs(peer.sin_port),
                         clients_.size());
        }
    }

    /// 广播一帧 JPEG；发不出去（慢客户端）直接断开
    void broadcast(const uint8_t* jpeg, size_t len) {
        if (clients_.empty()) return;
        char hdr[128];
        const int hl = std::snprintf(hdr, sizeof(hdr),
                                     "--ulinkframe\r\nContent-Type: image/jpeg\r\n"
                                     "Content-Length: %zu\r\n\r\n",
                                     len);
        for (size_t i = 0; i < clients_.size();) {
            const int fd = clients_[i];
            const bool ok =
                ::send(fd, hdr, static_cast<size_t>(hl), MSG_DONTWAIT | MSG_NOSIGNAL) == hl &&
                ::send(fd, jpeg, len, MSG_DONTWAIT | MSG_NOSIGNAL) ==
                    static_cast<ssize_t>(len);
            if (ok) {
                ++i;
            } else {
                std::fprintf(stderr, "[http] 客户端断开（慢/关闭）\n");
                ::close(fd);
                clients_.erase(clients_.begin() + static_cast<long>(i));
            }
        }
    }

    size_t clientCount() const { return clients_.size(); }

    void close() {
        for (int c : clients_) ::close(c);
        clients_.clear();
        if (lfd_ >= 0) {
            ::close(lfd_);
            lfd_ = -1;
        }
    }

    ~MjpegServer() { close(); }

private:
    int              lfd_ = -1;
    std::vector<int> clients_;
};

// ---------------------------------------------------------------------------
static void usage(const char* a0) {
    std::printf(
        "用法: %s [选项]\n"
        "  --device <路径>   摄像头 (默认 /dev/video0)\n"
        "  --size <WxH>      分辨率 (默认 640x480，可选 800x600/1280x720/1920x1080)\n"
        "  --fps <n>         帧率 (默认 30)\n"
        "  --port <n>        HTTP 端口 (默认 8080)\n"
        "  -h, --help\n", a0);
}

int main(int argc, char** argv) {
    std::string dev = "/dev/video0";
    int w = 640, h = 480, fps = 30, port = 8080;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "-h" || a == "--help") { usage(argv[0]); return 0; }
        else if (a == "--device" && i + 1 < argc) dev = argv[++i];
        else if (a == "--size" && i + 1 < argc) {
            std::sscanf(argv[++i], "%dx%d", &w, &h);
        }
        else if (a == "--fps" && i + 1 < argc)   fps  = std::atoi(argv[++i]);
        else if (a == "--port" && i + 1 < argc)  port = std::atoi(argv[++i]);
        else { usage(argv[0]); return 2; }
    }

    signal(SIGINT, onSignal);
    signal(SIGTERM, onSignal);
    signal(SIGPIPE, SIG_IGN);

    V4l2Capture cap;
    if (!cap.open(dev, w, h, fps)) return 1;

    MjpegServer srv;
    if (!srv.listen(static_cast<uint16_t>(port))) return 1;

    double  last_stat = nowS();
    uint64_t frames = 0, bytes = 0;
    uint64_t max_ms = 0;
    double  t_prev = nowS();

    while (g_run) {
        srv.acceptClients();

        const uint8_t* data = nullptr;
        size_t         len  = 0;
        int            idx  = 0;
        if (!cap.dequeue(&data, &len, &idx)) {
            // 无帧：等一下再来（10ms 后重试，避免空转吃满 CPU）
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        const double t_dq = nowS();
        srv.broadcast(data, len);
        cap.requeue(idx);

        ++frames;
        bytes += len;
        const double dt_ms = (t_dq - t_prev) * 1000.0;
        if (dt_ms < 1000.0 && static_cast<uint64_t>(dt_ms) > max_ms) {
            max_ms = static_cast<uint64_t>(dt_ms);
        }
        t_prev = t_dq;

        const double now = nowS();
        if (now - last_stat >= 1.0) {
            std::fprintf(stderr,
                         "[stat] %dx%d  %.1f fps  %.0f KB/s  客户端 %zu  最大帧间隔 %llu ms\n",
                         cap.width(), cap.height(), frames / (now - last_stat),
                         (bytes / 1024.0) / (now - last_stat), srv.clientCount(),
                         static_cast<unsigned long long>(max_ms));
            frames = 0;
            bytes  = 0;
            max_ms = 0;
            last_stat = now;
        }
    }

    std::fprintf(stderr, "\n[main] 退出\n");
    srv.close();
    cap.close();
    return 0;
}
