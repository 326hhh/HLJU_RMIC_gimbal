#include "gimbal/device/fire_control.hpp"

#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>

namespace gimbal {

FireControl::FireControl(const std::string& script, int pin) {
    int p[2];
    if (::pipe(p) != 0) {
        std::fprintf(stderr, "[fire] pipe: %s\n", std::strerror(errno));
        return;
    }

    pid_ = ::fork();
    if (pid_ < 0) {
        std::fprintf(stderr, "[fire] fork: %s\n", std::strerror(errno));
        ::close(p[0]);
        ::close(p[1]);
        return;
    }

    if (pid_ == 0) {   // ---- 子进程：python3 fire_worker.py <pin> ----
        ::close(p[1]);
        ::dup2(p[0], STDIN_FILENO);
        ::close(p[0]);
        const std::string pin_s = std::to_string(pin);
        ::execl("/usr/bin/python3", "python3", script.c_str(), pin_s.c_str(),
                static_cast<char*>(nullptr));
        std::perror("[fire] execl");
        ::_exit(127);
    }

    ::close(p[0]);
    fd_ = p[1];
    std::printf("[fire] worker 已启动 pid=%d pin=%d (%s)\n", pid_, pin, script.c_str());
}

FireControl::~FireControl() {
    if (fd_ >= 0) {
        // "0\nq\n"：先停火再退出，worker 自己 cleanup
        const char buf[] = "0\nq\n";
        const ssize_t n  = ::write(fd_, buf, sizeof(buf) - 1);
        (void)n;
        ::close(fd_);
        fd_ = -1;
    }
    if (pid_ > 0) {
        int st = 0;
        for (int i = 0; i < 100; ++i) {          // 最多等 1s
            const pid_t r = ::waitpid(pid_, &st, WNOHANG);
            if (r == pid_ || r < 0) { pid_ = -1; return; }
            ::usleep(10000);
        }
        ::kill(pid_, SIGTERM);
        ::waitpid(pid_, &st, 0);
        pid_ = -1;
    }
}

void FireControl::setFire(bool on) {
    if (fd_ < 0 || fire_ == on) return;
    // 注意：worker 按行读 stdin，必须带换行符
    const char buf[2] = { static_cast<char>(on ? '1' : '0'), '\n' };
    if (::write(fd_, buf, 2) != 2)
        std::fprintf(stderr, "[fire] write: %s\n", std::strerror(errno));
    fire_ = on;
}

}  // namespace gimbal
