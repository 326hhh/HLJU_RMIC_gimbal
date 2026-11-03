#pragma once

#include <string>
#include <sys/types.h>

namespace gimbal {

// 水弹波箱火力控制：fork/exec 一个 python worker，经管道发 '0'/'1'。
//   · worker 用 Jetson.GPIO 直接驱动 40-pin GPIO（BOARD 模式 pin 12）
//   · 本对象析构时写 "0\nq\n"：停火 + worker cleanup
//   · worker 检测 stdin EOF（本进程崩溃）也会自己停火 —— 双保险
class FireControl {
public:
    /// script: fire_worker.py 路径；pin: BOARD 引脚号
    explicit FireControl(const std::string& script, int pin = 12);
    ~FireControl();

    FireControl(const FireControl&)            = delete;
    FireControl& operator=(const FireControl&) = delete;

    bool ok() const { return fd_ >= 0; }
    /// 请求开火/停火（幂等：只在状态变化时写管道）
    void setFire(bool on);

private:
    int   fd_  = -1;    // 写端
    pid_t pid_ = -1;
    bool  fire_ = false;
};

}  // namespace gimbal
