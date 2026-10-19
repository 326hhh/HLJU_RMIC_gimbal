// ===========================================================================
//  UDP 操作端链路（Jetson 侧）
//
//  职责：
//    · 非阻塞收 UDP 控制包 -> 解码 -> 交给上层（主循环取用）
//    · 按固定频率把云台遥测回发给「最后一个发过包的客户端」
//    · 统计收包频率 / 丢包 / 时效，供看门狗与状态显示使用
//
//  线程模型：**单线程、非阻塞**。poll() 在控制循环里被调用，
//            不额外开线程 —— 控制循环本来就要跑 100Hz，顺手收包即可。
// ===========================================================================
#pragma once

#include <cstdint>
#include <deque>
#include <string>

#include "gimbal/link/control_link.hpp"

namespace gimbal {

class UdpControlLink {
public:
    struct Config {
        uint16_t port       = CTRL_DEFAULT_PORT;
        int      telem_hz   = static_cast<int>(CTRL_TELEM_HZ);
        double   peer_hold_s = 10.0;   ///< 多久没有上行就停止回发遥测
    };

    explicit UdpControlLink(Config c);
    ~UdpControlLink();

    bool        open();
    void        close();
    bool        isOpen() const { return fd_ >= 0; }
    std::string name() const;

    /// 控制循环里调用：收包 + 按频率回发遥测
    void poll();

    /// 取出一条收到的指令；无则返回 false
    bool popCommand(ControlCommand& out);

    /// 主循环每拍把最新遥测交进来；poll() 按频率自动回发
    void setTelemetry(const ControlTelemetry& t) { last_telem_ = t; }

    /// 立即回发一次（一般不直接用）
    void sendTelemetry(const ControlTelemetry& t);

    const ControlLinkStats& stats() const { return st_; }
    bool hasPeer() const { return have_peer_; }

    /// 最近一次收到的指令的 age（秒）——供上层看门狗参考
    double commandAgeS() const { return st_.age_s; }

private:
    static double nowS();

    Config                         cfg_;
    int                            fd_ = -1;
    bool                           have_peer_ = false;
    std::string                    peer_ip_;
    uint16_t                       peer_port_ = 0;
    double                         last_rx_s_ = 0.0;
    double                         last_telem_s_ = 0.0;
    double                         hz_window_s_ = 0.0;
    int                            hz_count_ = 0;
    uint32_t                       last_seq_ = 0;
    bool                           have_seq_ = false;
    std::deque<ControlCommand>     cmds_;
    ControlLinkStats               st_;
    ControlTelemetry               last_telem_{};
};

}  // namespace gimbal
