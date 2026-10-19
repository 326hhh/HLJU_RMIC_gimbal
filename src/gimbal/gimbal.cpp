#include "gimbal/gimbal/gimbal.hpp"

#include <algorithm>
#include <cmath>

namespace gimbal {

Gimbal::Gimbal(std::shared_ptr<ZdtMotor> yaw_motor,
               std::shared_ptr<ZdtMotor> pitch_motor, Config cfg)
    : yaw_motor_(std::move(yaw_motor)),
      pitch_motor_(std::move(pitch_motor)),
      yaw_(yaw_motor_, cfg.yaw),
      pitch_(pitch_motor_, cfg.pitch),
      cfg_(std::move(cfg)) {
    collectTransports();
    if (cfg_.auto_enable) enable(true);
}

void Gimbal::collectTransports() {
    transports_.clear();
    for (const auto& m : {yaw_motor_, pitch_motor_}) {
        if (!m) continue;
        auto t = m->transportPtr();
        if (!t) continue;
        bool seen = false;
        for (const auto& e : transports_) {
            if (e.get() == t.get()) { seen = true; break; }
        }
        if (!seen) transports_.push_back(t);
    }
}

void Gimbal::pollAllTransports() {
    for (auto& t : transports_) {
        if (t) t->poll();
    }
}

void Gimbal::enable(bool on) {
    // ⚠️ 必须幂等：操作端每个控制包都会带 Enable 标志，
    //    如果每次都重置空档计时，enable_delay_s 就永远走不完，
    //    两轴会被永久压在 hold() 上（这个坑实地踩过一次）。
    if (on == enabled_) return;

    enabled_      = on;
    enable_age_s_ = 0.0;
    yaw_.enable(on);
    pitch_.enable(on);
    refreshCommandClock();
}

void Gimbal::setRate(double yaw_dps, double pitch_dps) {
    yaw_.setRate(yaw_dps);
    pitch_.setRate(pitch_dps);
    refreshCommandClock();
    watchdog_ = false;
}

void Gimbal::hold() {
    yaw_.hold();
    pitch_.hold();
    refreshCommandClock();
}

void Gimbal::home() {
    yaw_.goHome();
    pitch_.goHome();
    refreshCommandClock();
}

void Gimbal::estop() {
    enabled_ = false;
    yaw_.estop();
    pitch_.estop();
    refreshCommandClock();
}

void Gimbal::step(double dt) {
    if (dt <= 0.0) return;
    if (dt > 0.2) dt = 0.2;

    since_cmd_s_  += dt;
    enable_age_s_ += dt;

    // ---- 1. 先把命令推到总线（节流发送在里面）----
    pollAllTransports();

    // ---- 2. 操作端看门狗（安全层）----
    if (enabled_ && since_cmd_s_ > cfg_.command_timeout_s) {
        watchdog_ = true;
        // 只归零速度，**不失能**（云台失能会松轴下垂）
        yaw_.hold();
        pitch_.hold();
    }

    // ---- 3. 使能后空档期 ----
    const bool in_delay = enabled_ && enable_age_s_ < cfg_.enable_delay_s;

    // ---- 4. 两轴控制 ----
    yaw_.update(in_delay ? 0.0 : dt);
    pitch_.update(in_delay ? 0.0 : dt);
    if (in_delay) {
        yaw_.hold();
        pitch_.hold();
    }

    // ---- 5. 收帧（把驱动板回帧填进状态）----
    pollAllTransports();
}

}  // namespace gimbal
