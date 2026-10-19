#include "gimbal/gimbal/axis.hpp"

#include <algorithm>
#include <cmath>

namespace gimbal {

Axis::Axis(std::shared_ptr<ZdtMotor> motor, Config cfg)
    : motor_(std::move(motor)), cfg_(std::move(cfg)) {}

bool Axis::limitSwitchActive() const {
    const auto& s = motor_->status();
    return s.lim_left || s.lim_right;
}

double Axis::clampToLimits(double rate) const {
    if (!cfg_.use_soft_limits) return rate;
    // 接近上限且还要继续正向 -> 不允许
    if (rate > 0.0 && est_angle_ >= cfg_.max_deg) return 0.0;
    if (rate < 0.0 && est_angle_ <= cfg_.min_deg) return 0.0;
    // 限位开关触发时，只允许往反方向走（用于退出限位）
    if (cfg_.use_limit_switches) {
        const auto& s = motor_->status();
        if (s.valid) {
            if (s.lim_right && rate > 0.0) return 0.0;
            if (s.lim_left && rate < 0.0) return 0.0;
        }
    }
    return rate;
}

// ---------------------------------------------------------------------------
void Axis::setRate(double dps) {
    since_cmd_s_ = 0.0;
    double r = dps;
    if (r > cfg_.max_rate_dps) r = cfg_.max_rate_dps;
    if (r < -cfg_.max_rate_dps) r = -cfg_.max_rate_dps;
    rate_cmd_ = r;
}

void Axis::hold() {
    rate_cmd_ = 0.0;
    homing_   = false;
}

void Axis::zeroHere() {
    est_angle_ = 0.0;
    at_limit_  = false;
    motor_->zeroPosition();
}

void Axis::goHome(double dps, double tol_deg) {
    homing_      = true;
    home_target_ = 0.0;
    home_tol_    = tol_deg;
    home_rate_   = std::fabs(dps);
}

void Axis::estop() {
    rate_cmd_ = 0.0;
    rate_act_ = 0.0;
    homing_   = false;
    enabled_  = false;
    motor_->estop();
    motor_->setEnabled(false);
    motor_->resyncCommands();
}

void Axis::enable(bool on) {
    enabled_ = on;
    motor_->setEnabled(on);
    if (!on) {
        rate_cmd_ = 0.0;
        rate_act_ = 0.0;
    }
}

// ---------------------------------------------------------------------------
void Axis::update(double dt) {
    // 遥测下发与收帧跟 dt 无关，必须每拍都跑
    motor_->update();

    if (dt <= 0.0) return;
    if (dt > 0.2) dt = 0.2;   // 调试断点后不要大步长跳变

    // ---- 1. 回中逻辑（若有）----
    if (homing_) {
        const double err = home_target_ - est_angle_;
        if (std::fabs(err) <= home_tol_) {
            homing_   = false;
            rate_cmd_ = 0.0;
        } else {
            rate_cmd_ = (err > 0 ? 1.0 : -1.0) * home_rate_;
        }
    }

    // ---- 1b. 停手平滑衰减（手感层）----
    since_cmd_s_ += dt;
    if (!homing_ && since_cmd_s_ > cfg_.rate_hold_s) {
        const double k = std::exp(-dt / std::max(1e-3, cfg_.rate_decay_tau));
        rate_cmd_ *= k;
        if (std::fabs(rate_cmd_) < 0.05) rate_cmd_ = 0.0;
    }

    // ---- 2. 斜坡限幅（角加速度约束）----
    const double max_step = cfg_.rate_accel_dps2 * dt;
    double       d        = rate_cmd_ - rate_act_;
    if (d > max_step) d = max_step;
    if (d < -max_step) d = -max_step;
    rate_act_ += d;

    // ---- 3. 死区（防抖）----
    if (std::fabs(rate_act_) < cfg_.rate_deadband_dps && std::fabs(rate_cmd_) < 1e-9) {
        rate_act_ = 0.0;
    }

    // ---- 4. 限位 ----
    const double before = rate_act_;
    rate_act_ = clampToLimits(rate_act_);
    at_limit_ = (std::fabs(before) > 1e-9 && std::fabs(rate_act_) < 1e-9);

    // ---- 5. 未使能一律不动 ----
    if (!enabled_) rate_act_ = 0.0;

    // ---- 6. 下发到电机 ----
    // 输出 °/s -> 电机 RPM：360°/s = 60RPM，再乘传动比
    const double gear = (cfg_.gear_ratio > 1e-6) ? cfg_.gear_ratio : 1.0;
    motor_->setSpeedRpm(rate_act_ * gear / 6.0);

    // ---- 7. 角度估计：优先用实测转速，无回帧时退化为指令值 ----
    const auto& s = motor_->status();
    double      rpm_used;
    if (s.valid && s.age_s < 0.5) {
        rpm_used = s.speed_rpm;                       // 电机实测 RPM
    } else {
        rpm_used = rate_act_ * gear / 6.0;            // 退化用指令值
    }
    // 电机 RPM -> 输出 °/s：×6 再除传动比
    est_angle_ += (rpm_used * 6.0 / gear) * dt;

    // 软限位硬夹（即使估计漂了也不越界）
    if (cfg_.use_soft_limits) {
        if (est_angle_ > cfg_.max_deg + 5.0) est_angle_ = cfg_.max_deg + 5.0;
        if (est_angle_ < cfg_.min_deg - 5.0) est_angle_ = cfg_.min_deg - 5.0;
    }
}

}  // namespace gimbal
