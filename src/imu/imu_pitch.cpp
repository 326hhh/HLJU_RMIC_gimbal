#include "gimbal/imu/imu.hpp"

#include <algorithm>
#include <cmath>

namespace gimbal {

namespace {

constexpr double kPi = 3.14159265358979323846;

/// 把角度折到 (-180, 180]
double wrap180(double d) {
    d = std::fmod(d + 180.0, 360.0);
    if (d < 0.0) d += 360.0;
    return d - 180.0;
}

}  // namespace

void ImuPitch::update(const ImuSample& s, double dt) {
    if (!s.valid || dt <= 0.0) return;

    // ---- 加速度计角度：θ = asin(u·â) ----
    //   u = 枪管方向（IMU 坐标下的一条轴），â = 归一化比力。
    //   这样只要枪管对齐了某条轴，绕枪管怎么滚转都不影响仰角。
    const double amag = std::sqrt(s.ax * s.ax + s.ay * s.ay + s.az * s.az);
    const double inv  = (amag > 1e-6) ? (1.0 / amag) : 0.0;
    const double a_ax = (cfg_.accel_axis == 0) ? s.ax
                      : (cfg_.accel_axis == 1) ? s.ay
                                               : s.az;
    double       acc  = std::asin(std::max(-1.0, std::min(1.0, a_ax * inv))) *
                        180.0 / kPi;
    if (cfg_.invert) acc = -acc;
    acc_deg_ = wrap180(acc + cfg_.offset_deg);

    // ---- 陀螺：俯仰角速度 ----
    double g = (cfg_.gyro_axis == 0) ? s.gx
             : (cfg_.gyro_axis == 1) ? s.gy
                                     : s.gz;
    g -= cfg_.gyro_bias_dps;
    if (cfg_.invert) g = -g;
    rate_ = g;

    // 加速度可信的前提：|a| 接近 1 g，且此刻角速度不大（排除转弯/抖动）
    confident_ = (std::fabs(amag - 1.0) <= cfg_.accel_tol_g) &&
                 (std::fabs(g) <= cfg_.accel_gate_dps);

    if (!seeded_) {
        pitch_ = acc_deg_;
        seeded_ = true;
        return;
    }

    pitch_ = wrap180(pitch_ + rate_ * dt);
    if (confident_) {
        // 互补滤波：只按 (1-alpha) 的权重把加速度角度"拉"过来。
        // 创新门限：底盘加减速会让加速度计角度整体偏掉（0.3 g 在 0° 时能骗出 17°），
        // 这时融合角和加速度角会先分开——差太大就只信陀螺，等它自己回来。
        const double inn = wrap180(acc_deg_ - pitch_);
        if (std::fabs(inn) <= cfg_.inn_tol_deg) {
            pitch_ = wrap180(pitch_ + (1.0 - cfg_.alpha) * inn);
        }
    }
}

void ImuPitch::zeroHere() { zero_deg_ = pitch_; }

void ImuPitch::reset() {
    seeded_    = false;
    confident_ = false;
    pitch_     = 0.0;
    acc_deg_   = 0.0;
    rate_      = 0.0;
}

double ImuPitch::pitchDeg() const { return wrap180(pitch_ - zero_deg_); }

double ImuPitch::rawAccelDeg() const { return wrap180(acc_deg_ - zero_deg_); }

}  // namespace gimbal
