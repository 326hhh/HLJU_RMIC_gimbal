#include "gimbal/device/zdt_motor.hpp"

#include <algorithm>
#include <cmath>

namespace gimbal {

double ZdtMotor::nowMs() {
    using namespace std::chrono;
    return duration<double, std::milli>(steady_clock::now().time_since_epoch())
        .count();
}

ZdtMotor::ZdtMotor(std::shared_ptr<ITransport> t, Config c)
    : transport_(std::move(t)), cfg_(c) {
    const double t0 = nowMs();
    const double gap = 5.0;   // 初始错开，避免所有遥测挤在同一拍
    polls_ = {
        {zdt::Func::GetSpeed,  cfg_.poll_speed_ms,  t0 + gap * 1},
        {zdt::Func::GetPos,    cfg_.poll_pos_ms,    t0 + gap * 2},
        {zdt::Func::GetStatus, cfg_.poll_status_ms, t0 + gap * 3},
        {zdt::Func::GetVbus,   cfg_.poll_vbus_ms,   t0 + gap * 4},
        {zdt::Func::GetTemp,   cfg_.poll_temp_ms,   t0 + gap * 5},
    };
}

// ---------------------------------------------------------------------------
// 角度 <-> 脉冲
//   Emm 固件位置模式用的是【脉冲数】：脉冲/圈 = 200 × 细分
//   而回读 0x36 返回的是 0-65535 对应一圈 0-360°，两套单位必须在设备层收口
// ---------------------------------------------------------------------------
int32_t ZdtMotor::degToPulses(double deg) const {
    const double p = deg / 360.0 * static_cast<double>(cfg_.pulses_per_rev);
    return static_cast<int32_t>(std::lround(p));
}

double ZdtMotor::pulsesToDeg(int32_t p) const {
    if (cfg_.pulses_per_rev == 0) return 0.0;
    return static_cast<double>(p) * 360.0 / static_cast<double>(cfg_.pulses_per_rev);
}

// ---------------------------------------------------------------------------
// 命令
// ---------------------------------------------------------------------------
void ZdtMotor::setEnabled(bool on) {
    if (shadow_en_valid_ && shadow_enabled_ == on) return;
    transport_->send(zdt::mkEnable(cfg_.addr, on));
    shadow_enabled_  = on;
    shadow_en_valid_ = true;
    if (!on) {
        shadow_rpm_       = 0.0;
        shadow_spd_valid_ = false;   // 失能后速度影子值失效
    }
}

void ZdtMotor::setSpeedRpm(double signed_rpm) {
    double a = std::fabs(signed_rpm);
    if (a > cfg_.max_rpm) a = cfg_.max_rpm;
    if (a < 0.5) a = 0.0;

    if (shadow_spd_valid_ && std::fabs(shadow_rpm_ - a) < 0.5) return;  // 无变化

    const bool positive = signed_rpm >= 0.0;
    const bool ccw      = (positive != cfg_.invert_dir);

    transport_->send(zdt::mkSpeed(cfg_.addr,
                                  static_cast<uint16_t>(std::lround(a)),
                                  ccw, cfg_.acc_default));
    shadow_rpm_       = a;
    shadow_spd_valid_ = true;
}

void ZdtMotor::setPositionDeg(double deg, uint16_t rpm, zdt::MoveMode mode) {
    const int32_t pulses = degToPulses(deg);
    const bool    ccw    = (pulses >= 0) != cfg_.invert_dir;

    if (cfg_.firmware != zdt::Firmware::Emm) {
        // X 固件的位置模式帧与 Emm 不同（0.1° 单位、加减速为 RPM/S），
        // 本工程暂只实现 Emm；需要 X 时按手册 §5.3.10 扩展 mkPositionX()。
        return;
    }
    transport_->send(zdt::mkPositionEmm(
        cfg_.addr, std::abs(pulses) , rpm,
        cfg_.acc_default, mode, ccw, /*cache=*/false));
    // 位置命令会改变运动状态，速度影子值失效
    shadow_rpm_       = 0.0;
    shadow_spd_valid_ = false;
}

void ZdtMotor::estop() {
    transport_->send(zdt::mkEStop(cfg_.addr), /*urgent=*/true);
    shadow_rpm_       = 0.0;
    shadow_spd_valid_ = false;
}

void ZdtMotor::zeroPosition() {
    transport_->send(zdt::mkZeroPos(cfg_.addr));
}

void ZdtMotor::clearStall() {
    transport_->send(zdt::mkClearStall(cfg_.addr));
}

void ZdtMotor::resyncCommands() {
    shadow_en_valid_  = false;
    shadow_spd_valid_ = false;
}

// ---------------------------------------------------------------------------
// 周期
// ---------------------------------------------------------------------------
void ZdtMotor::update() {
    const double t = nowMs();

    // ---- 到期遥测（每项独立计时）----
    for (auto& p : polls_) {
        if (t >= p.next_ms) {
            transport_->send(zdt::mkGet(cfg_.addr, p.func));
            p.next_ms = t + static_cast<double>(p.period_ms);
        }
    }

    // ---- 收帧 ----
    zdt::Frame f;
    while (transport_->popFrameFor(cfg_.addr, f)) handleFrame(f);

    // ---- 通信时效 ----
    if (has_rx_tp_) {
        st_.age_s = std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - last_rx_tp_)
                        .count();
    }
}

void ZdtMotor::handleFrame(const zdt::Frame& f) {
    if (f.data.size() < 2) return;   // 至少要有 返回码 + 校验码

    st_.frames = transport_->stats().rx_frames;
    last_rx_tp_ = std::chrono::steady_clock::now();
    has_rx_tp_  = true;
    st_.age_s   = 0.0;

    switch (static_cast<zdt::Func>(f.func)) {
        // ---- 控制类回帧：只有返回码 ----
        case zdt::Func::Enable:
        case zdt::Func::Speed:
        case zdt::Func::Position:
        case zdt::Func::EStop:
        case zdt::Func::ZeroPos:
        case zdt::Func::ClearStall: {
            st_.last_ret = f.data[0];
            st_.valid    = true;
            if (f.data[0] == 0x02 || f.data[0] == 0x12 || f.data[0] == 0x22) {
                ++st_.acks_ok;
            } else {
                ++st_.acks_err;
            }
            break;
        }

        // ---- 实时转速：data = [符号, rpmH, rpmL, chk] ----
        case zdt::Func::GetSpeed: {
            if (f.data.size() < 4) break;
            const double rpm = static_cast<double>((f.data[1] << 8) | f.data[2]);
            st_.speed_rpm = (f.data[0] != 0) ? -rpm : rpm;
            if (cfg_.invert_dir) st_.speed_rpm = -st_.speed_rpm;
            st_.valid = true;
            break;
        }

        // ---- 实时位置：data = [符号, p3,p2,p1,p0, chk] ----
        case zdt::Func::GetPos: {
            if (f.data.size() < 6) break;
            const uint32_t raw = (static_cast<uint32_t>(f.data[1]) << 24) |
                                 (static_cast<uint32_t>(f.data[2]) << 16) |
                                 (static_cast<uint32_t>(f.data[3]) << 8) |
                                 static_cast<uint32_t>(f.data[4]);
            st_.pos_raw = raw;
            // Emm 固件：0-65535 表示一圈 0-360°
            double deg = static_cast<double>(raw & 0xFFFFu) * 360.0 / 65536.0;
            if (f.data[0] != 0) deg = -deg;   // 符号位
            st_.pos_deg = deg;
            st_.valid   = true;
            break;
        }

        // ---- 状态标志：data = [flags, chk] ----
        case zdt::Func::GetStatus: {
            const uint8_t fl = f.data[0];
            st_.enabled     = (fl & zdt::st::Enabled) != 0;
            st_.in_position = (fl & zdt::st::InPos) != 0;
            st_.stalled     = (fl & zdt::st::Stalled) != 0;
            st_.stall_prot  = (fl & zdt::st::StallProt) != 0;
            st_.lim_left    = (fl & zdt::st::LimLeft) != 0;
            st_.lim_right   = (fl & zdt::st::LimRight) != 0;
            st_.power_loss  = (fl & zdt::st::PowerLoss) != 0;
            st_.valid       = true;
            break;
        }

        // ---- 母线电压：data = [vH, vL, chk] ----
        case zdt::Func::GetVbus: {
            if (f.data.size() < 3) break;
            st_.vbus_mv = (f.data[0] << 8) | f.data[1];
            break;
        }

        // ---- 驱动温度：data = [符号, ℃, chk] ----
        case zdt::Func::GetTemp: {
            if (f.data.size() < 3) break;
            st_.temp_c = (f.data[0] != 0) ? -static_cast<int>(f.data[1])
                                          : static_cast<int>(f.data[1]);
            break;
        }

        default:
            break;
    }
}

}  // namespace gimbal
