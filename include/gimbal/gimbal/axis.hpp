// ===========================================================================
//  Axis —— 云台单轴（速率控制）
//
//  控制模型：操作端给「角速度指令」，本类负责
//      · 斜坡限幅（避免机械冲击）
//      · 软限位（用软件积分角度估计）
//      · 驱动板左右限位开关联动
//      · 死区（防抖）
//
//  ⚠️ 角度估计的局限（诚实说明）:
//      ZDT 的 0x36 回读对 Emm 固件是【单圈 0-360°】，拿不到多圈绝对值，
//      所以这里的 est_angle_ 是靠转速积分出来的**软件估计**，会缓慢漂移。
//      真正的绝对保护应依赖驱动板的限位开关输入（状态位 bit4/bit5）。
//      需要绝对角度时用 zeroPosition() 在机械零点重新对齐。
// ===========================================================================
#pragma once

#include <memory>
#include <string>

#include "gimbal/device/zdt_motor.hpp"

namespace gimbal {

class Axis {
public:
    struct Config {
        std::string name = "axis";

        double min_deg = -180.0;   ///< 软限位下限
        double max_deg = 180.0;    ///< 软限位上限
        double max_rate_dps = 180.0;      ///< 最大角速度 °/s
        double rate_accel_dps2 = 900.0;   ///< 角加速度 °/s²（斜坡）
        double rate_deadband_dps = 0.5;   ///< 死区
        /// 传动比 = 电机转数 / 输出轴转数。2:1 减速就填 2.0。
        /// 影响：电机 RPM = 输出 °/s × gear_ratio / 6
        double gear_ratio        = 1.0;
        double rate_hold_s       = 0.12;  ///< 指令停手后开始衰减的延时
        double rate_decay_tau    = 0.05;  ///< 衰减时间常数（越小停得越快）
        bool   use_soft_limits = true;
        bool   use_limit_switches = true; ///< 联动机驱动板限位输入
    };

    Axis(std::shared_ptr<ZdtMotor> motor, Config cfg);

    // ---- 操作端接口 ----
    /// 设定角速度指令（°/s，正 = 角度增大）
    void setRate(double dps);
    /// 停止转动（保持使能，抱死）
    void hold();
    /// 把当前角度估计与电机零点对齐（应在机械零点调用）
    void zeroHere();
    /// 回到 0°
    void goHome(double dps = 60.0, double tol_deg = 2.0);
    /// 急停：驱动板立即停止 + 失能
    void estop();
    void enable(bool on);

    // ---- 周期 ----
    void update(double dt);

    // ---- 查询 ----
    const Config&           cfg() const { return cfg_; }
    double                  cmdRateDps() const { return rate_cmd_; }
    double                  actRateDps() const { return rate_act_; }
    double                  estAngleDeg() const { return est_angle_; }
    bool                    atSoftLimit() const { return at_limit_; }
    bool                    limitSwitchActive() const;
    bool                    homing() const { return homing_; }
    double                  targetDeg() const { return home_target_; }
    bool                    enabled() const { return enabled_; }
    ZdtMotor*               motor() const { return motor_.get(); }
    const ZdtMotor::Status& motorStatus() const { return motor_->status(); }

private:
    double clampToLimits(double rate) const;

    std::shared_ptr<ZdtMotor> motor_;
    Config                    cfg_;

    bool   enabled_    = false;
    double rate_cmd_   = 0.0;
    double rate_act_   = 0.0;
    double est_angle_  = 0.0;
    bool   at_limit_   = false;
    double since_cmd_s_ = 0.0;   ///< 距上次 setRate 的秒数

    bool   homing_      = false;
    double home_target_ = 0.0;
    double home_tol_    = 2.0;
    double home_rate_   = 60.0;
};

}  // namespace gimbal
