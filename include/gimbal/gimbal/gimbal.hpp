// ===========================================================================
//  Gimbal —— 双轴云台（yaw + pitch）
//
//  职责：
//    · 把操作端的「角速度指令」分发给两根轴
//    · 统一时间基准（一个 dt 走完两轴 + 传输层 poll）
//    · 操作端看门狗（断流 -> 归零 -> 硬停）
//    · 传输层 poll 去重（两轴共总线时只 poll 一次）
//
//  看门狗分两级（联调经验）：
//    1) rate_hold_s 之后开始指数衰减  —— 手感层，鼠标停手要平滑停住
//    2) command_timeout_s 之后强制    —— 安全层，操作端失联必须停
//       注意：安全层**只把速度归零，不失能**。云台一旦失能会松轴下垂，
//       比继续转更危险。
// ===========================================================================
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "gimbal/gimbal/axis.hpp"

namespace gimbal {

class Gimbal {
public:
    struct Config {
        Axis::Config yaw;
        Axis::Config pitch;

        double command_timeout_s = 0.5;  ///< 操作端失联判定（安全层）
        bool   auto_enable       = true;
        double enable_delay_s    = 0.3;  ///< 使能后空档，避免上电抖动被立刻执行
    };

    Gimbal(std::shared_ptr<ZdtMotor> yaw_motor,
           std::shared_ptr<ZdtMotor> pitch_motor, Config cfg);

    // ---- 操作端接口 -------------------------------------------------------
    void enable(bool on);
    /// 角速度指令：正 = 角度增大。yaw 正 = 逆时针，pitch 正 = 抬头
    void setRate(double yaw_dps, double pitch_dps);
    void hold();
    void home();
    void estop();

    // ---- 周期 -------------------------------------------------------------
    /// 一次控制拍：poll 传输层 -> 更新两轴 -> （由调用方随后再 poll 一次收帧）
    /// 实际实现里 step() = pollTx + 两轴 update + pollRx
    void step(double dt);

    // ---- 查询 -------------------------------------------------------------
    Axis&       yaw() { return yaw_; }
    Axis&       pitch() { return pitch_; }
    const Axis& yaw() const { return yaw_; }
    const Axis& pitch() const { return pitch_; }

    bool   watchdogTripped() const { return watchdog_; }
    double sinceCommandS() const { return since_cmd_s_; }
    bool   enabled() const { return enabled_; }
    const Config& cfg() const { return cfg_; }

    /// 去重后的传输层列表（两轴共总线时只会有一个）
    const std::vector<std::shared_ptr<ITransport>>& transports() const {
        return transports_;
    }

private:
    void refreshCommandClock() { since_cmd_s_ = 0.0; }
    void collectTransports();
    void pollAllTransports();

    std::shared_ptr<ZdtMotor> yaw_motor_;
    std::shared_ptr<ZdtMotor> pitch_motor_;

    Axis   yaw_;
    Axis   pitch_;
    Config cfg_;

    std::vector<std::shared_ptr<ITransport>> transports_;

    bool   enabled_      = false;
    bool   watchdog_     = false;
    double since_cmd_s_  = 0.0;
    double enable_age_s_ = 0.0;
};

}  // namespace gimbal
