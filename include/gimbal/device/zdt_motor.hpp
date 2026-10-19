// ===========================================================================
//  ZdtMotor —— 单个 ZDT 闭环步进电机
//
//  封装: 使能 / 速度 / 位置 / 急停 / 归零 / 状态回读
//  职责: 把「物理量」翻译成 ZDT 帧，并把回帧解析成结构化状态
//
//  通信节流与总线轮询由 ITransport 负责；本类不直接碰 fd。
// ===========================================================================
#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "gimbal/protocol/zdt_codes.hpp"
#include "gimbal/protocol/zdt_frame.hpp"
#include "gimbal/transport/itransport.hpp"

namespace gimbal {

class ZdtMotor {
public:
    struct Config {
        uint8_t        addr              = 1;
        zdt::Firmware  firmware          = zdt::Firmware::Emm;
        uint32_t       pulses_per_rev    = 3200;  ///< 200 步/圈 × 细分
        uint8_t        acc_default       = 200;   ///< 加速度档位
        bool           invert_dir        = false; ///< 逻辑正方向是否对应 CW
        double         max_rpm           = 3000.0;

        // 遥测轮询周期（ms）—— 每项独立计时，不用轮转指针
        int poll_speed_ms  = 100;
        int poll_pos_ms    = 100;
        int poll_status_ms = 200;
        int poll_vbus_ms   = 1000;
        int poll_temp_ms   = 2000;
    };

    struct Status {
        bool   valid       = false;  ///< 是否收到过有效回帧
        bool   enabled     = false;
        bool   in_position = false;
        bool   stalled     = false;
        bool   stall_prot  = false;
        bool   lim_left    = false;
        bool   lim_right   = false;
        bool   power_loss  = false;

        double   speed_rpm = 0.0;    ///< 带符号（按逻辑正方向）
        double   pos_deg   = 0.0;    ///< 单圈 0..360（Emm: raw*360/65536）
        uint32_t pos_raw   = 0;

        int     vbus_mv  = 0;
        int     temp_c   = 0;
        uint8_t last_ret = 0;        ///< 最近一次控制命令的返回码

        uint64_t acks_ok  = 0;
        uint64_t acks_err = 0;
        uint64_t frames   = 0;
        double   age_s    = 999.0;   ///< 距上次收到回帧的秒数
    };

    ZdtMotor(std::shared_ptr<ITransport> t, Config c);

    // ---- 命令 -------------------------------------------------------------
    void setEnabled(bool on);
    void setSpeedRpm(double signed_rpm);
    /// @param deg  目标角度（Emm 固件：内部换算成脉冲数）
    void setPositionDeg(double deg, uint16_t rpm, zdt::MoveMode mode);
    void estop();
    void zeroPosition();
    void clearStall();

    // ---- 周期 -------------------------------------------------------------
    /// 控制循环里调用：下发到期遥测 + 收帧更新状态。**不**负责 poll 传输层。
    void update();

    /// 清空"已下发"影子值，让下个周期强制重发（断线重连后调用）
    void resyncCommands();

    // ---- 查询 -------------------------------------------------------------
    const Status&     status() const { return st_; }
    const Config&     cfg() const { return cfg_; }
    uint8_t           addr() const { return cfg_.addr; }
    ITransport*                transport() const { return transport_.get(); }
    std::shared_ptr<ITransport> transportPtr() const { return transport_; }

    bool communicationOk(double timeout_s) const {
        return st_.valid && st_.age_s < timeout_s;
    }

    // ---- 换算 -------------------------------------------------------------
    int32_t degToPulses(double deg) const;
    double  pulsesToDeg(int32_t p) const;
    void    setPulsesPerRev(uint32_t v) { cfg_.pulses_per_rev = v ? v : 3200; }

private:
    struct PollItem {
        zdt::Func func;
        int       period_ms;
        double    next_ms;
    };

    void handleFrame(const zdt::Frame& f);
    static double nowMs();

    std::shared_ptr<ITransport>   transport_;
    Config                        cfg_;
    Status                        st_;
    std::vector<PollItem>         polls_;
    bool                          shadow_en_valid_  = false;
    bool                          shadow_enabled_   = false;
    bool                          shadow_spd_valid_ = false;
    double                        shadow_rpm_       = 0.0;
    std::chrono::steady_clock::time_point last_rx_tp_{};
    bool                          has_rx_tp_ = false;
};

}  // namespace gimbal
