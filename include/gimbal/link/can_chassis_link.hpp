// ===========================================================================
//  CAN 底盘链路：Jetson <-> F407（SocketCAN，标准帧）
//
//  协议见 docs/CAN_CONTROL_PROTOCOL.md（底盘仓库）：
//    0x000 广播急停（DLC=0，Jetson 发 / F407 只收）——最高优先级
//    0x020 速度命令 100 Hz：cmd seq vx(i16) vy(i16) wz(i16 0.01rad/s)
//    0x021 标志帧 10 Hz 保活 + flags 变化时立即发：flags(bit0 EN)
//    0x110~0x114 反馈 5 帧 @100Hz：串口 40B 反馈按 8 字节切开
//
//  组完整性规则（接收端）：
//    1. 0x110 开新组；2. 0x111~0x114 按序填入；
//    3. 下一个 0x110 到来时上组未满 → 丢弃；
//    4. 距 0x110 超过 group_timeout_s(5ms) 未满 → 丢弃；
//    5. CRC8 失败 → 丢弃整组，不输出。
// ===========================================================================
#pragma once

#include <cstdint>
#include <string>

#include "gimbal/link/chassis_link.hpp"

namespace gimbal {

// ---------------------------------------------------------------------------
//  CanFbGroup —— 0x110~0x114 → 40B 反馈帧的组拼装（纯逻辑，可独立自测）
// ---------------------------------------------------------------------------
class CanFbGroup {
public:
    static constexpr size_t kFrameLen = CHASSIS_FB_LEN;  // 40
    static constexpr size_t kChunk    = 8;
    static constexpr uint32_t kBaseId = 0x110;

    explicit CanFbGroup(uint32_t base_id = kBaseId) : base_(base_id) {}

    /**
     * @brief 喂一帧反馈
     * @return 1 = 组完整且 CRC/尾字节通过（out 有效）；0 = 未完成/忽略；
     *         -1 = 组完成但 CRC 校验失败（已丢弃）
     */
    int feed(uint32_t id, const uint8_t* data, uint8_t dlc,
             uint8_t out[kFrameLen]);

    bool   pending() const { return active_; }
    double startS()  const { return t_start_; }
    double timeoutS() const { return timeout_s_; }
    void   setTimeoutS(double s) { timeout_s_ = (s > 0.0) ? s : 0.005; }
    void   drop() { active_ = false; }

private:
    static double nowS();

    uint32_t base_;
    uint8_t  buf_[kFrameLen];
    uint8_t  got_ = 0;              // 位图：bit0=0x110 … bit4=0x114
    double   t_start_ = 0.0;
    bool     active_ = false;
    double   timeout_s_ = 0.005;
};

// ---------------------------------------------------------------------------
//  CanChassisLink —— SocketCAN 实现的 IChassisLink
// ---------------------------------------------------------------------------
class CanChassisLink : public IChassisLink {
public:
    struct Config {
        std::string iface           = "can0";
        double      cmd_period_s    = 0.010;  ///< 0x020 发送周期（100Hz）
        double      flags_period_s  = 0.100;  ///< 0x021 保活周期（10Hz）
        double      group_timeout_s = 0.005;  ///< 反馈组超时（F407 连续发，组内 ≤2ms）
        double      fb_timeout_s    = 0.100;  ///< 反馈超时 -> state().valid=0
        double      reconnect_s     = 1.0;    ///< socket 重开周期
        uint32_t    id_estop        = 0x000;
        uint32_t    id_speed        = 0x020;
        uint32_t    id_flags        = 0x021;
        uint32_t    id_fb_base      = 0x110;  ///< 0x110..0x114
    };

    explicit CanChassisLink(Config c);
    ~CanChassisLink() override;

    bool        open()  override;
    void        close() override;
    bool        isOpen() const override;
    std::string name()  const override;
    void        poll()  override;

    void                setCommand(const ChassisCommand& c) override;
    const ChassisState& state() const override;

    const ChassisLinkStats& stats() const { return st_; }

private:
    static double nowS();
    bool   tryOpen();
    void   drop(const std::string& why);
    bool   sendFrame(uint32_t id, const uint8_t* data, uint8_t dlc);
    void   sendSpeedFrame(uint8_t cmd);
    void   sendFlagsFrame();

    Config  cfg_;
    int     fd_ = -1;
    bool    opened_ = false;

    double  last_open_try_s_ = -1e9;
    double  last_cmd_s_      = 0.0;
    double  last_flags_s_    = 0.0;
    double  last_fb_s_       = -1e9;

    uint8_t flags_ = 0;
    uint8_t seq_   = 0;
    bool    estop_active_  = false;   ///< 当前是否处于急停
    bool    clear_pending_ = false;   ///< 里程计清零一次性标志

    ChassisCommand   cmd_{};
    ChassisState     state_{};
    ChassisLinkStats st_{};
    CanFbGroup       group_;
};

}  // namespace gimbal
