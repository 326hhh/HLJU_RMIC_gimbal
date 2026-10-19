/**
  ******************************************************************************
  * @file    unified_serial.hpp
  * @brief   单串口统一链路：底盘（F407 USART2 协议）与云台（ZDT 协议）共用一个 fd
  *
  * 硬件拓扑（三块 MCU 挂一条 CH340 总线）：
  *
  *   Jetson CH340 (USB-TTL)
  *        TX ──┬──────────────┬──────────────┬────────────┐
  *             │              │              │            │
  *        F407 USART2_RX   云台 ZDT yaw RX  云台 ZDT pitch RX   （接收）
  *        RX ──┬──────────────┬──────────────┬────────────┐
  *             │              │              │            │
  *        F407 USART2_TX   云台 yaw TX      云台 pitch TX     （发射，三机共线）
  *
  * 帧方向（两条线，互不干扰）：
  *   · Jetson TX -> 三机 RX：底盘命令帧（5A 5A 帧族，200Hz）+ ZDT 命令帧（12ms 节流）
  *   · 三机 TX -> Jetson RX：F407 反馈帧（100Hz）+ ZDT 返回帧（对命令的应答）
  *
  * 本类把「一个 fd」抽象成两个上层接口：
  *   · ITransport（云台）：send()/popFrameFor()/poll() —— 与原来 UartTransport 完全一致
  *   · IChassisLink（底盘）：setCommand()/state()/poll() —— 与原来 UartChassisLink 完全一致
  *
  * 关键设计：
  *   1. **RX 按帧边界分流**（RxDemux）：不是把原始字节同时喂给两个解析器（那样底盘
  *      反馈帧里的字节可能被 ZDT 解析器误拆成假帧、污染云台遥测），而是按
  *        `5A A5 81` + 40B + CRC8 + `0D`  -> 底盘解码器
  *        `Addr(1..32)` + Func + ... + `6B` -> ZDT 帧队列
  *      切流，剩余字节滚 1 字节重同步（与 F407 固件 ulink_parse_rx 同策略）。
  *   2. **发送静默窗调度**：F407 反馈 100Hz、每帧 40B 占线约 3.5ms，帧尾到下一帧头
  *      之间有约 6.5ms 空档。ZDT 命令尽量排在「刚收完一帧底盘反馈」之后 6ms 内发出，
  *      让 ZDT 的应答落在 F407 不发字的缝里，避免三机 TX 抢线。
  *      底盘 100ms 没帧（没接底盘/没上电）就退化为不限制。
  *   3. **节流**：ZDT 帧沿用 12ms 最小间隔（防 ZDT 粘包）；底盘命令固定 5ms（200Hz）。
  *
  * ⚠️ 接线前提：三机 TX 直接并线属于 TTL 多点（非 485），软件只能降低抢线概率，
  *    不能物理消除。实测若 CRC/sync 错误偏多，再加 485 收发器最稳。
  ******************************************************************************
  */
#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

#include "gimbal/link/chassis_link.hpp"
#include "gimbal/protocol/zdt_frame.hpp"
#include "gimbal/transport/itransport.hpp"

namespace gimbal {

// ===========================================================================
//  RxDemux —— 一条字节流上的帧分流器（纯函数逻辑，可单独自测）
// ===========================================================================
class RxDemux {
public:
    /// 喂入原始字节；内部按帧边界切分，把完整帧放进各自的队列
    void feed(const uint8_t* p, size_t n);

    /// 取出一帧底盘反馈；没有返回 false
    bool popChassis(ChassisState& s);
    /// 取出一帧 ZDT 返回帧；没有返回 false
    bool popZdt(zdt::Frame& f);

    size_t   pending()    const { return rx_.size(); }
    uint64_t chassisFrames() const { return chassis_ok_; }
    uint64_t chassisBad()    const { return chassis_bad_; }
    uint64_t zdtFrames()     const { return zdt_ok_; }
    uint64_t zdtBad()        const { return zdt_bad_; }
    void     clear();

private:
    static constexpr uint8_t kZdtAddrMin = 1;    // ZDT 地址范围：1..32
    static constexpr uint8_t kZdtAddrMax = 32;

    std::vector<uint8_t>     rx_;        // 接收缓冲（未消费字节）
    std::deque<ChassisState> chassis_;   // 已切出的底盘反馈帧
    std::deque<zdt::Frame>   zdt_;       // 已切出的 ZDT 返回帧
    uint64_t chassis_ok_ = 0;
    uint64_t chassis_bad_ = 0;
    uint64_t zdt_ok_ = 0;
    uint64_t zdt_bad_ = 0;
};

// ===========================================================================
//  UnifiedSerial —— 底盘 + 云台共用的单串口链路
// ===========================================================================
class UnifiedSerial : public ITransport, public IChassisLink {
public:
    struct Config {
        std::string device              = "/dev/ttyUSB0";
        int         baud                = 115200;
        int         zdt_gap_ms          = 12;      ///< ZDT 命令最小间隔（防粘包）
        double      chassis_tx_period_s = 0.005;   ///< 底盘命令节流（200Hz）
        double      chassis_timeout_s   = 0.3;     ///< 反馈超时 -> state().valid=0
        double      reconnect_s         = 0.5;     ///< 断线重连周期
        bool        zdt_after_fb        = true;    ///< ZDT 命令排在底盘反馈静默窗
    };

    explicit UnifiedSerial(Config c);
    ~UnifiedSerial() override;

    // ---- ITransport / IChassisLink（同名同签名，一份实现同时满足两个接口）----
    bool        open()  override;
    void        close() override;
    bool        isOpen() const override;
    std::string name() const override;
    void        poll() override;   // 收帧分流 + 节流发送 + 看门狗时效

    // ---- IChassisLink ----
    void                setCommand(const ChassisCommand& c) override;
    const ChassisState& state() const override;

    // ---- 诊断 ----
    const ChassisLinkStats& chassisStats() const { return chst_; }
    const RxDemux&          demux() const { return demux_; }

private:
    static int64_t nowUs();
    bool  tryOpen();
    void  drop(const std::string& why);
    bool  writeAll(const std::vector<uint8_t>& out);
    void  sendChassis();
    /// ZDT 帧现在能不能发（静默窗调度）
    bool  zdtWindowOpen(int64_t now_us) const;

    Config    cfg_;
    int       fd_ = -1;
    int64_t   last_poll_us_       = 0;
    int64_t   last_open_us_       = -1000000000000LL;
    int64_t   last_chassis_rx_us_ = -1000000000000LL;
    int64_t   last_chassis_tx_us_ = 0;
    uint8_t   rbuf_[1024];

    RxDemux         demux_;         ///< 收帧分流器
    ChassisCommand  cmd_{};         ///< 最新底盘命令
    ChassisState    state_{};       ///< 最新底盘状态（反馈解码）
    bool            clear_pending_ = false;  ///< 里程计清零一次性标志
    uint8_t         tx_seq_ = 0;    ///< 底盘命令帧序号
    ChassisLinkStats chst_{};       ///< 底盘链路统计
};

}  // namespace gimbal
