// ===========================================================================
//  FakeTransport —— 纯软件模拟的 ZDT 总线（可挂多台电机）
//
//  用途: --dry / --selftest。**不需要任何硬件**就能验证：
//        帧构造是否正确 -> 解析器是否能切帧 -> 地址路由是否对
//        -> 角度/转速换算是否正确 -> 斜坡与限位逻辑是否符合预期
//
//  它走的是和真实传输层完全相同的代码路径：
//      上层 send() -> txq_ -> poll() 取出 -> 生成响应 -> feedRx() -> Parser
//  唯一被替换掉的只有「OS 串口/CAN 读写」这一层。
//
//  一台 FakeTransport 可以模拟挂在同一总线上的多台电机（按地址区分），
//  这样能顺带验证 popFrameFor() 的地址路由。
// ===========================================================================
#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "gimbal/transport/itransport.hpp"

namespace gimbal {

class FakeTransport : public ITransport {
public:
    struct Sim {
        bool    enabled    = false;
        double  rpm        = 0.0;
        double  target_rpm = 0.0;
        double  accel_rpm_per_s = 4000.0;
        double  pos_deg    = 0.0;   ///< 单圈 0..360
        int     vbus_mv    = 11200;
        int     temp_c     = 27;
        uint8_t flags      = 0x00;
        bool    lim_left   = false;
        bool    lim_right  = false;
        uint64_t tx_seen   = 0;     ///< 收到的命令条数（自测断言用）
        uint64_t get_seen  = 0;     ///< 收到的读取命令条数
    };

    FakeTransport() {
        stats_.connected = true;
        setGapUs(2000);
        last_ = std::chrono::steady_clock::now();
    }

    void addMotor(uint8_t addr) {
        if (motors_.find(addr) == motors_.end()) motors_[addr] = Sim{};
    }

    Sim&       sim(uint8_t addr) { return motors_[addr]; }
    const Sim& sim(uint8_t addr) const { return motors_.at(addr); }
    const std::map<uint8_t, Sim>& motors() const { return motors_; }

    bool        open()  override { stats_.connected = true; return true; }
    void        close() override { stats_.connected = false; }
    bool        isOpen() const override { return true; }
    std::string name() const override { return "fake"; }

    void poll() override {
        using namespace std::chrono;
        const auto now = steady_clock::now();
        double     dt  = duration<double>(now - last_).count();
        last_ = now;
        if (dt <= 0.0) return;
        if (dt > 0.5) dt = 0.5;

        for (auto& kv : motors_) step(kv.second, dt);

        std::vector<uint8_t> f;
        while (nextTx(f)) {
            handleTx(f);
            onTxDone();
        }
    }

private:
    static uint32_t degToRaw(double deg) {
        double d = std::fmod(deg, 360.0);
        if (d < 0) d += 360.0;
        return static_cast<uint32_t>(d * 65536.0 / 360.0) & 0xFFFFu;
    }

    static void step(Sim& s, double dt) {
        const double max_step = s.accel_rpm_per_s * dt;
        double       d        = s.target_rpm - s.rpm;
        if (d > max_step) d = max_step;
        if (d < -max_step) d = -max_step;
        s.rpm += d;
        if (!s.enabled) { s.rpm = 0.0; s.target_rpm = 0.0; }

        s.pos_deg = std::fmod(s.pos_deg + s.rpm * 6.0 * dt, 360.0);
        if (s.pos_deg < 0) s.pos_deg += 360.0;

        s.flags = static_cast<uint8_t>(s.enabled ? zdt::st::Enabled : 0);
        if (s.enabled && std::fabs(s.rpm) < 1.0) s.flags |= zdt::st::InPos;
        if (s.lim_right) s.flags |= zdt::st::LimRight;
        if (s.lim_left)  s.flags |= zdt::st::LimLeft;
    }

    void handleTx(const std::vector<uint8_t>& f) {
        if (f.size() < 3) return;
        const uint8_t addr = f[0];
        const uint8_t func = f[1];
        if (addr == zdt::ADDR_BROADCAST) return;   // 广播不回（简化）

        auto it = motors_.find(addr);
        if (it == motors_.end()) return;           // 总线上没这台电机
        Sim& s = it->second;
        ++s.tx_seen;

        std::vector<uint8_t> rx;
        rx.push_back(addr);
        rx.push_back(func);

        switch (static_cast<zdt::Func>(func)) {
            case zdt::Func::Enable:
                s.enabled = (f.size() > 3 && f[3] != 0);
                if (!s.enabled) { s.target_rpm = 0.0; s.rpm = 0.0; }
                rx.push_back(0x02);
                break;

            case zdt::Func::Speed: {
                if (f.size() < 5) return;
                const bool     ccw = (f[2] != 0);
                const uint16_t r   = static_cast<uint16_t>((f[3] << 8) | f[4]);
                // 约定：ccw(=1) 对应逻辑正方向、正转速
                s.target_rpm = ccw ? static_cast<double>(r) : -static_cast<double>(r);
                rx.push_back(0x02);
                break;
            }

            case zdt::Func::EStop:
                s.target_rpm = 0.0;
                s.rpm        = 0.0;
                rx.push_back(0x02);
                break;

            case zdt::Func::ZeroPos:
                s.pos_deg = 0.0;
                rx.push_back(0x02);
                break;

            case zdt::Func::ClearStall:
                s.flags &= static_cast<uint8_t>(~zdt::st::StallProt);
                rx.push_back(0x02);
                break;

            case zdt::Func::GetSpeed: {
                ++s.get_seen;
                const double   v = s.rpm;
                const double   a = std::fabs(v) > 3000.0 ? 3000.0 : std::fabs(v);
                const uint16_t m = static_cast<uint16_t>(std::lround(a));
                rx.push_back(static_cast<uint8_t>(v < -0.5 ? 1 : 0));
                rx.push_back(static_cast<uint8_t>((m >> 8) & 0xFF));
                rx.push_back(static_cast<uint8_t>(m & 0xFF));
                break;
            }

            case zdt::Func::GetPos: {
                ++s.get_seen;
                const uint32_t p = degToRaw(s.pos_deg);
                rx.push_back(0x00);
                rx.push_back(static_cast<uint8_t>((p >> 24) & 0xFF));
                rx.push_back(static_cast<uint8_t>((p >> 16) & 0xFF));
                rx.push_back(static_cast<uint8_t>((p >> 8) & 0xFF));
                rx.push_back(static_cast<uint8_t>(p & 0xFF));
                break;
            }

            case zdt::Func::GetStatus:
                ++s.get_seen;
                rx.push_back(s.flags);
                break;

            case zdt::Func::GetVbus:
                ++s.get_seen;
                rx.push_back(static_cast<uint8_t>((s.vbus_mv >> 8) & 0xFF));
                rx.push_back(static_cast<uint8_t>(s.vbus_mv & 0xFF));
                break;

            case zdt::Func::GetTemp:
                ++s.get_seen;
                rx.push_back(0x00);
                rx.push_back(static_cast<uint8_t>(s.temp_c & 0xFF));
                break;

            default:
                return;   // 未仿真的命令不回，正好测超时路径
        }

        rx.push_back(zdt::CHECKSUM_FIXED);
        feedRx(rx.data(), rx.size());
    }

    std::map<uint8_t, Sim>                motors_;
    std::chrono::steady_clock::time_point last_;
};

}  // namespace gimbal
