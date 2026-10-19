// ===========================================================================
//  gimbal —— 麦轮小车云台控制器（正式工程）
//
//  范围: 只做云台（两个 ZDT 闭环步进）
//            yaw   ID = 2   （导电滑环，360° 连续旋转，无机械限位）
//            pitch ID = 1
//        细分 16 -> 3200 脉冲/电机圈
//        传动比 2:1（电机转 2 圈 = 输出轴 1 圈）
//
//  控制入口:
//    ① UDP 40001  —— PC 鼠标客户端（主通道）
//    ② stdin      —— 本地调试命令
//
//  用法:
//      gimbal                          # 真机
//      gimbal --dry                    # 纯软件模拟
//      gimbal --selftest               # 自检后退出
//      gimbal --device /dev/ttyTHS1 --gear 2.0 --ctrl-port 40001
//
//  本地命令（一行一条）:
//      y <dps> / p <dps> / r <y> <p>   设角速度
//      h 停手   home 回中   zero 归零   en 0|1   x 急停   s 状态   q 退出
// ===========================================================================
#include <fcntl.h>
#include <signal.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "gimbal/device/zdt_motor.hpp"
#include "gimbal/gimbal/gimbal.hpp"
#include "gimbal/link/chassis_link.hpp"
#include "gimbal/link/control_link.hpp"
#include "gimbal/link/udp_control_link.hpp"
#include "gimbal/protocol/zdt_frame.hpp"
#include "gimbal/transport/fake_transport.hpp"
#include "gimbal/transport/itransport.hpp"
#include "gimbal/transport/uart_transport.hpp"

using namespace gimbal;

static std::atomic<bool> g_run{true};
static void onSignal(int) { g_run = false; }

// ---------------------------------------------------------------------------
struct Options {
    std::string device     = "/dev/ttyUSB0";
    int         baud       = 115200;
    uint8_t     yaw_addr   = 2;          ///< 用户指定
    uint8_t     pitch_addr = 1;          ///< 用户指定
    bool        dry        = false;
    bool        selftest   = false;
    double      loop_hz    = 100.0;

    /// 传动比 = 电机转数 / 输出轴转数（2:1 减速填 2.0）
    double      gear       = 2.0;
    /// 脉冲/【电机】圈 = 200 × 细分（细分 16 -> 3200）
    uint32_t    pulses_per_motor_rev = 3200;

    double      yaw_min = -180.0, yaw_max = 180.0,  yaw_rate = 180.0;
    double      pitch_min = -45.0, pitch_max = 45.0, pitch_rate = 120.0;
    bool        yaw_no_limit = true;     ///< 导电滑环：yaw 无机械限位

    int         acc        = 200;
    uint16_t    ctrl_port  = CTRL_DEFAULT_PORT;
    bool        no_udp     = false;

    /// 底盘链路（F407 USART2）：第二个 USB-TTL 默认 ttyUSB1
    std::string chassis_device = "/dev/ttyUSB1";
    int         chassis_baud   = 115200;
    bool        no_chassis     = false;
};

static void usage(const char* a0) {
    std::printf(
        "用法: %s [选项]\n"
        "  --device <路径>        串口设备 (默认 /dev/ttyUSB0)\n"
        "  --baud <n>             波特率 (默认 115200)\n"
        "  --yaw-addr <n>         yaw 电机地址 (默认 2)\n"
        "  --pitch-addr <n>       pitch 电机地址 (默认 1)\n"
        "  --gear <f>             传动比=电机转数/输出转数 (默认 2.0)\n"
        "  --pulses <n>           脉冲/电机圈 = 200×细分 (默认 3200)\n"
        "  --yaw-limit <a> <b>    给 yaw 启用软限位 (默认关闭)\n"
        "  --pitch-limit <a> <b>  pitch 软限位 (默认 -45 45)\n"
        "  --yaw-rate <dps>       yaw 最大角速度 (默认 180)\n"
        "  --pitch-rate <dps>     pitch 最大角速度 (默认 120)\n"
        "  --acc <0-255>          加速度档位 (默认 200)\n"
        "  --ctrl-port <n>        UDP 控制口 (默认 40001)\n"
        "  --no-udp               不开 UDP（只用本地 stdin）\n"
        "  --hz <n>               控制频率 (默认 100)\n"
        "  --chassis-dev <路径>   底盘串口 (默认 /dev/ttyUSB1)\n"
        "  --chassis-baud <n>     底盘波特率 (默认 115200)\n"
        "  --no-chassis           不接底盘（只控云台）\n"
        "  --dry                  纯软件模拟\n"
        "  --selftest             自检后退出\n"
        "  -h, --help\n", a0);
}

// ===========================================================================
// 自检
// ===========================================================================
static int g_fail = 0;
#define CHECK(cond, msg)                                                   \
    do {                                                                   \
        const bool _ok_ = (cond);   /* 只求值一次，避免副作用被执行两遍 */  \
        std::cout << (_ok_ ? "  [ OK ] " : "  [FAIL] ") << msg << "\n";    \
        if (!_ok_) ++g_fail;                                               \
    } while (0)

static std::string hex(const std::vector<uint8_t>& v) {
    char        buf[8];
    std::string s;
    for (auto b : v) {
        std::snprintf(buf, sizeof(buf), "%02X ", b);
        s += buf;
    }
    if (!s.empty()) s.pop_back();
    return s;
}

static int runSelftest() {
    std::printf("=== gimbal 自检（纯软件，无硬件）===\n\n");

    std::printf("[1] 帧构造\n");
    {
        auto f = zdt::mkEnable(2, true);
        CHECK(hex(f) == "02 F3 AB 01 00 6B", "使能帧 yaw(addr=2): " + hex(f));
        f = zdt::mkSpeed(2, 300, /*ccw=*/true, 200);
        CHECK(hex(f) == "02 F6 01 01 2C C8 00 6B", "速度帧 300RPM: " + hex(f));
        f = zdt::mkEStop(1);
        CHECK(hex(f) == "01 FE 98 00 6B", "急停帧 pitch(addr=1): " + hex(f));
        f = zdt::mkGet(1, zdt::Func::GetStatus);
        CHECK(hex(f) == "01 3A 6B", "读状态帧: " + hex(f));
        f = zdt::mkPositionEmm(2, 6400, 300, 200, zdt::MoveMode::AbsoluteFromZero, true);
        CHECK(f.size() == 13, "Emm 位置帧长度 13");
    }

    std::printf("\n[2] 解析器切帧 + 校验\n");
    {
        zdt::Parser p;
        std::vector<uint8_t> s = {0xFF, 0x00, 0x7A};
        const uint8_t f1[] = {1, 0x3A, 0x03, 0x6B};
        const uint8_t f2[] = {2, 0x35, 0x00, 0x01, 0x2C, 0x6B};
        s.insert(s.end(), f1, f1 + sizeof(f1));
        s.insert(s.end(), f2, f2 + sizeof(f2));
        p.feed(s.data(), s.size());
        zdt::Frame a, b;
        CHECK(p.pop(a) && a.addr == 1 && a.func == 0x3A && a.data[0] == 0x03,
              "切出 pitch 状态帧");
        CHECK(p.pop(b) && b.addr == 2 && b.func == 0x35, "切出 yaw 转速帧");
        CHECK(!p.pop(a), "无多余帧");
    }

    std::printf("\n[3] 角度 <-> 脉冲（细分 16）\n");
    {
        ZdtMotor::Config c;
        c.pulses_per_rev = 3200;
        auto ft = std::make_shared<FakeTransport>();
        ft->addMotor(1);
        ZdtMotor m(ft, c);
        CHECK(m.degToPulses(360.0) == 3200, "360° -> 3200 脉冲");
        CHECK(m.degToPulses(90.0) == 800, "90°  -> 800 脉冲");
    }

    std::printf("\n[4] UDP 线协议编解码（两端必须逐字节一致）\n");
    {
        ControlCommand c;
        c.seq       = 12345;
        c.yaw_dps   = 90.0;
        c.pitch_dps = -45.5;
        c.flags     = ctrl_flag::Enable | ctrl_flag::Home;
        const auto b = c.encode();
        CHECK(b.size() == CTRL_UP_LEN, "上行包长 16 字节");

        ControlCommand d;
        CHECK(ControlCommand::decode(b.data(), b.size(), d), "上行解码成功");
        CHECK(d.seq == 12345, "seq 往返一致");
        CHECK(std::fabs(d.yaw_dps - 90.0) < 1e-6, "yaw 角速度往返一致");
        CHECK(std::fabs(d.pitch_dps + 45.5) < 1e-6, "pitch 角速度往返一致");
        CHECK(d.flags == c.flags, "flags 往返一致");

        auto bad = b;
        bad[13] ^= 0xFF;
        CHECK(!ControlCommand::decode(bad.data(), bad.size(), d), "校验和错误被拒");
        bad = b;
        bad[0] = 0;
        CHECK(!ControlCommand::decode(bad.data(), bad.size(), d), "magic 错误被拒");
        std::vector<uint8_t> trunc(b.begin(), b.begin() + 8);
        CHECK(!ControlCommand::decode(trunc.data(), trunc.size(), d), "短包被拒");

        ControlTelemetry t;
        t.echo_seq = 777;  t.yaw_dps = 12.34;  t.yaw_rpm = -15.5;
        t.yaw_deg  = -1234.56; t.vbus_mv = 11230; t.temp_c = 27;
        t.status   = ctrl_status::Enabled | ctrl_status::YawOk;
        t.rx_hz    = 100;
        const auto tb = t.encode();
        CHECK(tb.size() == CTRL_DOWN_LEN, "下行包长 32 字节");
        ControlTelemetry u;
        CHECK(ControlTelemetry::decode(tb.data(), tb.size(), u), "下行解码成功");
        CHECK(u.echo_seq == 777 && std::fabs(u.yaw_rpm + 15.5) < 1e-6 &&
                  std::fabs(u.yaw_deg + 1234.56) < 0.01 && u.vbus_mv == 11230 &&
                  u.temp_c == 27 && u.rx_hz == 100,
              "下行各字段往返一致");
    }

    std::printf("\n[5] 端到端：一总线双电机 + 传动比 2:1\n");
    {
        auto bus = std::make_shared<FakeTransport>();
        bus->addMotor(1);
        bus->addMotor(2);

        ZdtMotor::Config yc; yc.addr = 2; yc.poll_speed_ms = 50; yc.poll_pos_ms = 50;
        ZdtMotor::Config pc; pc.addr = 1; pc.poll_speed_ms = 50; pc.poll_pos_ms = 50;
        auto yaw_m   = std::make_shared<ZdtMotor>(bus, yc);
        auto pitch_m = std::make_shared<ZdtMotor>(bus, pc);

        Gimbal::Config gc;
        gc.yaw.name = "yaw";       gc.yaw.gear_ratio = 2.0;
        gc.yaw.use_soft_limits = false;              // 导电滑环：无机械限位
        gc.yaw.max_rate_dps = 180;
        gc.pitch.name = "pitch";   gc.pitch.gear_ratio = 2.0;
        gc.pitch.min_deg = -45;    gc.pitch.max_deg = 45;  gc.pitch.max_rate_dps = 120;
        gc.enable_delay_s = 0.05;
        gc.command_timeout_s = 0.4;

        Gimbal g(yaw_m, pitch_m, gc);
        CHECK(g.transports().size() == 1, "两轴共总线 -> 传输层去重为 1");

        const double DT = 0.004;
        auto tick = [&](int n) {
            for (int i = 0; i < n; ++i) {
                g.step(DT);
                std::this_thread::sleep_for(
                    std::chrono::microseconds(static_cast<int>(DT * 1e6)));
            }
        };

        tick(40);
        for (int i = 0; i < 150; ++i) {
            g.setRate(90.0, 45.0);
            g.step(DT);
            std::this_thread::sleep_for(
                std::chrono::microseconds(static_cast<int>(DT * 1e6)));
        }

        const auto& ys = bus->sim(2);
        const auto& ps = bus->sim(1);
        std::printf("      传动比 2:1: yaw 90°/s -> 电机 %.1f RPM (期望 30.0)\n", ys.rpm);
        std::printf("                  pitch 45°/s -> 电机 %.1f RPM (期望 15.0)\n", ps.rpm);
        CHECK(std::fabs(ys.rpm - 30.0) < 1.5, "yaw 传动比换算正确 (90°/s -> 30 RPM)");
        CHECK(std::fabs(ps.rpm - 15.0) < 1.0, "pitch 传动比换算正确 (45°/s -> 15 RPM)");
        CHECK(ys.enabled && ps.enabled, "两轴都已使能");

        // 停发指令 0.6s（> command_timeout_s=0.4）。注意不能调 hold()，
        // 那会刷新指令时钟，反而让看门狗不触发。
        tick(150);
        CHECK(g.watchdogTripped(), "操作端断流 -> 看门狗触发");
        CHECK(std::fabs(bus->sim(2).rpm) < 1.0, "看门狗后 yaw 停转");
        CHECK(bus->sim(2).enabled, "看门狗**未失能**（云台需抱死）");

        g.enable(true);
        tick(40);
        for (int i = 0; i < 400; ++i) {
            g.setRate(0.0, 120.0);
            g.step(DT);
            std::this_thread::sleep_for(
                std::chrono::microseconds(static_cast<int>(DT * 1e6)));
        }
        std::printf("      pitch 顶限位后输出角估计 = %.2f° (上限 45)\n",
                    g.pitch().estAngleDeg());
        CHECK(g.pitch().estAngleDeg() <= 50.0, "pitch 软限位生效");
        CHECK(std::fabs(g.yaw().estAngleDeg()) > 0.0, "yaw 关软限位后可持续转");

        g.estop();
        tick(20);
        CHECK(!bus->sim(1).enabled && !bus->sim(2).enabled, "急停后两轴失能");
    }

    std::printf("\n[6] 底盘协议帧（与 F407 逐字节一致，向量由 Python 参考实现生成）\n");
    {
        ChassisCommand c;
        c.vx_mmps   = 500;
        c.vy_mmps   = -200;
        c.wz_mradps = 1000;              // 1.0 rad/s -> 帧里编码 100 (0.01 rad/s)
        c.enable    = true;
        c.seq       = 1;
        uint8_t f[CHASSIS_CMD_LEN];
        chassisEncodeCmd(f, c);
        const std::string cmd_hex = hex(std::vector<uint8_t>(f, f + CHASSIS_CMD_LEN));
        CHECK(cmd_hex == "A5 5A 01 01 F4 01 38 FF 64 00 01 16", "命令帧编码: " + cmd_hex);

        static const uint8_t fb[CHASSIS_FB_LEN] = {
            0x5A, 0xA5, 0x81, 0x21, 0x24, 0xFA, 0xFF, 0xFF, 0xC4, 0x09, 0x00,
            0x00, 0xD8, 0xDC, 0xF4, 0x01, 0x38, 0xFF, 0x5E, 0x01, 0xE0, 0x2E,
            0x06, 0xFF, 0x13, 0x00, 0xD2, 0x04, 0x4D, 0x00, 0xB0, 0x04, 0xE0,
            0xFC, 0x2C, 0x01, 0x00, 0x00, 0xFE, 0x0D};
        ChassisState st;
        CHECK(chassisDecodeFb(fb, st), "反馈帧解码成功");
        CHECK(st.odom_x_mm == -1500 && st.odom_y_mm == 2500, "里程计 x/y");
        CHECK(std::fabs(st.odom_yaw_deg + 90.0) < 1e-9, "里程计 yaw = -90°");
        CHECK(st.vx_mmps == 500 && st.vy_mmps == -200, "正解 vx/vy");
        CHECK(std::fabs(st.wz_mradps - 3500.0) < 1e-9, "正解 wz = 3.5 rad/s");
        CHECK(std::fabs(st.gyro_yaw_deg - 120.0) < 1e-9, "陀螺 yaw = 120°");
        CHECK(st.status == (chassis_status::ImuOnline | chassis_status::LinkOk |
                            chassis_status::DriverOn),
              "status 位");
        CHECK(st.cmd_cnt == 1234 && st.fb_seq == 77, "计数字段");
        CHECK(std::fabs(st.wheel_rpm[0] - 12.0) < 1e-9 &&
                  std::fabs(st.wheel_rpm[1] + 8.0) < 1e-9 &&
                  std::fabs(st.wheel_rpm[2] - 3.0) < 1e-9 &&
                  std::fabs(st.wheel_rpm[3]) < 1e-9,
              "四轮转速解码");

        uint8_t bad[CHASSIS_FB_LEN];
        std::memcpy(bad, fb, sizeof(bad));
        bad[20] ^= 0xFF;
        ChassisState sb;
        CHECK(!chassisDecodeFb(bad, sb), "反馈帧单字节破坏被拒");

        // 急停帧命令字必须变成 0x02
        ChassisCommand e;
        e.estop = true;
        chassisEncodeCmd(f, e);
        CHECK(f[2] == chassis_cmd::EStop && f[10] == 0, "急停帧 cmd=0x02 且不带使能");
    }

    std::printf("\n=== 自检结束: %s（失败 %d 项）===\n",
                g_fail == 0 ? "全部通过" : "有失败", g_fail);
    return g_fail == 0 ? 0 : 1;
}

// ===========================================================================
// 运行时
// ===========================================================================
static void printStatus(const Gimbal& g, const ITransport& bus,
                        const UdpControlLink* link, const IChassisLink* chassis) {
    auto line = [](const Axis& a) {
        const auto& s = a.motorStatus();
        std::printf("  %-6s 指令=%7.1f°/s 实际=%7.1f°/s 输出角=%9.2f° "
                    "电机=%6.1fRPM 母线=%5.2fV %2d°C flags=0x%02X%s\n",
                    a.cfg().name.c_str(), a.cmdRateDps(), a.actRateDps(),
                    a.estAngleDeg(), s.speed_rpm, s.vbus_mv / 1000.0, s.temp_c,
                    static_cast<unsigned>((s.enabled ? 0x01 : 0) |
                                          (s.in_position ? 0x02 : 0) |
                                          (s.stalled ? 0x04 : 0) |
                                          (s.lim_left ? 0x10 : 0) |
                                          (s.lim_right ? 0x20 : 0)),
                    a.atSoftLimit() ? "  [限位]" : "");
    };
    std::printf("--- 状态 ---\n");
    line(g.yaw());
    line(g.pitch());
    const auto& st = bus.stats();
    std::printf("  电机总线: %-18s 连接=%d 发送=%llu 丢弃=%llu 收帧=%llu 重连=%llu\n",
                bus.name().c_str(), st.connected ? 1 : 0,
                static_cast<unsigned long long>(st.tx_frames),
                static_cast<unsigned long long>(st.tx_dropped),
                static_cast<unsigned long long>(st.rx_frames),
                static_cast<unsigned long long>(st.reconnects));
    if (link) {
        const auto& ls = link->stats();
        std::printf("  操作端  : %-18s 收=%llu 坏=%llu 丢=%llu @%dHz 时效=%.2fs\n",
                    ls.peer.empty() ? "(等待连接)" : ls.peer.c_str(),
                    static_cast<unsigned long long>(ls.rx_ok),
                    static_cast<unsigned long long>(ls.rx_bad),
                    static_cast<unsigned long long>(ls.rx_lost),
                    ls.rx_hz, ls.age_s);
    }
    if (g.watchdogTripped())
        std::printf("  看门狗已触发（操作端 %.1fs 无指令）\n", g.sinceCommandS());
    if (chassis) {
        const auto& cs = chassis->state();
        std::printf("  底盘    : %-18s 有效=%d 位姿=(%.0f,%.0f)mm yaw=%.1f° "
                    "v=(%.0f,%.0f)mm/s wz=%.2frad/s 陀螺yaw=%.1f° st=0x%02X 时效=%.2fs\n",
                    chassis->name().c_str(), cs.valid ? 1 : 0, cs.odom_x_mm,
                    cs.odom_y_mm, cs.odom_yaw_deg, cs.vx_mmps, cs.vy_mmps,
                    cs.wz_mradps / 1000.0, cs.gyro_yaw_deg,
                    static_cast<unsigned>(cs.status & 0xFF), cs.age_s);
        std::printf("  底盘轮速: FL %+6.1f FR %+6.1f BL %+6.1f BR %+6.1f RPM\n",
                    cs.wheel_rpm[0], cs.wheel_rpm[1], cs.wheel_rpm[2],
                    cs.wheel_rpm[3]);
    }
}

static ControlTelemetry buildTelemetry(const Gimbal& g, uint32_t echo_seq, int rx_hz,
                                       const IChassisLink* chassis) {
    ControlTelemetry t;
    t.echo_seq  = echo_seq;
    t.yaw_dps   = g.yaw().actRateDps();
    t.pitch_dps = g.pitch().actRateDps();
    t.yaw_rpm   = g.yaw().motorStatus().speed_rpm;
    t.pitch_rpm = g.pitch().motorStatus().speed_rpm;
    t.yaw_deg   = g.yaw().estAngleDeg();
    t.pitch_deg = g.pitch().estAngleDeg();
    t.vbus_mv   = g.yaw().motorStatus().vbus_mv;
    t.temp_c    = g.yaw().motorStatus().temp_c;
    t.rx_hz     = rx_hz;

    uint8_t s = 0;
    if (g.enabled())                    s |= ctrl_status::Enabled;
    if (g.yaw().motorStatus().valid)    s |= ctrl_status::YawOk;
    if (g.pitch().motorStatus().valid)  s |= ctrl_status::PitchOk;
    if (g.watchdogTripped())            s |= ctrl_status::Watchdog;
    if (g.yaw().atSoftLimit())          s |= ctrl_status::YawLim;
    if (g.pitch().atSoftLimit())        s |= ctrl_status::PitchLim;
    if (g.yaw().motorStatus().stalled || g.pitch().motorStatus().stalled ||
        g.yaw().motorStatus().stall_prot || g.pitch().motorStatus().stall_prot)
        s |= ctrl_status::Stall;
    t.status = s;

    if (chassis) {
        const auto& cs = chassis->state();
        t.chassis_x_mm            = cs.odom_x_mm;
        t.chassis_y_mm            = cs.odom_y_mm;
        t.chassis_yaw_deg         = cs.odom_yaw_deg;
        t.chassis_vx_mmps         = cs.vx_mmps;
        t.chassis_vy_mmps         = cs.vy_mmps;
        t.chassis_wz_mradps       = cs.wz_mradps;
        t.chassis_gyro_yaw_deg    = cs.gyro_yaw_deg;
        t.chassis_gyro_wz_mradps  = cs.gyro_wz_mradps;
        t.chassis_status          = static_cast<uint8_t>(cs.status & 0xFF);
        t.chassis_valid           = cs.valid;
        for (int i = 0; i < 4; ++i) t.chassis_wheel_rpm[i] = cs.wheel_rpm[i];
    }
    return t;
}

static int runMain(const Options& o) {
    // ---- 电机总线 ----
    std::shared_ptr<ITransport> bus;
    if (o.dry) {
        auto f = std::make_shared<FakeTransport>();
        f->addMotor(o.yaw_addr);
        f->addMotor(o.pitch_addr);
        bus = f;
        std::printf("[main] DRY：FakeTransport（无硬件）\n");
    } else {
        UartTransport::Config uc;
        uc.device = o.device;
        uc.baud   = o.baud;
        bus       = std::make_shared<UartTransport>(uc);
        if (!bus->open()) {
            std::printf("[main] 打开 %s 失败：%s\n", o.device.c_str(),
                        bus->stats().last_error.c_str());
            std::printf("       提示：用 --dry 可在无硬件下先跑通逻辑\n");
            return 1;
        }
    }

    // ---- 两台电机 ----
    ZdtMotor::Config yc;
    yc.addr           = o.yaw_addr;
    yc.pulses_per_rev = o.pulses_per_motor_rev;
    yc.acc_default    = static_cast<uint8_t>(o.acc);
    ZdtMotor::Config pc = yc;
    pc.addr = o.pitch_addr;

    auto yaw_motor   = std::make_shared<ZdtMotor>(bus, yc);
    auto pitch_motor = std::make_shared<ZdtMotor>(bus, pc);

    // ---- 云台 ----
    Gimbal::Config gc;
    gc.yaw.name            = "yaw";
    gc.yaw.gear_ratio      = o.gear;
    gc.yaw.max_rate_dps    = o.yaw_rate;
    gc.yaw.use_soft_limits = !o.yaw_no_limit;      // 导电滑环：默认无限位
    gc.yaw.min_deg         = o.yaw_min;
    gc.yaw.max_deg         = o.yaw_max;

    gc.pitch.name          = "pitch";
    gc.pitch.gear_ratio    = o.gear;
    gc.pitch.min_deg       = o.pitch_min;
    gc.pitch.max_deg       = o.pitch_max;
    gc.pitch.max_rate_dps  = o.pitch_rate;
    gc.auto_enable         = true;

    Gimbal gimbal(yaw_motor, pitch_motor, gc);

    // ---- 底盘链路（USART2 -> F407）----
    std::unique_ptr<IChassisLink> chassis;
    bool chassis_open_failed = false;
    if (!o.no_chassis) {
        UartChassisLink::Config cc;
        cc.device = o.chassis_device;
        cc.baud   = o.chassis_baud;
        auto ul = std::make_unique<UartChassisLink>(cc);
        if (!ul->open()) {
            std::printf("[main] 底盘串口 %s 打开失败：%s\n",
                        o.chassis_device.c_str(), ul->stats().last_error.c_str());
            std::printf("       云台照常工作；插上串口后会自动重连\n");
            chassis_open_failed = true;
        }
        chassis = std::move(ul);
    }

    // ---- UDP 操作端链路 ----
    std::unique_ptr<UdpControlLink> link;
    if (!o.no_udp) {
        UdpControlLink::Config lc;
        lc.port = o.ctrl_port;
        link.reset(new UdpControlLink(lc));
        if (!link->open()) {
            std::printf("[main] UDP 控制口打开失败，降级为仅本地 stdin\n");
            link.reset();
        }
    }

    signal(SIGINT, onSignal);
    signal(SIGTERM, onSignal);

    const int fl = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, fl | O_NONBLOCK);

    std::printf("========================================================\n");
    std::printf("  gimbal 云台控制器\n");
    std::printf("  电机总线: %s\n", bus->name().c_str());
    std::printf("  yaw     : addr=%u 传动比 %.2f %s 最大 %.0f°/s\n",
                o.yaw_addr, o.gear,
                o.yaw_no_limit ? "无限位(导电滑环)" : "有软限位", o.yaw_rate);
    std::printf("  pitch   : addr=%u 传动比 %.2f 限位[%.0f, %.0f] 最大 %.0f°/s\n",
                o.pitch_addr, o.gear, o.pitch_min, o.pitch_max, o.pitch_rate);
    std::printf("  脉冲/电机圈=%u  加速度档位=%d\n", o.pulses_per_motor_rev, o.acc);
    std::printf("  控制频率: %.0f Hz   看门狗: %.1f s\n", o.loop_hz,
                gc.command_timeout_s);
    if (link) {
        std::printf("  操作端  : UDP 0.0.0.0:%u\n", o.ctrl_port);
    } else {
        std::printf("  操作端  : 仅本地 stdin\n");
    }
    if (chassis) {
        std::printf("  底盘    : %s%s\n", chassis->name().c_str(),
                    chassis_open_failed ? "（未连上，自动重连中）" : "");
    } else {
        std::printf("  底盘    : 未接入（--no-chassis）\n");
    }
    std::printf("--------------------------------------------------------\n");
    std::printf("  本地命令: y/p/r 角速度  h 停手  home 回中  zero 归零\n");
    std::printf("            en 0|1 使能  x 急停  s 状态  q 退出\n");
    if (chassis)
        std::printf("  底盘命令: c <vx> <vy> <wz>（mm/s, mm/s, mrad/s）  cx 底盘急停\n");
    std::printf("========================================================\n");

    const double dt_target = 1.0 / o.loop_hz;
    auto         t_next    = std::chrono::steady_clock::now();

    std::string inbuf;
    double      local_yaw = 0.0, local_pitch = 0.0;
    bool        local_has = false;
    uint32_t    echo_seq  = 0;

    // ---- 底盘状态 ----
    double local_cvx = 0.0, local_cvy = 0.0, local_cwz = 0.0;
    bool   local_c_has = false;
    bool   local_c_estop = false;
    ChassisCommand udp_chassis{};
    double last_udp_chassis_s = -1e9;
    bool   udp_chassis_seen = false;
    auto now_s = []() {
        return std::chrono::duration<double>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    };

    while (g_run) {
        // ---- 1. 本地 stdin ----
        char c;
        while (::read(STDIN_FILENO, &c, 1) == 1) {
            if (c != '\n') {
                inbuf.push_back(c);
                if (inbuf.size() > 128) inbuf.clear();
                continue;
            }
            std::istringstream iss(inbuf);
            std::string        cmd;
            iss >> cmd;
            if (cmd == "y")      { iss >> local_yaw; local_has = true; std::printf("[cmd] yaw=%.1f\n", local_yaw); }
            else if (cmd == "p") { iss >> local_pitch; local_has = true; std::printf("[cmd] pitch=%.1f\n", local_pitch); }
            else if (cmd == "r") { iss >> local_yaw >> local_pitch; local_has = true;
                                   std::printf("[cmd] yaw=%.1f pitch=%.1f\n", local_yaw, local_pitch); }
            else if (cmd == "h" || cmd == "hold") { gimbal.hold(); local_has = false;
                                   local_yaw = local_pitch = 0; std::printf("[cmd] hold\n"); }
            else if (cmd == "home") { gimbal.home(); std::printf("[cmd] home\n"); }
            else if (cmd == "zero") { gimbal.yaw().zeroHere(); gimbal.pitch().zeroHere();
                                   std::printf("[cmd] 归零\n"); }
            else if (cmd == "en") { int v = 1; iss >> v; gimbal.enable(v != 0);
                                   std::printf("[cmd] enable=%d\n", v); }
            else if (cmd == "x" || cmd == "estop") { gimbal.estop(); local_has = false;
                                   std::printf("[cmd] 急停\n"); }
            else if (cmd == "s" || cmd == "status") { printStatus(gimbal, *bus, link.get(), chassis.get()); }
            else if (cmd == "c") { iss >> local_cvx >> local_cvy >> local_cwz; local_c_has = true;
                                   std::printf("[cmd] 底盘 vx=%.0f vy=%.0f wz=%.0f\n",
                                               local_cvx, local_cvy, local_cwz); }
            else if (cmd == "cx") { local_c_estop = true; local_c_has = false;
                                   std::printf("[cmd] 底盘急停\n"); }
            else if (cmd == "q" || cmd == "quit") { g_run = false; }
            else if (!cmd.empty()) { std::printf("[cmd] 未知: %s\n", cmd.c_str()); }
            inbuf.clear();
        }

        // ---- 2. UDP 收包并应用 ----
        if (link) {
            link->poll();
            ControlCommand cc;
            while (link->popCommand(cc)) {
                echo_seq = cc.seq;
                if (cc.estop()) {
                    gimbal.estop();
                    local_has = false;
                    continue;
                }
                if (cc.flags & ctrl_flag::Disable)      gimbal.enable(false);
                else if (cc.enable())                   gimbal.enable(true);
                if (cc.zero()) { gimbal.yaw().zeroHere(); gimbal.pitch().zeroHere(); }
                if (cc.home()) gimbal.home();
                if (cc.hold()) {
                    gimbal.hold();
                    local_has = false;
                } else {
                    local_has   = false;      // 远端接管本地设定
                    local_yaw   = cc.yaw_dps;
                    local_pitch = cc.pitch_dps;
                    gimbal.setRate(cc.yaw_dps, cc.pitch_dps);
                }

                // ---- 底盘指令（坐标约定：vx 前进 / vy 左移 / wz 逆时针）----
                if (chassis) {
                    ChassisCommand sc;
                    sc.vx_mmps    = cc.chassis_vx_mmps;
                    sc.vy_mmps    = cc.chassis_vy_mmps;
                    sc.wz_mradps  = cc.chassis_wz_mradps;
                    sc.enable     = cc.chassisEnable();
                    sc.estop      = cc.chassisEstop();
                    sc.clear_odom = cc.chassisClear();
                    chassis->setCommand(sc);   // clear_odom 由链路内部做一次性处理
                    udp_chassis   = sc;
                    udp_chassis.clear_odom = false;
                    udp_chassis_seen = true;
                    last_udp_chassis_s = now_s();
                    if (cc.chassisClear()) {
                        std::printf("[cmd] 底盘里程计清零\n");
                    }
                }
            }
        }

        // ---- 3. 本地指令下发 ----
        if (local_has) gimbal.setRate(local_yaw, local_pitch);

        // ---- 3b. 底盘：本地 > UDP（250 ms 内）> 停车 ----
        //     注意：操作端断流时必须主动发 EN=0，否则 F407 的看门狗虽然也会停车，
        //     但这多一层保险——云台断流与底盘断流用同一套时效判断。
        if (chassis) {
            ChassisCommand sc;
            const double t = now_s();
            if (local_c_estop) {
                sc.estop = true;                       // 粘滞，直到发 c 命令才解除
            } else if (local_c_has) {
                sc.vx_mmps   = local_cvx;
                sc.vy_mmps   = local_cvy;
                sc.wz_mradps = local_cwz;
                sc.enable    = true;
            } else if (udp_chassis_seen && (t - last_udp_chassis_s) <= 0.25) {
                sc = udp_chassis;
            } else {
                sc.enable = false;                     // 全部 0：F407 走斜坡停车
            }
            chassis->setCommand(sc);
            chassis->poll();
        }

        // ---- 4. 控制一拍 ----
        gimbal.step(dt_target);

        // ---- 5. 把最新遥测交给链路（链路按频率自动回发）----
        if (link) {
            link->setTelemetry(
                buildTelemetry(gimbal, echo_seq, link->stats().rx_hz, chassis.get()));
        }

        // ---- 6. 定频 ----
        t_next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(dt_target));
        const auto now = std::chrono::steady_clock::now();
        if (t_next > now) {
            std::this_thread::sleep_until(t_next);
        } else {
            t_next = now;
        }
    }

    std::printf("\n[main] 退出中：速度归零…\n");
    gimbal.hold();
    for (int i = 0; i < 30; ++i) gimbal.step(dt_target);
    std::printf("[main] 电机保持使能（抱死）；如需松轴请手动失能\n");

    if (chassis) {
        // 发 0.1 s 的停车帧（EN=0），让 F407 平滑停下
        chassis->setCommand(ChassisCommand{});
        for (int i = 0; i < 20; ++i) {
            chassis->poll();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        chassis->close();
    }
    return 0;
}

// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next  = [&](double def) { return (i + 1 < argc) ? std::atof(argv[++i]) : def; };
        auto nexti = [&](int def)    { return (i + 1 < argc) ? std::atoi(argv[++i]) : def; };

        if (a == "-h" || a == "--help") { usage(argv[0]); return 0; }
        else if (a == "--device" && i + 1 < argc) o.device = argv[++i];
        else if (a == "--baud")        o.baud = nexti(o.baud);
        else if (a == "--yaw-addr")    o.yaw_addr = static_cast<uint8_t>(nexti(2));
        else if (a == "--pitch-addr")  o.pitch_addr = static_cast<uint8_t>(nexti(1));
        else if (a == "--gear")        o.gear = next(o.gear);
        else if (a == "--pulses")      o.pulses_per_motor_rev = static_cast<uint32_t>(nexti(3200));
        else if (a == "--yaw-limit")   { o.yaw_min = next(o.yaw_min); o.yaw_max = next(o.yaw_max);
                                         o.yaw_no_limit = false; }
        else if (a == "--pitch-limit") { o.pitch_min = next(o.pitch_min); o.pitch_max = next(o.pitch_max); }
        else if (a == "--yaw-rate")    o.yaw_rate = next(o.yaw_rate);
        else if (a == "--pitch-rate")  o.pitch_rate = next(o.pitch_rate);
        else if (a == "--acc")         o.acc = nexti(200);
        else if (a == "--ctrl-port")   o.ctrl_port = static_cast<uint16_t>(nexti(CTRL_DEFAULT_PORT));
        else if (a == "--no-udp")      o.no_udp = true;
        else if (a == "--hz")          o.loop_hz = next(o.loop_hz);
        else if (a == "--chassis-dev" && i + 1 < argc)  o.chassis_device = argv[++i];
        else if (a == "--chassis-baud") o.chassis_baud = nexti(o.chassis_baud);
        else if (a == "--no-chassis")   o.no_chassis = true;
        else if (a == "--dry")         o.dry = true;
        else if (a == "--selftest")    o.selftest = true;
        else { std::printf("未知参数: %s\n", a.c_str()); usage(argv[0]); return 2; }
    }
    if (o.selftest) return runSelftest();
    return runMain(o);
}
