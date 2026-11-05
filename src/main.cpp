// ===========================================================================
//  gimbal —— 麦轮小车云台控制器（正式工程）
//
//  范围: 云台（两个 ZDT 闭环步进）+ 底盘（STM32F407）
//            yaw   ID = 1   （导电滑环，360° 连续旋转，无机械限位）
//            pitch ID = 2
//        细分 16 -> 3200 脉冲/电机圈
//        传动比 2:1（电机转 2 圈 = 输出轴 1 圈）
//
//  一条 CH340 挂三块 MCU（F407 + 两个 ZDT 驱动板）：
//      底盘命令帧（12B）+ 云台 ZDT 帧 共用一个串口（`link/unified_serial.hpp`）。
//
//  控制入口:
//    ① UDP 40001  —— PC 鼠标客户端（主通道，有线 / Jetson AP 热点都行）
//    ② stdin      —— 本地调试命令
//
//  pitch 会话（默认开，--no-pitch-session 关闭）:
//    PC 首包到达 -> 相对当前姿态上抬 rise_deg，把停住的位置定义为 0°（"水平"），
//    之后 ±pitch-limit 软限位内由 PC 速率控制；
//    PC 控制结束（end_s 无包）-> 放回会话起点的物理姿态并保持使能锁死。
//
//  用法:
//      gimbal                          # 真机（底盘 + 云台共用 /dev/ttyUSB0）
//      gimbal --yaw-can can0           # yaw 走 CAN（can0），pitch + 底盘走串口
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

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdlib>
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
#include "gimbal/link/unified_serial.hpp"   // 底盘 + 云台共用一条串口
#include "gimbal/link/can_chassis_link.hpp"  // SocketCAN 后端：底盘标准帧
#include "gimbal/transport/can_transport.hpp"  // SocketCAN 后端：云台 ZDT 扩展帧
#include "gimbal/link/fire_link.hpp"           // 水弹波箱：F401 USART1 -> PA12 -> MOS
#include "gimbal/imu/imu.hpp"                  // 俯仰角融合（IMU）
#include "gimbal/imu/imu_i2c.hpp"              // ICM-42688-P over I²C

using namespace gimbal;

static std::atomic<bool> g_run{true};
static void onSignal(int) { g_run = false; }

// ---------------------------------------------------------------------------
struct Options {
    std::string device     = "/dev/ttyUSB0";
    int         baud       = 115200;
    std::string fire_dev    = "/dev/ttyTHS1";   ///< 开火链路串口（F401 USART2，40pin UART）
    int         fire_baud   = 115200;           ///< 开火链路波特率
    bool        no_fire     = false;            ///< 不开火链路
    uint8_t     yaw_addr   = 1;          ///< 实机接线：yaw 电机在 ID 1（原默认 2，与 pitch 互换）
    uint8_t     pitch_addr = 2;          ///< 实机接线：pitch 电机在 ID 2（暂未接入 CAN）
    bool        selftest   = false;
    double      loop_hz    = 100.0;

    /// 传动比 = 电机转数 / 输出轴转数（2:1 减速填 2.0）
    double      gear       = 2.0;
    double      pitch_gear = 1.0;   ///< pitch 传动比：本机构直连无减速 -> 1.0

    /// 脉冲/【电机】圈 = 200 × 细分（细分 16 -> 3200）
    uint32_t    pulses_per_motor_rev = 3200;

    double      yaw_min = -180.0, yaw_max = 180.0,  yaw_rate = 180.0;
    double      pitch_min = -30.0, pitch_max = 30.0, pitch_rate = 120.0;   ///< 软限位：上抬 30° / 下压 30°
    bool        yaw_no_limit = true;     ///< 导电滑环：yaw 无机械限位

    int         acc        = 200;
    uint16_t    ctrl_port  = CTRL_DEFAULT_PORT;
    bool        no_udp     = false;

    /// 底盘与云台共用 --device 指定的串口（--no-chassis 时只控云台）
    bool        no_chassis = false;
    /// 底盘后端：空=自动（云台串口则统一串口；云台 CAN 则同 CAN）；
    /// "serial"=与云台共用 --device；"can0"/"can1"=SocketCAN
    std::string chassis    = "";

    /// 仅 yaw 走 SocketCAN 的接口名（如 can0）；非空时 pitch 仍走 --device 串口
    std::string yaw_can    = "";

    // ---- pitch 会话状态机（PC 接管 -> 抬 30° 定零；PC 结束 -> 放下锁死）----
    bool        pitch_session   = true;    ///< --no-pitch-session 关闭，速率直通
    double      pitch_rise_deg  = 40.0;    ///< 相对会话起点上抬的角度 = 新的 0°
    double      pitch_rise_rate = 60.0;    ///< 抬升/放下期间最大角速度 °/s
    double      pitch_servo_kp  = 2.0;     ///< 位置伺服 P 增益（°/s 每 °）
    double      pitch_servo_tol = 1.5;     ///< 到位容差 °
    double      pitch_end_s     = 0.5;     ///< 无控制包多久算"PC 控制结束"
    double      pitch_raise_timeout_s = 8.0;  ///< 抬升/放下最长允许时间

    // ---- IMU（ICM-42688-P，I²C；空 dev = 不启用，回退纯编码器方案）----
    std::string imu_dev       = "";        ///< 如 /dev/i2c-1
    int         imu_addr      = 0x68;      ///< 0x68 / 0x69
    bool        imu_invert    = false;     ///< 安装方向反了翻一下
    int         imu_gyro_axis = 2;         ///< 俯仰转轴对应的陀螺轴（实机为 Z）
    double      imu_offset    = 0.0;       ///< 安装偏置 °
    double      imu_alpha     = 0.98;      ///< 互补滤波系数
    double      imu_gyro_bias = 0.0;       ///< 陀螺零偏 °/s（0 = 开机自动标定）
    bool        imu_probe     = false;     ///< --imu-probe：只读并打印，不控车

    // ---- 绝对仰角保持（动态补偿；需要 IMU 在抬升时定过"水平"）----
    bool        pitch_abs     = false;     ///< 速率指令积分成绝对目标，用 IMU 闭环保持
    double      abs_kp        = 3.0;       ///< 绝对伺服 P 增益 (1/s)

    // ---- 手感（每轴独立，可命令行调）----
    int         tx_gap_ms   = 8;      ///< ZDT 命令最小间隔（越小指令越及时，别低于 3）
    double      yaw_ramp    = 600.0;  ///< yaw 角加速度 °/s²（0→180 约 0.3 s）
    double      pitch_ramp  = 600.0;  ///< pitch 角加速度 °/s²（0→120 约 0.2 s）
    double      hold_delay  = 0.12;   ///< 停手后开始衰减的延时 s
    double      hold_tau    = 0.05;   ///< 停手衰减时间常数 s
    double      deadband    = 0.5;    ///< 速度死区 °/s
};

static void usage(const char* a0) {
    std::printf(
        "用法: %s [选项]\n"
        "  --device <路径>        串口设备 (默认 /dev/ttyUSB0)\n"
        "  --baud <n>             波特率 (默认 115200)\n"
        "  --yaw-addr <n>         yaw 电机地址 (默认 1)\n"
        "  --pitch-addr <n>       pitch 电机地址 (默认 2)\n"
        "  --gear <f>             传动比=电机转数/输出转数 (默认 2.0)\n"
        "  --pitch-gear <f>       pitch 传动比：直连填 1.0 (默认 1.0)\n"
        "  --pulses <n>           脉冲/电机圈 = 200×细分 (默认 3200)\n"
        "  --yaw-limit <a> <b>    给 yaw 启用软限位 (默认关闭)\n"
        "  --pitch-limit <a> <b>  pitch 软限位 (默认 -30 30)\n"
        "  --yaw-rate <dps>       yaw 最大角速度 (默认 180)\n"
        "  --pitch-rate <dps>     pitch 最大角速度 (默认 120)\n"
        "  --acc <0-255>          加速度档位 (默认 200)\n"
        "  --ctrl-port <n>        UDP 控制口 (默认 40001)\n"
        "  --no-udp               不开 UDP（只用本地 stdin）\n"
        "  --hz <n>               控制频率 (默认 100)\n"
        "  --can <iface>          SocketCAN 后端（如 can0；等价于 --device can0，两轴同总线）\n"
        "  --yaw-can <iface>      仅 yaw 走 SocketCAN（如 can0），pitch 仍走 --device 串口\n"
        "  --chassis <serial|canX> 底盘后端：serial=与云台共用 --device（默认），can0=底盘走 CAN\n"
        "  --no-chassis           不接底盘（只控云台）\n"
        "  --fire-dev <路径>      开火串口（F401 USART2，默认 /dev/ttyTHS1）\n"
        "  --fire-baud <n>        开火串口波特率 (默认 115200)\n"
        "  --no-fire              不开火链路\n"
        "  --imu <dev>            IMU 设备（如 /dev/i2c-1；不给则纯编码器）\n"
        "  --imu-addr <n>         IMU 地址 0x68/0x69 (默认 0x68)\n"
        "  --imu-invert           安装方向反了时翻转俯仰符号\n"
        "  --imu-gyro-axis <x|y|z> 俯仰转轴对应的陀螺轴 (默认 z)\n"
        "  --imu-offset <deg>     安装偏置 (默认 0)\n"
        "  --imu-alpha <f>        互补滤波系数 (默认 0.98)\n"
        "  --imu-gyro-bias <dps>  陀螺零偏（0=开机自动标定，默认 0）\n"
        "  --imu-probe            只看 IMU（打印 WHO_AM_I 与角度），不控车\n"
        "  --pitch-abs            绝对仰角保持：方向键给速率，松手保持绝对仰角\n"
        "  --abs-kp <f>           绝对伺服 P 增益 (默认 3.0)\n"
        "  --tx-gap <ms>          ZDT 命令最小间隔 (默认 8；12 会在抬/降速时挤占带宽)\n"
        "  --yaw-ramp <dps2>      yaw 角加速度 (默认 600)\n"
        "  --pitch-ramp <dps2>    pitch 角加速度 (默认 600)\n"
        "  --hold-delay <s>       停手后开始衰减的延时 (默认 0.12)\n"
        "  --hold-tau <s>         停手衰减时间常数 (默认 0.05)\n"
        "  --deadband <dps>       速度死区 (默认 0.5)\n"
        "  --no-pitch-session     关闭 pitch 会话状态机（旧行为：PC 速率直通）\n"
        "  --pitch-rise <deg>     会话开始时相对起点上抬并定为 0° (默认 40)\n"
        "  --pitch-rise-rate <dps> 抬升/放下最大角速度 (默认 60)\n"
        "  --pitch-session-end <s> 无控制包多久后放下锁死 (默认 0.5)\n"
        "  --pitch-servo-kp <f>   抬升/放下位置伺服 P 增益 (默认 2.0)\n"
        "  --pitch-servo-tol <deg> 到位容差 (默认 1.5)\n"
        "  --pitch-raise-timeout <s> 抬升/放下超时 (默认 8)\n"
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

static void pitchSessionSelftest();   // 定义在 PitchSession 之后（依赖其定义）
static void smoothnessSelftest();     // 起停指令流平滑度（定义在文件后部）
static void imuFusionSelftest();      // IMU 融合（纯软件）
static void absHoldSelftest();        // 绝对仰角保持（FakeTransport + 假 IMU）

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
        f = zdt::mkClearStall(1);
        CHECK(hex(f) == "01 0E 52 6B", "解除堵转帧（带 0x52 固件要求）: " + hex(f));
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
        CHECK(f[2] == chassis_cmd::EStop && f[10] == 0, "急停帧 cmd=0x02 且不带 enable");
    }

    std::printf("\n[7] 单总线帧分流（RxDemux：底盘帧 + ZDT 帧 + 垃圾字节）\n");
    {
        // 与 [6] 相同的 40B 底盘反馈帧
        static const uint8_t fb[CHASSIS_FB_LEN] = {
            0x5A, 0xA5, 0x81, 0x21, 0x24, 0xFA, 0xFF, 0xFF, 0xC4, 0x09, 0x00,
            0x00, 0xD8, 0xDC, 0xF4, 0x01, 0x38, 0xFF, 0x5E, 0x01, 0xE0, 0x2E,
            0x06, 0xFF, 0x13, 0x00, 0xD2, 0x04, 0x4D, 0x00, 0xB0, 0x04, 0xE0,
            0xFC, 0x2C, 0x01, 0x00, 0x00, 0xFE, 0x0D};
        // ZDT GetSpeed 应答：addr=1, func=0x35, 正转 300RPM（0x012C）
        const uint8_t zdt1[] = {0x01, 0x35, 0x00, 0x01, 0x2C, 0x6B};
        const uint8_t junk[] = {0xFF, 0x00, 0x12, 0x7A};

        RxDemux dx;
        // 1) 半包：只喂前 20 字节 -> 不应产出任何帧
        dx.feed(fb, 20);
        ChassisState st;
        zdt::Frame fr;
        CHECK(!dx.popChassis(st) && !dx.popZdt(fr), "半包不产出帧");

        // 2) 补全 + 垃圾 + ZDT 帧 + 又整帧，交错喂入
        dx.feed(fb + 20, CHASSIS_FB_LEN - 20);
        dx.feed(junk, sizeof(junk));
        dx.feed(zdt1, sizeof(zdt1));
        dx.feed(fb, CHASSIS_FB_LEN);

        int  nfb = 0;
        bool fb_ok = false;
        while (dx.popChassis(st)) {
            ++nfb;
            if (st.odom_x_mm == -1500 && st.odom_y_mm == 2500 &&
                std::fabs(st.odom_yaw_deg + 90.0) < 1e-9)
                fb_ok = true;
        }
        int  nz = 0;
        bool zdt_ok = false;
        while (dx.popZdt(fr)) {
            ++nz;
            if (fr.addr == 1 && fr.func == 0x35 && fr.data.size() == 4 &&
                fr.data[0] == 0x00 && fr.data[1] == 0x01 &&
                fr.data[2] == 0x2C && fr.data[3] == 0x6B)
                zdt_ok = true;
        }
        CHECK(nfb == 2 && fb_ok, "两帧底盘反馈被正确切出");
        CHECK(nz == 1 && zdt_ok, "一帧 ZDT 回帧被正确切出");
        CHECK(dx.chassisBad() == 0 && dx.zdtBad() == 0, "无坏帧");

        // 3) 逐字节喂入（极端半包），结果应一致
        RxDemux dx2;
        for (size_t i = 0; i < CHASSIS_FB_LEN; ++i) dx2.feed(fb + i, 1);
        for (size_t i = 0; i < sizeof(zdt1); ++i)  dx2.feed(zdt1 + i, 1);
        int c2 = 0;
        while (dx2.popChassis(st)) ++c2;
        int z2 = 0;
        while (dx2.popZdt(fr)) ++z2;
        CHECK(c2 == 1 && z2 == 1, "逐字节喂入同样正确");
    }

    std::printf("\n[8] CAN 反馈组拼装（0x110~0x114 → 40B 反馈帧）\n");
    {
        static const uint8_t fb[CHASSIS_FB_LEN] = {
            0x5A, 0xA5, 0x81, 0x21, 0x24, 0xFA, 0xFF, 0xFF, 0xC4, 0x09, 0x00,
            0x00, 0xD8, 0xDC, 0xF4, 0x01, 0x38, 0xFF, 0x5E, 0x01, 0xE0, 0x2E,
            0x06, 0xFF, 0x13, 0x00, 0xD2, 0x04, 0x4D, 0x00, 0xB0, 0x04, 0xE0,
            0xFC, 0x2C, 0x01, 0x00, 0x00, 0xFE, 0x0D};
        uint8_t out[CHASSIS_FB_LEN];
        CanFbGroup g;

        // 顺序拼装
        CHECK(g.feed(0x110, fb + 0, 8, out) == 0, "0x110 开组未完成");
        CHECK(g.feed(0x111, fb + 8, 8, out) == 0, "0x111 未完成");
        CHECK(g.feed(0x112, fb + 16, 8, out) == 0, "0x112 未完成");
        CHECK(g.feed(0x113, fb + 24, 8, out) == 0, "0x113 未完成");
        CHECK(g.feed(0x114, fb + 32, 8, out) == 1, "0x114 完成且 CRC 通过");
        CHECK(std::memcmp(out, fb, CHASSIS_FB_LEN) == 0, "拼装结果与原帧一致");

        // 乱序拼装
        CHECK(g.feed(0x110, fb + 0, 8, out) == 0, "乱序：开组");
        CHECK(g.feed(0x113, fb + 24, 8, out) == 0, "乱序：0x113");
        CHECK(g.feed(0x111, fb + 8, 8, out) == 0, "乱序：0x111");
        CHECK(g.feed(0x114, fb + 32, 8, out) == 0, "乱序：0x114");
        CHECK(g.feed(0x112, fb + 16, 8, out) == 1, "乱序补全后完成");

        // 半组被新 0x110 覆盖
        CHECK(g.feed(0x110, fb + 0, 8, out) == 0, "半组");
        CHECK(g.feed(0x110, fb + 0, 8, out) == 0, "新组覆盖半组");

        // CRC 失败整组丢弃（破坏 offset 20，落在 0x112 帧里）
        uint8_t bad[CHASSIS_FB_LEN];
        std::memcpy(bad, fb, sizeof(bad));
        bad[20] ^= 0xFF;
        CHECK(g.feed(0x110, bad + 0, 8, out) == 0, "坏组：开组");
        CHECK(g.feed(0x111, bad + 8, 8, out) == 0, "坏组：0x111");
        CHECK(g.feed(0x112, bad + 16, 8, out) == 0, "坏组：0x112");
        CHECK(g.feed(0x113, bad + 24, 8, out) == 0, "坏组：0x113");
        CHECK(g.feed(0x114, bad + 32, 8, out) == -1, "坏组：CRC 失败被丢弃");

        // 非反馈 ID / 错误长度忽略
        CHECK(g.feed(0x020, fb, 8, out) == 0, "非反馈 ID 被忽略");
        CHECK(g.feed(0x110, fb, 4, out) == 0, "短帧被忽略");
    }

    std::printf("\n[9] 混合传输：yaw 走 CAN、pitch 走 UART（两轴不同总线）\n");
    {
        auto ybus = std::make_shared<FakeTransport>();
        auto pbus = std::make_shared<FakeTransport>();
        ybus->addMotor(2);
        pbus->addMotor(1);

        ZdtMotor::Config yc; yc.addr = 2; yc.poll_speed_ms = 50; yc.poll_pos_ms = 50;
        ZdtMotor::Config pc; pc.addr = 1; pc.poll_speed_ms = 50; pc.poll_pos_ms = 50;
        auto yaw_m   = std::make_shared<ZdtMotor>(ybus, yc);
        auto pitch_m = std::make_shared<ZdtMotor>(pbus, pc);

        Gimbal::Config gc;
        gc.yaw.name = "yaw";       gc.yaw.gear_ratio = 2.0;
        gc.yaw.use_soft_limits = false;
        gc.pitch.name = "pitch";   gc.pitch.gear_ratio = 2.0;
        gc.pitch.min_deg = -45;    gc.pitch.max_deg = 45;
        gc.enable_delay_s = 0.05;
        gc.command_timeout_s = 0.4;

        Gimbal g(yaw_m, pitch_m, gc);
        CHECK(g.transports().size() == 2, "两轴不同总线 -> 传输层 2 个");

        const double DT = 0.004;
        for (int i = 0; i < 120; ++i) {
            g.step(DT);
            std::this_thread::sleep_for(
                std::chrono::microseconds(static_cast<int>(DT * 1e6)));
        }
        CHECK(ybus->sim(2).tx_seen > 0, "yaw 命令只落在 CAN 总线");
        CHECK(pbus->sim(1).tx_seen > 0, "pitch 命令只落在串口总线");
        CHECK(ybus->sim(2).get_seen > 0, "yaw 读取在 CAN 总线回读");
        CHECK(pbus->sim(1).get_seen > 0, "pitch 读取在串口总线回读");
    }

    std::printf("\n[10] pitch 会话状态机（纯软件，FakeTransport 模拟电机运动）\n");
    pitchSessionSelftest();

    std::printf("\n[11] 起停手感：指令流台阶（FakeTransport）\n");
    smoothnessSelftest();

    std::printf("\n[12] IMU 俯仰融合（互补滤波 + 创新门限）\n");
    imuFusionSelftest();

    std::printf("\n[13] 绝对仰角保持（底盘倾斜补偿）\n");
    absHoldSelftest();

    std::printf("\n=== 自检结束: %s（失败 %d 项）===\n",
                g_fail == 0 ? "全部通过" : "有失败", g_fail);
    return g_fail == 0 ? 0 : 1;
}

// ===========================================================================
// 运行时
// ===========================================================================
static void printStatus(const Gimbal& g, const ITransport& yaw_bus,
                        const ITransport& pitch_bus, const UdpControlLink* link,
                        const IChassisLink* chassis) {
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
    auto busLine = [](const char* tag, const ITransport& b) {
        const auto& st = b.stats();
        std::printf("  %s总线: %-18s 连接=%d 发送=%llu 丢弃=%llu 收帧=%llu 重连=%llu\n",
                    tag, b.name().c_str(), st.connected ? 1 : 0,
                    static_cast<unsigned long long>(st.tx_frames),
                    static_cast<unsigned long long>(st.tx_dropped),
                    static_cast<unsigned long long>(st.rx_frames),
                    static_cast<unsigned long long>(st.reconnects));
    };
    busLine("yaw  ", yaw_bus);
    if (&yaw_bus != &pitch_bus) busLine("pitch", pitch_bus);
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

// ===========================================================================
// pitch 会话状态机（--pitch-session）
//
//   PC 首包到达 -> 以当前姿态为起点向上抬 rise_deg，停住后把该位置定义为 0°
//   （即"水平"），±limit 软限位随即生效，之后由 PC 速率控制；
//   PC 控制结束（end_s 内无包）-> 放回会话起点的物理姿态（新基准下 = -rise_deg）
//   并保持使能锁死。抬升/放下是角度伺服：rate = clamp(kp*err, ±rate_max)，
//   斜坡、死区、限位仍由 Axis 负责。
//
//   注意：0° 是用"实际停住的位置"建立的，所以无论落地误差多少，
//   "会话起点 = -rise_deg、抬升位 = 0°" 这层关系始终自洽。
// ===========================================================================
class PitchSession {
public:
    enum class Mode { kLocked, kRaising, kActive, kLowering };

    struct Params {
        double rise_deg  = 40.0;   ///< 相对会话起点上抬的角度（= 新 0°）
        double rate_max  = 60.0;   ///< 过渡期间最大角速度 °/s
        double kp        = 2.0;    ///< 位置伺服 P 增益（°/s 每 °）
        double tol_deg   = 1.5;    ///< 到位容差
        double timeout_s = 8.0;    ///< 过渡最长允许时间
        double limit_min = -30.0;  ///< 到位后恢复的软限位
        double limit_max = 30.0;

        // ---- IMU（每拍由主循环填）----
        bool   imu_ok        = false;  ///< 本拍 IMU 可用
        double imu_pitch_deg = 0.0;    ///< 当前枪管绝对仰角（已含零位）
        bool*  imu_zero_req  = nullptr;///< 抬升到位时置 true，让主循环给 IMU 清零

        // ---- 绝对仰角保持（动态补偿）----
        bool   abs_hold      = false;  ///< 速率指令积分成绝对目标，再用 IMU 闭环
        double abs_kp        = 3.0;    ///< P 增益 (1/s)
        double abs_min       = -30.0;  ///< 目标仰角下限
        double abs_max       = 30.0;   ///< 目标仰角上限
    };

    /// 每拍调用：返回要下发的 pitch 角速度；*send=false 表示本拍不下发
    /// （交给 Gimbal 的看门狗自然把速度归零，同时保留看门狗状态位）。
    double step(double dt, bool udp_live, bool udp_hold, double pc_pitch_dps,
                Axis& pitch, const Params& p, bool* send);

    void onEstop() { mode_ = Mode::kLocked; latched_ = true; age_s_ = 0.0; }
    void onDisable() { saw_disable_ = true; }
    void onEnable() {
        if (saw_disable_) {
            latched_     = false;   // g 关再开 = 重新授权
            saw_disable_ = false;
        }
    }
    void onLocalEnable() { latched_ = false; saw_disable_ = false; }

    /// 退出前调用：抬升中/控制中 -> 直接切到"放下"，避免先抬到位再落下去
    void prepareExit(Axis& pitch, const Params& p) {
        if (mode_ == Mode::kRaising || mode_ == Mode::kActive) beginLower(pitch, p);
    }

    Mode        mode() const { return mode_; }
    bool        latched() const { return latched_; }
    double      restDeg() const { return rest_deg_; }
    double      targetDeg() const { return target_deg_; }
    const char* modeName() const;

private:
    void beginRaise(Axis& pitch, const Params& p);
    void beginLower(Axis& pitch, const Params& p);
    void abortTo(Axis& pitch, const Params& p, const char* why);

    Mode   mode_        = Mode::kLocked;
    bool   latched_     = false;  ///< 急停/异常后待重新授权
    bool   saw_disable_ = false;  ///< 客户端 g 关 -> g 开 的重新授权握手
    bool   warned_      = false;  ///< "等待反馈/使能"告警去重
    bool   zeroed_      = false;  ///< 0° 是否已由本会话建立（决定放下目标的坐标系）
    double entry_deg_   = 0.0;    ///< 进入过渡模式时的角度（无进展检测用）
    bool   used_imu_    = false;  ///< 本次抬升是否用 IMU 定的"水平"
    double rest_est_    = 0.0;    ///< 会话起点在新基准（0°=水平）下的位置
    double abs_target_deg_ = 0.0; ///< 绝对模式的目标仰角
    bool   abs_armed_   = false;  ///< 绝对模式已用当前姿态起步
    double rest_deg_    = 0.0;    ///< 会话起点（旧基准）
    double target_deg_  = 0.0;    ///< 当前过渡目标
    double age_s_       = 0.0;    ///< 当前模式持续时长
};

const char* PitchSession::modeName() const {
    switch (mode_) {
        case Mode::kLocked:   return "Locked";
        case Mode::kRaising:  return "Raising";
        case Mode::kActive:   return "Active";
        case Mode::kLowering: return "Lowering";
    }
    return "?";
}

void PitchSession::beginRaise(Axis& pitch, const Params& p) {
    rest_deg_   = pitch.estAngleDeg();
    zeroed_     = false;
    entry_deg_  = rest_deg_;
    abs_armed_  = false;
    if (p.imu_ok) {
        // 有 IMU：目标不是"抬固定角度"，而是"抬到枪管水平"。
        // 当前仰角为 imu_pitch_deg，要转平就得让 est 减少这么多。
        target_deg_ = rest_deg_ - p.imu_pitch_deg;
        if (target_deg_ < rest_deg_)        target_deg_ = rest_deg_;
        if (target_deg_ > rest_deg_ + 120.0) target_deg_ = rest_deg_ + 120.0;
        used_imu_ = true;
        std::printf("[pitch] 会话开始（IMU）：当前仰角 %.1f° -> 抬到水平（est %.2f°）\n",
                    p.imu_pitch_deg, target_deg_);
    } else {
        target_deg_ = rest_deg_ + p.rise_deg;
        used_imu_   = false;
        std::printf("[pitch] 会话开始：起点 %.2f° -> 目标 %.2f°（%+.0f°），到达后定为 0°\n",
                    rest_deg_, target_deg_, p.rise_deg);
    }
    // 过渡期间放宽软限位：此刻 est 仍按旧基准计算，±limit 会误拦
    pitch.setSoftLimits(-180.0, 180.0);
    pitch.motor()->clearStall();   // 顺手清一次旧的堵转保护（帧已按手册补 0x52）
    mode_  = Mode::kRaising;
    age_s_ = 0.0;
}

void PitchSession::beginLower(Axis& pitch, const Params& p) {
    // 已由本会话定义过 0°：会话起点 = -rise_deg（新基准）；
    // 没定义过（例如抬升途中退出）：起点就是 rest_deg_（旧基准）。
    // 注意：rise 可能大于 limit（如抬 40° 而限位 ±30°），放下必须能回到真正的
    // 静止位，所以过渡期把下限临时放宽到目标下方 5°，到位后仍是 ±limit。
    target_deg_ = zeroed_ ? rest_est_ : rest_deg_;
    entry_deg_  = pitch.estAngleDeg();
    abs_armed_  = false;
    pitch.setSoftLimits(target_deg_ - 5.0, p.limit_max);
    mode_  = Mode::kLowering;
    age_s_ = 0.0;
    std::printf("[pitch] PC 控制结束：放下到 %.1f°（会话起点姿态）\n", target_deg_);
}

void PitchSession::abortTo(Axis& pitch, const Params& p, const char* why) {
    pitch.setSoftLimits(p.limit_min, p.limit_max);
    mode_    = Mode::kLocked;
    latched_ = true;
    age_s_   = 0.0;
    std::printf("[pitch] %s中止：停在 %.2f°，保持使能。"
                "按 g 关再开（或本地 en 1）重新授权\n",
                why, pitch.estAngleDeg());
}

double PitchSession::step(double dt, bool udp_live, bool udp_hold,
                          double pc_pitch_dps, Axis& pitch, const Params& p,
                          bool* send) {
    *send = true;
    age_s_ += dt;

    const auto stalled = [&pitch]() {
        const auto& s = pitch.motorStatus();
        return s.stalled || s.stall_prot;
    };
    const auto servo = [&p](double err) {
        double r = err * p.kp;
        if (r >  p.rate_max) r =  p.rate_max;
        if (r < -p.rate_max) r = -p.rate_max;
        return r;
    };

    if (mode_ == Mode::kLocked) {
        // 未授权 / 没控制端 / 缺反馈：本拍不下发，交给看门狗抱死
        if (!udp_live || latched_) {
            *send = false;
            return 0.0;
        }
        if (!pitch.enabled()) {
            if (!warned_) {
                warned_ = true;
                std::printf("[pitch] pitch 未使能，等待客户端使能后再抬升\n");
            }
            *send = false;
            return 0.0;
        }
        if (!pitch.motor()->hasPosFeedback()) {
            if (!warned_) {
                warned_ = true;
                std::printf("[pitch] 等待位置反馈（0x36），暂不抬升\n");
            }
            *send = false;
            return 0.0;
        }
        warned_ = false;
        beginRaise(pitch, p);   // 落到下面的 kRaising
    }

    if (mode_ == Mode::kRaising) {
        const double err = target_deg_ - pitch.estAngleDeg();
        if (std::fabs(err) <= p.tol_deg) {
            pitch.zeroHere(/*zero_driver=*/false);   // 只在软件侧定义 0°
            pitch.setSoftLimits(p.limit_min, p.limit_max);
            rest_est_ = -(target_deg_ - rest_deg_);  // 会话起点在新基准下的位置
            if (used_imu_ && p.imu_zero_req) *p.imu_zero_req = true;
            std::printf("[pitch] 抬升到位（停于 %.2f°），该姿态定为 0°（%s），"
                        "限位 %+.0f/%+.0f°；会话起点 %.1f°\n",
                        pitch.estAngleDeg(), used_imu_ ? "IMU 真水平" : "相对水平",
                        p.limit_min, p.limit_max, rest_est_);
            mode_   = Mode::kActive;
            age_s_   = 0.0;
            zeroed_  = true;
            return 0.0;
        }
        if (stalled()) {
            abortTo(pitch, p, "抬升时检测到堵转");
            return 0.0;
        }
        if (age_s_ > p.timeout_s) {
            abortTo(pitch, p, "抬升超时");
            return 0.0;
        }
        // 护栏：1 s 内角度毫无变化 = 方向接反/机构卡死，不能让它继续死推
        if (age_s_ > 1.0 && std::fabs(pitch.estAngleDeg() - entry_deg_) < 0.5) {
            abortTo(pitch, p, "抬升 1 s 无位移（检查方向/机械）");
            return 0.0;
        }
        return servo(err);
    }

    if (mode_ == Mode::kActive) {
        if (!udp_live) {
            beginLower(pitch, p);   // 落到下面的 kLowering
        } else {
            if (!pitch.enabled()) {
                std::printf("[pitch] pitch 被失能，会话结束并锁死\n");
                mode_    = Mode::kLocked;
                latched_ = false;
                age_s_   = 0.0;
                return 0.0;
            }
            if (stalled()) {
                abortTo(pitch, p, "保持中检测到堵转");
                return 0.0;
            }
            if (p.abs_hold && p.imu_ok && used_imu_) {
                // 绝对仰角保持：方向键速率积分成目标仰角，用 IMU 闭环。
                // 底盘倾斜/加减速时枪口保持世界坐标下的仰角。
                if (!abs_armed_) {
                    abs_target_deg_ = p.imu_pitch_deg;
                    abs_armed_      = true;
                }
                if (!udp_hold) abs_target_deg_ += pc_pitch_dps * dt;
                if (abs_target_deg_ < p.abs_min) abs_target_deg_ = p.abs_min;
                if (abs_target_deg_ > p.abs_max) abs_target_deg_ = p.abs_max;
                double r = (abs_target_deg_ - p.imu_pitch_deg) * p.abs_kp;
                if (r >  p.rate_max) r =  p.rate_max;
                if (r < -p.rate_max) r = -p.rate_max;
                return r;
            }
            return udp_hold ? 0.0 : pc_pitch_dps;
        }
    }

    // ---- kLowering ----
    if (udp_live) {   // 控制端又回来了：中止放下，回到速率控制
        mode_     = Mode::kActive;
        age_s_    = 0.0;
        abs_armed_ = false;
        std::printf("[pitch] 控制端重新在线，中止放下\n");
        return udp_hold ? 0.0 : pc_pitch_dps;
    }
    const double err = target_deg_ - pitch.estAngleDeg();
    if (std::fabs(err) <= p.tol_deg) {
        mode_    = Mode::kLocked;
        latched_ = false;
        age_s_   = 0.0;
        std::printf("[pitch] 已放下并锁死（%.2f°），保持使能抱死\n",
                    pitch.estAngleDeg());
        return 0.0;
    }
    if (stalled()) {
        mode_  = Mode::kLocked;
        age_s_ = 0.0;
        std::printf("[pitch] 放下时检测到堵转，停在 %.2f° 并保持使能\n",
                    pitch.estAngleDeg());
        return 0.0;
    }
    if (age_s_ > p.timeout_s) {
        mode_  = Mode::kLocked;
        age_s_ = 0.0;
        std::printf("[pitch] 放下超时，停在 %.2f° 并保持使能\n",
                    pitch.estAngleDeg());
        return 0.0;
    }
    if (age_s_ > 1.0 && std::fabs(pitch.estAngleDeg() - entry_deg_) < 0.5) {
        mode_  = Mode::kLocked;
        age_s_ = 0.0;
        std::printf("[pitch] 放下 1 s 无位移（检查方向/机械），"
                    "停在 %.2f° 并保持使能\n", pitch.estAngleDeg());
        return 0.0;
    }
    return servo(err);
}

// ---------------------------------------------------------------------------
// pitch 会话状态机的纯软件自检：FakeTransport 会按**真实时间**积分电机位置，
// 所以这里用 sleep 10 ms 的实时步进，跑完"无控制端 -> 抬升 -> 限位 -> 放下"全流程。
// ---------------------------------------------------------------------------
static void pitchSessionSelftest() {
    using Clock = std::chrono::steady_clock;

    auto bus = std::make_shared<FakeTransport>();
    bus->addMotor(1);   // yaw
    bus->addMotor(2);   // pitch

    // 轮询周期必须与真机一致（100ms）：更激进会让速度指令超过总线 12ms
    // 节流（83 帧/s）的排空能力，发送队列积压后电机执行的是旧命令。
    ZdtMotor::Config yc;
    yc.addr          = 1;
    yc.poll_speed_ms = 100;
    yc.poll_pos_ms   = 100;
    yc.poll_status_ms = 200;
    auto yaw_motor = std::make_shared<ZdtMotor>(bus, yc);
    ZdtMotor::Config pc = yc;
    pc.addr = 2;
    auto pitch_motor = std::make_shared<ZdtMotor>(bus, pc);

    Gimbal::Config gc;
    gc.yaw.name            = "yaw";
    gc.yaw.gear_ratio      = 2.0;
    gc.yaw.use_soft_limits = false;
    gc.pitch.name          = "pitch";
    gc.pitch.gear_ratio    = 1.0;
    gc.pitch.min_deg       = -30.0;
    gc.pitch.max_deg       = 30.0;
    gc.command_timeout_s   = 0.5;
    gc.enable_delay_s      = 0.05;

    Gimbal g(yaw_motor, pitch_motor, gc);
    PitchSession      sess;
    PitchSession::Params sp;

    const bool dbg_on = std::getenv("PITCH_DBG") != nullptr;
    int  dbg = 0;
    auto run_for = [&](double secs, bool udp_live, double rate) {
        const auto t0 = Clock::now();
        while (std::chrono::duration<double>(Clock::now() - t0).count() < secs) {
            bool         send = false;
            const double pr =
                sess.step(0.01, udp_live, false, rate, g.pitch(), sp, &send);
            if (send) g.setRate(0.0, pr);
            g.step(0.01);
            if (dbg_on && ++dbg % 5 == 0) {
                std::printf("    [dbg] %-8s pr=%6.1f act=%6.1f est=%7.2f "
                            "pos_sim=%6.1f raw=%5u hfb=%d rpm_sim=%5.1f fbrpm=%5.1f "
                            "age=%.2f\n",
                            sess.modeName(), pr, g.pitch().actRateDps(),
                            g.pitch().estAngleDeg(), bus->sim(2).pos_deg,
                            g.pitch().motorStatus().pos_raw & 0xFFFFu,
                            g.pitch().motor()->hasPosFeedback() ? 1 : 0,
                            bus->sim(2).rpm, g.pitch().motorStatus().speed_rpm,
                            g.pitch().motorStatus().age_s);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    };

    run_for(0.6, /*udp_live=*/false, 0.0);   // 没有控制端：应保持锁死
    CHECK(sess.mode() == PitchSession::Mode::kLocked, "无控制包 -> 保持 Locked");
    CHECK(std::fabs(bus->sim(2).rpm) < 1.0, "无控制包 -> pitch 不转");

    run_for(5.0, /*udp_live=*/true, 0.0);    // 控制端上线：抬升 40° 并定为 0°
    CHECK(sess.mode() == PitchSession::Mode::kActive, "首包 -> 抬升到位后进入 Active");
    double est = g.pitch().estAngleDeg();
    CHECK(std::fabs(est) <= 3.0, "抬升位被定为 0°（水平）: est=" + std::to_string(est));

    run_for(1.2, /*udp_live=*/true, 60.0);   // 上抬：应被 +30° 软限位挡住
    est = g.pitch().estAngleDeg();
    CHECK(est >= 10.0, "上抬指令生效: est=" + std::to_string(est));
    CHECK(est <= 36.0, "上抬被 +30° 软限位挡住: est=" + std::to_string(est));

    run_for(1.0, /*udp_live=*/true, 0.0);    // 松手保持
    const double est_hold = g.pitch().estAngleDeg();
    CHECK(std::fabs(est_hold - est) <= 2.0,
          "松手保持（不回漂）: est=" + std::to_string(est_hold));

    run_for(5.0, /*udp_live=*/false, 0.0);   // 控制端消失：放下并锁死
    CHECK(sess.mode() == PitchSession::Mode::kLocked, "断流 -> 放下后回到 Locked");
    est = g.pitch().estAngleDeg();
    CHECK(est >= -43.0 && est <= -33.0,
          "放到会话起点（约 -40°）: est=" + std::to_string(est));
}

// ---------------------------------------------------------------------------
// 起停平滑度自检：记录电机"实际收到的目标转速序列"（FakeTransport 回显），
// 检查起停过程中每帧只变 1 RPM、无来回抖、松手能回 0。
// ---------------------------------------------------------------------------
static void smoothnessSelftest() {
    auto bus = std::make_shared<FakeTransport>();
    bus->addMotor(1);
    bus->addMotor(2);

    ZdtMotor::Config yc;
    yc.addr          = 1;
    yc.poll_speed_ms = 100;
    yc.poll_pos_ms   = 100;
    auto yaw_motor = std::make_shared<ZdtMotor>(bus, yc);
    ZdtMotor::Config pc = yc;
    pc.addr = 2;
    auto pitch_motor = std::make_shared<ZdtMotor>(bus, pc);

    Gimbal::Config gc;
    gc.yaw.name            = "yaw";
    gc.yaw.gear_ratio      = 2.0;
    gc.yaw.max_rate_dps    = 180.0;
    gc.yaw.use_soft_limits = false;
    gc.pitch.name          = "pitch";
    gc.pitch.gear_ratio    = 1.0;
    gc.pitch.max_rate_dps  = 120.0;
    gc.pitch.min_deg       = -30.0;
    gc.pitch.max_deg       = 30.0;
    gc.yaw.rate_accel_dps2   = 600.0;
    gc.pitch.rate_accel_dps2 = 600.0;
    gc.enable_delay_s        = 0.05;

    Gimbal g(yaw_motor, pitch_motor, gc);

    std::vector<double> cmd;   // 电机实际收到的目标转速（去重后）
    // 节奏必须和真机一致：10 ms 控制拍 + 总线节流（这里 2 ms）。
    // 若按 1 ms 睡而 dt 用 10 ms，轴的变化会比总线发送还快，相邻帧被合并成
    // 2 RPM 的台阶——那是测试假象，不是真机行为。
    auto tick = [&](double rate) {
        g.setRate(0.0, rate);
        g.step(0.01);
        const double rpm = bus->sim(2).target_rpm;
        if (cmd.empty() || std::fabs(cmd.back() - rpm) > 1e-9) cmd.push_back(rpm);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    };

    for (int i = 0; i < 40; ++i) tick(120.0);   // 起步
    for (int i = 0; i < 40; ++i) tick(0.0);     // 松手

    size_t peak = 0;
    for (size_t i = 1; i < cmd.size(); ++i) {
        if (std::fabs(cmd[i]) > std::fabs(cmd[peak])) peak = i;
    }
    double max_step = 0.0;
    bool   monotone = true;
    for (size_t i = 1; i <= peak; ++i) {
        const double d = cmd[i] - cmd[i - 1];
        max_step = std::max(max_step, std::fabs(d));
        if (d < -1e-9) monotone = false;
    }
    for (size_t i = peak + 1; i < cmd.size(); ++i) {
        const double d = cmd[i] - cmd[i - 1];
        max_step = std::max(max_step, std::fabs(d));
        if (d > 1e-9) monotone = false;
    }

    if (std::getenv("PITCH_DBG")) {
        std::string seq;
        for (double v : cmd) seq += std::to_string(static_cast<int>(std::lround(v))) + " ";
        std::printf("    [dbg] rpm 指令序列: %s\n", seq.c_str());
    }
    CHECK(std::fabs(cmd[peak]) >= 19.0,
          "起步爬到 20 RPM: peak=" + std::to_string(cmd[peak]));
    CHECK(max_step <= 2.0,
          "单帧转速跳变 <= 2 RPM: max=" + std::to_string(max_step));
    CHECK(monotone, "起停转速单调，无来回抖");
    CHECK(std::fabs(cmd.back()) < 1e-9,
          "松手后回到 0: last=" + std::to_string(cmd.back()));
}

// ---------------------------------------------------------------------------
// IMU 俯仰融合自检：喂合成样本，检查静态精度 / 动态跟踪 / 线性加速度门限
// ---------------------------------------------------------------------------
static void imuFusionSelftest() {
    // 造一个"枪管仰角 = deg"的静止样本；g_along 模拟沿枪管的线性加速度污染
    auto make = [](double deg, double g_along = 0.0) {
        ImuSample s;
        s.valid = true;
        const double rad = deg * 3.14159265358979323846 / 180.0;
        s.ax = std::sin(rad) + g_along;
        s.ay = 0.0;
        s.az = -std::cos(rad);
        return s;
    };

    // --- 静态精度 ---
    {
        ImuPitch f;
        ImuSample s = make(30.0);
        f.update(s, 0.01);
        for (int i = 0; i < 100; ++i) f.update(make(30.0), 0.01);
        CHECK(std::fabs(f.pitchDeg() - 30.0) < 0.2,
              "静态 30° 融合角: " + std::to_string(f.pitchDeg()));
        CHECK(f.confident(), "静态时加速度可信");
        CHECK(std::fabs(f.rawAccelDeg() - 30.0) < 0.2,
              "加速度角: " + std::to_string(f.rawAccelDeg()));
    }

    // --- 动态跟踪：20°/s 转 3 s，陀螺与真实角速度一致 ---
    {
        ImuPitch f;
        ImuSample s = make(0.0);
        f.update(s, 0.01);
        double truth = 0.0;
        double max_err = 0.0;
        for (int i = 0; i < 300; ++i) {
            truth += 20.0 * 0.01;
            s = make(truth);
            s.gz = 20.0;   // 默认俯仰转轴是陀螺 Z 轴
            f.update(s, 0.01);
            max_err = std::max(max_err, std::fabs(f.pitchDeg() - truth));
        }
        CHECK(max_err < 2.0, "动态跟踪误差 < 2°: max=" + std::to_string(max_err));
    }

    // --- 线性加速度污染：0.3 g 沿枪管，加速度角被骗偏，融合应靠陀螺 ---
    {
        ImuPitch f;
        ImuSample s = make(0.0);
        f.update(s, 0.01);
        double truth = 0.0;
        double max_err = 0.0;
        for (int i = 0; i < 200; ++i) {
            truth += 15.0 * 0.01;
            s = make(truth, /*g_along=*/0.3);
            s.gz = 15.0;
            f.update(s, 0.01);
            max_err = std::max(max_err, std::fabs(f.pitchDeg() - truth));
        }
        CHECK(max_err < 3.0, "有线性加速度时仍靠陀螺跟踪: max=" + std::to_string(max_err));
    }
}

// ---------------------------------------------------------------------------
// 绝对仰角保持自检：FakeTransport 模拟电机 + 手工构造 IMU 角度，
// 验证"底盘倾斜时枪口保持世界坐标仰角"。
// ---------------------------------------------------------------------------
static void absHoldSelftest() {
    auto bus = std::make_shared<FakeTransport>();
    bus->addMotor(1);
    bus->addMotor(2);

    ZdtMotor::Config yc;
    yc.addr          = 1;
    yc.poll_speed_ms = 100;
    yc.poll_pos_ms   = 100;
    auto yaw_motor = std::make_shared<ZdtMotor>(bus, yc);
    ZdtMotor::Config pc = yc;
    pc.addr = 2;
    auto pitch_motor = std::make_shared<ZdtMotor>(bus, pc);

    Gimbal::Config gc;
    gc.yaw.name            = "yaw";
    gc.yaw.gear_ratio      = 2.0;
    gc.yaw.use_soft_limits = false;
    gc.pitch.name          = "pitch";
    gc.pitch.gear_ratio    = 1.0;
    gc.pitch.min_deg       = -30.0;
    gc.pitch.max_deg       = 30.0;
    gc.pitch.rate_accel_dps2 = 600.0;
    gc.enable_delay_s        = 0.05;

    Gimbal       g(yaw_motor, pitch_motor, gc);
    PitchSession sess;
    PitchSession::Params sp;
    bool imu_zero_req = false;
    sp.imu_zero_req   = &imu_zero_req;
    sp.abs_hold       = true;
    sp.abs_kp         = 3.0;

    double tilt = 0.0;      // 底盘倾斜（世界坐标下枪口多出来的仰角）
    bool   zeroed = false;  // IMU 是否已被会话清零

    auto tick = [&](double secs, double pc_rate) {
        const auto t0 = std::chrono::steady_clock::now();
        while (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
                   .count() < secs) {
            // 假 IMU：抬升前 est=0 表示静止位（朝下 40°），清零后 0° = 水平
            sp.imu_ok        = true;
            sp.imu_pitch_deg = zeroed ? (g.pitch().estAngleDeg() + tilt)
                                      : (g.pitch().estAngleDeg() - 40.0);
            bool         send = false;
            const double pr =
                sess.step(0.01, /*udp_live=*/true, /*udp_hold=*/false, pc_rate,
                          g.pitch(), sp, &send);
            if (send) g.setRate(0.0, pr);
            g.step(0.01);
            if (imu_zero_req) { imu_zero_req = false; zeroed = true; }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    };

    tick(4.0, 0.0);   // 抬升：应把朝下 40° 的枪口抬到水平（est 到 40 后清零）
    CHECK(sess.mode() == PitchSession::Mode::kActive, "IMU 抬升后进入 Active");
    CHECK(zeroed, "抬升到位后 IMU 被清零（0° = 水平）");
    CHECK(std::fabs(g.pitch().estAngleDeg()) <= 3.0,
          "抬升位 = 水平: est=" + std::to_string(g.pitch().estAngleDeg()));

    tick(1.0, 0.0);   // 稳定一下
    {
        const double est0 = g.pitch().estAngleDeg();
        tilt = 8.0;   // 底盘抬头 8°
        tick(2.0, 0.0);
        const double est1 = g.pitch().estAngleDeg();
        // 保持绝对仰角 => est 应该向 -8° 运动
        CHECK(std::fabs((est1 - est0) + 8.0) < 2.5,
              "底盘倾斜 8° 后枪口补偿: est 变化=" + std::to_string(est1 - est0));
        CHECK(std::fabs(est1 + tilt) < 2.5,
              "枪口绝对仰角保持 0°: 绝对角=" + std::to_string(est1 + tilt));
    }
}

// ---------------------------------------------------------------------------
static int runImuProbe(const Options& o) {
    const std::string dev = o.imu_dev.empty() ? "/dev/i2c-1" : o.imu_dev;
    Icm42688I2c::Config ic;
    ic.dev  = dev;
    ic.addr = o.imu_addr;
    Icm42688I2c imu(ic);
    std::printf("[probe] 打开 %s addr=0x%02X ...\n", dev.c_str(), o.imu_addr);
    if (!imu.open()) {
        std::printf("[probe] 失败：%s\n", imu.lastError().c_str());
        return 1;
    }
    std::printf("[probe] WHO_AM_I = 0x%02X（ICM-42688-P 应为 0x47）\n", imu.whoAmI());

    ImuPitch::Config icfg;
    icfg.invert     = o.imu_invert;
    icfg.gyro_axis  = o.imu_gyro_axis;
    icfg.offset_deg = o.imu_offset;
    icfg.alpha      = o.imu_alpha;
    ImuPitch f;
    f.setConfig(icfg);

    for (int i = 0; i < 30; ++i) {
        ImuSample s;
        if (imu.read(&s)) {
            f.update(s, 0.02);
            std::printf("  a=(%+.2f,%+.2f,%+.2f)g  w=(%+7.1f,%+7.1f,%+7.1f)°/s  "
                        "俯仰=%6.2f°  加速度角=%6.2f°%s\n",
                        s.ax, s.ay, s.az, s.gx, s.gy, s.gz, f.pitchDeg(),
                        f.rawAccelDeg(), f.confident() ? "" : "  [加速度不可信]");
        } else {
            std::printf("  读失败：%s\n", imu.lastError().c_str());
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return 0;
}

static int runMain(Options o) {
    // 没有 UDP 控制端时 pitch 会话无从启动：退回旧行为，让本地 stdin 可调试
    if (o.no_udp && o.pitch_session) {
        std::printf("[main] --no-udp：pitch 会话自动关闭（本地 stdin 可直接给速率）\n");
        o.pitch_session = false;
    }
    // ---- 后端：云台（串口 或 CAN）+ 底盘（统一串口 或 CAN）----
    //   典型组合：
    //     默认（全部串口）           ./build/gimbal
    //     底盘 CAN / 云台 UART      ./build/gimbal --chassis can0
    //     全部 CAN                  ./build/gimbal --device can0
    //     yaw CAN / pitch UART      ./build/gimbal --yaw-can can0
    const bool split      = !o.yaw_can.empty();               // 仅 yaw 走 CAN
    const bool gimbal_can = (o.device.rfind("can", 0) == 0) && !split;

    if (split && o.device.rfind("can", 0) == 0) {
        std::printf("[main] --yaw-can 与 --device canX 冲突（后者表示两轴同走 CAN）\n");
        return 1;
    }

    bool        chassis_can   = false;
    std::string chassis_iface = o.device;
    if (!o.no_chassis) {
        if (o.chassis.empty()) {
            chassis_can   = gimbal_can;                  // 自动：两轴同 CAN -> 底盘同 CAN
            chassis_iface = gimbal_can ? o.device : "";
        } else if (o.chassis.rfind("can", 0) == 0) {
            if (gimbal_can) {
                std::printf("[main] 云台已走 CAN（%s），底盘直接共用该 CAN 即可（去掉 --chassis）\n",
                            o.device.c_str());
                return 1;
            }
            chassis_can   = true;
            chassis_iface = o.chassis;
        } else {
            if (gimbal_can) {
                std::printf("[main] 云台走 CAN 时底盘只能走 CAN（--chassis can0）或 --no-chassis\n");
                return 1;
            }
            chassis_can = false;                         // 统一串口
        }
    }

    std::shared_ptr<ITransport>      yaw_bus;
    std::shared_ptr<ITransport>      pitch_bus;
    std::shared_ptr<UnifiedSerial>   serial_link;
    std::unique_ptr<CanChassisLink>  can_chassis;

    if (split) {
        // pitch + 底盘：串口
        UnifiedSerial::Config ucc;
        ucc.device          = o.device;
        ucc.baud            = o.baud;
        ucc.zdt_gap_ms      = o.tx_gap_ms;   // 指令间隔：速度帧要比遥测更及时
        ucc.chassis_enabled = !o.no_chassis && !chassis_can;   // 底盘走 CAN 时串口只跑云台
        serial_link = std::make_shared<UnifiedSerial>(ucc);
        if (!serial_link->open()) {
            std::printf("[main] 打开 %s 失败：%s\n", o.device.c_str(),
                        serial_link->stats().last_error.c_str());
            return 1;
        }
        pitch_bus = serial_link;
        std::printf("[main] pitch 后端: 串口 %s%s\n", o.device.c_str(),
                    ucc.chassis_enabled ? "（底盘也走此串口）" : "（只跑云台 ZDT）");

        // yaw：CAN
        CanTransport::Config ctc;
        ctc.iface = o.yaw_can;
        auto yb = std::make_shared<CanTransport>(ctc);
        if (!yb->open()) {
            std::printf("[main] 打开 %s 失败：%s\n", o.yaw_can.c_str(),
                        yb->stats().last_error.c_str());
            return 1;
        }
        yaw_bus = yb;
        std::printf("[main] yaw 后端: SocketCAN %s（ZDT 扩展帧）\n", o.yaw_can.c_str());
    } else if (gimbal_can) {
        CanTransport::Config ctc;
        ctc.iface = o.device;
        auto b = std::make_shared<CanTransport>(ctc);
        if (!b->open()) {
            std::printf("[main] 打开 %s 失败：%s\n", o.device.c_str(),
                        b->stats().last_error.c_str());
            return 1;
        }
        yaw_bus = pitch_bus = b;
        std::printf("[main] 云台后端: SocketCAN %s（ZDT 扩展帧，两轴同总线）\n", o.device.c_str());
    } else {
        UnifiedSerial::Config ucc;
        ucc.device          = o.device;
        ucc.baud            = o.baud;
        ucc.zdt_gap_ms      = o.tx_gap_ms;   // 指令间隔：速度帧要比遥测更及时
        ucc.chassis_enabled = !o.no_chassis && !chassis_can;   // 底盘走 CAN 时串口只跑云台
        serial_link = std::make_shared<UnifiedSerial>(ucc);
        if (!serial_link->open()) {
            std::printf("[main] 打开 %s 失败：%s\n", o.device.c_str(),
                        serial_link->stats().last_error.c_str());
            return 1;
        }
        yaw_bus = pitch_bus = serial_link;
        std::printf("[main] 云台后端: 串口 %s%s\n", o.device.c_str(),
                    ucc.chassis_enabled ? "（底盘也走此串口）" : "（只跑云台 ZDT）");
    }

    if (!o.no_chassis && chassis_can) {
        CanChassisLink::Config cc;
        cc.iface = chassis_iface;
        can_chassis = std::make_unique<CanChassisLink>(cc);
        if (!can_chassis->open()) {
            std::printf("[main] CAN 底盘链路打开失败：%s\n",
                        can_chassis->stats().last_error.c_str());
            return 1;
        }
        std::printf("[main] 底盘后端: SocketCAN %s（标准帧）\n", chassis_iface.c_str());
    }

    // ---- 两台电机（可分别挂在不同的总线上）----
    ZdtMotor::Config yc;
    yc.addr           = o.yaw_addr;
    yc.pulses_per_rev = o.pulses_per_motor_rev;
    yc.acc_default    = static_cast<uint8_t>(o.acc);
    ZdtMotor::Config pc = yc;
    pc.addr = o.pitch_addr;

    auto yaw_motor   = std::make_shared<ZdtMotor>(yaw_bus,   yc);
    auto pitch_motor = std::make_shared<ZdtMotor>(pitch_bus, pc);

    // ---- 云台 ----
    Gimbal::Config gc;
    gc.yaw.name            = "yaw";
    gc.yaw.gear_ratio      = o.gear;
    gc.yaw.max_rate_dps    = o.yaw_rate;
    gc.yaw.use_soft_limits = !o.yaw_no_limit;      // 导电滑环：默认无限位
    gc.yaw.min_deg         = o.yaw_min;
    gc.yaw.max_deg         = o.yaw_max;

    gc.pitch.name          = "pitch";
    gc.pitch.gear_ratio    = o.pitch_gear;
    gc.pitch.min_deg       = o.pitch_min;
    gc.pitch.max_deg       = o.pitch_max;
    gc.pitch.max_rate_dps  = o.pitch_rate;
    gc.auto_enable         = true;

    // ---- 手感：两轴同一套口径，pitch 是 1 RPM=6°/s、yaw 是 3°/s，坡道按"输出°/s"
    //      统一，看起来才一致 ----
    gc.yaw.rate_accel_dps2   = o.yaw_ramp;
    gc.pitch.rate_accel_dps2 = o.pitch_ramp;
    gc.yaw.rate_hold_s       = o.hold_delay;
    gc.pitch.rate_hold_s     = o.hold_delay;
    gc.yaw.rate_decay_tau    = o.hold_tau;
    gc.pitch.rate_decay_tau  = o.hold_tau;
    gc.yaw.rate_deadband_dps = o.deadband;
    gc.pitch.rate_deadband_dps = o.deadband;

    Gimbal gimbal(yaw_motor, pitch_motor, gc);

    // ---- 底盘：与云台共用同一条串口（--no-chassis 时只控云台）----
    IChassisLink* chassis = nullptr;
    if (!o.no_chassis) {
        if (can_chassis)      chassis = can_chassis.get();
        else if (serial_link) chassis = serial_link.get();
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
            if (o.pitch_session) {
                std::printf("[main] 无 UDP 操作端：pitch 会话关闭，本地 stdin 可直接给速率\n");
                o.pitch_session = false;
            }
        }
    }

    // ---- 水弹波箱开火链路（F401 USART1，PA12 -> MOS）----
    std::unique_ptr<UartFireLink> fire;
    if (!o.no_fire) {
        UartFireLink::Config fcfg;
        fcfg.device = o.fire_dev;
        fcfg.baud   = o.fire_baud;
        fire.reset(new UartFireLink(fcfg));
        if (!fire->open()) {
            std::printf("[main] 警告：开火串口 %s 打开失败（%s）——后台每 0.5 s 重试，"
                        "云台/底盘不受影响\n",
                        o.fire_dev.c_str(), fire->stats().last_error.c_str());
        }
    }

    // ---- IMU（ICM-42688-P，I²C）：给 pitch 提供绝对仰角 ----
    std::unique_ptr<IImu> imu;
    ImuPitch              imu_pitch;
    bool                  imu_zero_req     = false;
    int                   imu_fail         = 0;
    double                next_imu_retry_s = 0.0;
    // 陀螺零偏：0 = 开机自动标定（2 s 窗口，且要求数据安静）
    bool                  imu_bias_done    = (o.imu_gyro_bias != 0.0);
    int                   imu_bias_n       = 0;
    double                imu_bias_sum     = 0.0;
    double                imu_bias_sum2    = 0.0;
    const int             kImuBiasWin      = 200;   // 100 Hz × 2 s
    if (!o.imu_dev.empty()) {
        ImuPitch::Config icfg;
        icfg.invert     = o.imu_invert;
        icfg.gyro_axis  = o.imu_gyro_axis;
        icfg.offset_deg = o.imu_offset;
        icfg.alpha      = o.imu_alpha;
        imu_pitch.setConfig(icfg);

        Icm42688I2c::Config ic;
        ic.dev  = o.imu_dev;
        ic.addr = o.imu_addr;
        auto dev = std::make_unique<Icm42688I2c>(ic);
        if (!dev->open()) {
            std::printf("[main] IMU 打开失败（%s）：%s —— 回退纯编码器方案\n",
                        o.imu_dev.c_str(), dev->lastError().c_str());
        } else {
            imu = std::move(dev);   // 上转到 IImu*
        }
    }

    signal(SIGINT, onSignal);
    signal(SIGTERM, onSignal);

    const int fl = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, fl | O_NONBLOCK);

    std::printf("========================================================\n");
    std::printf("  gimbal 云台控制器\n");
    std::printf("  yaw 总线 : %s\n", yaw_bus->name().c_str());
    std::printf("  pitch 总线: %s\n", pitch_bus->name().c_str());
    std::printf("  yaw     : addr=%u 传动比 %.2f %s 最大 %.0f°/s\n",
                o.yaw_addr, o.gear,
                o.yaw_no_limit ? "无限位(导电滑环)" : "有软限位", o.yaw_rate);
    std::printf("  pitch   : addr=%u 传动比 %.2f 限位[%.0f, %.0f] 最大 %.0f°/s\n",
                o.pitch_addr, o.pitch_gear, o.pitch_min, o.pitch_max, o.pitch_rate);
    std::printf("  脉冲/电机圈=%u  加速度档位=%d\n", o.pulses_per_motor_rev, o.acc);
    std::printf("  手感    : 坡道 yaw %.0f / pitch %.0f °/s²  停手 %.2fs/τ%.2fs  "
                "死区 %.1f°/s  总线间隔 %d ms\n",
                o.yaw_ramp, o.pitch_ramp, o.hold_delay, o.hold_tau, o.deadband,
                o.tx_gap_ms);
    std::printf("  控制频率: %.0f Hz   看门狗: %.1f s\n", o.loop_hz,
                gc.command_timeout_s);
    if (link) {
        std::printf("  操作端  : UDP 0.0.0.0:%u\n", o.ctrl_port);
    } else {
        std::printf("  操作端  : 仅本地 stdin\n");
    }
    if (chassis) {
        std::printf("  底盘    : %s\n", chassis->name().c_str());
    } else {
        std::printf("  底盘    : 未接入（--no-chassis）\n");
    }
    if (fire) {
        std::printf("  开火    : %s @%d（F401 USART1 -> PA12 -> MOS）%s\n",
                    o.fire_dev.c_str(), o.fire_baud,
                    fire->isOpen() ? "" : "  [串口未连接，重试中]");
    } else {
        std::printf("  开火    : 未接入（--no-fire）\n");
    }
    if (imu) {
        std::printf("  IMU     : %s @0x%02X（俯仰融合）%s\n", o.imu_dev.c_str(),
                    o.imu_addr, o.pitch_abs ? "  绝对仰角保持=开" : "");
    } else {
        std::printf("  IMU     : 未接入（纯编码器方案）\n");
    }
    if (o.pitch_session) {
        if (imu) {
            std::printf("  pitch 会话: 开（首包后抬到 IMU 真水平并定 0°；%.1f s 无包放下锁死）\n",
                        o.pitch_end_s);
        } else {
            std::printf("  pitch 会话: 开（首包后上抬 %+.0f° 定为 0°；%.1f s 无包放下并锁死）\n",
                        o.pitch_rise_deg, o.pitch_end_s);
        }
    } else {
        std::printf("  pitch 会话: 关（--no-pitch-session，速率直通）\n");
    }
    std::printf("--------------------------------------------------------\n");
    std::printf("  本地命令: y/p/r 角速度  h 停手  home 回中  zero 归零\n");
    std::printf("            en 0|1 使能  x 急停  s 状态  q 退出  f 1|0 开/停火\n");
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

    // ---- 发射扳机：UDP Fire 位（0.3s 内有效）或本地 f 命令 ----
    bool   fire_cmd   = false;   // 最后一帧命令的扳机状态
    double fire_cmd_s = -1e9;    // 该帧到达时刻
    bool   local_fire = false;   // 本地 stdin f 命令
    bool   last_fire_req = false;  // 日志去重：上一拍发给 F401 的扳机状态
    bool   fire_block_warned = false;

    // ---- UDP 控制端状态（会话判定用）----
    bool   udp_seen     = false;   // 收到过控制包
    double last_udp_s   = -1e9;    // 最近一次控制包时刻
    bool   udp_hold     = false;   // 最后一帧的 HOLD 标志
    double udp_yaw_dps  = 0.0;     // 最后一帧的 yaw 角速度
    double udp_pitch_dps = 0.0;    // 最后一帧的 pitch 角速度

    // ---- pitch 会话 ----
    PitchSession sess;
    PitchSession::Params pp;
    pp.rise_deg  = o.pitch_rise_deg;
    pp.rate_max  = o.pitch_rise_rate;
    pp.kp        = o.pitch_servo_kp;
    pp.tol_deg   = o.pitch_servo_tol;
    pp.timeout_s = o.pitch_raise_timeout_s;
    pp.limit_min = o.pitch_min;
    pp.limit_max = o.pitch_max;
    pp.imu_zero_req = &imu_zero_req;
    pp.abs_hold     = o.pitch_abs;
    pp.abs_kp       = o.abs_kp;
    pp.abs_min      = o.pitch_min;
    pp.abs_max      = o.pitch_max;

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
            if (cmd == "y" || cmd == "p" || cmd == "r") {
                if (o.pitch_session) {
                    std::printf("[cmd] 会话模式下 y/p/r 由 PC 控制（调试请用 --no-pitch-session）\n");
                } else if (cmd == "y") {
                    iss >> local_yaw; local_has = true;
                    std::printf("[cmd] yaw=%.1f\n", local_yaw);
                } else if (cmd == "p") {
                    iss >> local_pitch; local_has = true;
                    std::printf("[cmd] pitch=%.1f\n", local_pitch);
                } else {
                    iss >> local_yaw >> local_pitch; local_has = true;
                    std::printf("[cmd] yaw=%.1f pitch=%.1f\n", local_yaw, local_pitch);
                }
            }
            else if (cmd == "h" || cmd == "hold") { gimbal.hold(); local_has = false;
                                   local_yaw = local_pitch = 0; std::printf("[cmd] hold\n"); }
            else if (cmd == "home") { gimbal.home(); std::printf("[cmd] home\n"); }
            else if (cmd == "zero") {
                gimbal.yaw().zeroHere();
                if (o.pitch_session) {
                    std::printf("[cmd] 会话模式：只归零 yaw（pitch 的 0° 由会话在抬升到位时建立）\n");
                } else {
                    gimbal.pitch().zeroHere();
                    std::printf("[cmd] 归零\n");
                }
            }
            else if (cmd == "en") { int v = 1; iss >> v; gimbal.enable(v != 0);
                                   if (v != 0) sess.onLocalEnable();
                                   std::printf("[cmd] enable=%d\n", v); }
            else if (cmd == "x" || cmd == "estop") { gimbal.estop(); local_has = false;
                                   sess.onEstop();
                                   std::printf("[cmd] 急停\n"); }
            else if (cmd == "s" || cmd == "status") {
                printStatus(gimbal, *yaw_bus, *pitch_bus, link.get(), chassis);
                if (o.pitch_session) {
                    std::printf("  pitch 会话: %-8s 起点=%.2f° 目标=%.2f°%s\n",
                                sess.modeName(), sess.restDeg(), sess.targetDeg(),
                                sess.latched() ? "  [已锁死，按 g 关再开重新授权]" : "");
                }
                if (imu) {
                    std::printf("  IMU      : %s 仰角=%.2f° ω=%.1f°/s 加速度角=%.2f°%s\n",
                                imu->isOpen() ? "在线" : "离线",
                                imu_pitch.pitchDeg(), imu_pitch.rateDps(),
                                imu_pitch.rawAccelDeg(),
                                imu_pitch.confident() ? "" : " [加速度不可信]");
                }
            }
            else if (cmd == "f") { int v = 0; iss >> v; local_fire = (v != 0);
                                   std::printf("[cmd] fire=%d\n", local_fire ? 1 : 0); }
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
                echo_seq    = cc.seq;
                udp_seen    = true;
                last_udp_s  = now_s();
                fire_cmd   = (cc.flags & ctrl_flag::Fire) != 0;
                fire_cmd_s = now_s();
                if (cc.estop()) {
                    gimbal.estop();
                    sess.onEstop();          // 会话锁死 + latch，等 g 关再开
                    local_has  = false;
                    local_fire = false;      // 急停一并停火
                    continue;
                }
                if (cc.flags & ctrl_flag::Disable) {
                    gimbal.enable(false);
                    sess.onDisable();
                } else if (cc.enable()) {
                    gimbal.enable(true);
                    sess.onEnable();
                }
                if (cc.zero()) {
                    gimbal.yaw().zeroHere();
                    if (!o.pitch_session) {
                        gimbal.pitch().zeroHere();
                    } else {
                        std::printf("[pitch] 会话模式忽略 z 归零"
                                    "（0° 在抬升到位时自动建立）\n");
                    }
                }
                if (cc.home()) gimbal.home();

                udp_hold      = cc.hold();
                udp_yaw_dps   = cc.yaw_dps;
                udp_pitch_dps = cc.pitch_dps;
                if (cc.hold()) {
                    gimbal.hold();
                    local_has = false;
                } else if (!o.pitch_session) {
                    local_has   = false;      // 远端接管本地设定
                    local_yaw   = cc.yaw_dps;
                    local_pitch = cc.pitch_dps;
                    gimbal.setRate(cc.yaw_dps, cc.pitch_dps);
                } else {
                    local_has = false;        // 会话模式：pitch 由状态机统一下发
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

        // ---- 3. 本地指令下发（会话模式下 y/p/r 已被拒绝，local_has 恒 false）----
        if (!o.pitch_session && local_has) gimbal.setRate(local_yaw, local_pitch);

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

        // ---- 3c. 水弹波箱：UDP 扳机（0.3s 内有效）或本地 f 命令 ----
        if (fire) {
            const bool udp_fire = fire_cmd && (now_s() - fire_cmd_s) <= 0.30;
            // 会话未就绪（抬升/放下/锁死）时不允许开火
            const bool fire_ok =
                !o.pitch_session || sess.mode() == PitchSession::Mode::kActive;
            const bool fire_req = (local_fire || udp_fire) && fire_ok;
            if (!fire_ok && (local_fire || udp_fire) && !fire_block_warned) {
                fire_block_warned = true;
                std::printf("[fire] 会话未就绪（%s），忽略发射请求\n", sess.modeName());
            }
            if (fire_ok) fire_block_warned = false;
            if (fire_req != last_fire_req) {
                std::printf("[fire] %s（F401 %s）\n", fire_req ? "开火" : "停火",
                            fire->linkOk() ? "在线" : "未连");
                last_fire_req = fire_req;
            }
            fire->setFire(fire_req);
            fire->poll();     // 收发一拍（断线时内部自动重连）
        }

        // ---- 3c2. IMU：读一帧、融合成枪管绝对仰角（控制拍同频）----
        bool   imu_ok  = false;
        double imu_deg = 0.0;
        if (imu) {
            const double t_imu = now_s();
            if (!imu->isOpen()) {
                if (t_imu >= next_imu_retry_s) {
                    next_imu_retry_s = t_imu + 5.0;
                    if (imu->open()) std::printf("[imu] 重连成功\n");
                }
            } else {
                ImuSample ismp;
                if (imu->read(&ismp)) {
                    if (!imu_bias_done) {
                        // 开机静止 2 s 估陀螺零偏；期间有明显运动（方差大）就重来
                        const double g = (o.imu_gyro_axis == 0) ? ismp.gx
                                       : (o.imu_gyro_axis == 1) ? ismp.gy : ismp.gz;
                        imu_bias_sum  += g;
                        imu_bias_sum2 += g * g;
                        if (++imu_bias_n >= kImuBiasWin) {
                            const double mean = imu_bias_sum / imu_bias_n;
                            const double var  = imu_bias_sum2 / imu_bias_n - mean * mean;
                            const double sd   = (var > 0.0) ? std::sqrt(var) : 0.0;
                            if (sd < 1.5) {
                                ImuPitch::Config c = imu_pitch.cfg();
                                c.gyro_bias_dps = mean;
                                imu_pitch.setConfig(c);
                                imu_pitch.reset();   // 用扣掉零偏的数据重新播种
                                imu_bias_done = true;
                                std::printf("[imu] 陀螺零偏标定: %+.2f°/s（轴 %d，σ=%.2f）\n",
                                            mean, o.imu_gyro_axis, sd);
                            } else {
                                imu_bias_n = 0;
                                imu_bias_sum = imu_bias_sum2 = 0.0;
                                std::printf("[imu] 标定期间检测到运动（σ=%.2f°/s），"
                                            "重新标定\n", sd);
                            }
                        }
                    } else {
                        imu_pitch.update(ismp, dt_target);
                        imu_ok   = true;
                        imu_deg  = imu_pitch.pitchDeg();
                    }
                    imu_fail = 0;
                } else if (++imu_fail > 50) {
                    imu_fail = 0;
                    imu->close();
                    std::printf("[imu] 连续读失败，5 s 后重试（期间回退编码器方案）\n");
                }
            }
        }
        if (imu_zero_req) {
            imu_pitch.zeroHere();
            imu_zero_req = false;
            std::printf("[pitch] IMU 已在当前姿态清零（0° = 真水平）\n");
        }

        // ---- 3d. 云台速率：pitch 走会话状态机，yaw 跟随 PC ----
        if (o.pitch_session) {
            const double t_now    = now_s();
            const bool   udp_live =
                udp_seen && (t_now - last_udp_s) <= o.pitch_end_s;
            bool send = false;
            pp.imu_ok        = imu_ok;
            pp.imu_pitch_deg = imu_deg;
            const double pr = sess.step(dt_target, udp_live, udp_hold,
                                        udp_pitch_dps, gimbal.pitch(), pp, &send);
            if (send) {
                const double yr = (udp_live && !udp_hold) ? udp_yaw_dps : 0.0;
                gimbal.setRate(yr, pr);   // 每拍下发 => 刷新命令时钟，看门狗不会打断过渡
            }
        }

        // ---- 4. 控制一拍 ----
        gimbal.step(dt_target);

        // ---- 5. 把最新遥测交给链路（链路按频率自动回发）----
        if (link) {
            link->setTelemetry(
                buildTelemetry(gimbal, echo_seq, link->stats().rx_hz, chassis));
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
    if (fire) {
        fire->setFire(false);           // 先停火
        for (int i = 0; i < 20; ++i) {  // 发 0.1 s 停火帧，确认 F401 收到
            fire->poll();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        fire->close();
    }
    // 会话开着且不在 Locked：先把枪口按放下逻辑走完（最多 6 s），再抱死
    if (o.pitch_session && sess.mode() != PitchSession::Mode::kLocked) {
        std::printf("[pitch] 退出：放下枪口并锁死（最多 6 s）\n");
        sess.prepareExit(gimbal.pitch(), pp);
        bool send = false;
        for (int i = 0; i < 600 && sess.mode() != PitchSession::Mode::kLocked; ++i) {
            const double pr = sess.step(dt_target, /*udp_live=*/false,
                                        /*udp_hold=*/false, 0.0, gimbal.pitch(),
                                        pp, &send);
            gimbal.setRate(0.0, pr);
            gimbal.step(dt_target);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
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
        else if (a == "--yaw-addr")    o.yaw_addr = static_cast<uint8_t>(nexti(1));
        else if (a == "--pitch-addr")  o.pitch_addr = static_cast<uint8_t>(nexti(2));
        else if (a == "--gear")        o.gear = next(o.gear);
        else if (a == "--pitch-gear")  o.pitch_gear = next(o.pitch_gear);
        else if (a == "--pulses")      o.pulses_per_motor_rev = static_cast<uint32_t>(nexti(3200));
        else if (a == "--yaw-limit")   { o.yaw_min = next(o.yaw_min); o.yaw_max = next(o.yaw_max);
                                         o.yaw_no_limit = false; }
        else if (a == "--pitch-limit") { o.pitch_min = next(o.pitch_min); o.pitch_max = next(o.pitch_max); }
        else if (a == "--yaw-rate")    o.yaw_rate = next(o.yaw_rate);
        else if (a == "--pitch-rate")  o.pitch_rate = next(o.pitch_rate);
        else if (a == "--acc")         o.acc = nexti(200);
        else if (a == "--ctrl-port")   o.ctrl_port = static_cast<uint16_t>(nexti(CTRL_DEFAULT_PORT));
        else if (a == "--can" && i + 1 < argc) o.device = argv[++i];
        else if (a == "--yaw-can" && i + 1 < argc) o.yaw_can = argv[++i];
        else if (a == "--chassis" && i + 1 < argc) o.chassis = argv[++i];
        else if (a == "--no-udp")      o.no_udp = true;
        else if (a == "--hz")          o.loop_hz = next(o.loop_hz);
        else if (a == "--no-chassis")   o.no_chassis = true;
        else if (a == "--fire-dev" && i + 1 < argc)   o.fire_dev = argv[++i];
        else if (a == "--fire-baud")                   o.fire_baud = nexti(o.fire_baud);
        else if (a == "--no-fire")                     o.no_fire = true;
        else if (a == "--imu")                         o.imu_dev = argv[++i];
        else if (a == "--imu-addr" && i + 1 < argc)    o.imu_addr = static_cast<int>(std::strtol(argv[++i], nullptr, 0));
        else if (a == "--imu-invert")                  o.imu_invert = true;
        else if (a == "--imu-gyro-axis" && i + 1 < argc) {
            const std::string t = argv[++i];
            o.imu_gyro_axis = (t == "x") ? 0 : (t == "y") ? 1 : 2;
        }
        else if (a == "--imu-offset")                  o.imu_offset = next(o.imu_offset);
        else if (a == "--imu-alpha")                   o.imu_alpha = next(o.imu_alpha);
        else if (a == "--imu-gyro-bias")               o.imu_gyro_bias = next(o.imu_gyro_bias);
        else if (a == "--imu-probe")                   o.imu_probe = true;
        else if (a == "--pitch-abs")                   o.pitch_abs = true;
        else if (a == "--abs-kp")                      o.abs_kp = next(o.abs_kp);
        else if (a == "--tx-gap")                      o.tx_gap_ms = nexti(o.tx_gap_ms);
        else if (a == "--yaw-ramp")                    o.yaw_ramp = next(o.yaw_ramp);
        else if (a == "--pitch-ramp")                  o.pitch_ramp = next(o.pitch_ramp);
        else if (a == "--hold-delay")                  o.hold_delay = next(o.hold_delay);
        else if (a == "--hold-tau")                    o.hold_tau = next(o.hold_tau);
        else if (a == "--deadband")                    o.deadband = next(o.deadband);
        else if (a == "--no-pitch-session")            o.pitch_session = false;
        else if (a == "--pitch-rise")                  o.pitch_rise_deg = next(o.pitch_rise_deg);
        else if (a == "--pitch-rise-rate")             o.pitch_rise_rate = next(o.pitch_rise_rate);
        else if (a == "--pitch-session-end")           o.pitch_end_s = next(o.pitch_end_s);
        else if (a == "--pitch-servo-kp")              o.pitch_servo_kp = next(o.pitch_servo_kp);
        else if (a == "--pitch-servo-tol")             o.pitch_servo_tol = next(o.pitch_servo_tol);
        else if (a == "--pitch-raise-timeout")         o.pitch_raise_timeout_s = next(o.pitch_raise_timeout_s);
        else if (a == "--selftest")    o.selftest = true;
        else { std::printf("未知参数: %s\n", a.c_str()); usage(argv[0]); return 2; }
    }
    if (o.imu_probe) return runImuProbe(o);
    if (o.selftest) return runSelftest();
    return runMain(o);
}
