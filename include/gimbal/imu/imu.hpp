// ===========================================================================
//  IMU（ICM-42688-P，I²C）—— 只服务 pitch 轴：给出枪管的"绝对仰角"
//
//  两个抽象：
//    IImu      : 与总线无关的"读一帧"接口；i2c 实现在 src/imu/imu_i2c.cpp
//    ImuPitch  : 俯仰角融合器（陀螺短时 + 加速度长时，互补滤波）
//
//  安装约定（可用命令行参数校正）：
//    · **只有一条硬约定：枪管方向对齐 IMU 的某一条轴**（默认 X 指向枪口）。
//      绕枪管的滚转随意——仰角只用到枪管轴向分量：θ = asin(u·â)。
//    · 俯仰转轴用陀螺的哪一条轴由 --imu-gyro-axis 指定（滚转 90° 的安装
//      就是 Z 轴，见 --imu-probe 的实测）。
//    · 方向反了用 --imu-invert 翻。
//
//  为什么只算 1 个自由度：pitch 轴只有俯仰，自由度越少，融合越稳，
//  安装误差的影响也越小（roll/yaw 不参与这里的控制）。
//
//  注意：加速度计会被车体加减速污染（0.3 g 在 30° 仰角上投影≈0.15 g≈8°）。
//  所以融合里用"|a|≈1g 且角速度小"作为可信门限，超了就只信陀螺短时积分。
// ===========================================================================
#pragma once

#include <string>

namespace gimbal {

struct ImuSample {
    bool   valid = false;
    double t_s   = 0.0;              ///< 采样时刻（steady_clock 秒）
    double ax = 0, ay = 0, az = 0;   ///< 比力，单位 g（IMU 安装坐标系）
    double gx = 0, gy = 0, gz = 0;   ///< 角速度，单位 °/s
};

class IImu {
public:
    virtual ~IImu() = default;
    virtual bool        open()  = 0;
    virtual void        close() = 0;
    virtual bool        isOpen() const = 0;
    virtual std::string name() const = 0;
    /// 读一帧；失败返回 false（调用方可据此重连）
    virtual bool        read(ImuSample* out) = 0;
};

class ImuPitch {
public:
    struct Config {
        double alpha          = 0.98;  ///< 互补系数（100 Hz 下 ≈0.5 s 时间常数）
        double accel_tol_g    = 0.20;  ///< |a|-1 超过它认为有线性加速度，不信加速度
        double accel_gate_dps = 25.0;  ///< 角速度超过它也不信加速度（抖动/转弯）
        double inn_tol_deg    = 10.0;  ///< 创新门限：加速度角与融合角差超过它就不信加速度
        double gyro_bias_dps  = 0.0;   ///< 陀螺零偏（标定后写参数）
        int    accel_axis     = 0;     ///< 枪管方向在 IMU 坐标系中的轴：0=x 1=y 2=z
        int    gyro_axis      = 2;     ///< 俯仰转轴对应的陀螺轴：0=x 1=y 2=z
        bool   invert         = false; ///< 方向反了翻一下
        double offset_deg     = 0.0;   ///< 安装偏置（标定后可写死）
    };

    void          setConfig(const Config& c) { cfg_ = c; }
    const Config& cfg() const { return cfg_; }

    /// 每拍调用；dt 为距上一帧的秒数
    void update(const ImuSample& s, double dt);

    /// 把当前融合角记为 0°（标定"水平"时用）
    void zeroHere();
    /// 清融合状态，下一帧重新用加速度播种
    void reset();

    double pitchDeg() const;       ///< 融合后的枪管仰角（0° = 调用 zeroHere 时的姿态）
    double rawAccelDeg() const;    ///< 纯加速度计角度（没融合，调试用）
    double rateDps() const { return rate_; }
    bool   confident() const { return confident_; }  ///< 上一帧加速度可信
    bool   seeded() const { return seeded_; }

private:
    Config cfg_;
    bool   seeded_    = false;
    bool   confident_ = false;
    double pitch_     = 0.0;   ///< 未减零位的融合角
    double acc_deg_   = 0.0;
    double rate_      = 0.0;
    double zero_deg_  = 0.0;
};

}  // namespace gimbal
