#pragma once
#include <stddef.h>
#include <stdint.h>

//===================================================================================================
// flight_control_params.hpp
// FlightController 的全部参数集中定义于此:
//   - 机身物理布局(电机编号 / 旋转方向 / 机臂方向)
//   - 控制参数(以 enum class FlightParamId 具名声明, 通过参数表读写)
//
// 已知布局:
//   X 布局, 机头为 IMU x+(机头朝上), 从左上角开始顺时针的逻辑电机序号 0 1 2 3:
//             机头 x+
//       M0(左前)   M1(右前)
//            +----+
//            |    |
//            +----+
//       M3(左后)   M2(右后)
//   机体系约定: x 前, y 左, z 上 (右手系)
//===================================================================================================

namespace dlx
{
    // 逻辑电机编号
    enum class MotorId : uint8_t
    {
        M0 = 0, // 左前 (左上角)
        M1 = 1, // 右前
        M2 = 2, // 右后
        M3 = 3, // 左后 (左下角)
        COUNT = 4,
    };

    // 电机旋转方向 (从上方俯视)
    enum class RotorDirection : uint8_t
    {
        CW = 0,  // 顺时针
        CCW = 1, // 逆时针
    };

    // 机臂方向 (机体系归一化单位向量, 乘以 ARM_LENGTH_M 得到电机位置 [m])
    struct MotorArmUnit
    {
        float x;
        float y;
    };

    // 与 MotorId 一一对应: 对角线电机同向, 相邻电机反向 (X 布局标准)
    static constexpr MotorArmUnit kMotorArmUnit[static_cast<size_t>(MotorId::COUNT)] = {
        {0.70710678f, 0.70710678f},  // M0 左前 (+x, +y)
        {0.70710678f, -0.70710678f}, // M1 右前 (+x, -y)
        {-0.70710678f, -0.70710678f}, // M2 右后 (-x, -y)
        {-0.70710678f, 0.70710678f},  // M3 左后 (-x, +y)
    };

    // 与 MotorId 一一对应
    static constexpr RotorDirection kMotorSpinDirection[static_cast<size_t>(MotorId::COUNT)] = {
        RotorDirection::CW,
        RotorDirection::CCW,
        RotorDirection::CW,
        RotorDirection::CCW,
    };

    //================================================================================================
    // 控制参数 ID (具名声明)
    //================================================================================================
    enum class FlightParamId : uint8_t
    {
        // ---- 物理 ----
        ARM_LENGTH_M = 0, // 机臂长度 [m]
        MASS_KG,          // 总质量 [kg]
        INERTIA_XX,       // 绕机体系 x 轴转动惯量 [kg*m^2]
        INERTIA_YY,       // 绕机体系 y 轴转动惯量 [kg*m^2]
        INERTIA_ZZ,       // 绕机体系 z 轴转动惯量 [kg*m^2]
        YAW_TORQUE_ARM_M, // 旋翼偏航反力矩等效力臂 [m] (力矩/推力比, 混控换算用)

        // ---- 推力 ----
        HOVER_THROTTLE, // 悬停油门 (归一化 0~1)
        THROTTLE_MIN,   // 油门下限 (归一化 0~1)
        THROTTLE_MAX,   // 油门上限 (归一化 0~1)
        TILT_MAX_RAD,   // 允许的最大倾角 [rad]

        // ---- 姿态环 (倾转误差 -> 期望角速度) ----
        ATT_ROLL_P,     // roll 角增益
        ATT_PITCH_P,    // pitch 角增益
        ATT_YAW_P,      // yaw 角增益 (预留, 单 IMU 偏航不准, 暂不控制)
        ATT_YAW_WEIGHT, // 偏航权重 [0,1], 现阶段保持 0
        RATE_ROLL_MAX,  // roll 角速度限幅 [rad/s]
        RATE_PITCH_MAX, // pitch 角速度限幅 [rad/s]
        RATE_YAW_MAX,   // yaw 角速度限幅 [rad/s]

        // ---- 角速度环 (角速度误差 -> 期望角加速度) ----
        RATE_ROLL_P,  // roll 角速度比例
        RATE_ROLL_I,  // roll 角速度积分
        RATE_ROLL_D,  // roll 角速度微分 (作用于实测角加速度)
        RATE_PITCH_P, // pitch 角速度比例
        RATE_PITCH_I, // pitch 角速度积分
        RATE_PITCH_D, // pitch 角速度微分
        RATE_YAW_P,   // yaw 角速度比例
        RATE_YAW_I,   // yaw 角速度积分
        RATE_YAW_D,   // yaw 角速度微分
        RATE_ROLL_FF,  // roll 角速度前馈 (预留, 默认 0)
        RATE_PITCH_FF, // pitch 角速度前馈 (预留, 默认 0)
        RATE_YAW_FF,   // yaw 角速度前馈 (预留, 默认 0)
        ANGACC_MAX,    // 期望角加速度限幅 [rad/s^2]
        RATE_INT_LIMIT, // 角速度环积分限幅 [rad/s^2]
        RATE_D_TAU_S,  // 角速度微分项低通时间常数 [s]

        // ---- 高度环 ----
        POS_Z_P,       // 高度误差 -> 期望垂向速度 [1/s]
        VEL_Z_P,       // 垂向速度误差 -> 期望垂向加速度 [1/s]
        VEL_Z_I,       // 垂向速度积分增益 [1/s^2]
        VZ_MAX_UP,     // 最大爬升速度 [m/s]
        VZ_MAX_DOWN,   // 最大下降速度 (正值) [m/s]
        VZ_EST_TAU_S,  // 垂向速度估计低通时间常数 [s]

        COUNT,
    };

    // 参数默认值表, 与 FlightParamId 顺序一一对应
    static constexpr float kFlightControlParamDefaults[static_cast<size_t>(FlightParamId::COUNT)] = {
        0.125f, // ARM_LENGTH_M
        0.45f,  // MASS_KG
        0.004f, // INERTIA_XX
        0.004f, // INERTIA_YY
        0.007f, // INERTIA_ZZ
        0.03f,  // YAW_TORQUE_ARM_M
        0.45f,  // HOVER_THROTTLE
        0.05f,  // THROTTLE_MIN
        1.0f,   // THROTTLE_MAX
        0.6f,   // TILT_MAX_RAD (~34°)
        7.0f,   // ATT_ROLL_P
        7.0f,   // ATT_PITCH_P
        3.0f,   // ATT_YAW_P
        0.0f,   // ATT_YAW_WEIGHT
        6.0f,   // RATE_ROLL_MAX
        6.0f,   // RATE_PITCH_MAX
        3.0f,   // RATE_YAW_MAX
        20.0f,  // RATE_ROLL_P
        5.0f,   // RATE_ROLL_I
        0.02f,  // RATE_ROLL_D
        20.0f,  // RATE_PITCH_P
        5.0f,   // RATE_PITCH_I
        0.02f,  // RATE_PITCH_D
        15.0f,  // RATE_YAW_P
        3.0f,   // RATE_YAW_I
        0.01f,  // RATE_YAW_D
        0.0f,   // RATE_ROLL_FF
        0.0f,   // RATE_PITCH_FF
        0.0f,   // RATE_YAW_FF
        120.0f, // ANGACC_MAX
        10.0f,  // RATE_INT_LIMIT
        0.01f,  // RATE_D_TAU_S
        2.0f,   // POS_Z_P
        3.5f,   // VEL_Z_P
        0.6f,   // VEL_Z_I
        1.5f,   // VZ_MAX_UP
        0.8f,   // VZ_MAX_DOWN
        0.25f,  // VZ_EST_TAU_S
    };

    //================================================================================================
    // 参数表
    //================================================================================================
    class FlightControlParams
    {
    public:
        FlightControlParams()
        {
            resetToDefaults();
        }

        float get(FlightParamId id) const
        {
            return _values[static_cast<size_t>(id)];
        }

        void set(FlightParamId id, float value)
        {
            _values[static_cast<size_t>(id)] = value;
        }

        float &operator[](FlightParamId id)
        {
            return _values[static_cast<size_t>(id)];
        }

        float operator[](FlightParamId id) const
        {
            return _values[static_cast<size_t>(id)];
        }

        void resetToDefaults()
        {
            for (size_t i = 0; i < static_cast<size_t>(FlightParamId::COUNT); i++) {
                _values[i] = kFlightControlParamDefaults[i];
            }
        }

    private:
        float _values[static_cast<size_t>(FlightParamId::COUNT)];
    };

} // namespace dlx
