#pragma once
#include <math.h>
#include "flight_controller.hpp"

//===================================================================================================
// flight_control_mixer.hpp
// 混控器 (control allocation): 把 FlightController 的输出
//   (总油门 + 三轴期望力矩) 转换为四个电机的实际油门 (0~1).
//
// 原理 (X 布局, 机体系 x 前 / y 左 / z 上):
//   - 总油门: 四个电机共同承担;
//   - roll  力矩: 左(M0,M3) 与右(M1,M2) 差动;
//   - pitch 力矩: 前(M0,M1) 与后(M2,M3) 差动;
//   - yaw   力矩: 顺时针(M0,M2) 与逆时针(M1,M3) 差动 (旋翼反力矩).
//   差动量由几何关系换算: 单电机油门单位推力 k = (m*g/hover)/4,
//     roll/pitch 满差动对应的力矩 = 2*sqrt(2)*L*k,
//     yaw     满差动对应的力矩 = 4*YAW_TORQUE_ARM_M*k.
//
// 饱和处理: 推力优先. 当差动导致某个电机越界时, 按比例收缩三轴差动,
//   保证总油门不变 (牺牲力矩而不是推力).
//===================================================================================================

namespace dlx
{
    // 四电机油门输出 (0~1), 下标对应 MotorId (M0 左前, 顺时针 M0..M3)
    struct FlightControlMotorOutput
    {
        float motor[static_cast<size_t>(MotorId::COUNT)]; // 0~1
        float mix_scale{1.0f}; // 混控缩放: 1=未饱和; <1 表示差动(力矩)被收缩以保住总油门
    };

    // 将控制输出(总油门 + 三轴期望力矩)转换为四电机油门 (0~1)
    inline FlightControlMotorOutput mixMotors(const FlightControlOutput &out, const FlightControlParams &params)
    {
        FlightControlMotorOutput res;

        const float L = params.get(FlightParamId::ARM_LENGTH_M);
        const float mass = params.get(FlightParamId::MASS_KG);
        const float hover = params.get(FlightParamId::HOVER_THROTTLE);
        const float yaw_arm = params.get(FlightParamId::YAW_TORQUE_ARM_M);

        // 单电机油门单位推力 [N]
        const float k = (mass * 9.80665f / hover) / 4.0f;

        // 力矩 -> 每个电机的差动油门量 (正 = 增加油门)
        const float roll_d = out.torque.x() / (2.0f * 1.41421356f * L * k);
        const float pitch_d = out.torque.y() / (2.0f * 1.41421356f * L * k);
        const float yaw_d = out.torque.z() / (4.0f * yaw_arm * k);

        const float T = out.throttle;

        // 组合:  M0 左前(+x,+y)  M1 右前(+x,-y)  M2 右后(-x,-y)  M3 左后(-x,+y)
        // roll :   +              -              -              +
        // pitch:   -              -              +              +
        // yaw  :   +              -              +              -   (M0/M2 顺时针)
        float d[4];
        d[0] = roll_d - pitch_d + yaw_d;
        d[1] = -roll_d - pitch_d - yaw_d;
        d[2] = -roll_d + pitch_d + yaw_d;
        d[3] = roll_d + pitch_d - yaw_d;

        // 推力优先缩放: 使 T + s*d[i] 落在 [0,1] 内
        float d_max = 0.0f;
        float d_min = 0.0f;
        for (int i = 0; i < 4; i++) {
            if (d[i] > d_max) {
                d_max = d[i];
            }
            if (d[i] < d_min) {
                d_min = d[i];
            }
        }

        float s = 1.0f;
        if (d_max > 0.0f && (T + d_max) > 1.0f) {
            s = min(s, (1.0f - T) / d_max);
        }
        if (d_min < 0.0f && (T + d_min) < 0.0f) {
            s = min(s, T / (-d_min));
        }
        res.mix_scale = s;

        for (int i = 0; i < 4; i++) {
            res.motor[i] = constrain(T + s * d[i], 0.0f, 1.0f);
        }
        return res;
    }

} // namespace dlx
