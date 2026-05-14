#include "stdint.h"
#pragma once

/*=================================================================================================*/
#define strWithLen(s) (uint8_t *)s, strlen(s)

/*=================================================================================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*=================================================================================================*/

/*=================================================================================================*/

extern volatile uint32_t SystemClockMilliseconds;

extern uint32_t globalErrorCode;

/*=================================================================================================*/

struct DataTransferGroudStation {
};

/*=================================================================================================*/

// namespace dl
// {
//     class DShot;
// } // namespace dl
// extern dl::DShot foc;
extern uint16_t throttleValue[4];

extern float throttleValueF[4];

/*=================================================================================================*/

// SD卡参数
/*=================================================================================================*/

extern uint32_t DL_SD_CardBlockSize;
extern uint64_t DL_SD_CardCapacity;
extern uint8_t DL_SD_CardType;

/*=================================================================================================*/

/*IMU数据*/
extern volatile float sampleFreq;
/*=================================================================================================*/

namespace dl
{
    struct IMUGData;
}
extern dl::IMUGData mpu9250_data;

/*=================================================================================================*/

/*姿态解算算法MadgwickAHRS的全局变量声明*/
/*=================================================================================================*/
extern volatile float beta;             // algorithm gain
extern volatile float q0, q1, q2, q3;   // quaternion of sensor frame relative to auxiliary frame
extern volatile float roll, pitch, yaw; // Euler angles of sensor frame relative to auxiliary frame

inline void convertQuantToEuler()
{
    pitch = fmod(asinf(-2 * q1 * q3 + 2 * q0 * q2) * 57.3f + 180, 360);
    roll  = fmod((atan2f(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3f + 180), 360);
    yaw   = fmod(atan2f(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3f + 180, 360);
}

/*=================================================================================================*/

/*PID参数*/
/*=================================================================================================*/

// 默认电机摆放.↓面向
//    x    //
/*---------*/
/*X0  |  X1*/
/*----+----*/ /*yaw*/
/*X2  |  X3*/
/*---------*/
/*   roll  */

extern uint32_t last_pid_time;
extern float dt;

extern float last_lose_roll_rate;
extern float integral_delta_roll_rate; // 积分值

extern float target_roll_rate; // 目标角速度(弧度/s),控制y方向角速度
extern float last_lose_roll;
extern float integral_delta_roll; // 积分值

extern float target_roll; // 目标角度(弧度),控制y方向加速度g*tan(roll)

extern float last_lose_pitch_rate;
extern float integral_delta_pitch_rate; // 积分值

extern float target_pitch_rate; // 目标角速度(弧度/s),控制x方向角速度
extern float last_lose_pitch;
extern float integral_delta_pitch; // 积分值

extern float target_pitch; // 目标角度(弧度),控制x方向加速度g*tan(pitch)

extern float last_lose_yaw_rate;
extern float integral_delta_yaw_rate; // 积分值

extern float target_yaw_rate; // 目标角速度(弧度/s),控制z方向角速度
extern float last_lose_yaw;
extern float integral_delta_yaw; // 积分值

extern float target_yaw; // 目标角度(弧度).

// 实际角速度直接访问IMU数据
#define roll_rate  mpu9250_data.gyro[1]
#define pitch_rate mpu9250_data.gyro[2]
#define yaw_rate   mpu9250_data.gyro[0]

// 这些是目标量,定义在main或control中.

extern float target_z; // 目标z速度(m/s)

#define PID_THROTTLE_LIMIT_MAX          1900
#define PID_THROTTLE_LIMIT_MIN          200

#define PID_ROLL_RATE_KP                0.01f   // 滚转角速度Kp
#define PID_ROLL_RATE_KI                0.0000f // 滚转角速度Ki
#define PID_ROLL_RATE_KD                0.001f  // 滚转角速度Kd
#define PID_ROLL_RATE_ALPHA             10
#define PID_ROLL_THROTTLE_ADJUST_LIMIT  1
#define PID_ROLL_INTEGERAL_LIMIT        100

#define PID_ROLL_KP                     0.01f   // 滚转角Kp
#define PID_ROLL_KI                     0.0000f // 滚转角Ki
#define PID_ROLL_KD                     0.001f  // 滚转角Kd
#define PID_ROLL_ALPHA                  10
#define PID_ROLL_RATE_ADJUST_LIMIT      0.5f // 角速度最大调整量
#define PID_ROLL_RATE_LIMIT             5    // 角速度最大限制

#define PID_ROLL_RATE_INTEGERAL_LIMIT   100

#define PID_ROLL_ADJUST_LIMIT           0.5f
#define PID_ROLL_LIMIT                  30

#define PID_PITCH_RATE_KP               0.01f
#define PID_PITCH_RATE_KI               0.0000f
#define PID_PITCH_RATE_KD               0.001f
#define PID_PITCH_RATE_ALPHA            10
#define PID_PITCH_THROTTLE_ADJUST_LIMIT 1

#define PID_PITCH_INTEGERAL_LIMIT       100

#define PID_PITCH_KP                    0.01f
#define PID_PITCH_KI                    0.0000f
#define PID_PITCH_KD                    0.001f
#define PID_PITCH_ALPHA                 10
#define PID_PITCH_RATE_ADJUST_LIMIT     0.5f
#define PID_PITCH_RATE_LIMIT            5

#define PID_PITCH_RATE_INTEGRAL_LIMIT   100

#define PID_PITCH_ADJUST_LIMIT          0.5f // 最大调整量
#define PID_PITCH_LIMIT                 30

#define PID_YAW_RATE_KP                 0.01f   // 滚转角速度Kp
#define PID_YAW_RATE_KI                 0.0000f // 滚转角速度Ki
#define PID_YAW_RATE_KD                 0.001f  // 滚转角速度Kd
#define PID_YAW_RATE_ALPHA              10
#define PID_YAW_THROTTLE_ADJUST_LIMIT   1

#define PID_YAW_INTEGERAL_LIMIT         100

#define PID_YAW_KP                      0.01f   // 滚转角Kp
#define PID_YAW_KI                      0.0000f // 滚转角Ki
#define PID_YAW_KD                      0.001f  // 滚转角Kd
#define PID_YAW_ALPHA                   10
#define PID_YAW_RATE_ADJUST_LIMIT       0.5f // 角速度最大调整量
#define PID_YAW_RATE_LIMIT              5    // 角速度最大限制

#define PID_YAW_RATE_INTEGERAL_LIMIT    100

/*=================================================================================================*/

namespace dl
{

    namespace BME280
    {
        struct bmeData;
    } // namespace BME280
}

extern dl::BME280::bmeData bmeData;

extern uint16_t height;

/*=================================================================================================*/

#ifdef __cplusplus
}
#endif