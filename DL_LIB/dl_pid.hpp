#pragma once
#include "stm32f4xx.h"
#include "dl_imu.hpp"
#include "global.h"
#include "dl_dshot.hpp"
#include "dl_log.h"
uint32_t last_pid_time    = 0;
float dt                  = 0.0f; // 单位 s
uint16_t throttleValue[4] = {200, 200, 200, 200};

float throttleValueF[4] = {200, 200, 200, 200};

inline uint32_t delta_pid_time()
{
    uint32_t delta = SystemClockMilliseconds - last_pid_time;
    last_pid_time  = SystemClockMilliseconds;
    dt             = delta / 1000.0f;
    return delta;
}

inline float limitRange(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}
inline float limitRange(float value, float range)
{
    if (value <= -range) return -range;
    if (value >= range) return range;
    return value;
}
//单位 度
inline float deltaAngle(float target, float actual){
   return (float) atan2(sin((target - actual)/57.2957 ), cos(target - actual)/57.2957);
}

inline void updatePID(dl::DShot *ds)
{
    throttleValueF[0] = limitRange(throttleValueF[0], PID_THROTTLE_LIMIT_MIN, PID_THROTTLE_LIMIT_MAX);
    throttleValueF[1] = limitRange(throttleValueF[1], PID_THROTTLE_LIMIT_MIN, PID_THROTTLE_LIMIT_MAX);
    throttleValueF[2] = limitRange(throttleValueF[2], PID_THROTTLE_LIMIT_MIN, PID_THROTTLE_LIMIT_MAX);
    throttleValueF[3] = limitRange(throttleValueF[3], PID_THROTTLE_LIMIT_MIN, PID_THROTTLE_LIMIT_MAX);
    throttleValue[0] = (uint16_t)throttleValueF[0];
    throttleValue[1] = (uint16_t)throttleValueF[1];
    throttleValue[2] = (uint16_t)throttleValueF[2];
    throttleValue[3] = (uint16_t)throttleValueF[3];
    ds->encodePreLoadDshotData(throttleValue);
}

float last_lose_roll_rate      = 0;
float integral_delta_roll_rate = 0;

inline void pid_roll_rate()
{
    float lose = deltaAngle( target_roll_rate , roll_rate);
    float diff = -(last_lose_roll_rate - lose);
    float inte = integral_delta_roll_rate;

    float sigma = (lose * PID_ROLL_RATE_KP) + (inte * PID_ROLL_RATE_KI) + (diff * PID_ROLL_RATE_KD); // 计算PID输出

    sigma *= PID_ROLL_RATE_ALPHA;
    sigma = limitRange(sigma, PID_ROLL_THROTTLE_ADJUST_LIMIT);
    
    // logger<<"pid-rate-roll "<<lose<<" "<<diff<<" "<<inte<<" "<<sigma<<LCMD::NFLUSH;

    throttleValueF[0] -= sigma;
    throttleValueF[1] -= sigma;
    throttleValueF[2] += sigma;
    throttleValueF[3] += sigma;

    integral_delta_roll_rate += (lose + last_lose_roll_rate) / 2 * dt; // 积分
    integral_delta_roll_rate = limitRange(integral_delta_roll_rate, PID_ROLL_RATE_INTEGERAL_LIMIT); // 限幅
    last_lose_roll_rate = lose;
}
// roll角度由roll角速度控制

float target_roll_rate    = 0; // 向上层传递
float last_lose_roll      = 0;
float integral_delta_roll = 0;

inline void pid_roll()
{
    // 改 02 13
    float lose  = deltaAngle( target_roll , roll);                                                 // 计算误差,target > actual时为正,正反馈
    float diff  = -(roll_rate);                                      // 计算角速度(微分),取负值负反馈
    float inte  = integral_delta_roll;                                                // 积分制,同delta
    float sigma = (lose * PID_ROLL_KP) + (inte * PID_ROLL_KI) + (diff * PID_ROLL_KD); // 计算PID输出
    sigma *= PID_ROLL_ALPHA;
    sigma = limitRange(sigma, PID_ROLL_RATE_ADJUST_LIMIT);
    
    // logger<<"pid-roll "<<lose<<" "<<diff<<" "<<inte<<" "<<sigma<<LCMD::NFLUSH;

    integral_delta_roll += (lose + last_lose_roll) / 2 * dt; // 积分
    integral_delta_roll = limitRange(integral_delta_roll, PID_ROLL_INTEGERAL_LIMIT);
    last_lose_roll = lose;

    target_roll_rate += sigma;
    target_roll_rate = limitRange(target_roll_rate, PID_ROLL_RATE_LIMIT); // 限幅
}

// y方向速度依赖roll角度(加速度)

float target_roll = 0; // 向上层传递

/*=====================================================================================================*/

/*=====================================================================================================*/

float last_lose_pitch_rate;
float integral_delta_pitch_rate = 0;

inline void pid_pitch_rate()
{
    float lose = deltaAngle( target_pitch_rate , pitch_rate);
    float diff = -(last_lose_pitch_rate - lose) / dt;
    float inte = integral_delta_pitch_rate;

    float sigma = (lose * PID_PITCH_RATE_KP) + (inte * PID_PITCH_RATE_KI) + (diff * PID_PITCH_RATE_KD); // 计算PID输出

    sigma *= PID_PITCH_RATE_ALPHA;
    sigma = limitRange(sigma, PID_PITCH_THROTTLE_ADJUST_LIMIT);

    
    // logger<<"pid-rate-pitch "<<lose<<" "<<diff<<" "<<inte<<" "<<sigma<<LCMD::NFLUSH;

    throttleValueF[3] += sigma;
    throttleValueF[1] += sigma;
    throttleValueF[2] -= sigma;
    throttleValueF[0] -= sigma;

    integral_delta_pitch_rate += (lose + last_lose_pitch_rate) / 2 * dt; // 积分
    integral_delta_pitch_rate = limitRange(integral_delta_pitch_rate, PID_PITCH_RATE_INTEGRAL_LIMIT); // 限幅
    last_lose_pitch_rate = lose;
}

float target_pitch_rate    = 0; // 向上层传递
float last_lose_pitch      = 0;
float integral_delta_pitch = 0;

inline void pid_pitch()
{
    // 改 02 13
    float lose  = deltaAngle( target_pitch , pitch);                                                  // 计算误差,target > actual时为正,正反馈
    float diff  = -(pitch_rate) ;                                        // 计算角速度(微分),取负值负反馈
    float inte  = integral_delta_pitch;                                                  // 积分制,同delta
    float sigma = (lose * PID_PITCH_KP) + (inte * PID_PITCH_KI) + (diff * PID_PITCH_KD); // 计算PID输出
    sigma *= PID_PITCH_ALPHA;
    sigma = limitRange(sigma, PID_PITCH_RATE_ADJUST_LIMIT);

    // logger<<"pid-pitch "<<lose<<" "<<diff<<" "<<inte<<" "<<sigma<<LCMD::NFLUSH;


    integral_delta_pitch += (lose + last_lose_pitch) / 2 * dt; // 积分
    integral_delta_pitch = limitRange(integral_delta_pitch, PID_PITCH_INTEGERAL_LIMIT);
    last_lose_pitch = lose;

    target_pitch_rate += sigma;
    target_pitch_rate = limitRange(target_pitch_rate, PID_PITCH_RATE_LIMIT); // 限幅
}

// y方向速度依赖pitch角度(加速度)
float target_pitch = 0; // 向上层传递

/*============================================================================================================*/

float last_lose_yaw_rate;
float integral_delta_yaw_rate = 0;
// 目标值直接设置,无需传递.

inline void pid_yaw_rate()
{
    float lose =deltaAngle( target_yaw_rate , yaw_rate);
    //logger << "pid_yaw_rate lose=" <<lose << LCMD::NFLUSH;
    float diff = -(last_lose_yaw_rate - lose) / dt;
    
    //logger << "pid_yaw_rate diff=" <<diff << LCMD::NFLUSH;
    float inte = integral_delta_yaw_rate;
    
    //logger << "pid_yaw_rate inte=" <<inte << LCMD::NFLUSH;

    float sigma = (lose * PID_YAW_RATE_KP) + (inte * PID_YAW_RATE_KI) + (diff * PID_YAW_RATE_KD); // 计算PID输出


    sigma *= PID_YAW_RATE_ALPHA;
    sigma = limitRange(sigma, PID_YAW_THROTTLE_ADJUST_LIMIT);

    

    throttleValueF[0] += sigma;
    throttleValueF[3] += sigma;

    throttleValueF[2] += sigma;
    throttleValueF[1] -= sigma;

    

    integral_delta_yaw_rate += (lose + last_lose_yaw_rate) / 2 * dt; // 积分
    integral_delta_yaw_rate = limitRange(integral_delta_yaw_rate, PID_YAW_RATE_INTEGERAL_LIMIT);
    last_lose_yaw_rate = lose;
    
}
// yaw角度由yaw角速度控制

float target_yaw_rate    = 0;
float last_lose_yaw      = 0;
float integral_delta_yaw = 0;

inline void pid_yaw()
{
    // 改 02 13
    float lose  =deltaAngle( target_yaw , yaw);                                                // 计算误差,target > actual时为正,正反馈
    float diff  = -(yaw_rate);                                    // 计算角速度(微分),取负值负反馈
    float inte  = integral_delta_yaw;                                              // 积分制,同delta
    float sigma = (lose * PID_YAW_KP) + (inte * PID_YAW_KI) + (diff * PID_YAW_KD); // 计算PID输出
    sigma *= PID_YAW_ALPHA;
    sigma = limitRange(sigma, PID_YAW_RATE_ADJUST_LIMIT);

    

    integral_delta_yaw += (lose + last_lose_yaw) / 2 * dt; // 积分
    integral_delta_yaw_rate = limitRange(integral_delta_yaw_rate, PID_YAW_INTEGERAL_LIMIT);
    last_lose_yaw = lose;

    target_yaw_rate += sigma;
    target_yaw_rate = limitRange(target_yaw_rate, PID_YAW_RATE_LIMIT); // 限幅
}
float target_yaw = 0;

float last_lose_z      = 0;
float integral_delta_z = 0;

#define PID_Z_KP                         0.01f   // z方向速度Kp
#define PID_Z_KI                         0.0001f // z方向速度Ki
#define PID_Z_KD                         0.1f    // z方向速度Kd
#define PID_Z_ALPHA                      10
#define PID_Z_INTEGRAL_LIMIT             100

#define PID_HEIGHT_THROTTLE_ADJUST_LIMIT 100

inline void pid_z()
{
    float lose  = target_z - height;
    float diff  = -(last_lose_z - lose) / dt;
    float inte  = integral_delta_z;
    float sigma = (lose * PID_Z_KP) + (inte * PID_Z_KI) + (diff * PID_Z_KD);

    sigma *= PID_Z_ALPHA;
    sigma = limitRange(sigma, PID_HEIGHT_THROTTLE_ADJUST_LIMIT);

    // 直接改油门
    throttleValueF[0] += sigma;
    throttleValueF[2] += sigma;
    throttleValueF[1] += sigma;
    throttleValueF[3] += sigma;

    integral_delta_z += (lose + last_lose_z) / 2 * dt; // 积分

    limitRange(integral_delta_z, PID_Z_INTEGRAL_LIMIT);

    last_lose_z = lose;
}

void PIDAngleControl(dl::DShot *ds)
{
   // logger << "PIDAngleControl start ds=" << (uint32_t)ds << LCMD::NFLUSH;
    // pid_yaw();
    pid_pitch();
    pid_roll();
    updatePID(ds);
  //  logger << "PIDAngleControl end" << LCMD::NFLUSH;
}
void PIDRateControl()
{
    delta_pid_time(); // 更新 dt，保证速率 PID 使用正确的时间差
  //  logger << "PIDRateControl start dt=" << (double)dt << LCMD::NFLUSH;
    // pid_yaw_rate();
    pid_pitch_rate();
    pid_roll_rate();
   // logger << "PIDRateControl end thr: " << (uint32_t)throttleValueF[0] << " " << (uint32_t)throttleValueF[1] << " " << (uint32_t)throttleValueF[2] << " " << (uint32_t)throttleValueF[3] << LCMD::NFLUSH;
}