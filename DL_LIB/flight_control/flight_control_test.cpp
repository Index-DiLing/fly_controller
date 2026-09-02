//===================================================================================================
// flight_control_test.cpp
// 仅用于 PC 端验证 (g++ / clang++), 不参与单片机工程.
//   g++ -std=c++11 -O2 -I ../.. flight_control_test.cpp -o flight_control_test
//   运行: ./flight_control_test
//
// 内容:
//   1. dlx_math 单元校验 (四元数乘法 / 旋转 / z 轴 / 两向量构造);
//   2. 混控器单元校验 (力矩方向 -> 电机差动方向, 饱和推力优先);
//   3. 悬停仿真 (控制器 -> 混控器 -> 四电机模型): 带倾角 + 高度偏差收敛;
//   4. 偏航不控制验证: 期望偏航改变时飞机保持当前偏航;
//   5. 姿态跟踪: 期望 roll 阶跃后收敛;
//   6. 垂向速度未提供时 (高度差分估计) 同样收敛;
//   7. 扰动恢复与输入鲁棒性;
//   8. 垂向速度估计器: 气压高度 + 四元数 + IMU 融合 (悬停/爬升/倾角/毛刺/鲁棒性).
//   9. 端到端定高: 控制器只拿到 气压+四元数+IMU, 由估计器提供垂向速度完成定高.
//===================================================================================================

#include "flight_control/flight_controller.hpp"
#include "flight_control/flight_control_mixer.hpp"
#include "flight_control/vertical_velocity_estimator.hpp"

#include <cmath>
#include <cstdio>

using dlx::FlightControlOutput;
using dlx::FlightControlSetpoint;
using dlx::FlightControlState;
using dlx::FlightController;
using dlx::Quaternion;
using dlx::Vector3f;

static int g_failures = 0;

#define CHECK(cond)                                                                                                    \
    do {                                                                                                               \
        if (!(cond)) {                                                                                                 \
            g_failures++;                                                                                              \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                                               \
        }                                                                                                              \
    } while (0)

static bool near(float a, float b, float tol)
{
    return std::fabs(a - b) <= tol;
}

//===================================================================================================
// 1. 数学单元校验
//===================================================================================================
static void testMath()
{
    const float s2 = 0.70710678f;

    // 绕 z 轴 90°: (1,0,0) -> (0,1,0)
    {
        const Quaternion q = dlx::quatFromAxisAngle(Vector3f{0.0f, 0.0f, 1.0f}, 1.5707963f);
        CHECK(near(q.data[0], s2, 1e-5f));
        CHECK(near(q.data[3], s2, 1e-5f));
        const Vector3f v = dlx::quatRotate(q, Vector3f{1.0f, 0.0f, 0.0f});
        CHECK(near(v.x(), 0.0f, 1e-4f));
        CHECK(near(v.y(), 1.0f, 1e-4f));
        CHECK(near(v.z(), 0.0f, 1e-4f));
    }

    // 绕 x 轴 90°: 机体 z 轴 -> 世界 (0,-1,0)
    {
        const Quaternion q = dlx::quatFromAxisAngle(Vector3f{1.0f, 0.0f, 0.0f}, 1.5707963f);
        const Vector3f ez = dlx::quatDcmZ(q);
        CHECK(near(ez.x(), 0.0f, 1e-4f));
        CHECK(near(ez.y(), -1.0f, 1e-4f));
        CHECK(near(ez.z(), 0.0f, 1e-4f));
    }

    // 两向量构造: 从 (0,0,1) 到 (0,-1,0) 应为绕 x 轴 90°
    {
        const Quaternion q = dlx::quatFromTwoVectors(Vector3f{0.0f, 0.0f, 1.0f}, Vector3f{0.0f, -1.0f, 0.0f});
        const Vector3f v = dlx::quatRotate(q, Vector3f{0.0f, 0.0f, 1.0f});
        CHECK(near(v.x(), 0.0f, 1e-4f));
        CHECK(near(v.y(), -1.0f, 1e-4f));
        CHECK(near(v.z(), 0.0f, 1e-4f));
    }

    // 共轭 * 原四元数 = 单位四元数
    {
        const Quaternion q = dlx::quatFromAxisAngle(Vector3f{1.0f, -0.5f, 0.3f}, 0.8f);
        const Quaternion id = dlx::quatMul(dlx::quatConjugate(q), q);
        CHECK(near(id.data[0], 1.0f, 1e-5f));
        CHECK(near(id.data[1], 0.0f, 1e-5f));
        CHECK(near(id.data[2], 0.0f, 1e-5f));
        CHECK(near(id.data[3], 0.0f, 1e-5f));
    }

    // 180° 反向退化: 两向量相反时仍应返回有效单位四元数
    {
        const Quaternion q = dlx::quatFromTwoVectors(Vector3f{0.0f, 0.0f, 1.0f}, Vector3f{0.0f, 0.0f, -1.0f});
        const Vector3f v = dlx::quatRotate(q, Vector3f{0.0f, 0.0f, 1.0f});
        CHECK(near(v.z(), -1.0f, 1e-4f));
    }

    std::printf("[math] done\n");
}

//===================================================================================================
// 2. 混控器单元校验
//===================================================================================================
static void testMixer()
{
    dlx::FlightControlParams p;
    FlightControlOutput out;
    out.throttle = 0.5f;

    // 纯 roll 正力矩: 左(M0,M3) 增, 右(M1,M2) 减
    out.torque = Vector3f{0.05f, 0.0f, 0.0f};
    {
        const dlx::FlightControlMotorOutput mo = dlx::mixMotors(out, p);
        CHECK(mo.motor[0] > mo.motor[1]);
        CHECK(mo.motor[3] > mo.motor[2]);
        CHECK(mo.motor[0] > 0.5f);
        CHECK(mo.motor[1] < 0.5f);
        CHECK(std::fabs(mo.motor[0] - mo.motor[3]) < 1e-4f); // 同侧电机差动量相同
        CHECK(std::fabs(mo.motor[1] - mo.motor[2]) < 1e-4f);
    }

    // 纯 pitch 正力矩: 后(M2,M3) 增, 前(M0,M1) 减
    out.torque = Vector3f{0.0f, 0.05f, 0.0f};
    {
        const dlx::FlightControlMotorOutput mo = dlx::mixMotors(out, p);
        CHECK(mo.motor[2] > mo.motor[0]);
        CHECK(mo.motor[3] > mo.motor[1]);
        CHECK(std::fabs(mo.motor[0] - mo.motor[1]) < 1e-4f);
        CHECK(std::fabs(mo.motor[2] - mo.motor[3]) < 1e-4f);
    }

    // 纯 yaw 正力矩: 顺时针(M0,M2) 增, 逆时针(M1,M3) 减
    out.torque = Vector3f{0.0f, 0.0f, 0.01f};
    {
        const dlx::FlightControlMotorOutput mo = dlx::mixMotors(out, p);
        CHECK(mo.motor[0] > mo.motor[1]);
        CHECK(mo.motor[2] > mo.motor[3]);
        CHECK(std::fabs(mo.motor[0] - mo.motor[2]) < 1e-4f);
        CHECK(std::fabs(mo.motor[1] - mo.motor[3]) < 1e-4f);
    }

    // 饱和: 大油门 + 大力矩 -> 电机不越界, 总油门不变 (推力优先)
    out.throttle = 0.9f;
    out.torque = Vector3f{0.3f, 0.3f, 0.05f};
    {
        const dlx::FlightControlMotorOutput mo = dlx::mixMotors(out, p);
        float sum = 0.0f;
        for (int i = 0; i < 4; i++) {
            CHECK(mo.motor[i] >= 0.0f && mo.motor[i] <= 1.0f);
            sum += mo.motor[i];
        }
        CHECK(std::fabs(sum - 4.0f * 0.9f) < 1e-4f); // 总油门守恒
        CHECK(mo.mix_scale < 1.0f);                    // 确实发生了收缩
    }

    std::printf("[mixer] done\n");
}

//===================================================================================================
// 四旋翼仿真模型: 控制器输出 -> 混控器 -> 四个电机推力 -> 合力/力矩 (验证完整链路)
//===================================================================================================
namespace
{
const float kG = 9.80665f;

struct QuadSim
{
    float Ixx, Iyy, Izz;
    float m;  // 质量 [kg]
    float L;  // 机臂长度 [m]
    float k;  // 单电机油门单位推力 [N]
    float ky; // 偏航反力矩等效力臂 [m]

    Quaternion q{{1.0f, 0.0f, 0.0f, 0.0f}}; // 姿态
    Vector3f w;                              // 机体角速度 [rad/s]
    float h{1.0f};                           // 高度 [m]
    float vz{0.0f};                          // 垂向速度 [m/s] 向上为正

    explicit QuadSim(const dlx::FlightControlParams &p)
        : Ixx(p.get(dlx::FlightParamId::INERTIA_XX))
        , Iyy(p.get(dlx::FlightParamId::INERTIA_YY))
        , Izz(p.get(dlx::FlightParamId::INERTIA_ZZ))
        , m(p.get(dlx::FlightParamId::MASS_KG))
        , L(p.get(dlx::FlightParamId::ARM_LENGTH_M))
        , k((m * kG / p.get(dlx::FlightParamId::HOVER_THROTTLE)) / 4.0f)
        , ky(p.get(dlx::FlightParamId::YAW_TORQUE_ARM_M))
    {
    }

    void step(const FlightControlOutput &out, const dlx::FlightControlParams &p, float dt)
    {
        // 混控: 输出 -> 四电机油门
        const dlx::FlightControlMotorOutput motors = dlx::mixMotors(out, p);

        // 电机位置 (机体系): M0(+x,+y) M1(+x,-y) M2(-x,-y) M3(-x,+y)
        static const float arm_x[4] = {1.0f, 1.0f, -1.0f, -1.0f};
        static const float arm_y[4] = {1.0f, -1.0f, -1.0f, 1.0f};
        // 旋转方向: M0/M2 顺时针(+), M1/M3 逆时针(-)
        static const float spin[4] = {1.0f, -1.0f, 1.0f, -1.0f};

        Vector3f tau; // 电机产生的净力矩
        float F_sum = 0.0f;
        for (int i = 0; i < 4; i++) {
            const float F = motors.motor[i] * k;
            F_sum += F;
            tau.x() += (L / 1.41421356f) * arm_y[i] * F;
            tau.y() += -(L / 1.41421356f) * arm_x[i] * F;
            tau.z() += spin[i] * ky * F;
        }

        // 欧拉方程: I * w_dot = tau - w x (I*w)
        const Vector3f Iw{w.x() * Ixx, w.y() * Iyy, w.z() * Izz};
        const Vector3f gyro_term = dlx::cross(w, Iw);
        Vector3f w_dot;
        w_dot.x() = (tau.x() - gyro_term.x()) / Ixx;
        w_dot.y() = (tau.y() - gyro_term.y()) / Iyy;
        w_dot.z() = (tau.z() - gyro_term.z()) / Izz;
        w += w_dot * dt;

        // 姿态积分: q_dot = 0.5 * q (x) (0, w)
        Quaternion qd;
        qd.data[0] = 0.5f * (-q.data[1] * w.x() - q.data[2] * w.y() - q.data[3] * w.z());
        qd.data[1] = 0.5f * (q.data[0] * w.x() + q.data[2] * w.z() - q.data[3] * w.y());
        qd.data[2] = 0.5f * (q.data[0] * w.y() - q.data[1] * w.z() + q.data[3] * w.x());
        qd.data[3] = 0.5f * (q.data[0] * w.z() + q.data[1] * w.y() - q.data[2] * w.x());
        q.data[0] += qd.data[0] * dt;
        q.data[1] += qd.data[1] * dt;
        q.data[2] += qd.data[2] * dt;
        q.data[3] += qd.data[3] * dt;
        q = dlx::quatNormalize(q);

        // 高度: 总推力垂直分量 - 重力, 带一点速度阻尼
        const float tilt_cos = dlx::quatDcmZ(q).z();
        const float a_z = (F_sum * tilt_cos) / m - kG - 0.2f * vz;
        vz += a_z * dt;
        h += vz * dt;
    }

    FlightControlState state() const
    {
        FlightControlState s;
        s.attitude = q;
        s.height_m = h;
        s.body_rate.data[0] = w.x();
        s.body_rate.data[1] = w.y();
        s.body_rate.data[2] = w.z();
        s.vertical_velocity_mps = vz;
        s.vertical_velocity_valid = true;
        return s;
    }
};
} // namespace

//===================================================================================================
// 3. 悬停仿真: 初始 25° 倾角 + 0.5m 高度偏差 -> 收敛
//===================================================================================================
static void runHoverSim(bool use_vz)
{
    FlightController fc;
    QuadSim sim(fc.params());

    // 初始姿态: 绕轴 (1,-0.5,0) 倾 25°
    sim.q = dlx::quatFromAxisAngle(Vector3f{1.0f, -0.5f, 0.0f}, 25.0f * 3.14159265f / 180.0f);
    sim.h = 1.5f;

    FlightControlSetpoint sp;
    sp.attitude = dlx::quatIdentity();
    sp.height_m = 1.0f;

    const float dt = 0.002f;
    const int steps = static_cast<int>(10.0f / dt);
    bool nan_detected = false;
    bool motor_ok = true;

    for (int i = 0; i < steps; i++) {
        FlightControlState st = sim.state();
        if (!use_vz) {
            st.vertical_velocity_valid = false;
            st.vertical_velocity_mps = 0.0f;
        }
        const float throttle = fc.update(sp, st, dt);
        const FlightControlOutput &out = fc.lastOutput();
        const dlx::FlightControlMotorOutput motors = dlx::mixMotors(out, fc.params());
        sim.step(out, fc.params(), dt);

        if (!std::isfinite(throttle) || !std::isfinite(out.torque.x()) || !std::isfinite(out.torque.y())
            || !std::isfinite(out.torque.z())) {
            nan_detected = true;
        }
        for (int i = 0; i < 4; i++) {
            if (motors.motor[i] < 0.0f || motors.motor[i] > 1.0f) {
                motor_ok = false;
            }
        }

        if (i % 1000 == 0) {
            const Vector3f eul = dlx::quatToEuler(sim.q);
            std::printf("  t=%5.1fs roll=%7.2f pitch=%7.2f yaw=%7.2f h=%6.3f thr=%5.3f m=%5.3f %5.3f %5.3f %5.3f\n",
                        i * dt, eul.z() * 57.29578f, eul.y() * 57.29578f, eul.x() * 57.29578f, sim.h, throttle,
                        motors.motor[0], motors.motor[1], motors.motor[2], motors.motor[3]);
        }
    }

    const Vector3f eul = dlx::quatToEuler(sim.q);
    const float tilt_deg = std::fabs(eul.z()) * 57.29578f;
    std::printf("  final: roll=%7.2f pitch=%7.2f yaw=%7.2f h=%6.3f |w|=%6.3f\n",
                eul.z() * 57.29578f, eul.y() * 57.29578f, eul.x() * 57.29578f, sim.h, dlx::norm(sim.w));

    CHECK(!nan_detected);
    CHECK(motor_ok);
    CHECK(tilt_deg < 2.0f);
    CHECK(std::fabs(sim.h - sp.height_m) < 0.05f);
    CHECK(dlx::norm(sim.w) < 0.1f);
}

//===================================================================================================
// 4. 偏航不控制: 期望偏航 90°, 实际偏航应保持接近 0
//===================================================================================================
static void runYawIgnoreTest()
{
    FlightController fc;
    QuadSim sim(fc.params());

    FlightControlSetpoint sp;
    sp.attitude = dlx::quatFromAxisAngle(Vector3f{0.0f, 0.0f, 1.0f}, 1.5707963f); // 期望 yaw 90°
    sp.height_m = 1.0f;

    const float dt = 0.002f;
    const int steps = static_cast<int>(5.0f / dt);
    for (int i = 0; i < steps; i++) {
        const float throttle = fc.update(sp, sim.state(), dt);
        (void)throttle;
        sim.step(fc.lastOutput(), fc.params(), dt);
    }

    const Vector3f eul = dlx::quatToEuler(sim.q);
    std::printf("  yaw after 5s (should stay ~0): %.2f deg\n", eul.x() * 57.29578f);
    CHECK(std::fabs(eul.x()) * 57.29578f < 5.0f); // 不追偏航
    CHECK(std::fabs(sim.h - 1.0f) < 0.05f);       // 高度仍保持
}

//===================================================================================================
// 5. 姿态跟踪: t=2s 时期望 roll 阶跃 +30°, 应收敛
//===================================================================================================
static void runStepTrackingTest()
{
    FlightController fc;
    QuadSim sim(fc.params());

    FlightControlSetpoint sp;
    sp.attitude = dlx::quatIdentity();
    sp.height_m = 1.0f;

    const float dt = 0.002f;
    const int steps = static_cast<int>(10.0f / dt);
    const int step_at = static_cast<int>(2.0f / dt);

    for (int i = 0; i < steps; i++) {
        if (i == step_at) {
            sp.attitude = dlx::quatFromAxisAngle(Vector3f{1.0f, 0.0f, 0.0f}, 30.0f * 3.14159265f / 180.0f);
        }
        fc.update(sp, sim.state(), dt);
        sim.step(fc.lastOutput(), fc.params(), dt);
    }

    const Vector3f eul = dlx::quatToEuler(sim.q);
    std::printf("  final roll (want 30): %.2f deg, h=%.3f\n", eul.z() * 57.29578f, sim.h);
    CHECK(std::fabs(eul.z() * 57.29578f - 30.0f) < 2.0f);
    CHECK(std::fabs(sim.h - 1.0f) < 0.05f);
}

//===================================================================================================
// 6. 扰动恢复: 悬停稳定后给一个瞬时角速度冲击, 应收敛回水平
//===================================================================================================
static void runDisturbanceTest()
{
    FlightController fc;
    QuadSim sim(fc.params());

    FlightControlSetpoint sp;
    sp.attitude = dlx::quatIdentity();
    sp.height_m = 1.0f;

    const float dt = 0.002f;
    const int disturb_at = static_cast<int>(2.0f / dt);
    const int steps = static_cast<int>(8.0f / dt);
    float min_throttle = 1.0f;
    float max_throttle = 0.0f;

    for (int i = 0; i < steps; i++) {
        const float throttle = fc.update(sp, sim.state(), dt);
        min_throttle = min_throttle < throttle ? min_throttle : throttle;
        max_throttle = max_throttle > throttle ? max_throttle : throttle;

        if (i == disturb_at) {
            sim.w.x() += 6.0f; // roll 角速度冲击 ~6 rad/s
            sim.w.y() -= 4.0f;
        }
        sim.step(fc.lastOutput(), fc.params(), dt);
    }

    const Vector3f eul = dlx::quatToEuler(sim.q);
    std::printf("  after disturbance: roll=%.2f pitch=%.2f h=%.3f |w|=%.3f (thr range %.2f~%.2f)\n",
                eul.z() * 57.29578f, eul.y() * 57.29578f, sim.h, dlx::norm(sim.w), min_throttle, max_throttle);
    CHECK(std::fabs(eul.z()) * 57.29578f < 3.0f);
    CHECK(std::fabs(eul.y()) * 57.29578f < 3.0f);
    CHECK(std::fabs(sim.h - 1.0f) < 0.1f);
    CHECK(dlx::norm(sim.w) < 0.2f);
    CHECK(min_throttle >= 0.0f);
    CHECK(max_throttle <= 1.0f);
}

//===================================================================================================
// 9. 端到端定高: 控制器看不到仿真真值, 垂向速度完全由估计器 (气压+四元数+IMU) 融合提供
//===================================================================================================
static void runHoverSimWithEstimator()
{
    FlightController fc;
    QuadSim sim(fc.params());
    dlx::VerticalVelocityEstimator vz_est;
    vz_est.reset(sim.h); // 起飞前用当前高度锁存

    // 初始姿态: 绕轴 (1,-0.5,0) 倾 25°, 高度偏差 +0.5m
    sim.q = dlx::quatFromAxisAngle(Vector3f{1.0f, -0.5f, 0.0f}, 25.0f * 3.14159265f / 180.0f);
    sim.h = 1.5f;

    FlightControlSetpoint sp;
    sp.attitude = dlx::quatIdentity();
    sp.height_m = 1.0f;

    const float dt = 0.002f;
    const int steps = static_cast<int>(12.0f / dt);
    bool nan_detected = false;

    for (int i = 0; i < steps; i++) {
        // 模拟 IMU 读数: 当前推力 (上一周期油门) 沿机体 z 产生的比力 [g]
        float F_per_m_g = 1.0f; // 首周期用悬停比力近似
        if (i > 0) {
            const dlx::FlightControlMotorOutput motors = dlx::mixMotors(fc.lastOutput(), fc.params());
            float F_sum = 0.0f;
            for (int m = 0; m < 4; m++) {
                F_sum += motors.motor[m] * sim.k;
            }
            F_per_m_g = F_sum / sim.m / 9.80665f;
        }
        dlx::AccelerometerG acc;
        acc.data[0] = 0.0f;
        acc.data[1] = 0.0f;
        acc.data[2] = F_per_m_g;

        // 气压: 真值 + 确定性小噪声
        const float baro = sim.h + 0.01f * std::sin(i * 0.1f);
        const dlx::VerticalVelocityEstimate e = vz_est.update(baro, sim.q, acc, dt);

        // 控制器只能看到估计值, 拿不到仿真真值
        FlightControlState st = sim.state();
        st.height_m = e.height_m;
        st.vertical_velocity_mps = e.vertical_velocity_mps;
        st.vertical_velocity_valid = e.valid;

        const float throttle = fc.update(sp, st, dt);
        if (!std::isfinite(throttle) || !std::isfinite(e.height_m) || !std::isfinite(e.vertical_velocity_mps)) {
            nan_detected = true;
        }
        sim.step(fc.lastOutput(), fc.params(), dt);

        if (i % 1000 == 0) {
            std::printf("  t=%5.1fs h_true=%6.3f h_est=%6.3f vz_est=%6.3f thr=%5.3f\n",
                        i * dt, sim.h, e.height_m, e.vertical_velocity_mps, throttle);
        }
    }

    const Vector3f eul = dlx::quatToEuler(sim.q);
    std::printf("  final: roll=%.2f pitch=%.2f h_true=%.3f |w|=%.3f\n",
                eul.z() * 57.29578f, eul.y() * 57.29578f, sim.h, dlx::norm(sim.w));

    CHECK(!nan_detected);
    CHECK(std::fabs(sim.h - sp.height_m) < 0.08f);
    CHECK(dlx::norm(sim.w) < 0.1f);
}

//===================================================================================================
// 7. 鲁棒性: 非法输入不应产生 NaN / 崩溃
//===================================================================================================
static void runRobustnessTest()
{
    FlightController fc;
    FlightControlSetpoint sp;
    sp.attitude = dlx::quatIdentity();
    sp.height_m = 1.0f;

    FlightControlState st;
    st.attitude = dlx::quatIdentity();
    st.height_m = 1.0f;
    st.body_rate.data[0] = st.body_rate.data[1] = st.body_rate.data[2] = 0.0f;
    st.vertical_velocity_valid = false;

    // 非法 dt (0 与 NaN)
    float thr0 = fc.update(sp, st, 0.0f);
    float thrNaN = fc.update(sp, st, 0.0f / 0.0f);
    CHECK(std::isfinite(thr0));
    CHECK(std::isfinite(thrNaN));

    // 全零四元数 (视为无效 -> 归一化为单位四元数)
    st.attitude = Quaternion{{0.0f, 0.0f, 0.0f, 0.0f}};
    float thrBad = fc.update(sp, st, 0.002f);
    CHECK(std::isfinite(thrBad));

    // 异常大角速度不应导致 NaN
    st.attitude = dlx::quatIdentity();
    st.body_rate.data[0] = 1e6f;
    float thrHuge = fc.update(sp, st, 0.002f);
    CHECK(std::isfinite(thrHuge));

    std::printf("  robustness OK (thr0=%.3f thrNaN=%.3f thrBad=%.3f thrHuge=%.3f)\n",
                thr0, thrNaN, thrBad, thrHuge);
}

//===================================================================================================
// 8. 垂向速度估计器: 气压高度 + 姿态四元数 + IMU 融合
//===================================================================================================
static void setHoverAccel(dlx::AccelerometerG &acc)
{
    acc.data[0] = 0.0f;
    acc.data[1] = 0.0f;
    acc.data[2] = 1.0f; // 水平悬停: 比力 = 1g 沿机体 z
}

// 8.1 水平悬停: 气压恒定, 估计垂向速度应保持 ~0, 高度不漂移
static void testVzLevelHover()
{
    dlx::VerticalVelocityEstimator est;
    est.reset(1.0f);

    const float dt = 0.005f;
    const int steps = 2000; // 10s @ 200Hz
    float h = 0.0f, vz = 0.0f;
    for (int i = 0; i < steps; i++) {
        dlx::AccelerometerG acc;
        setHoverAccel(acc);
        const dlx::VerticalVelocityEstimate e = est.update(1.0f, dlx::quatIdentity(), acc, dt);
        h = e.height_m;
        vz = e.vertical_velocity_mps;
        CHECK(e.valid);
    }
    std::printf("  level hover: h=%.3f vz=%.4f\n", h, vz);
    CHECK(std::fabs(h - 1.0f) < 0.02f);
    CHECK(std::fabs(vz) < 0.02f);
}

// 8.2 匀加速后匀速爬升: 估计垂向速度应收敛到真值 (气压带确定性小噪声)
static void testVzClimb()
{
    dlx::VerticalVelocityEstimator est;
    est.reset(1.0f);

    const float dt = 0.005f;
    const int steps = 4000; // 20s @ 200Hz
    float h_true = 1.0f, vz_true = 0.0f;
    float h = 0.0f, vz = 0.0f;
    for (int i = 0; i < steps; i++) {
        // 前 5s 匀加速 0.5 m/s^2, 之后匀速 2.5 m/s
        const float az_true = (i < 1000) ? 0.5f : 0.0f;
        vz_true += az_true * dt;
        h_true += vz_true * dt;

        dlx::AccelerometerG acc;
        acc.data[0] = 0.0f;
        acc.data[1] = 0.0f;
        acc.data[2] = 1.0f + az_true / 9.80665f; // 比力 = g + a (水平爬升)

        const float baro = h_true + 0.02f * std::sin(i * 0.05f); // 确定性小噪声
        const dlx::VerticalVelocityEstimate e = est.update(baro, dlx::quatIdentity(), acc, dt);
        h = e.height_m;
        vz = e.vertical_velocity_mps;
    }
    std::printf("  climb: h_true=%.2f h=%.2f vz_true=%.2f vz=%.2f\n", h_true, h, vz_true, vz);
    CHECK(std::fabs(vz - vz_true) < 0.1f);
    CHECK(std::fabs(h - h_true) < 0.15f);
}

// 8.3 倾角飞行: 推力 mg/cos(tilt) 沿机体 z, 世界系垂向加速度应为 0 (验证四元数旋转)
static void testVzTilted()
{
    dlx::VerticalVelocityEstimator est;
    est.reset(1.0f);

    const float tilt = 20.0f * 3.14159265f / 180.0f;
    const dlx::Quaternion q = dlx::quatFromAxisAngle(dlx::Vector3f{1.0f, 0.0f, 0.0f}, tilt);
    const float fz = 1.0f / std::cos(tilt); // 定高飞行所需比力 (水平分量被外力/侧向加速度抵消)

    const float dt = 0.005f;
    const int steps = 2000;
    float h = 0.0f, vz = 0.0f;
    for (int i = 0; i < steps; i++) {
        dlx::AccelerometerG acc;
        acc.data[0] = 0.0f;
        acc.data[1] = 0.0f;
        acc.data[2] = fz;
        const dlx::VerticalVelocityEstimate e = est.update(1.0f, q, acc, dt);
        h = e.height_m;
        vz = e.vertical_velocity_mps;
    }
    std::printf("  tilted 20deg: h=%.3f vz=%.4f\n", h, vz);
    CHECK(std::fabs(vz) < 0.05f); // 旋转校正后不应向上/向下漂移
    CHECK(std::fabs(h - 1.0f) < 0.1f);
}

// 8.4 气压毛刺: 单样本 +5m 应被丢弃, 高度/速度不受影响
static void testVzBaroSpike()
{
    dlx::VerticalVelocityEstimator est;
    est.reset(1.0f);

    const float dt = 0.005f;
    const int steps = 2000;
    float h = 0.0f, vz = 0.0f;
    for (int i = 0; i < steps; i++) {
        dlx::AccelerometerG acc;
        setHoverAccel(acc);
        float baro = 1.0f;
        if (i == 200) {
            baro += 5.0f; // 单样本毛刺
        }
        const dlx::VerticalVelocityEstimate e = est.update(baro, dlx::quatIdentity(), acc, dt);
        h = e.height_m;
        vz = e.vertical_velocity_mps;
    }
    std::printf("  baro spike: h=%.3f vz=%.4f\n", h, vz);
    CHECK(std::fabs(h - 1.0f) < 0.05f);
    CHECK(std::fabs(vz) < 0.05f);
}

// 8.5 气压长时间丢失后重新获得: 应直接对齐高度 (避免持续错误积分)
static void testVzBaroReacquire()
{
    dlx::VerticalVelocityEstimator est;
    est.reset(1.0f);

    const float dt = 0.005f;
    for (int i = 0; i < 400; i++) { // 2s 无气压
        dlx::AccelerometerG acc;
        setHoverAccel(acc);
        const dlx::VerticalVelocityEstimate e = est.update(0.0f, dlx::quatIdentity(), acc, dt, false);
        (void)e;
    }

    dlx::AccelerometerG acc;
    setHoverAccel(acc);
    const dlx::VerticalVelocityEstimate e = est.update(2.0f, dlx::quatIdentity(), acc, dt, true);
    std::printf("  baro reacquire: h=%.3f (expect 2.0)\n", e.height_m);
    CHECK(e.valid);
    CHECK(std::fabs(e.height_m - 2.0f) < 1e-3f);
}

// 8.6 鲁棒性: 非法输入不应产生 NaN
static void testVzRobustness()
{
    dlx::VerticalVelocityEstimator est; // 未初始化
    dlx::AccelerometerG acc;
    acc.data[0] = 0.0f;
    acc.data[1] = 0.0f;
    acc.data[2] = 0.0f;

    // 未初始化 + 气压无效: 保持无效且全零
    const dlx::VerticalVelocityEstimate e0 = est.update(0.0f, dlx::quatIdentity(), acc, 0.005f, false);
    CHECK(!e0.valid);
    CHECK(std::isfinite(e0.height_m) && std::isfinite(e0.vertical_velocity_mps));

    // 非法 dt (0 与 NaN)
    const dlx::VerticalVelocityEstimate e1 = est.update(1.0f, dlx::quatIdentity(), acc, 0.0f);
    const dlx::VerticalVelocityEstimate e2 = est.update(1.0f, dlx::quatIdentity(), acc, 0.0f / 0.0f);
    CHECK(std::isfinite(e1.height_m) && std::isfinite(e1.vertical_velocity_mps));
    CHECK(std::isfinite(e2.height_m) && std::isfinite(e2.vertical_velocity_mps));

    // 全零四元数
    const dlx::VerticalVelocityEstimate e3 = est.update(1.0f, dlx::Quaternion{{0.0f, 0.0f, 0.0f, 0.0f}}, acc, 0.005f);
    CHECK(std::isfinite(e3.height_m) && std::isfinite(e3.vertical_velocity_mps));

    // 异常大加速度 (超出有效区间, 应跳过积分)
    acc.data[2] = 100.0f;
    const dlx::VerticalVelocityEstimate e4 = est.update(1.0f, dlx::quatIdentity(), acc, 0.005f);
    CHECK(std::isfinite(e4.height_m) && std::isfinite(e4.vertical_velocity_mps));
    CHECK(std::fabs(e4.vertical_velocity_mps) < 1e-3f);

    std::printf("  estimator robustness OK (h=%.3f vz=%.4f)\n", e4.height_m, e4.vertical_velocity_mps);
}

int main()
{
    std::printf("== math unit tests ==\n");
    testMath();

    std::printf("== mixer unit tests ==\n");
    testMixer();

    std::printf("== hover sim (with vz) ==\n");
    runHoverSim(true);

    std::printf("== hover sim (vz estimated from height) ==\n");
    runHoverSim(false);

    std::printf("== yaw ignore test ==\n");
    runYawIgnoreTest();

    std::printf("== step tracking test ==\n");
    runStepTrackingTest();

    std::printf("== disturbance recovery test ==\n");
    runDisturbanceTest();

    std::printf("== robustness test ==\n");
    runRobustnessTest();

    std::printf("== vertical velocity estimator tests ==\n");
    testVzLevelHover();
    testVzClimb();
    testVzTilted();
    testVzBaroSpike();
    testVzBaroReacquire();
    testVzRobustness();

    std::printf("== hover sim (vz from estimator) ==\n");
    runHoverSimWithEstimator();

    std::printf("\n%s (%d failures)\n", g_failures == 0 ? "ALL PASSED" : "FAILED", g_failures);
    return g_failures == 0 ? 0 : 1;
}
