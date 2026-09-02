#pragma once
#include <math.h>
#include <stddef.h>
#include "dlx_sensor_data.hpp"

//===================================================================================================
// dlx_math.hpp
// 独立于单片机的向量 / 四元数数学工具, 供 DL_LIB 内各模块复用.
//
// 约定:
//   - Quaternion 使用 dlx_sensor_data.hpp 中的类型, data[0..3] = (w, x, y, z).
//   - 姿态四元数为“世界系 -> 机体系”的旋转四元数, 与 MadgwickAHRS 输出一致.
//   - Vector3f.data[0..2] = (x, y, z).
//   - 所有函数为纯浮点计算, 无动态内存, 可直接用于单片机.
//===================================================================================================

namespace dlx
{
    //================================================================================================
    // 三维向量
    //================================================================================================
    struct Vector3f
    {
        float data[3];

        Vector3f()
            : data{0.0f, 0.0f, 0.0f}
        {
        }

        Vector3f(float x, float y, float z)
            : data{x, y, z}
        {
        }

        float &x() { return data[0]; }
        float &y() { return data[1]; }
        float &z() { return data[2]; }
        float x() const { return data[0]; }
        float y() const { return data[1]; }
        float z() const { return data[2]; }

        float &operator[](size_t i) { return data[i]; }
        float operator[](size_t i) const { return data[i]; }
    };

    inline Vector3f operator+(const Vector3f &a, const Vector3f &b)
    {
        Vector3f r;
        r.x() = a.x() + b.x();
        r.y() = a.y() + b.y();
        r.z() = a.z() + b.z();
        return r;
    }

    inline Vector3f operator-(const Vector3f &a, const Vector3f &b)
    {
        Vector3f r;
        r.x() = a.x() - b.x();
        r.y() = a.y() - b.y();
        r.z() = a.z() - b.z();
        return r;
    }

    inline Vector3f operator*(const Vector3f &a, float s)
    {
        Vector3f r;
        r.x() = a.x() * s;
        r.y() = a.y() * s;
        r.z() = a.z() * s;
        return r;
    }

    inline Vector3f operator*(float s, const Vector3f &a)
    {
        return a * s;
    }

    inline Vector3f operator/(const Vector3f &a, float s)
    {
        Vector3f r;
        r.x() = a.x() / s;
        r.y() = a.y() / s;
        r.z() = a.z() / s;
        return r;
    }

    inline Vector3f &operator+=(Vector3f &a, const Vector3f &b)
    {
        a = a + b;
        return a;
    }

    inline Vector3f &operator-=(Vector3f &a, const Vector3f &b)
    {
        a = a - b;
        return a;
    }

    inline Vector3f &operator*=(Vector3f &a, float s)
    {
        a = a * s;
        return a;
    }

    inline Vector3f &operator/=(Vector3f &a, float s)
    {
        a = a / s;
        return a;
    }

    inline float dot(const Vector3f &a, const Vector3f &b)
    {
        return a.x() * b.x() + a.y() * b.y() + a.z() * b.z();
    }

    inline Vector3f cross(const Vector3f &a, const Vector3f &b)
    {
        Vector3f r;
        r.x() = a.y() * b.z() - a.z() * b.y();
        r.y() = a.z() * b.x() - a.x() * b.z();
        r.z() = a.x() * b.y() - a.y() * b.x();
        return r;
    }

    inline float normSquared(const Vector3f &a)
    {
        return dot(a, a);
    }

    inline float norm(const Vector3f &a)
    {
        return sqrtf(normSquared(a));
    }

    // 归一化; 模长过小(零向量)时返回零向量, 避免 NaN
    inline Vector3f normalized(const Vector3f &a)
    {
        float n = norm(a);
        if (n < 1e-12f) {
            return Vector3f{};
        }
        return a / n;
    }

    // 分量限幅
    inline float constrain(float v, float lo, float hi)
    {
        if (v < lo) {
            return lo;
        }
        if (v > hi) {
            return hi;
        }
        return v;
    }

    inline Vector3f constrain(const Vector3f &v, float lo, float hi)
    {
        Vector3f r;
        r.x() = constrain(v.x(), lo, hi);
        r.y() = constrain(v.y(), lo, hi);
        r.z() = constrain(v.z(), lo, hi);
        return r;
    }

    inline float min(float a, float b)
    {
        return a < b ? a : b;
    }

    inline float max(float a, float b)
    {
        return a > b ? a : b;
    }

    //================================================================================================
    // 四元数工具 (w, x, y, z)
    //================================================================================================
    inline Quaternion quatIdentity()
    {
        return Quaternion{{1.0f, 0.0f, 0.0f, 0.0f}};
    }

    inline float quatNormSquared(const Quaternion &q)
    {
        return q.data[0] * q.data[0] + q.data[1] * q.data[1] + q.data[2] * q.data[2] + q.data[3] * q.data[3];
    }

    inline Quaternion quatNormalize(const Quaternion &q)
    {
        float n2 = quatNormSquared(q);
        if (n2 < 1e-12f) {
            return quatIdentity();
        }
        float inv = 1.0f / sqrtf(n2);
        Quaternion r;
        r.data[0] = q.data[0] * inv;
        r.data[1] = q.data[1] * inv;
        r.data[2] = q.data[2] * inv;
        r.data[3] = q.data[3] * inv;
        return r;
    }

    // 单位四元数的共轭即逆
    inline Quaternion quatConjugate(const Quaternion &q)
    {
        Quaternion r;
        r.data[0] = q.data[0];
        r.data[1] = -q.data[1];
        r.data[2] = -q.data[2];
        r.data[3] = -q.data[3];
        return r;
    }

    // Hamilton 乘法: 先旋转 b, 再旋转 a (与矩阵 R(a)R(b) 一致)
    inline Quaternion quatMul(const Quaternion &a, const Quaternion &b)
    {
        Quaternion r;
        r.data[0] = a.data[0] * b.data[0] - a.data[1] * b.data[1] - a.data[2] * b.data[2] - a.data[3] * b.data[3];
        r.data[1] = a.data[0] * b.data[1] + a.data[1] * b.data[0] + a.data[2] * b.data[3] - a.data[3] * b.data[2];
        r.data[2] = a.data[0] * b.data[2] - a.data[1] * b.data[3] + a.data[2] * b.data[0] + a.data[3] * b.data[1];
        r.data[3] = a.data[0] * b.data[3] + a.data[1] * b.data[2] - a.data[2] * b.data[1] + a.data[3] * b.data[0];
        return r;
    }

    // 旋转向量: v' = q * v * q^-1
    inline Vector3f quatRotate(const Quaternion &q, const Vector3f &v)
    {
        Vector3f qv;
        qv.x() = q.data[1];
        qv.y() = q.data[2];
        qv.z() = q.data[3];

        Vector3f t = cross(qv, v) * 2.0f;
        return v + t * q.data[0] + cross(qv, t);
    }

    // 机体系 z 轴在世界系中的表示 (旋转矩阵第三列)
    inline Vector3f quatDcmZ(const Quaternion &q)
    {
        Vector3f r;
        r.x() = 2.0f * (q.data[1] * q.data[3] + q.data[0] * q.data[2]);
        r.y() = 2.0f * (q.data[2] * q.data[3] - q.data[0] * q.data[1]);
        r.z() = q.data[0] * q.data[0] - q.data[1] * q.data[1] - q.data[2] * q.data[2] + q.data[3] * q.data[3];
        return r;
    }

    // 轴角 -> 四元数 (axis 无需归一化)
    inline Quaternion quatFromAxisAngle(const Vector3f &axis, float angle)
    {
        Vector3f n = normalized(axis);
        float half = 0.5f * angle;
        float s = sinf(half);
        Quaternion r;
        r.data[0] = cosf(half);
        r.data[1] = n.x() * s;
        r.data[2] = n.y() * s;
        r.data[3] = n.z() * s;
        return r;
    }

    // 由两个单位向量构造旋转四元数: 将 a 旋转到 b 的最小旋转
    inline Quaternion quatFromTwoVectors(const Vector3f &a, const Vector3f &b)
    {
        Vector3f an = normalized(a);
        Vector3f bn = normalized(b);

        float d = constrain(dot(an, bn), -1.0f, 1.0f);
        Vector3f cr = cross(an, bn);
        float cr2 = normSquared(cr);

        if (cr2 < 1e-12f) {
            if (d < 0.0f) {
                // 180° 情形: 任取一个与 a 正交的轴
                Vector3f ref;
                ref.x() = 0.0f;
                ref.y() = 0.0f;
                ref.z() = 1.0f;
                if (fabsf(an.z()) > 0.9f) {
                    ref.x() = 1.0f;
                    ref.z() = 0.0f;
                }
                Vector3f axis = normalized(cross(an, ref));
                Quaternion r;
                r.data[0] = 0.0f;
                r.data[1] = axis.x();
                r.data[2] = axis.y();
                r.data[3] = axis.z();
                return r;
            }
            return quatIdentity();
        }

        // q ∝ (1 + a·b, a×b)
        Quaternion r;
        r.data[0] = 1.0f + d;
        r.data[1] = cr.x();
        r.data[2] = cr.y();
        r.data[3] = cr.z();
        return quatNormalize(r);
    }

    // 规范化: 保证 w >= 0 (q 与 -q 表示同一旋转)
    inline void quatCanonicalize(Quaternion &q)
    {
        if (q.data[0] < 0.0f) {
            q.data[0] = -q.data[0];
            q.data[1] = -q.data[1];
            q.data[2] = -q.data[2];
            q.data[3] = -q.data[3];
        }
    }

    // 四元数 -> 欧拉角 (ZYX: yaw, pitch, roll) [rad]
    inline Vector3f quatToEuler(const Quaternion &q)
    {
        Quaternion n = quatNormalize(q);
        Vector3f r;
        r.x() = atan2f(2.0f * (n.data[0] * n.data[3] + n.data[1] * n.data[2]),
                       1.0f - 2.0f * (n.data[2] * n.data[2] + n.data[3] * n.data[3])); // yaw
        float sp = 2.0f * (n.data[0] * n.data[2] - n.data[3] * n.data[1]);
        sp = constrain(sp, -1.0f, 1.0f);
        r.y() = asinf(sp); // pitch
        r.z() = atan2f(2.0f * (n.data[0] * n.data[1] + n.data[2] * n.data[3]),
                       1.0f - 2.0f * (n.data[1] * n.data[1] + n.data[2] * n.data[2])); // roll
        return r;
    }

} // namespace dlx
