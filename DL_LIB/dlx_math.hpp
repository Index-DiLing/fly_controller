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
//   - 姿态四元数为“机体系 -> 世界系”的旋转四元数, 与 MadgwickAHRS 输出一致
//     (即 v_world = R(q) * v_body; quatDcmZ / quatRotate 均按此解释).
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

    // 欧拉角 (ZYX: yaw, pitch, roll) [rad] -> 四元数. 与 quatToEuler 互为逆.
    // 机体系约定: x 前, y 左, z 上 (右手系); 旋转顺序 R = Rz(yaw) * Ry(pitch) * Rx(roll).
    // 其中 yaw 绕 z 轴, pitch 绕 y 轴, roll 绕 x 轴. 由 quatFromAxisAngle + quatMul 组合实现.
    inline Quaternion quatFromEulerZYX(float yaw, float pitch, float roll)
    {
        const Quaternion qz = quatFromAxisAngle(Vector3f{0.0f, 0.0f, 1.0f}, yaw);
        const Quaternion qy = quatFromAxisAngle(Vector3f{0.0f, 1.0f, 0.0f}, pitch);
        const Quaternion qx = quatFromAxisAngle(Vector3f{1.0f, 0.0f, 0.0f}, roll);
        return quatMul(qz, quatMul(qy, qx)); // = Rz * Ry * Rx
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

    //================================================================================================
    // 固定尺寸浮点矩阵 (无动态内存, 供 EKF 等滤波算法复用)
    //   列向量用 MatF<N,1> 表示, 标量用 MatF<1,1>.
    //================================================================================================
    template <size_t R, size_t C>
    struct MatF
    {
        float d[R][C];

        MatF()
        {
            for (size_t i = 0; i < R; ++i) {
                for (size_t j = 0; j < C; ++j) {
                    d[i][j] = 0.0f;
                }
            }
        }

        float &operator()(size_t i, size_t j) { return d[i][j]; }
        float operator()(size_t i, size_t j) const { return d[i][j]; }
    };

    template <size_t R, size_t C>
    inline MatF<C, R> matTranspose(const MatF<R, C> &a)
    {
        MatF<C, R> r;
        for (size_t i = 0; i < R; ++i) {
            for (size_t j = 0; j < C; ++j) {
                r.d[j][i] = a.d[i][j];
            }
        }
        return r;
    }

    template <size_t A, size_t K, size_t B>
    inline MatF<A, B> matMul(const MatF<A, K> &a, const MatF<K, B> &b)
    {
        MatF<A, B> r;
        for (size_t i = 0; i < A; ++i) {
            for (size_t j = 0; j < B; ++j) {
                float s = 0.0f;
                for (size_t k = 0; k < K; ++k) {
                    s += a.d[i][k] * b.d[k][j];
                }
                r.d[i][j] = s;
            }
        }
        return r;
    }

    template <size_t R, size_t C>
    inline MatF<R, C> matAdd(const MatF<R, C> &a, const MatF<R, C> &b)
    {
        MatF<R, C> r;
        for (size_t i = 0; i < R; ++i) {
            for (size_t j = 0; j < C; ++j) {
                r.d[i][j] = a.d[i][j] + b.d[i][j];
            }
        }
        return r;
    }

    template <size_t R, size_t C>
    inline MatF<R, C> matSub(const MatF<R, C> &a, const MatF<R, C> &b)
    {
        MatF<R, C> r;
        for (size_t i = 0; i < R; ++i) {
            for (size_t j = 0; j < C; ++j) {
                r.d[i][j] = a.d[i][j] - b.d[i][j];
            }
        }
        return r;
    }

    template <size_t R, size_t C>
    inline MatF<R, C> matScale(const MatF<R, C> &a, float s)
    {
        MatF<R, C> r;
        for (size_t i = 0; i < R; ++i) {
            for (size_t j = 0; j < C; ++j) {
                r.d[i][j] = a.d[i][j] * s;
            }
        }
        return r;
    }

    template <size_t R, size_t C>
    inline MatF<R, C> matAddScaled(const MatF<R, C> &a, float sa, const MatF<R, C> &b, float sb)
    {
        MatF<R, C> r;
        for (size_t i = 0; i < R; ++i) {
            for (size_t j = 0; j < C; ++j) {
                r.d[i][j] = a.d[i][j] * sa + b.d[i][j] * sb;
            }
        }
        return r;
    }

    template <size_t N>
    inline MatF<N, N> matIdent()
    {
        MatF<N, N> r;
        for (size_t i = 0; i < N; ++i) {
            r.d[i][i] = 1.0f;
        }
        return r;
    }

    // 3x3 行列式 (用于求逆)
    inline float mat3Det(const MatF<3, 3> &m)
    {
        return m.d[0][0] * (m.d[1][1] * m.d[2][2] - m.d[1][2] * m.d[2][1])
             - m.d[0][1] * (m.d[1][0] * m.d[2][2] - m.d[1][2] * m.d[2][0])
             + m.d[0][2] * (m.d[1][0] * m.d[2][1] - m.d[1][1] * m.d[2][0]);
    }

    // 3x3 逆 (伴随除以行列式)
    inline MatF<3, 3> mat3Inverse(const MatF<3, 3> &m)
    {
        const float det = mat3Det(m);
        MatF<3, 3> r;
        if (fabsf(det) < 1e-12f) {
            return r; // 奇异, 返回零
        }
        const float inv = 1.0f / det;
        r.d[0][0] = (m.d[1][1] * m.d[2][2] - m.d[1][2] * m.d[2][1]) * inv;
        r.d[0][1] = -(m.d[0][1] * m.d[2][2] - m.d[0][2] * m.d[2][1]) * inv;
        r.d[0][2] = (m.d[0][1] * m.d[1][2] - m.d[0][2] * m.d[1][1]) * inv;
        r.d[1][0] = -(m.d[1][0] * m.d[2][2] - m.d[1][2] * m.d[2][0]) * inv;
        r.d[1][1] = (m.d[0][0] * m.d[2][2] - m.d[0][2] * m.d[2][0]) * inv;
        r.d[1][2] = -(m.d[0][0] * m.d[1][2] - m.d[0][2] * m.d[1][0]) * inv;
        r.d[2][0] = (m.d[1][0] * m.d[2][1] - m.d[1][1] * m.d[2][0]) * inv;
        r.d[2][1] = -(m.d[0][0] * m.d[2][1] - m.d[0][1] * m.d[2][0]) * inv;
        r.d[2][2] = (m.d[0][0] * m.d[1][1] - m.d[0][1] * m.d[1][0]) * inv;
        return r;
    }

    // 反对称阵 [v]x (用于叉乘: v x a = [v]x * a)
    inline MatF<3, 3> skew(const Vector3f &v)
    {
        MatF<3, 3> m;
        m.d[0][1] = -v.z();
        m.d[0][2] = v.y();
        m.d[1][0] = v.z();
        m.d[1][2] = -v.x();
        m.d[2][0] = -v.y();
        m.d[2][1] = v.x();
        return m;
    }

    // 按四元数(w,x,y,z, 机体系->世界系)构造机体系->世界系的旋转矩阵 R
    inline MatF<3, 3> matFromQuaternion(const Quaternion &q)
    {
        Quaternion n = quatNormalize(q);
        const float w = n.data[0], x = n.data[1], y = n.data[2], z = n.data[3];
        MatF<3, 3> r;
        r.d[0][0] = 1.0f - 2.0f * (y * y + z * z);
        r.d[0][1] = 2.0f * (x * y - w * z);
        r.d[0][2] = 2.0f * (x * z + w * y);
        r.d[1][0] = 2.0f * (x * y + w * z);
        r.d[1][1] = 1.0f - 2.0f * (x * x + z * z);
        r.d[1][2] = 2.0f * (y * z - w * x);
        r.d[2][0] = 2.0f * (x * z - w * y);
        r.d[2][1] = 2.0f * (y * z + w * x);
        r.d[2][2] = 1.0f - 2.0f * (x * x + y * y);
        return r;
    }

    // Vector3f -> MatF<3,1>
    inline MatF<3, 1> vecToMat(const Vector3f &v)
    {
        MatF<3, 1> m;
        m.d[0][0] = v.x();
        m.d[1][0] = v.y();
        m.d[2][0] = v.z();
        return m;
    }

    // MatF<3,1> -> Vector3f
    inline Vector3f matToVec(const MatF<3, 1> &m)
    {
        return Vector3f(m.d[0][0], m.d[1][0], m.d[2][0]);
    }

} // namespace dlx
