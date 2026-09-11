#include "stm32f4xx.h"
#include "dlx_gpio.hpp"
#include "dlx_gpio_profile.h"
#include "dlx_usart.hpp"
#include "dlx_bytebuffer.hpp"
#include "stdio.h"
#include "stdarg.h"
#include "string.h"
#include "math.h"
#include "dlx.hpp"
#include "rtthread.h"
#include "dlx_delay.hpp"
#include "dsp/matrix_functions.h"
using namespace dlx;

/* ---------------------------------------------------------------
 * 纯计算矩阵基准: 对比 CMSIS-DSP 与 纯C(无加速) 的开销
 * 覆盖: 16x16 乘法 / 加法 / 转置, 3x3 / 6x6 矩阵求逆
 * 计时: DWT->CYCCNT 周期计数器 (168MHz 下 1 周期 ~= 5.95ns)
 * 输出: USART1 (TX=PA9) 115200 波特率
 * --------------------------------------------------------------- */

/* ---------------- 输出缓冲(串口 DMA) ---------------- */
static char g_txbuf[2048];
static int  g_txpos = 0;

static void tx_reset(void)
{
    g_txpos = 0;
    g_txbuf[0] = '\0';
}

static void tx_printf(const char *fmt, ...)
{
    va_list args;
    char line[160];
    va_start(args, fmt);
    int n = vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);
    if (n <= 0)
        return;
    if (g_txpos + n >= (int)sizeof(g_txbuf))
        n = (int)sizeof(g_txbuf) - 1 - g_txpos;
    memcpy(g_txbuf + g_txpos, line, n);
    g_txpos += n;
    g_txbuf[g_txpos] = '\0';
}

/* ---------------- DWT 周期计数 ---------------- */
static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL         |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline uint32_t dwt_cycles(void)
{
    return DWT->CYCCNT;
}

/* ---------------- 矩阵存储 ---------------- */
static float mA[16 * 16], mB[16 * 16], mC[16 * 16], mD[16 * 16];
static float m3A[3 * 3], m3C[3 * 3], m3X[3 * 3], m3Y[3 * 3];
static float m6A[6 * 6], m6C[6 * 6], m6X[6 * 6], m6Y[6 * 6];

static arm_matrix_instance_f32 I16_A, I16_B, I16_D;
static arm_matrix_instance_f32 I3_SRC, I3_DST;
static arm_matrix_instance_f32 I6_SRC, I6_DST;

/* 定点版本: q31(32位) 与 q15(16位, 在CM4上使用DSP的__SMLALD双16位SIMD) */
static q31_t qA[16 * 16], qB[16 * 16], qC[16 * 16], qD[16 * 16];
static q15_t hA[16 * 16], hB[16 * 16], hC[16 * 16], hD[16 * 16];
static q15_t hState[16 * 16]; /* arm_mat_mult_q15 内部转置B所需的中间缓冲 */
static arm_matrix_instance_q31 Q_A, Q_B, Q_D;
static arm_matrix_instance_q15 H_A, H_B, H_D;

/* ---------------- 纯 C 实现(无 CMSIS 加速) ---------------- */
template <int N>
static void mat_mul_naive(const float *a, const float *b, float *c)
{
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++)
            c[i * N + j] = 0.0f;
        for (int k = 0; k < N; k++) {
            float aik = a[i * N + k];
            const float *bk = b + k * N;
            float *ci       = c + i * N;
            for (int j = 0; j < N; j++)
                ci[j] += aik * bk[j];
        }
    }
}

template <int N>
static void mat_add_naive(const float *a, const float *b, float *c)
{
    for (int i = 0; i < N * N; i++)
        c[i] = a[i] + b[i];
}

template <int N>
static void mat_trans_naive(const float *a, float *d)
{
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            d[j * N + i] = a[i * N + j];
}

/* 高斯-约当消元 + 列主元, 返回 false 表示矩阵奇异 */
template <int N>
static bool mat_inv_naive(const float *a, float *inv)
{
    float A[N * N];
    memcpy(A, a, sizeof(A));
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            inv[i * N + j] = (i == j) ? 1.0f : 0.0f;

    for (int col = 0; col < N; col++) {
        int piv = col;
        float m = fabsf(A[col * N + col]);
        for (int r = col + 1; r < N; r++) {
            float v = fabsf(A[r * N + col]);
            if (v > m) { m = v; piv = r; }
        }
        if (m < 1e-12f)
            return false;

        if (piv != col) {
            for (int j = 0; j < N; j++) {
                float t = A[col * N + j];
                A[col * N + j] = A[piv * N + j];
                A[piv * N + j] = t;
                t = inv[col * N + j];
                inv[col * N + j] = inv[piv * N + j];
                inv[piv * N + j] = t;
            }
        }

        float pivv = A[col * N + col];
        for (int j = 0; j < N; j++) {
            A[col * N + j]   /= pivv;
            inv[col * N + j] /= pivv;
        }

        for (int r = 0; r < N; r++) {
            if (r == col)
                continue;
            float f = A[r * N + col];
            if (f != 0.0f) {
                for (int j = 0; j < N; j++) {
                    A[r * N + j]   -= f * A[col * N + j];
                    inv[r * N + j] -= f * inv[col * N + j];
                }
            }
        }
    }
    return true;
}

/* Q31 朴素版本(纯C, 与 CMSIS 相同语义: int64 累加, 输出=(sum>>31)) */
template <int N>
static void mat_mul_naive_q31(const q31_t *a, const q31_t *b, q31_t *c)
{
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++) {
            int64_t sum = 0;
            for (int k = 0; k < N; k++)
                sum += (int64_t)a[i * N + k] * (int64_t)b[k * N + j];
            c[i * N + j] = (q31_t)(sum >> 31);
        }
}

template <int N>
static void mat_add_naive_q31(const q31_t *a, const q31_t *b, q31_t *c)
{
    for (int i = 0; i < N * N; i++)
        c[i] = a[i] + b[i];
}

template <int N>
static void mat_trans_naive_q31(const q31_t *a, q31_t *d)
{
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            d[j * N + i] = a[i * N + j];
}

/* Q15 朴素版本(纯C, 与 CMSIS 相同语义: int64 累加, 输出=(sum>>15) 并饱和到16位) */
template <int N>
static void mat_mul_naive_q15(const q15_t *a, const q15_t *b, q15_t *c)
{
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++) {
            int64_t sum = 0;
            for (int k = 0; k < N; k++)
                sum += (int64_t)a[i * N + k] * (int64_t)b[k * N + j];
            int64_t v = sum >> 15;
            if (v > 32767)      c[i * N + j] = 32767;
            else if (v < -32768) c[i * N + j] = -32768;
            else                 c[i * N + j] = (q15_t)v;
        }
}

template <int N>
static void mat_add_naive_q15(const q15_t *a, const q15_t *b, q15_t *c)
{
    for (int i = 0; i < N * N; i++)
        c[i] = a[i] + b[i];
}

template <int N>
static void mat_trans_naive_q15(const q15_t *a, q15_t *d)
{
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            d[j * N + i] = a[i * N + j];
}

/* ---------------- 数据初始化 ---------------- */
static void fill_plain(float *m, int r, int c)
{
    for (int i = 0; i < r; i++)
        for (int j = 0; j < c; j++)
            m[i * c + j] = 0.01f * (float)(((i * c + j) % 13) + 1) + 0.001f * (float)(i - j);
}

/* 对角线占优, 保证可逆且条件数良好 */
static void fill_inv(float *m, int n)
{
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            m[i * n + j] = (i == j) ? (2.0f + 0.1f * (float)i)
                                    : (0.01f * (float)(((i * n + j) % 7) + 1));
}

/* 定点: 值域约 ±0.125 / ±0.122, 保证 16 项累加不溢出 int64、不改动结果 */
static void fill_q31(q31_t *m, int r, int c)
{
    for (int i = 0; i < r; i++)
        for (int j = 0; j < c; j++)
            m[i * c + j] = (q31_t)(((i * c + j) % 9 - 4) << 26);
}

static void fill_q15(q15_t *m, int r, int c)
{
    for (int i = 0; i < r; i++)
        for (int j = 0; j < c; j++)
            m[i * c + j] = (q15_t)(((i * c + j) % 9 - 4) * 1000);
}

static void data_init(void)
{
    fill_plain(mA, 16, 16);
    fill_plain(mB, 16, 16);
    fill_inv(m3A, 3);
    fill_inv(m6A, 6);
    fill_q31(qA, 16, 16);
    fill_q31(qB, 16, 16);
    fill_q15(hA, 16, 16);
    fill_q15(hB, 16, 16);

    arm_mat_init_f32(&I16_A, 16, 16, mA);
    arm_mat_init_f32(&I16_B, 16, 16, mB);
    arm_mat_init_f32(&I16_D, 16, 16, mD);
    arm_mat_init_f32(&I3_SRC, 3, 3, m3C);
    arm_mat_init_f32(&I3_DST, 3, 3, m3Y);
    arm_mat_init_f32(&I6_SRC, 6, 6, m6C);
    arm_mat_init_f32(&I6_DST, 6, 6, m6Y);
    arm_mat_init_q31(&Q_A, 16, 16, qA);
    arm_mat_init_q31(&Q_B, 16, 16, qB);
    arm_mat_init_q31(&Q_D, 16, 16, qD);
    arm_mat_init_q15(&H_A, 16, 16, hA);
    arm_mat_init_q15(&H_B, 16, 16, hB);
    arm_mat_init_q15(&H_D, 16, 16, hD);
}

/* ---------------- 计时基准 ---------------- */
template <typename Fn>
static uint32_t bench(uint32_t iters, Fn &&fn, volatile float *acc)
{
    uint32_t t0 = dwt_cycles();
    float s    = 0.0f;
    for (uint32_t k = 0; k < iters; k++)
        s += fn();
    uint32_t t1 = dwt_cycles();
    if (acc)
        *acc = s;
    return (t1 - t0) / iters;
}

static float max_abs_diff(const float *a, const float *b, int n)
{
    float mx = 0.0f;
    for (int i = 0; i < n; i++) {
        float d = a[i] - b[i];
        if (d < 0)
            d = -d;
        if (d > mx)
            mx = d;
    }
    return mx;
}

/* 定点: 直接返回整数最大绝对差 */
template <typename T>
static T max_abs_diff_t(const T *a, const T *b, int n)
{
    T mx = 0;
    for (int i = 0; i < n; i++) {
        T d = a[i] - b[i];
        if (d < 0)
            d = -d;
        if (d > mx)
            mx = d;
    }
    return mx;
}

/* ---------------- 报告输出 ---------------- */
static void report_op(const char *title, uint32_t iters,
                      uint32_t cyc_cmsis, uint32_t cyc_naive, float maxdiff)
{
    if (cyc_cmsis == 0) cyc_cmsis = 1; /* 防除零(DWT 未计数时不至于崩溃) */
    if (cyc_naive == 0) cyc_naive = 1;
    uint32_t cpu   = SystemCoreClock / 1000000u; /* 每微秒周期数 */
    uint32_t usc   = cyc_cmsis / cpu;
    uint32_t ufc   = ((cyc_cmsis % cpu) * 1000u) / cpu;
    uint32_t usn   = cyc_naive / cpu;
    uint32_t ufn   = ((cyc_naive % cpu) * 1000u) / cpu;
    uint32_t spd   = (uint32_t)(((uint64_t)cyc_naive * 100u) / cyc_cmsis);
    uint32_t md    = (uint32_t)(maxdiff * 1e6f);

    tx_printf("%-9s x%-6lu  CMSIS %6lu cyc %4lu.%03lu us | 纯C %6lu cyc %4lu.%03lu us | 加速 %lu.%02lu x | 误差 %lu e-6\n",
              title,
              (unsigned long)iters,
              (unsigned long)cyc_cmsis, (unsigned long)usc, (unsigned long)ufc,
              (unsigned long)cyc_naive, (unsigned long)usn, (unsigned long)ufn,
              (unsigned long)(spd / 100u), (unsigned long)(spd % 100u),
              (unsigned long)md);
}

/* 定点行的报告: 误差为整数(不放大) */
static void report_fx(const char *title, uint32_t iters,
                      uint32_t cyc_cmsis, uint32_t cyc_naive, int32_t maxdiff)
{
    if (cyc_cmsis == 0) cyc_cmsis = 1;
    if (cyc_naive == 0) cyc_naive = 1;
    uint32_t cpu   = SystemCoreClock / 1000000u;
    uint32_t usc   = cyc_cmsis / cpu;
    uint32_t ufc   = ((cyc_cmsis % cpu) * 1000u) / cpu;
    uint32_t usn   = cyc_naive / cpu;
    uint32_t ufn   = ((cyc_naive % cpu) * 1000u) / cpu;
    uint32_t spd   = (uint32_t)(((uint64_t)cyc_naive * 100u) / cyc_cmsis);

    tx_printf("%-9s x%-6lu  CMSIS %6lu cyc %4lu.%03lu us | 纯C %6lu cyc %4lu.%03lu us | 加速 %lu.%02lu x | 误差 %ld\n",
              title,
              (unsigned long)iters,
              (unsigned long)cyc_cmsis, (unsigned long)usc, (unsigned long)ufc,
              (unsigned long)cyc_naive, (unsigned long)usn, (unsigned long)ufn,
              (unsigned long)(spd / 100u), (unsigned long)(spd % 100u),
              (long)maxdiff);
}

/* ---------------- 基准主体 ---------------- */
static void run_bench(void)
{
    volatile float s1, s2;
    uint32_t it;
    uint32_t cpu = SystemCoreClock / 1000000u;
    uint32_t nsx = 100000u / cpu; /* 每周期约 ns(单位放大100倍): 1000ns/100*cpu */

    tx_printf("[CMSIS-DSP 矩阵计算基准] 内核 %lu MHz | FPU 单精度 | 每周期 ~= %lu.%02lu ns\n\n",
              (unsigned long)(SystemCoreClock / 1000000u),
              (unsigned long)(nsx / 100u),
              (unsigned long)(nsx % 100u));

    /* 16x16 乘法 */
    it = 1000;
    uint32_t c_mul = bench(it, [&]() -> float { arm_mat_mult_f32(&I16_A, &I16_B, &I16_D); return I16_D.pData[0] + I16_D.pData[255]; }, &s1);
    uint32_t n_mul = bench(it, [&]() -> float { mat_mul_naive<16>(mA, mB, mC);          return mC[0] + mC[255]; }, &s2);
    report_op("mul16x16", it, c_mul, n_mul, max_abs_diff(mC, mD, 256));

    /* 16x16 加法 */
    it = 10000;
    uint32_t c_add = bench(it, [&]() -> float { arm_mat_add_f32(&I16_A, &I16_B, &I16_D); return I16_D.pData[0] + I16_D.pData[255]; }, &s1);
    uint32_t n_add = bench(it, [&]() -> float { mat_add_naive<16>(mA, mB, mC);           return mC[0] + mC[255]; }, &s2);
    report_op("add16x16", it, c_add, n_add, max_abs_diff(mC, mD, 256));

    /* 16x16 转置 */
    it = 10000;
    uint32_t c_tr = bench(it, [&]() -> float { arm_mat_trans_f32(&I16_A, &I16_D); return I16_D.pData[0] + I16_D.pData[255]; }, &s1);
    uint32_t n_tr = bench(it, [&]() -> float { mat_trans_naive<16>(mA, mC);       return mC[0] + mC[255]; }, &s2);
    report_op("trans16x16", it, c_tr, n_tr, max_abs_diff(mC, mD, 256));

    /* 3x3 求逆 (源矩阵会被 CMSIS 修改, 每次先拷贝) */
    it = 10000;
    uint32_t c_i3 = bench(it, [&]() -> float { memcpy(m3C, m3A, sizeof m3A); arm_mat_inverse_f32(&I3_SRC, &I3_DST); return I3_DST.pData[0] + I3_DST.pData[8]; }, &s1);
    uint32_t n_i3 = bench(it, [&]() -> float { mat_inv_naive<3>(m3A, m3X);     return m3X[0] + m3X[8]; }, &s2);
    report_op("inv3x3", it, c_i3, n_i3, max_abs_diff(m3X, m3Y, 9));

    /* 6x6 求逆 */
    it = 2000;
    uint32_t c_i6 = bench(it, [&]() -> float { memcpy(m6C, m6A, sizeof m6A); arm_mat_inverse_f32(&I6_SRC, &I6_DST); return I6_DST.pData[0] + I6_DST.pData[35]; }, &s1);
    uint32_t n_i6 = bench(it, [&]() -> float { mat_inv_naive<6>(m6A, m6X);     return m6X[0] + m6X[35]; }, &s2);
    report_op("inv6x6", it, c_i6, n_i6, max_abs_diff(m6X, m6Y, 36));

    /* ===================== 定点 Q31 ===================== */
    tx_printf("\n[Q31 定点] 16x16 (值域约 ±0.125; CMSIS 标量仍为普通 C, 无 SIMD)\n");

    it = 1000;
    uint32_t q31_mul_c = bench(it, [&]() -> float { arm_mat_mult_q31(&Q_A, &Q_B, &Q_D); return (float)Q_D.pData[0] + (float)Q_D.pData[255]; }, &s1);
    uint32_t q31_mul_n = bench(it, [&]() -> float { mat_mul_naive_q31<16>(qA, qB, qC);   return (float)qC[0] + (float)qC[255]; }, &s2);
    report_fx("q31mul16", it, q31_mul_c, q31_mul_n, max_abs_diff_t(qC, qD, 256));

    it = 10000;
    uint32_t q31_add_c = bench(it, [&]() -> float { arm_mat_add_q31(&Q_A, &Q_B, &Q_D); return (float)Q_D.pData[0] + (float)Q_D.pData[255]; }, &s1);
    uint32_t q31_add_n = bench(it, [&]() -> float { mat_add_naive_q31<16>(qA, qB, qC);   return (float)qC[0] + (float)qC[255]; }, &s2);
    report_fx("q31add16", it, q31_add_c, q31_add_n, max_abs_diff_t(qC, qD, 256));

    it = 10000;
    uint32_t q31_tr_c = bench(it, [&]() -> float { arm_mat_trans_q31(&Q_A, &Q_D); return (float)Q_D.pData[0] + (float)Q_D.pData[255]; }, &s1);
    uint32_t q31_tr_n = bench(it, [&]() -> float { mat_trans_naive_q31<16>(qA, qC);      return (float)qC[0] + (float)qC[255]; }, &s2);
    report_fx("q31trans16", it, q31_tr_c, q31_tr_n, max_abs_diff_t(qC, qD, 256));

    /* ===================== 定点 Q15 (CM4 上 CMSIS 用 __SMLALD 双16位 SIMD) ===================== */
    tx_printf("\n[Q15 定点] 16x16 (值域约 ±0.122; CMSIS 走 DSP SIMD, 理应更快)\n");

    it = 1000;
    uint32_t q15_mul_c = bench(it, [&]() -> float { arm_mat_mult_q15(&H_A, &H_B, &H_D, hState); return (float)H_D.pData[0] + (float)H_D.pData[255]; }, &s1);
    uint32_t q15_mul_n = bench(it, [&]() -> float { mat_mul_naive_q15<16>(hA, hB, hC);    return (float)hC[0] + (float)hC[255]; }, &s2);
    report_fx("q15mul16", it, q15_mul_c, q15_mul_n, max_abs_diff_t(hC, hD, 256));

    it = 10000;
    uint32_t q15_add_c = bench(it, [&]() -> float { arm_mat_add_q15(&H_A, &H_B, &H_D); return (float)H_D.pData[0] + (float)H_D.pData[255]; }, &s1);
    uint32_t q15_add_n = bench(it, [&]() -> float { mat_add_naive_q15<16>(hA, hB, hC);    return (float)hC[0] + (float)hC[255]; }, &s2);
    report_fx("q15add16", it, q15_add_c, q15_add_n, max_abs_diff_t(hC, hD, 256));

    it = 10000;
    uint32_t q15_tr_c = bench(it, [&]() -> float { arm_mat_trans_q15(&H_A, &H_D); return (float)H_D.pData[0] + (float)H_D.pData[255]; }, &s1);
    uint32_t q15_tr_n = bench(it, [&]() -> float { mat_trans_naive_q15<16>(hA, hC);       return (float)hC[0] + (float)hC[255]; }, &s2);
    report_fx("q15trans16", it, q15_tr_c, q15_tr_n, max_abs_diff_t(hC, hD, 256));
}

int main()
{
    DLX_NVIC_AutoConfig();

    auto usart1 = USART::USART1_TA9_RAA();
    usart1.init(USARTModeProfile::WL8_SB1_PN_RXTX_FCN, 115200);

    dwt_init();
    data_init();

    tx_reset();
    run_bench();
    uint32_t txlen = (uint32_t)g_txpos;

    ByteBuffer txBuf((uint8_t *)g_txbuf, sizeof(g_txbuf));
    txBuf.cur = txBuf.src + txlen;
    auto serialDMA = usart1.setDMASend(txBuf);

    while (true) {
        serialDMA.start();
        serialDMA.wait();
        serialDMA.reset(txlen);
        dlx::delay_ms(1000);
    }
}
