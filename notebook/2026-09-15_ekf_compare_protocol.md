# 2026-09-15 EkfCompare：Madgwick vs 16 维 EKF 的对比回传报文

用途：`main_att_ekf.cpp` 里两个滤波器吃同一份 BMI088 数据。现有回传只给了
`Quat`(类型 2，Madgwick 四元数) 和 `EKF`(类型 10，其实**只发了四元数 4 个 float** + height + vh)，
拿不到 EKF 的速度/零偏/协方差，也拿不到两者的输入，做不了归因分析。

所以新增**一个**下行报文 `EkfCompare`（类型号 **16**，负载 **144 B**，500 Hz）：

- 一次拿到两个滤波器的姿态 → 可以算夹角、比 roll/pitch/yaw；
- 一次拿到 EKF 的**内部状态**（速度、陀螺零偏）与**协方差对角**（姿态/速度/位置z/陀螺零偏）→ 能看收敛、置信度、是否发散；
- 带**输入快照**（最后一个陀螺+加速度样本）与计数/标志 → 能判断抖动、FIFO 积压、倾角门限拒绝；
- 标量 + 数组混合，两端都用生成代码，字节序不用手抠（见 §2）。

本文档自包含：Kotlin 类定义、字节布局、C++ 生成接口、固件三处改动（缓冲区 / 滤波器加 getter /
填充代码）、开销核算、字段取舍理由、采集动作建议。

---

## 1. Kotlin 类定义（贴到 `Protocol.kt` 末尾）

```kotlin
/**
 * EkfCompare — 姿态估计对比包(下位机 -> 上位机, 单向, 类型号 16)
 *
 * 同一份 BMI088 输入下, 一次回传:
 *   - Madgwick 与 EKF 的姿态四元数(可直接算两者夹角);
 *   - EKF 名义状态里 Madgwick 没有的量: 世界系速度 + 陀螺零偏;
 *   - EKF 误差态协方差对角(δθ / δv / δpz / δbg): 看置信度与收敛;
 *   - 输入快照与计数/标志: 判断抖动、FIFO 积压、倾角门限拒绝、气压是否更新。
 *
 * 字节序: 标量大端, 定长数组原始小端(与 FlightLog 一致, 两端都用生成代码即不会错)。
 * 只在 main_att_ekf.cpp 发送; 不产生接收结构体/回调。
 */
class EkfCompare(
    // ---------------- 时序 / 计数 / 标志: 16 B ----------------
    @TargetBasicType(DLXBasicType.UINT32)
    val seq: Int,                 // 自增帧号(丢帧/乱序/与其它流的配对)
    @TargetBasicType(DLXBasicType.UINT32)
    val tickMs: Int,              // rt_tick_get(), 上电毫秒
    @TargetBasicType(DLXBasicType.UINT16)
    val loopPeriodUs: Int,        // 实测循环周期[us](DWT 测相邻两次唤醒间隔, 正常 2000)
    @TargetBasicType(DLXBasicType.UINT16)
    val flags: Int,               // 位定义见 §4
    @TargetBasicType(DLXBasicType.UINT8)
    val gyroCount: Int,           // 本帧处理的陀螺样本数(2000Hz/500Hz => 期望 4)
    @TargetBasicType(DLXBasicType.UINT8)
    val accelCount: Int,          // 本帧消费的加速度样本数(1600Hz/500Hz => 期望 3~4)
    @TargetBasicType(DLXBasicType.UINT8)
    val slowCount: Int,           // 本帧慢路径(协方差+倾角)执行次数(每 20 样本 1 次 => 0 或 1)
    @TargetBasicType(DLXBasicType.UINT8)
    val gateReject: Int,          // 本帧倾角更新被门限拒绝的次数(动态激励有多强)

    // ---------------- 输入快照: 24 B ----------------
    @TargetBasicType(DLXBasicType.FLOAT) @FixedLength(3)
    val gyro: FloatArray,         // 本帧最后一个陀螺样本 [rad/s], 机体系(已重映射)
    @TargetBasicType(DLXBasicType.FLOAT) @FixedLength(3)
    val accel: FloatArray,        // 与它配套的加速度 [g], 机体系(已重映射)

    // ---------------- 两个滤波器的姿态: 32 B ----------------
    @TargetBasicType(DLXBasicType.FLOAT) @FixedLength(4)
    val madgwickQuat: FloatArray, // Madgwick (w,x,y,z), 机体->世界
    @TargetBasicType(DLXBasicType.FLOAT) @FixedLength(4)
    val ekfQuat: FloatArray,      // EKF (w,x,y,z), 同约定

    // ---------------- EKF 名义状态: 32 B ----------------
    @TargetBasicType(DLXBasicType.FLOAT) @FixedLength(3)
    val vel: FloatArray,          // 世界系速度 (vx,vy,vz) [m/s]; vz 即垂速
    @TargetBasicType(DLXBasicType.FLOAT) @FixedLength(3)
    val gyroBias: FloatArray,     // EKF 估计的陀螺零偏 [rad/s] (Madgwick 没有这个量)
    @TargetBasicType(DLXBasicType.FLOAT)
    val height: Float,            // EKF 高度 pz [m]
    @TargetBasicType(DLXBasicType.FLOAT)
    val baroRel: Float,           // 最近一次喂给 EKF 的气压高度观测(相对起飞点) [m]

    // ---------------- EKF 协方差对角: 40 B ----------------
    // 顺序 = 误差态 FlightFilterErrorIdx: δθ(3) δv(3) δp(3) δbg(3) δba(3)
    @TargetBasicType(DLXBasicType.FLOAT) @FixedLength(3)
    val attVar: FloatArray,       // δθ 对角(姿态不确定度, 开方即姿态标准差[rad])
    @TargetBasicType(DLXBasicType.FLOAT) @FixedLength(3)
    val velVar: FloatArray,       // δv 对角(速度不确定度)
    @TargetBasicType(DLXBasicType.FLOAT) @FixedLength(3)
    val gyroBiasVar: FloatArray,  // δbg 对角(零偏估计的不确定度 = 收敛程度)
    @TargetBasicType(DLXBasicType.FLOAT)
    val posVarZ: Float,           // δpz 方差(高度不确定度)
) {
    /** 两个滤波器的姿态夹角[deg]: 0 = 完全一致。这是"两个滤波器差多少"的唯一可比指标 */
    fun attitudeDiffDeg(): Float {
        var dot = 0f
        for (i in 0..3) dot += madgwickQuat[i] * ekfQuat[i]
        val c = kotlin.math.abs(dot).coerceAtMost(1f)
        return (2.0 * kotlin.math.acos(c.toDouble()) * 180.0 / kotlin.math.PI).toFloat()
    }

    /** 姿态标准差[deg] = sqrt(P_θθ), 拿它和 attitudeDiffDeg() 一起看是否自洽 */
    fun attSigmaDeg(): Triple<Float, Float, Float> = Triple(
        (kotlin.math.sqrt(attVar[0]) * 180.0 / kotlin.math.PI).toFloat(),
        (kotlin.math.sqrt(attVar[1]) * 180.0 / kotlin.math.PI).toFloat(),
        (kotlin.math.sqrt(attVar[2]) * 180.0 / kotlin.math.PI).toFloat(),
    )

    override fun toString(): String =
        "EkfCompare(seq=$seq t=${tickMs}ms dt=${loopPeriodUs}us flags=0x${flags.toString(16)} " +
            "gy=$gyroCount ac=$accelCount slow=$slowCount gate=$gateReject) " +
            "gyro=${gyro.contentToString()} accel=${accel.contentToString()} " +
            "madgwick=${madgwickQuat.contentToString()} ekf=${ekfQuat.contentToString()} " +
            "diff=${attitudeDiffDeg()}deg vel=${vel.contentToString()} bg=${gyroBias.contentToString()} " +
            "h=$height baro=$baroRel attVar=${attVar.contentToString()}"
}
```

> 追加到文件**末尾**很关键：类型号 = 类在文件里的声明顺序（0…15 已占满），
> 插在中间会让后面所有报文集体错位（固件/上位机类型号不一致 → 解出一堆"看起来合法"的垃圾帧）。

---

## 2. 字节布局（负载共 144 B，帧 = 头 2 B + 144 B = 146 B）

| 偏移 | 字段 | 类型 | 字节 | 说明 |
| --- | --- | --- | --- | --- |
| 0 | `seq` | UINT32 大端 | 4 | 自增帧号 |
| 4 | `tickMs` | UINT32 大端 | 4 | 上电毫秒 |
| 8 | `loopPeriodUs` | UINT16 大端 | 2 | 实测循环周期 |
| 10 | `flags` | UINT16 大端 | 2 | 见 §4 |
| 12 | `gyroCount` | UINT8 | 1 | 期望 4 |
| 13 | `accelCount` | UINT8 | 1 | 期望 3~4 |
| 14 | `slowCount` | UINT8 | 1 | 0 或 1 |
| 15 | `gateReject` | UINT8 | 1 | 0…slowCount |
| 16 | `gyro[3]` | FLOAT 数组(小端) | 12 | @16/@20/@24 |
| 28 | `accel[3]` | FLOAT 数组 | 12 | @28/@32/@36 |
| 40 | `madgwickQuat[4]` | FLOAT 数组 | 16 | w,x,y,z |
| 56 | `ekfQuat[4]` | FLOAT 数组 | 16 | w,x,y,z |
| 72 | `vel[3]` | FLOAT 数组 | 12 | vx,vy,vz |
| 84 | `gyroBias[3]` | FLOAT 数组 | 12 | bgx,bgy,bgz |
| 96 | `height` | FLOAT 大端 | 4 | pz |
| 100 | `baroRel` | FLOAT 大端 | 4 | 气压观测 |
| 104 | `attVar[3]` | FLOAT 数组 | 12 | δθ 对角 |
| 116 | `velVar[3]` | FLOAT 数组 | 12 | δv 对角 |
| 128 | `gyroBiasVar[3]` | FLOAT 数组 | 12 | δbg 对角 |
| 140 | `posVarZ` | FLOAT 大端 | 4 | δpz 方差 |

负载全程 4 字节对齐、总长 144（4 的倍数）：上位机若要按固定偏移手解析也很省事。

---

## 3. 生成出来的 C++ 接口（KSP 生成，不用手写）

```cpp
// 单向报文: 只有写函数, 不产生 struct/callback, 也不进 check() 的 switch
bool EkfCompareW(uint32_t seq, uint32_t tickMs, uint16_t loopPeriodUs, uint16_t flags,
                 uint8_t gyroCount, uint8_t accelCount, uint8_t slowCount, uint8_t gateReject,
                 float *gyro, float *accel,
                 float *madgwickQuat, float *ekfQuat,
                 float *vel, float *gyroBias, float height, float baroRel,
                 float *attVar, float *velVar, float *gyroBiasVar, float posVarZ);
```

（签名里的数组参数是 `float*`，所以传进去的必须是非 const 的 `float[3]/[4]`。)

---

## 4. `flags` 位定义

| 位 | 含义 |
| --- | --- |
| 0 | BME280 初始化成功（`bmeOk`） |
| 1 | 气压基准已锁定（`baroRefSet`） |
| 2 | 本帧有气压观测送入 EKF |
| 3 | 本帧跑过慢路径（等价于 `slowCount > 0`，方便直接筛） |
| 4 | 本帧倾角更新被门限拒绝（等价于 `gateReject > 0`） |
| 5 | EKF `isValid()` |
| 6 | 检测到非有限数（四元数/高度里有 NaN/Inf） |
| 7 | 陀螺零偏撞限幅（需要 §5.2 的 getter，不加就恒 0） |
| 8…15 | 保留（填 0） |

---

## 5. 固件侧三处改动

### 5.1 `main_att_ekf.cpp`：发送缓冲区必须加大（**否则静默丢帧**）

三帧共享同一个 `dmaBuffer`：`Quat` 18 B + `EKF` 26 B + `EkfCompare` 146 B = **190 B**，
超过了现在的 128 B —— `buffer.remaining()` 不够时生成代码直接 `return false`（什么都不发），
现象就是"上位机一条对比包都收不到"，还不报错。

```cpp
    uint8_t dmaB[256];                 // 原来是 128
    ByteBuffer dmaBuffer(dmaB, 256);   // 原来是 128
    uint8_t fr[256];                   // 建议一起改 256(解帧临时区, 规则: 不小于最大 payload)
    ByteBuffer frame(fr, 256);
```

### 5.2 `DL_LIB/flight_control/flight_control_fliter.hpp`：加两个只读接口

`FlightControlFilter` 的 `_P` 是私有的，遥测要读协方差对角；倾角门限是否拒绝也只有它自己知道。
两处都是加法，不改现有行为（`updateAccelerometer` 由 `void` 改成 `bool`，原调用点忽略返回值即可）：

```cpp
        // 放在 public 的 getHeight()/getState16() 附近
        // 误差态协方差对角线, 顺序同 FlightFilterErrorIdx: δθ(3) δv(3) δp(3) δbg(3) δba(3)
        void  getCovDiag(float out[FF_ERROR_COUNT]) const
        {
            for (int i = 0; i < FF_ERROR_COUNT; ++i) { out[i] = _P.d[i][i]; }
        }
        float getAttVar(int i) const       { return _P.d[FF_E_DTH0 + i][FF_E_DTH0 + i]; }
        float getVelVar(int i) const       { return _P.d[FF_E_DV0 + i][FF_E_DV0 + i]; }
        float getPosVar(int i) const       { return _P.d[FF_E_DP0 + i][FF_E_DP0 + i]; }
        float getGyroBiasVar(int i) const  { return _P.d[FF_E_DBG0 + i][FF_E_DBG0 + i]; }
        float getAccelBiasVar(int i) const { return _P.d[FF_E_DBA0 + i][FF_E_DBA0 + i]; }
```

```cpp
        // 原: void updateAccelerometer(const Vector3f &accel_mss)
        // 返回 false = 被倾角门限拒绝, 没做修正(供遥测统计)
        bool updateAccelerometer(const Vector3f &accel_mss)
        {
            const float n = norm(accel_mss);
            if (fabsf(n - _params.gravity_mss) > _params.accel_tilt_gate_mss) {
                return false;                       // 高动态, 丢弃
            }
            ...                                     // 中间不动
            kalmanUpdate(H, Rz, innov);
            return true;                            // 末尾补这一行
        }

        bool updateAccelerometer(const AccelerometerG &accel_g)
        {
            Vector3f accel_mss(accel_g.data[0] * _params.gravity_mss,
                               accel_g.data[1] * _params.gravity_mss,
                               accel_g.data[2] * _params.gravity_mss);
            return updateAccelerometer(accel_mss);
        }
```

### 5.3 `main_att_ekf.cpp`：填充与发送

（1）文件顶部，`loopCostCnt` 那一组全局量旁边：

```cpp
uint32_t cmpSeq        = 0;   // EkfCompare 自增序号
uint32_t lastLoopStart = 0;   // 上一次循环起点(DWT 周期), 用于实测周期

constexpr uint8_t kCmpDecim = 1;   // 1 = 每轮都发(500Hz); 2 = 每两轮发(250Hz), 省一半带宽
uint8_t cmpDiv             = 0;
```

（2）主循环里，进入 `while (true)` 后立刻把"本帧"的统计量清零：

```cpp
    while (true) {
        rt_sem_take(mutex, RT_WAITING_FOREVER);

        const uint32_t loopStart = DWT->CYCCNT;
        bool    baroUpdatedThisFrame = false;   // 新增
        uint8_t slowCnt              = 0;       // 新增
        uint8_t gateReject           = 0;       // 新增

        bmi.fifoRead(ac, gy);
```

（3）慢路径处改成计数（`updateAccelerometer` 现在有返回值）：

```cpp
            if (++imuCnt >= kEkfDecim) {
                ekf.propagateCovariance(gyroR, curAcc, imuDtAccum);
                if (!ekf.updateAccelerometer(curAcc)) { ++gateReject; }  // 被门限拒绝
                ++slowCnt;
                imuCnt     = 0;
                imuDtAccum = 0.0f;
            }
```

（4）气压那块标记一下：

```cpp
            lastBaroObs = lastBaroAbs - baroRef;
            ekf.updateBarometer(lastBaroObs);
            baroUpdatedThisFrame = true;        // 新增
```

（5）上报区，紧跟在现有 `protocol.EKFW(...)` 之后（`quatMadgwick` 还在作用域内）：

```cpp
        // ---- 对比包: 两个滤波器 + EKF 内部量 (类型 16, 144B 负载) ----
        if (++cmpDiv >= kCmpDecim) {
            cmpDiv = 0;

            const Quaternion qE = ekf.getQuaternion();
            const Vector3f   vE = ekf.getVelocity();
            const Vector3f   bE = ekf.getGyroBias();

            float ekfQuat[4]   = {qE.data[0], qE.data[1], qE.data[2], qE.data[3]};
            float velArr[3]    = {vE.x(), vE.y(), vE.z()};
            float bgArr[3]     = {bE.x(), bE.y(), bE.z()};
            float attVar[3]    = {ekf.getAttVar(0), ekf.getAttVar(1), ekf.getAttVar(2)};
            float velVar[3]    = {ekf.getVelVar(0), ekf.getVelVar(1), ekf.getVelVar(2)};
            float bgVar[3]     = {ekf.getGyroBiasVar(0), ekf.getGyroBiasVar(1),
                                  ekf.getGyroBiasVar(2)};

            // 实测循环周期: DWT 是 168MHz, /(SystemCoreClock/1e6) = us
            uint32_t periodUs = (uint32_t)(loopStart - lastLoopStart) /
                                (SystemCoreClock / 1000000u);
            lastLoopStart = loopStart;
            if (periodUs > 0xFFFFu) { periodUs = 0xFFFFu; }   // 第一帧无意义, 会夹住

            uint16_t cmpFlags = 0;
            if (bmeOk)                { cmpFlags |= 1u << 0; }
            if (baroRefSet)           { cmpFlags |= 1u << 1; }
            if (baroUpdatedThisFrame) { cmpFlags |= 1u << 2; }
            if (slowCnt)              { cmpFlags |= 1u << 3; }
            if (gateReject)           { cmpFlags |= 1u << 4; }
            if (ekf.isValid())        { cmpFlags |= 1u << 5; }
            if (!(isfinite(qE.data[0]) && isfinite(qE.data[1]) && isfinite(qE.data[2]) &&
                  isfinite(qE.data[3]) && isfinite(ekf.getHeight()) && isfinite(vE.x()))) {
                cmpFlags |= 1u << 6;                          // 非有限数
            }

            protocol.EkfCompareW(
                ++cmpSeq, (uint32_t)rt_tick_get(), (uint16_t)periodUs, cmpFlags,
                (uint8_t)gySize, (uint8_t)acSize, slowCnt, gateReject,
                lastGyro.data, lastAcc.data,
                quatMadgwick, ekfQuat,
                velArr, bgArr, ekf.getHeight(), lastBaroObs,
                attVar, velVar, bgVar, ekf.getPosVar(2));
        }
```

注意：`gySize/acSize` 是 `fifoRead` 之后的队列长度（本帧样本数），队列容量 42，
一旦看到它们稳定 >4（比如 8、12），说明主循环跟不上 FIFO，数据是"追赶"状态，分析时要剔除；
读数停在 **42** 就是队列饱和（超过的部分已经被 BMI088 FIFO 丢掉），那一段数据作废。

---

## 6. 开销核算

| 项 | 数值 |
| --- | --- |
| `EkfCompare` 帧 | 2 + 144 = 146 B |
| 一轮三帧合计（Quat 18 + EKF 26 + 146） | 190 B |
| 2 Mbaud / 8N1（10 bit/字节）→ 一轮占用串口 | ≈ 950 µs（2 ms 周期的 48%） |
| 数据量（500 Hz） | 73 kB/s 负载 / 95 kB/s 线路 |

- 这个占用是**测试台**可接受的（不是飞控主程序，`main_att_ekf.cpp` 只跑估计器）；
  但 DMA 是阻塞等待的，若发现 `loopPeriodUs` 抖动或 `gyroCount` 往上涨，
  把 `kCmpDecim` 改成 2（250 Hz，占用减半）或 4（125 Hz）即可。
- 想再省：`Quat`(18 B) 与 `EKF`(26 B) 两帧的信息已被对比包含（两个四元数 + height + 垂速），
  确认上位机不再依赖它们后可以停发，一轮降到 146 B。

---

## 7. 字段取舍（为什么是这些）

| 分析目标 | 需要的字段 |
| --- | --- |
| 两个滤波器差多少、谁跟得上手速 | `madgwickQuat`、`ekfQuat`、`gyro`、`accel`（可离线算两路各自的倾角误差） |
| 谁在漂、漂的方向对不对 | `gyroBias`（EKF 估的零偏）+ Madgwick 的 yaw 漂移斜率：静态段应互相印证 |
| 门限/动态策略差异 | `gateReject`、`slowCount`、`accel` 的模长（可判 0.05 m/s² 门限有多"卡") |
| EKF 是否收敛、有没有过度自信 | `attVar`、`gyroBiasVar`、`velVar`、`posVarZ`（开方与实测残差对比） |
| 高度环 | `height`、`baroRel`、`flags.b2`、`vel[2]` |
| 数据是不是可信 | `seq`、`tickMs`、`loopPeriodUs`、`gyroCount`/`accelCount`、`flags.b6` |

**刻意没放的**（以及理由）：

| 未放字段 | 理由 |
| --- | --- |
| 加速度零偏 `ba[3]` 及其方差 | `ekfParams.estimate_accel_bias = false`，它恒等于 0，报了是浪费 12 B |
| 位置 `px,py`、`posVarX/Y` | 无 GNSS 时水平位置不可观、只体现漂移，与"对比两个姿态滤波器"无关（要的话下方有扩展） |
| 姿态倾角残差 `inno[3]` | 可由 `ekfQuat` + `accel` 在 PC 端精确重算（h = R(q)ᵀ[0,0,1]，inno = â − h） |
| 欧拉角 | 四元数信息完整，且欧拉角在 PC 端算更灵活（避免万向锁歧义） |
| 全 15 维协方差 | 只丢了 δba(3) 与 δpx/δpy(2)，其余全在；真要全的见下方扩展 |

### 扩展（可选，不建议一开始就上）

1. 要 `px,py,posVarX/Y,baVar`：在 `EkfCompare` 末尾再加
   `@FixedLength(3) val pos: FloatArray`、`@FixedLength(3) val posVar: FloatArray`、
   `@FixedLength(3) val accelBias: FloatArray`（+36 B → 180 B 负载）。
2. 要 **2 kHz 全速率原始 IMU** 做离线重放：那是另一条设计（500 Hz 的包只能带"最后一个样本"），
   需要按 2000 Hz 分包回传，带宽会翻 4 倍 —— 本报文不承担这个。

---

## 8. 顺带发现的三处"注释与代码不一致"（会影响你怎么解读数据）

1. **慢路径其实是 100 Hz，不是 250 Hz**：
   `constexpr uint32_t kEkfDecim = 8*2.5;` → `8*2.5 = 20.0f`，转成 `uint32_t` 是 **20**，
   即每 20 个样本一次 = 2000/20 = **100 Hz**（累计 dt 10 ms），
   与代码注释"每 8 个样本 = 250 Hz"不符。对比时要注意：EKF 的协方差/倾角更新是 100 Hz。
2. **倾角门限是"收紧"不是"放宽"**：`ekfParams.accel_tilt_gate_mss = 0.05f;`
   注释写"放宽门限，接近 Madgwick 行为"，但默认值是 1.0 —— 0.05 m/s² ≈ ±5 mg，
   手一拿起来基本每帧都被拒绝（`gateReject` 会直接显示出来）。
   Madgwick 侧完全没有门限（β=0.1、2 kHz 一直在修），两者的"动态策略"差异是**主要的**差异来源。
3. **EKF 报文的负载只有 4 个 float**：`main_att_ekf.cpp` 传的是 `st.data`（16 个 float），
   但 `Protocol.kt` 的 `EKF` 类只声明了 `@FixedLength(4) val data` → 生成代码只写 16 B，
   速度/位置/零偏**从来没上过线**。这正是本对比包要补的东西。
   （另外 `kBaroIntervalMs = 20` 是 50 Hz，注释写的是 30 ms。）

---

## 9. 我拿到数据后会做的分析（采集时按这个来最省事）

给出两种东西之一即可：**CSV（每帧一行）** 或**原始二进制帧流**。
CSV 列顺序建议（与 kotlin 生成端字段顺序一致，便于我直接读）：

```
seq,tickMs,loopPeriodUs,flags,gyroCount,accelCount,slowCount,gateReject,
gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z,
mw_w,mw_x,mw_y,mw_z,ekf_w,ekf_x,ekf_y,ekf_z,
vel_x,vel_y,vel_z,bg_x,bg_y,bg_z,height,baroRel,
attVar_x,attVar_y,attVar_z,velVar_x,velVar_y,velVar_z,bgVar_x,bgVar_y,bgVar_z,posVarZ
```

我会输出：

1. 两路 roll/pitch/yaw 叠加曲线 + **姿态夹角**（`attitudeDiffDeg`）时序；
2. 静态段对比：Madgwick 的 yaw 漂移斜率 vs EKF `gyroBias`（应互为印证）+ 噪声(1σ)对比；
3. 动态段对比：夹角峰值/恢复时间、`gateReject` 与夹角的因果对齐；
4. EKF 自洽性：姿态标准差 `sqrt(attVar)` 与实际夹角分布对比（置信度是否合理）；
5. 高度环：`height` vs `baroRel`、垂速 vs 气压微分；
6. 数据健康：`gyroCount/accelCount/loopPeriodUs` 异常段标注（这些段落要剔除再统计）。

### 采集动作建议（一段 60~90 s 就够，动作之间停 1~2 s 让我能切段）

1. 静置 30 s（最重要：零偏收敛、漂移率、静态噪声）；
2. 绕 x/y 轴缓慢倾斜 ±30°，各 3 次（静态精度）；
3. 快速左右摇 10 s（动态响应、门限拒绝）；
4. 快速 90° 转向再回位（瞬态）；
5. 竖直抬升/下降 30 cm × 3（高度环，有气压才有意义）；
6. 结束前静置 10 s。

---

## 10. 落地状态与验证（2026-09-15）

已改（`EkfCompareW` 已由 KSP 生成，类型号 16）：

| 文件 | 改动 |
| --- | --- |
| `main_att_ekf.cpp` | 发送缓冲区 128→256（必须，否则静默丢帧）；对比包填充与发送；`slowCnt/gateReject/baroUpdatedThisFrame` 统计；`cmpSeq/loopPeriodUs` |
| `DL_LIB/flight_control/flight_control_fliter.hpp` | 新增 `getCovDiag/getAttVar/getVelVar/getPosVar/getGyroBiasVar/getAccelBiasVar`；`updateAccelerometer` 由 `void` 改 `bool`（返回是否真的做修正，原调用点忽略返回值即可） |
| `test/release_check/check.cpp` | 新增 §6：对比包的帧长/类型号/CRC4/全部字段偏移与字节序回归检查 |

验证结果：

- `powershell -ExecutionPolicy Bypass -File .\test\release_check\run.ps1` → **通过 54 项 / 失败 0 项**
  （其中 8 项是本次新增：帧长 146、类型号 16、CRC4 覆盖 144B、各组字段偏移与字节序）；
- 把发送块逐字搬到主机上、用真实头文件跑通：输出 146B 帧，CRC 匹配，各字段与内存值一一对应
  （顺便确认 `height / baroRel / posVarZ` 是标量→**大端**，`gyro/accel/四元数/速度/零偏/协方差` 是数组→小端）；
- `DL_LIB/flight_control/flight_control_test.cpp` 主机跑测：**ALL PASSED (0 failures)**，说明这次只加只读接口，没动滤波行为；
- `main_att_ekf.cpp` 的最终编译要在 Keil 里做：本机 shell 调 armclang 会报
  "license found is assigned to another user"，取不到授权。

**没有改动的**（只登记，等你决定）：第 §8 节那三处注释与代码不一致的地方，尤其
`kEkfDecim`（实际 20 → 慢路径 100 Hz）与 `accel_tilt_gate_mss = 0.05f`（很严的门限）保持原样，
所以这次回传的数据就是"现状行为"的真实反映。
