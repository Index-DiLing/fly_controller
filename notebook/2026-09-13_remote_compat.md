# 遥控器侧要改什么(对比最初的 main_final_test.cpp)

> 面向：遥控器(接收 NRF 遥测那端)固件。
> 结论先说：**帧格式一个字节都没变**(类型 8、负载 30B、帧 32B、CRC4 覆盖 30B、50Hz)，
> 只有负载最后 2 字节的拆法 + `height` 字段的语义变了。

---

## 1. FlightCoreStatus(类型 8)负载布局

| 负载偏移 | 长度 | 字段 | final_test | 发布版 |
| --- | --- | --- | --- | --- |
| 0 ~ 15 | 16 | `Quaternion0..3`(float32×4, 大端) | 四元数 | **同** |
| 16 ~ 23 | 8 | `motor0..3`(uint16×4, 大端) | 四路电机 | **同** |
| 24 ~ 27 | 4 | `height`(float32, 大端) | 真实高度 [m] | **垂速 [m/s]**(气压计可用时)，详见 §2 |
| 28 | 1 | `status`(uint8) | — | 新增：实时状态位 |
| 29 | 1 | `error`(uint8) | `error` 是 **uint16**，占 28~29 | 8 位错误位，粘滞 |

**遥控器要做的第一件事**：把最后那个 `UInt16 error` 换成两个 `UInt8`：

```kotlin
// 遥控器侧 Protocol.kt 的 FlightCoreStatus 里
@TargetBasicType(DLXBasicType.UINT8)
val status: UByte,

@TargetBasicType(DLXBasicType.UINT8)
val error: UByte,
```

然后重新生成协议代码(KSP) —— 类型号、CRC、长度都不受影响，**不需要做任何版本兼容或迁移**。

## 2. `height` 字段：现在装的是垂速，不是高度

| 情况 | `height` 字段里的值 |
| --- | --- |
| `status` bit7(`0x80`) = 1 ⇒ 气压计可用 | **EKF 估计的垂向速度** [m/s] |
| `status` bit7 = 0 ⇒ 气压计不可用 | 退回 EKF 高度 [m] |

字段名不改(历史原因)，**解码必须看 `status` 的 bit7**：

```kotlin
if ((status.toInt() and 0x80) != 0) vz = height else alt = height
```

> ⚠️ 不要用 `error` 的 bit2 判断：`error` 是**粘滞**的(见 §3)，气压计出过一次错之后就再也
> 不会归零，用它判断会导致后续一直把垂速当高度显示。

## 3. `status`(实时) / `error`(粘滞) 位定义

`status` —— 电平型，每次回传(50Hz)都按当前状态重算：

| 位 | 值 | 含义 |
| --- | --- | --- |
| 0 | `0x01` | 电机已使能(解锁完成，控制已接管) |
| 1 | `0x02` | 启动序列进行中 |
| 2 | `0x04` | 角度环正在工作 |
| 3 | `0x08` | 遥控链路在线 |
| 4 | `0x10` | 收到硬停机命令 |
| 5 | `0x20` | 失控保护触发过(本会话锁存) |
| 6 | `0x40` | flash 日志写满，已停止记录 |
| 7 | `0x80` | 气压计可用 ⇒ `height` 字段是垂速 |

`error` —— 只放真故障，**飞控报的是实时值**：这一刻没问题就回 0，飞控端不做锁存：

| 位 | 值 | 含义 |
| --- | --- | --- |
| 0 | `0x01` | flash 日志写入失败 |
| 1 | `0x02` | BMI088 自检失败 |
| 2 | `0x04` | 气压计不可用 |

所以在遥控器上要做两件事：

1. **判"此刻健康"看 `error` 的实时值**(它会自己归零，别以为是丢包或者没刷新)；
2. **怕一闪而过就自己锁存**——按位或累积一份"本架次出现过的错误"，这是遥控器端的事
   (飞控故意不做，免得把"曾经错过"和"此刻还在错"混成一个字段)：

```kotlin
var errSeen = 0                    // 本架次累计(上电/重新连接时清零)
...
errSeen = errSeen or error.toInt() // 粘滞显示用
```

上位机 `Controller.kt` 就是这么做的：`teleErr`(实时) + `teleErrSeen`(累计)两条曲线并排看。

## 4. 没变的部分(不用动)

- **下行**：NRF 上仍然只有 `FlightCoreStatus` 这一条，50Hz(`kConfig.telTickMs = 20`)，
  和 final_test 的 `kTelTickMs = 20` 一致；
- **上行**：遥控器发的 `ControlToFlight`(类型 7)结构没动；
- `FlightDebug`(类型 10)在 final_test 里是给**串口/日志**用的(进 `sharedLog`，不发给遥控器)，
  发布版里由 `FlightLog`(类型 15)取代，**NRF 上不会出现**，遥控器不用管；
- 旧版 `UnBlock`(类型 1)兼容仍然保留。

## 5. 遥控器侧改动清单(照着勾)

- [ ] `Protocol.kt` 的 `FlightCoreStatus`：`error: UShort` → `status: UByte` + `error: UByte`
- [ ] 重新生成协议代码
- [ ] 解析 `height` 时改成看 `status` bit7，而不是 `error` bit2
- [ ] `error` 按**实时值**使用(飞控会自己归零)；需要"不漏看"就自己按位或锁存一份
      `errSeen |= error`，上电/重新连接时清零
- [ ] `status` 按**实时值**使用(8 位定义见 §3)，至少用它判断 `height` 字段语义
- [ ] 界面/指示灯建议：`status` 显示当前状态(已解锁/链路在线/气压计可用…)、
      `errSeen` 显示"本架次出过什么错"、`error` 显示"此刻有没有错"
- [ ] 有数据记录的话把 `status`/`error` 都存下来(各 1 字节，不占地方)
