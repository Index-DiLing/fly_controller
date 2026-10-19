#pragma once
#include <stdint.h>
namespace dlx
{

    // 修改于 2026/10/7

    // 去除了所有OK的异常. 异常应该是异常,哪有异常包含OK的道理. 不过仍然保留了而未使用

    // 调用函数应当通过返回值是否SUCCESS判断,不SUCCESS再看ExceptionCode

    enum class ExceptionCode : uint16_t {
        EXCEPTION_NONE = 0x0000,
        // ============================ Flash 文件系统 ============================
        FLASH_SYSTEM_CREATED    = 0x0101, ///< 未检测到文件系统, 已整片擦除并新建(调用仍算成功, 仅提示异常)
        FLASH_GEOMETRY_MISMATCH = 0x0102, ///< 已有文件系统与本次编译的段布局/结构体大小不一致(不自动重建, 防止误删数据)
        FLASH_HARDWARE_ERROR    = 0x0103, ///< 读不到正确的 Flash ID(SPI 未初始化或硬件异常)
        FLASH_ERASE_FAILED      = 0x0104, ///< 擦除超时/失败
        FLASH_WRITE_FAILED      = 0x0105, ///< 写入超时/失败
        FLASH_READ_FAILED       = 0x0106, ///< 读取失败
        FLASH_NOT_READY         = 0x0107, ///< 未成功 init() 就调用其它接口
        FLASH_LOG_FULL          = 0x0108, ///< 单次日志已达上限且策略为 Halt: 本次写入被拒绝(日志数据仍然完整)
        FLASH_LOG_OVERWRITTEN   = 0x0109, ///< 单次日志已达上限, 已环绕覆写最旧的日志槽(提示性异常, 写入本身成功)
        FLASH_SESSION_NOT_FOUND = 0x010A, ///< 找不到指定的日志会话
        FLASH_PARAM_NOT_FOUND   = 0x010B, ///< 参数段还没有任何可用参数(上层用默认值即可)
        FLASH_INVALID_ARGUMENT  = 0x010C, ///< 入参非法(空指针等)
        FLASH_ENTRY_NOT_FOUND   = 0x010D, ///< 指定会话里没有这个序号的日志条目

        // ============================ 链路 / E220 ============================
        LINK_MODULE_NOT_FOUND     = 0x0201, ///< 配置模式下 AT 无响应(模块没插好/串口接错/模块未上电)
        LINK_CONFIG_FAILED        = 0x0202, ///< AT 写入返回失败
        LINK_CONFIG_VERIFY_FAILED = 0x0203, ///< 寄存器**回读**与写入不一致(别信"写进去就生效了")
        LINK_DESYNC               = 0x0204, ///< 时分失同步(连续 2 个槽没收到有效上行) -> 转静默只收不发
        LINK_RESYNCED             = 0x0205, ///< 已重新同步(提示性, 不是故障)
        LINK_FRAME_CRC            = 0x0206, ///< 收到 CRC 错的帧(预置噪声, 单次不算故障)
        LINK_FRAME_TYPE           = 0x0207, ///< 收到类型非法的帧(同上)
        LINK_TX_TIMEOUT           = 0x0208, ///< 写入 UART 超时
        LINK_AUX_TIMEOUT          = 0x0209, ///< AUX 长时间不表示空闲(UAUX=1 下才有意义)
        LINK_RX_OVERRUN           = 0x020A, ///< 接收溢出/DMA 覆盖, 有字节丢失
        LINK_PROFILE_CHANGED      = 0x020B, ///< 链路 profile 已切换(提示性)
        LINK_AT_TIMEOUT           = 0x020C, ///< 等模块串口应答超时(没接/波特率不对/透传模式下不理 AT)
        LINK_AT_REPLY_BAD         = 0x020D, ///< 应答形态不对(含 HEX 的 FF FF FF 错误响应)
        LINK_REG_CLAMPED          = 0x020E, ///< 写进去的和回读的不一样: 参数被模块钳位(手册 §7 章首第 5 条)
        LINK_BAD_ARGUMENT         = 0x020F, ///< 调用参数非法(空指针/长度越界): 是代码写错了, 不是模块的问题

        // ============================ IMU / BMI088 ============================
        IMU_NOT_FOUND       = 0x0301, ///< 读不到 CHIP_ID
        IMU_SELFTEST_FAILED = 0x0302, ///< 自检失败(现在只上报不拦截 -> B0 起也阻止解锁)
        IMU_FIFO_OVERRUN    = 0x0303, ///< FIFO 溢出, 有样本丢失
        IMU_SPI_ERROR       = 0x0304, ///< SPI 传输失败/超时
        IMU_DATA_STALE      = 0x0305, ///< 超过若干周期没拿到新样本
        IMU_CHIP_ID_ACCEL   = 0x0306, ///< 加速度计 CHIP_ID 不是 0x1E ⇒ 那一根片选/接线
        IMU_CHIP_ID_GYRO    = 0x0307, ///< 陀螺 CHIP_ID 不是 0x0F ⇒ 另一根片选/接线

        // ============================ 气压计 BMP388 ============================
        BARO_NOT_FOUND        = 0x0401, ///< 总线上没响应
        BARO_CHIP_ID_MISMATCH = 0x0402, ///< CHIP_ID != 0x50(读的是保留寄存器 = I2C 地址/总线错)
        BARO_CALIB_INVALID    = 0x0403, ///< 标定系数全 0/全 FF(NVM 没读出来)
        BARO_READ_FAILED      = 0x0404, ///< 读数据寄存器失败
        BARO_DATA_STALE       = 0x0405, ///< 长时间没有新数据

        // ============================ 测距 VL53L1X ============================
        RANGE_NOT_FOUND           = 0x0501, ///< 模型 ID / 上电后无响应
        RANGE_BOOT_TIMEOUT        = 0x0502, ///< XSHUT 释放后没在预期时间内起来
        RANGE_IO_VOLTAGE_FAILED   = 0x0503, ///< 0x002E/0x002F 写失败 -> 3.3V 电平模式没切过去(不切就可能完全不通)
        RANGE_ULD_LOAD_FAILED     = 0x0504, ///< 91 字节 ULD 配置写入失败
        RANGE_INVALID             = 0x0505, ///< 状态码表示本次量程无效(目标太近/太远/反光)
        RANGE_DATA_STALE          = 0x0506, ///< 长时间没有新测距
        RANGE_INT_REGISTER_FAILED = 0x0507, ///< 数据就绪中断订阅失败(EXTI 线已被占用/引脚配不上)。
                                            ///< 原来这种情况被报成 RANGE_NOT_FOUND(器件没响应) —— 方向完全错了

        // ============================ 光流 PMW3901 ============================
        FLOW_NOT_FOUND          = 0x0601, ///< SPI 无响应
        FLOW_INVERSE_PRODUCT_ID = 0x0602, ///< 读到的不是 0x49 —— 十有八九是 bit7 读写方向搞反了
        FLOW_SPI_GARBAGE        = 0x0603, ///< 抖一下读到全 0/全 FF(供电或时序余量不足)
        FLOW_LOW_QUALITY        = 0x0604, ///< SQUAL 低于阈值(提示性; 悬停时地面纹理差就会持续出现)
        FLOW_VERIFY_FAILED      = 0x0605, ///< 厂家初始化表有寄存器没落地(非严格模式已放行, 但必须留痕)

        // ============================ GNSS ============================
        GNSS_NO_FIX         = 0x0701, ///< 还没定位(fix=0)
        GNSS_NMEA_CHECKSUM  = 0x0702, ///< NMEA 校验和错
        GNSS_UART_OVERRUN   = 0x0703, ///< 串口溢出丢字节
        GNSS_SPEED_REJECTED = 0x0704, ///< 速度观测被 EKF 门限拒绝

        // ============================ 电源 / ADC ============================
        POWER_ADC_FAILED    = 0x0801, ///< ADC 自检/转换失败
        POWER_VOLTAGE_WARN  = 0x0802, ///< 电压偏低, 告警
        POWER_VOLTAGE_LAND  = 0x0803, ///< 电压低到必须降落
        POWER_VOLTAGE_CUT   = 0x0804, ///< 电压低到必须断动力
        POWER_CURRENT_HIGH  = 0x0805, ///< 过流
        POWER_SAMPLE_FAILED = 0x0806, ///< 一次电压/电流采样失败(单次不一定算掉线, 连续失败才降级)

        // ============================ 控制 / 飞行 ============================
        CONTROL_EKF_DIVERGED    = 0x0901, ///< EKF 协方差/状态跑飞(已限幅仍不正)
        CONTROL_ATTITUDE_LOST   = 0x0902, ///< 姿态不可信(翻滚/发散)
        CONTROL_MOTOR_SATURATED = 0x0903, ///< 有电机输出饱和(提示性)
        CONTROL_VEL_REJECTED    = 0x0904, ///< 速度观测被门限拒绝(光流/级联用)
        CONTROL_HOVER_LAND      = 0x0905, ///< 进入原地降落
        CONTROL_FLIP_DETECTED   = 0x0906, ///< 检测到翻转 -> 断动力

        // ============================ 状态机 ============================
        STATE_ILLEGAL_TRANSITION = 0x0A01, ///< 该事件在当前状态下不合法(迁移表没这一行)
        STATE_COMMAND_REJECTED   = 0x0A02, ///< 命令被准入表拒绝
        STATE_ARM_REJECTED       = 0x0A03, ///< 解锁前置条件不满足
        STATE_MODE_DEGRADED      = 0x0A04, ///< 当前模式所需数据源失效, 已自动降级
        STATE_HEALTH_CHANGED     = 0x0A05, ///< 健康位发生变化(提示性)
        STATE_ARM_BLOCKED        = 0x0A06, ///< 解锁被门槛拒绝(具体缺什么看同一批异常里的根因码)
        STATE_TASK_HANDOVER      = 0x0A07, ///< 任务把高度环交回某个模式(自动起飞交付/自动降落接管), 提示性

        // ============================ 配置 / 参数 ============================
        CONFIG_PARAM_NOT_FOUND    = 0x0B01, ///< 参数号越界
        CONFIG_PARAM_OUT_OF_RANGE = 0x0B02, ///< 参数值超出允许范围
        CONFIG_VERSION_MISMATCH   = 0x0B03, ///< 持久化参数表版本与固件不符
        CONFIG_SAVE_FAILED        = 0x0B04, ///< 参数落盘失败

        // ============================ 日志 / 存储 ============================
        LOG_QUEUE_FULL   = 0x0C01, ///< 控制线程 -> 日志线程的队列满, 本条被丢弃
        LOG_WRITE_FAILED = 0x0C02, ///< 写 flash 失败
        LOG_DUMP_ABORTED = 0x0C03, ///< 回传被上位机中止
        LOG_FORMATTED    = 0x0C04, ///< 布局不符, 已整片格式化(提示性; 会丢全部历史日志)
        LOG_NO_SINK      = 0x0C05, ///< 飞行模式但没有 flash 日志出口(芯片认不到/文件系统没就绪) —— 照样飞, 但要留痕

        // ============================ 调度 / 线程 / 看门狗 ============================
        SCHED_THREAD_START_FAILED = 0x0D01, ///< 线程创建/启动失败
        SCHED_LOOP_OVERRUN        = 0x0D02, ///< 控制环超预算(loopMax > 告警线)
        SCHED_STACK_LOW           = 0x0D03, ///< 线程栈水位告警
        SCHED_TICK_LOST           = 0x0D04, ///< 调度节拍丢失

        // ============================ 系统 / 初始化 / 复位 ============================
        SYS_NVIC_FAILED    = 0x0E01, ///< NVIC 优先级分组设置失败
        SYS_DWT_FAILED     = 0x0E02, ///< DWT 周期计数器不可用(负载统计全废)
        SYS_CONSOLE_FAILED = 0x0E03, ///< 调试串口初始化失败
        SYS_LED_FAILED     = 0x0E04, ///< 指示灯初始化失败
        SYS_WDT_FAILED     = 0x0E05, ///< 看门狗初始化失败
        SYS_RESET_CAUSE    = 0x0E06, ///< 本次不是上电复位(提示性, 附带上一次复位原因)
        SYS_FAULT_TRAP     = 0x0E07, ///< 上次运行落进了 HardFault 等故障处理器(供下次上电上报)
        SYS_HEAP_LOW       = 0x0E08, ///< 堆剩余不足

        // ============================ 异常机制自身 (0x0F) ============================
        EXCEPTION_OVERFLOW = 0x0F01, ///< 记录器满了: 后面的异常只保留"溢出过"这一个事实
        EXCEPTION_NO_SINK  = 0x0F02, ///< 抛异常时**一个出口都没挂上**(初始化顺序错了, 见 app_exception_handler.hpp)
        EXCEPTION_UNKNOWN  = 0x0F04, ///< 驱动返回失败但没给错误码(0 码): 属于写错了, 必须可见
    };

    enum class FunctionResult : bool {
        SUCCESS = true,
        FAILED  = false
    };

} // namespace dlx
