# sim — 用 JSBSim 验证飞控算法

把固件的**控制算法 + 滤波算法原封不动**编译到 PC 上, 和 JSBSim 的**六自由度刚体动力学**
闭环跑起来, 用来在上机之前验证参数、找问题。

```
                  JSBSim (aircraft/DLX450, 2000 Hz)
                  刚体动力学 + 4x 无刷电机/螺旋桨 + 起落架 + IMU/气压计
                        |                                  ^
      sensor/imu/*  sensor/baro/*                  fcs/dlx/motorN-nd
                        v                                  |
                 传感器误差模型 (BMI088 噪声/零偏/振动, BME280 高度噪声)   <-- 可关
                        |                                  |
                        v                                  |
              fc_sim.dll  (fc_sim.cpp, 直接 include 固件源码)
                  EKF(16 维) 或 Madgwick+垂速估计
                    -> FlightController(角度环 + 角速度环)
                    -> mixMotors -> kConfig.motorMap
```

## 目录

| 文件 | 作用 |
| --- | --- |
| `fc_sim.cpp` | 固件控制/滤波链路的 PC 封装 (C ABI), **不含任何重写的算法** |
| `fc_sim.dll` | 上面这个文件编出来的动态库 (由 `build_fc_sim.ps1` 生成) |
| `build_fc_sim.ps1` | 用 g++ 编 DLL |
| `dlx_conf.py` | 固件参数副本 + 坐标系转换工具 (JSBSim NED  <->  固件 ENU/FLU) |
| `plant_check.py` | **飞机模型自检**: IMU 符号、电机布局/旋向/顺序、悬停油门、姿态换算 |
| `calibrate_from_flight_log.py` | 从实飞日志反推**悬停油门** (用来定标动力配置) |
| `fit_power_train.py` | 扫 桨直径 / KV / 电池节数, 算出悬停油门和满油门推重比 |
| `jsbsim_drone_sim.py` | 闭环仿真主程序 (场景、传感器误差、日志、画图) |
| `compare_filters.py` | **三路滤波对比**: EKF(固件默认) / EKF(优化参数) / Madgwick, 同一杆量同一噪声; `--case takeoff\|attitude` |
| `cg_trim_test.py` | **起飞 x 重心偏前** 扫描: 三套参数各能容忍多少重心偏移 |
| `sweep_rate_i.ps1` | **并发**跑整套场景扫描 `RATE_*_I` 倍数 (基线 + 多个 I 值同时跑) |
| `run_sim.ps1` | 一条命令跑完 `build -> plant_check -> 全部场景` |
| `out/` | 结果: 每个场景一份 `*.csv` (100 Hz) + `*.png` + `summary.txt` |
| `REPORT.md` | 仿真结论与建议 (含发现的三个问题) |

JSBSim 侧的飞机模型在 `E:\JSBSim\aircraft\DLX450` (由 F450 拷贝改造),
电机/螺旋桨定义在 `E:\JSBSim\engine\DLX_MOTOR.xml` / `DLX_PROP.xml` (6S + 10 in 桨 + 960 KV, 悬停 0.262)。

## 快速开始

```powershell
# 1. 编译 PC 端控制器 (需要 g++; 没有就在 DLX_GXX 里指路径)
powershell -ExecutionPolicy Bypass -File .\sim\build_fc_sim.ps1

# 2. 飞机模型自检 (会算出真实悬停油门)
python sim\plant_check.py

# 3. 全部场景闭环仿真 (含平地起飞)
python sim\jsbsim_drone_sim.py
python sim\jsbsim_drone_sim.py -s attitude height     # 只跑指定场景
python sim\jsbsim_drone_sim.py --clean                 # 关掉传感器噪声
python sim\cg_trim_test.py                            # 起飞能容忍多少重心偏前
powershell -ExecutionPolicy Bypass -File .\sim\sweep_rate_i.ps1        # 并发扫 RATE_*_I = 3/4/5/6
python sim\compare_filters.py --case takeoff --cg-mm 0 # 理想重心下的三套对比
```

也可以一条命令: `powershell -ExecutionPolicy Bypass -File .\sim\run_sim.ps1`

依赖: Python 3.10 + `jsbsim` + `numpy` + `matplotlib`
(`python -m pip install jsbsim numpy matplotlib`), 以及 MinGW-w64 的 `g++`。

## JSBSim 飞机模型 DLX450

### 参数来源 (唯一真值仍是固件)

| JSBSim 文件 | 字段 | 固件来源 |
| --- | --- | --- |
| `Mass.xml` | `emptywt` 1.566 kg | `MASS_KG` |
| `Mass.xml` | Ixx/Iyy/Izz 0.0234/0.0124/0.0338 | `INERTIA_XX/YY/ZZ` |
| `Propulsion.xml` | 电机位置 ±0.12933 / ±0.18106 m | `ARM_FORWARD_M` / `ARM_LATERAL_M` |
| `Propulsion.xml` | 旋向 M0 CW, M1 CCW, M2 CW, M3 CCW | `kMotorSpinDirection` |
| `Propulsion.xml` | engine 下标 = 物理 DShot 通道, 顺序按 `motorMap{0,2,3,1}` 摆放 | `kConfig.motorMap` |
| `DLX_MOTOR.xml` | 960 KV / 0.117 Ω / 0.45 A / **22.2 V (6S)** | 见下面"动力配置标定" |
| `DLX_PROP.xml` | **10 in** 双叶桨 Ct/Cp 表 | 悬停油门 0.262 对上实飞日志 |
| `SensorImu.xml` | 加速度计/陀螺/磁力计, 装在重心 | "配上 IMU" |
| `SensorBaro.xml` | 静压/温度 | BME280 定高用 |

### 两个坐标系约定 (改模型时最容易错的地方)

1. **机体轴**: 固件是 `x 前 / y 左 / z 上`, JSBSim 是 `x 前 / y 右 / z 下`,
   两者差一个绕 x 轴 180° 旋转 `S = diag(1,-1,-1)`。
   - 电机位置: `x_xml = -x_dlx, y_xml = -y_dlx`
   - 姿态: `R_dlx = S * R_jsb * S`  (`dlx_conf.py: jsb_attitude_to_dlx_quat`)
   - IMU 装在机上就是 `<roll>180</roll>`, 于是一上电水平静止时 `accelZ = +9.81`,
     和固件 Madgwick/EKF 的预期完全一致。
2. **JSBSim 的 `<location>` 用的是 "结构系": x 向**后**为正, y 向右, z 向**上**为正**
   (见 `include/models/propulsion/FGForce.h`)。所以 XML 里 x 要取负,
   `plant_check.py` 用"单电机出力 -> 力矩方向"把这条约定钉死了。

### 电机顺序

固件逻辑编号 `M0..M3 = 左前/右前/右后/左后`, `motorMap{0,2,3,1}` 把它映射到物理
DShot 通道。模型里 **engine 下标 = 物理 DShot 通道**, 通道上的位置按 motorMap 反推摆放,
所以仿真里跑的是 `mixMotors -> motorMap -> 物理通道` 的完整链路 (不是一步抄近路)。
`plant_check.py` 第 [3] 组会验证四个电机的位置和 +roll/+pitch/+yaw 三种力矩方向。

### 动力配置 (悬停油门) 是怎么定的

"多少油门能悬停" 完全由 **KV x 电池电压 x 桨直径** 决定, 而这是仿真里最容易搞错的一项
(第一版按 4S 建模, 悬停要 43.7% 油门, 和实飞完全对不上)。

现在这一版是这么定的:

1. `python sim\calibrate_from_flight_log.py` 从 `E:\kmpFly\desktopApp\flight_logs` 里量出
   真实悬停集合油门 ≈ **0.16 ~ 0.30** (5035 个稳定悬停采样点中位 0.160; 低空有地效所以偏低);
2. `python sim\fit_power_train.py` 把桨直径 / KV / 电池节数扫一遍, 选"悬停油门对得上
   **而且**满油门推重比物理合理"的组合;
3. 结果是 **6S (22.2 V) + 10 in 桨 + 960 KV**: 悬停 **0.262**, 满油门推力 37.3 N (T/W 2.43)。

   > 4S 那一列 (9.4~12 in 桨) 都压不到 0.30 以内; 12 in 桨又装不进这台机架
   > (相邻电机中心距只有 259 mm, 10 in 桨只剩 4.7 mm 间隙)。 如果电池确实是 4S,
   > 那就说明**质量或者 KV** 与记录不符, 需要称重 / 测一次台架推力。

换动力只改 `E:\JSBSim\engine\DLX_MOTOR.xml` 和 `DLX_PROP.xml` (或直接
`fit_power_train.py --pick 10/960/6`), 然后重跑 `plant_check.py` 拿新的悬停油门 —— 
`jsbsim_drone_sim.py` 会自动读 `out/plant_hover.json`。

### 驾驶员模型

角度环模式 (`updateAngle`) 的油门是遥控给的, 仿真里用一个最简模型代替人手:

    throttle = 基准油门 - pilot_kv * vz      (vz 向上为正, 默认 pilot_kv = 0.05)

因为固定油门时 "下沉 -> 桨进气角变小 -> 推力下降 -> 继续下沉" 是发散的,
不管它一定会慢慢沉到地面; `pilot_kv = 0` 就是纯固定油门 (最原始的行为)。

### 平地起飞与重心偏移 (重要)

`takeoff` 场景默认带 **5 mm 重心沿机体 x 偏前** (等价于 716 g 电池装偏约 1 cm)。
**把 `cg_mm` 设成 0 会得到过于乐观的结果** —— 第一版就是因为重心建在几何正中、
四个电机完全一致, 所以看起来"起飞完美", 和实飞不符。

仿真结论 (见 REPORT.md §2~§4):

- 重心偏前 4 mm 就明显前倾, **8 mm 直接翻掉**;
- 两个根因: (a) 姿态估计被加速度计拉平, 飞机前倾 45° 时估计只偏 6°, 角度环不下发回正指令;
  (b) 角速度环积分能抵消的重心偏移上限只有 `Iyy*RATE_PITCH_I*RATE_INT_LIMIT/(m*g)` = **8.1 mm**,
  而且要 ~8 s 才攒得出来, 飞机 3 s 就翻了;
- `RATE_ROLL_I / RATE_PITCH_I` 从 2 提到 6, 8 mm 工况从"翻掉"变成"末态 11.8°"。

相关开关: `--cg-mm <mm>` (正=偏前), `--int-limit <rad/s^2>`, `--rate-i-scale <倍数>`,
`--tilt-angle-gate <deg>` (倾角门控, **实测无效**), `--tilt-air-weak 1` (地面上强/解锁后弱)。

**不需要量重心的修正手段** (REPORT.md 附录 A.5 有九种组合的实测对比):
采用 `main_att_ekf.cpp` 那套加速度计约束 (gate .20 / sigma .08 / inno .15 / freeze)
+ **打开准静止判定** (hold 50ms / 0.15 / 150dps) + `RATE_ROLL_I = RATE_PITCH_I` ×3~5 (即 6~10)。

> `RATE_*_I` 不是越大越稳 (REPORT.md 附录 C): 抗常值扰动在 ×10 附近最优, 再大反弹;
> 动态超调/roll 稳态误差在整个区间里随 I 增大而略微变差。 而且仿真没有 ESC/IMU 延迟,
> 真机要比仿真保守。
效果: 8 mm 重心偏前 从「翻掉」变成「峰值 8.1° 能飞」, 空中持续倾角 47°->10.8°,
电机不平衡 30°->6.5°, 而定高/悬停几乎不变差 (悬停 0.57°->0.88°)。
仿真开关: `--freeze-bias 1 --accel-hold 1 --rate-i-scale 6` + `--tilt-sigma .08 --tilt-gate .20 --tilt-inno-limit .15`。

## PC 端控制器 fc_sim.dll

`fc_sim.cpp` 直接 `#include` 这些固件头文件, 不重写算法:

- `DL_LIB/flight_control/flight_control_fliter.hpp` — 16 维 EKF (`release.cpp` 在用的那套)
- `DL_LIB/DL_AHRS/MadgwickAHRS.hpp` — 备选姿态融合
- `DL_LIB/flight_control/vertical_velocity_estimator.hpp` — Madgwick 路线的定高
- `DL_LIB/flight_control/flight_controller.hpp` — 角度环 + 角速度环
- `DL_LIB/flight_control/flight_control_mixer.hpp` — 混控
- 参数: `flight_control_params.hpp` / `flight_config_struct.hpp`

时序与 `release.cpp` 一致:

| 环节 | 频率 | 说明 |
| --- | --- | --- |
| IMU 快路径 | 2000 Hz | `ekf.integrateNominal()` (或 Madgwick) |
| EKF 慢路径 | 250 Hz | `propagateCovariance()` + `updateAccelerometer()` (ekfDecim = 8) |
| 气压计 | 50 Hz | `updateBarometer()` (baroPeriodMs = 20) |
| 控制环 | 500 Hz | `updateAngle()` / `updateAngleHeight()` + `mixMotors()` + `motorMap` |

解锁前滤波器照跑、电机输出 0; 解锁后才进控制环 —— 和固件 `en && !stop` 一致。

### C ABI 摘要

```c
void fc_sim_step(const FcSimInput *in, FcSimOutput *out);
```

`FcSimInput`: 陀螺/加速度 (固件轴系, rad/s & m/s^2), 气压高度, 期望姿态/高度,
遥控油门, `height_mode`, `armed`, `reset`, `filter_mode` (0=EKF / 1=Madgwick),
`hover_throttle` 覆盖, 以及 `tilt_sigma/tilt_gate/tilt_inno_limit/freeze_gyro_bias`
四个 EKF 倾角修正参数覆盖 (用来做 A/B 对比)。

`FcSimOutput`: 物理通道油门 `motor_phys[4]`、逻辑电机 `motor_logical[4]`、估计姿态四元数、
估计高度/垂速、期望力矩/角速度、倾转误差角、混控收缩系数、陀螺零偏估计。

## 场景

| 场景 | 模式 | 内容 |
| --- | --- | --- |
| `hover` | 角度环 | 悬停 12 s, 看姿态保持和滤波精度 |
| `takeoff` | 角度环 | **平地起飞**: 地面停机 -> 解锁 -> 推油门离地 -> 起飞后 -6° 滚转修正再回平 |
| `attitude` | 角度环 | roll -10° / pitch +10° 阶跃 (固件默认 EKF 参数) |
| `attitude_tuned` | 角度环 | 同一段指令, 换 `notebook/2026-09-15_ekf_tilt_fix.md` 给的那组参数 |
| `attitude_weak` | 角度环 | 同一段指令, 把倾角修压到很弱 (sigma .5 / gate .03 / inno .05) |
| `gust` | 角度环 | 悬停中在机体 x 加 2.5 N 外力 0.4 s (`external_reactions`) |
| `imbalance` | 角度环 | 物理通道 3 号位置加 +3% 油门偏置 (模拟电调/桨不平衡) |
| `height` | 定高 | 2.0 m 保持 -> 3.5 m -> 2.5 m (悬停油门用实测值) |
| `height_fw_hover` | 定高 | 同上, 但悬停油门保持固件原值 `HOVER_THROTTLE = 0.25` (比实测低 13%) |
| `madgwick` | 角度环 | 换成 Madgwick + 垂速估计的滤波链路对比 |

传感器误差模型 (默认开, `--clean` 关): BMI088 陀螺白噪声 0.006 rad/s + 零偏 0.005 rad/s,
加速度计白噪声 0.045 m/s² + 零偏 ~0.02 m/s², 与转速相关的机体振动, BME280 高度噪声 σ=0.5 m。

## 常见改动

- **改质量/臂长/转动惯量**: 改 `flight_control_params.hpp` 的默认值, 同时改
  `E:\JSBSim\aircraft\DLX450\Mass.xml` / `Propulsion.xml` / `Gear.xml` (三者要一致)。
- **改电机/桨/电池**: `python sim\fit_power_train.py --pick 桨/KV/节数`, 再重跑 `plant_check.py`
  拿新的悬停油门 (`out/plant_hover.json`), 仿真脚本会自动读这个值。
- **改电机顺序/旋向**: 改 `Propulsion.xml` 的位置和 `<sense>`, 以及 `dlx_conf.py` 的
  `MOTOR_MAP`, 重跑 `plant_check.py` 验证。
- **换 EKF 参数**: 命令行 `--tilt-sigma / --tilt-gate / --tilt-inno-limit`, 或在场景表里加
  `ekf=dict(...)`。

## 已知边界

- JSBSim 的 `MSVC` 静态库不能和 MinGW `g++` 链接, 所以这里用 Python 绑定
  (`pip install jsbsim`) 驱动动力学, 控制算法编成 DLL 由 `ctypes` 调用。
- 气动只有 CD=1.0 的阻力, 没有旋翼下洗/地效/桨-桨干扰, 所以低速悬停的保真度
  主要在"推力-转速-姿态"这条主链上。
- 角度环模式 (`updateAngle`) 不做高度控制, 遥控油门固定时高度会缓慢下沉 ——
  这是固件本身的行为 (真实飞行靠遥控手补), 不是仿真误差。
- 动力用 6S + 9.4 in 桨 + 960 KV 标定, 悬停 0.288。 实际电机/桨/电池不同就重跑
  `fit_power_train.py`, 飞机本体不用动。
