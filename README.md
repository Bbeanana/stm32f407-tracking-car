# 寻迹避障小车（STM32F407）

这是在原 `工创f4小车` 工程基础上复制、裁剪并改写的新车主控固件。

新小车目前只做三件事：

1. **两路直流无刷电机行驶控制**：PWM + INA/INB + STBY，编码器速度闭环；
2. **车头八路数字循迹**：加权求偏差 + PD 转向；
3. **HC-SR04 超声波避障**：前方小于阈值时执行“刹车→后退→转出→直行绕过→转回→重新找线”的绕行动作。

原工程里的 DT7 遥控、MaixCam 视觉、舵机、救援状态机等均已移除。

> 驱动板的 `ADC` 输出只用于串口电压遥测，不接也不影响行驶；
> 如果要接，先确认它的输出电压不超过 3.3V（PC0 不耐 5V，必要时分压）。

---

## 1. 代码结构

```text
Core/
├── Inc/
│   ├── board_config.h        ← 硬件引脚表（先看这个）
│   ├── tim.h / gpio.h / usart.h / adc.h
│   └── stm32f4xx_hal_conf.h
└── Src/
    ├── main.c                ← 初始化 + 5ms 主循环
    ├── tim.c                 ← TIM1 PWM / TIM3/4 编码器 / TIM5 捕获
    ├── gpio.c                ← 电机方向、STBY、循迹输入、TRIG
    ├── usart.c               ← USART2 调试串口
    ├── adc.c                 ← 驱动板 ADC 输出（裸寄存器）
    └── stm32f4xx_it.c        ← TIM5 中断

Users/
├── application/
│   ├── car_config.h          ← 所有可调参数（速度、PID、阈值、极性）
│   ├── car_control.c/h       ← 循迹/避障状态机
│   ├── motion.c/h            ← 双电机差速 + 编码器速度环
│   └── telemetry.c/h         ← 串口遥测
├── device/
│   ├── bldc_motor.c/h        ← 无刷驱动板 PWM+INA/INB+STBY
│   ├── encoder.c/h           ← 正交编码器
│   ├── track_sensor.c/h      ← 八路循迹
│   ├── ultrasonic.c/h        ← HC-SR04
│   └── current_sense.c/h     ← 驱动板 ADC 电压
├── bsp/
│   ├── bsp_delay.c/h         ← DWT 微秒延时
│   └── bsp_uart.c/h          ← 调试串口发送/printf 重定向
└── alg/
    └── alg_pid.c/h           ← PID
```

硬件接线见 [`接线说明.md`](接线说明.md)。

---

## 2. 编译与烧录

### 编译

```bash
cd ~/Desktop/工创f4_寻迹避障小车
./build.sh
```

产物：

```text
build/tracking_car.elf
build/tracking_car.hex
build/tracking_car.bin
```

也可以使用 CMake：

```bash
cmake -S . -B build_cmake -DCMAKE_BUILD_TYPE=Debug
cmake --build build_cmake -j
```

### 烧录（ST-Link + OpenOCD）

```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c "program build/tracking_car.elf verify reset exit"
```

没有 NRST 复位线时：

```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c "reset_config none" \
  -c "program build/tracking_car.elf verify reset exit"
```

---

## 3. 第一次调试步骤（重要）

所有参数都在 `Users/application/car_config.h`。

> 调试模式 1/2 下不跑正常状态机，遥测里的 `S=` 会一直显示 `INIT`，属于正常现象。

### 第 1 步：电机与编码器方向（车轮必须架空）

```c
#define CAR_BRINGUP_MODE 1
```

烧录后，电机会按 **正转 2s → 停 2s → 反转 2s → 停 2s** 循环。此模式强制开环，
不会因为编码器方向接反而飞车。

打开串口助手（115200），观察：

- 左轮“正转”时 `V=左速度/右速度` 中左边应为正；如果为负，改：
  ```c
  #define CAR_ENCODER_L_INVERT 1
  ```
- 右轮同理，改 `CAR_ENCODER_R_INVERT`。
- 如果某个轮子的“正转”实际是后退，改：
  ```c
  #define CAR_MOTOR_L_INVERT 1   // 或 CAR_MOTOR_R_INVERT
  ```
- 如果左右轮转向相反，先确认电机相线/驱动板 A/B 路是否接反，再改软件。

> 注意：`CAR_BRINGUP_MODE = 1` 时 `Motion` 使用开环，目的是先确认方向和编码器
> 读数，不要在这个模式下直接下地跑。

### 第 2 步：八路循迹极性/顺序/阈值

当前代码按 **8 路数字输出** 读取（PD2~PD9）。如果你用的是数字循迹模块，
黑白阈值由模块上的**电位器/学习按键**决定，软件只能读“高/低”，不能直接改阈值。

#### 2.1 用校准固件看每一路状态

```bash
cd ~/Desktop/工创f4_寻迹避障小车
CAR_BRINGUP_MODE=3 ./build.sh
```

烧录生成的 `build/tracking_car_mode3.elf`。此模式电机会保持停止，串口每 100ms 输出：

```text
CAL RAW=18 ACT=18 |...##...| E=+0.00 D=999.0 U=0
```

| 字段 | 含义 |
|---|---|
| `RAW` | PD2~PD9 的原始电平，bit0=PD2（最左） |
| `ACT` | 按 `TRACK_ACTIVE_LEVEL` 转换后的“压线”掩码 |
| `\|...##...\|` | 8 路可视化，`#`=压线，`.`=未压线 |
| `E` | 当前偏差（-1 最左，+1 最右） |

把车放在**白色地板**上，逐路调模块电位器，目标是 8 路都显示 `.`；
再把某个探头放到**黑线**上，对应位应变成 `#`。如果黑白完全反了，改：

```c
#define TRACK_ACTIVE_LEVEL 1   // 或从 1 改回 0
```

如果最左探头对应的是最右边的显示位，改：

```c
#define TRACK_REVERSE_ORDER 1
```

#### 2.2 如果是模拟量模块

如果你说的“标注黑线和白色地板的阈值”是指**软件里记录黑白 ADC 值再算阈值**，
那当前接线（PD2~PD9 数字输入）做不到，需要把模块的 8 路模拟输出改接到 ADC 引脚，
并改代码。告诉我模块型号/照片，我再给你改模拟量版本。

#### 2.3 调好后

把 `CAR_BRINGUP_MODE` 改回 `0`，重新编译正常固件：

```bash
./build.sh
```

正常固件里 `M=` 是滤波后的压线掩码，`E=` 是加权偏差；直线时 `E` 应接近 0，
线偏右时 `E` 为正，偏左时为负。

### 第 3 步：标定编码器一圈计数

`CAR_ENCODER_CPR` 默认 1560，只是占位值。标定方法：

1. 保持 `CAR_BRINGUP_MODE = 2`（电机不动）；
2. 记下遥测里的 `C=左计数/右计数`；
3. 用手把左轮**正好转一圈**，再读一次左计数；
4. 差值就是左轮输出轴一圈计数，填到 `CAR_ENCODER_CPR`；
5. 右轮同样操作。若左右编码器线数/减速比不同，需要分别标定（当前代码左右共用一个 CPR，
   如果差异较大，告诉我再改成左右独立参数）。

### 第 4 步：正常循迹

```c
#define CAR_BRINGUP_MODE 0
```

先把车放在直线中间，手扶车壳，确认：

- 车能缓慢前进；
- 线偏右时左轮加速、右轮减速（车向右修正）；
- 线偏左时相反；
- 前方放障碍物，距离小于 `CAR_OBSTACLE_STOP_CM` 时车会先刹车，然后按
  `刹车 → 后退 → 右转出 → 直行绕过 → 左转回 → 低速找线` 的顺序绕过去。

---

## 4. 主要参数说明

### 循迹

| 参数 | 默认 | 说明 |
|---|---:|---|
| `CAR_LINE_BASE_SPEED_MPS` | 0.22 | 直线基础速度 |
| `CAR_LINE_MIN_SPEED_MPS` | 0.05 | 大偏差时最低基础速度 |
| `CAR_LINE_MAX_SPEED_MPS` | 0.45 | 单轮最大速度 |
| `CAR_LINE_KP` | 0.35 | 偏差比例系数，越大修正越猛 |
| `CAR_LINE_KD` | 0.05 | 偏差微分，抑制左右摆动 |
| `CAR_LINE_TURN_LIMIT_MPS` | 0.25 | 差速转向限幅 |
| `CAR_LINE_ERROR_FILTER` | 0.35 | 偏差低通，越小越平滑但越滞后 |
| `CAR_LINE_SLOWDOWN_GAIN` | 0.50 | 偏差大时降速比例 |
| `CAR_LOST_STOP_MS` | 120 | 丢线多久后进入找线 |
| `CAR_SEARCH_SPEED_MPS` | 0.10 | 原地找线速度 |
| `CAR_SEARCH_TIMEOUT_MS` | 1500 | 找线超时后停车 |

控制律：

```text
error = 八路加权平均位置（-1 最左，+1 最右）
turn  = KP * error + KD * (error - last_error)
base  = 基础速度 * (1 - 降速比例 * |error|)
左轮  = base + turn
右轮  = base - turn
```

### 超声波

| 参数 | 默认 | 说明 |
|---|---:|---|
| `CAR_OBSTACLE_STOP_CM` | 20 | 小于该距离触发绕行 |
| `CAR_OBSTACLE_ENABLE` | 1 | 0=关闭超声波避障 |

绕行参数（`car_config.h`）：

| 参数 | 默认 | 说明 |
|---|---:|---|
| `CAR_AVOID_DIR` | +1 | +1=右侧绕行，-1=左侧绕行 |
| `CAR_AVOID_BRAKE_MS` | 250 | 触发后先刹车多久 |
| `CAR_AVOID_BACK_SPEED_MPS` | 0.10 | 后退速度 |
| `CAR_AVOID_BACK_MS` | 200 | 最短后退时间 |
| `CAR_AVOID_BACK_CLEAR_CM` | 20 | 后退到该距离再转向 |
| `CAR_AVOID_BACK_MAX_MS` | 1000 | 后退最长时间 |
| `CAR_AVOID_TURN_SPEED_MPS` | 0.18 | 原地转出/转回速度 |
| `CAR_AVOID_TURN_MS` | 650 | 转出时间（约 70~90°，按实车调） |
| `CAR_AVOID_DRIVE_SPEED_MPS` | 0.15 | 绕行直行速度 |
| `CAR_AVOID_DRIVE_MS` | 1800 | 绕行直行时间（约 27cm） |
| `CAR_AVOID_DRIVE_MAX_MS` | 3000 | 还没绕开时的最长时间 |
| `CAR_AVOID_TURN_BACK_MS` | 650 | 转回原方向的时间 |
| `CAR_AVOID_CLEAR_CM` | 25 | 距离大于该值认为已绕开 |
| `CAR_AVOID_REACQUIRE_SPEED_MPS` | 0.10 | 绕过后低速找线速度 |
| `CAR_AVOID_REACQUIRE_TURN_MPS` | 0.03 | 找线时向原线方向回拐 |
| `CAR_AVOID_REACQUIRE_TIMEOUT_MS` | 3000 | 找不到线就进入原地找线 |
| `CAR_AVOID_MAX_ATTEMPTS` | 3 | 连续 3 次绕不过去就锁定停车 |

> 只有一个前向超声波，无法判断左右哪边更空，所以默认固定右绕。
> 如果赛道右侧空间不够，把 `CAR_AVOID_DIR` 改成 `-1` 左绕。
>
> 绕行角度/直行距离需要按实车尺寸和速度微调：转少了会蹭障碍物，
> 转多了会离原线太远。先调 `CAR_AVOID_TURN_MS`，再调 `CAR_AVOID_DRIVE_MS`。
>
> HC-SR04 在 4m 内没有障碍物时可能没有回波，代码按 999cm（前方空旷）处理。
> 串口 `U=0` 表示本次没有有效回波，可用来判断传感器是否真的在工作。

### 速度环

| 参数 | 默认 | 说明 |
|---|---:|---|
| `CAR_SPEED_CLOSED_LOOP` | 1 | 1=编码器闭环；0=开环占空比 |
| `CAR_SPEED_PID_KP` | 1.50 | 速度环 Kp |
| `CAR_SPEED_PID_KI` | 0.80 | 速度环 Ki |
| `CAR_SPEED_RAMP_MPS2` | 1.50 | 加减速斜坡，防止猛冲 |
| `CAR_MAX_WHEEL_SPEED_MPS` | 0.55 | 单轮最大速度 |
| `CAR_ENCODER_CPR` | 1560 | 输出轴一圈计数（必须标定） |
| `CAR_WHEEL_RADIUS_M` | 0.0325 | 车轮半径（65mm 轮径） |
| `CAR_TRACK_WIDTH_M` | 0.150 | 左右轮中心距，仅转向解算用 |

速度环整定建议：先把 `Ki=0`，从小到大加 `Kp`，直到速度能跟上且不持续振荡；
再加少量 `Ki` 消除稳态误差。如果电机驱动器内部已经有速度环，`Kp` 要适当减小。

### 安全检测

`Motion` 内置两种闭环保护：

| 故障码 | 含义 | 处理 |
|---|---|---|
| 1 | 目标为正但编码器持续为负 | 编码器 A/B 接反或方向配置错，停机 |
| 2 | 输出较大但编码器长时间无变化 | 编码器没接/电机堵转，停机 |

出现故障后串口 `F=1` 或 `F=2`，需要断电重启；修正接线/配置后再上电。

---

## 5. 状态机

```text
INIT ──1.2s──> LINE ──障碍<20cm──> AVOID ──绕完并重新看到线──> LINE
                 │
                 ├──丢线>120ms──> SEARCH ──找到线3次──> LINE
                 │                  └──1.5s未找到──> STOP
                 └──编码器故障──> FAULT

AVOID 内部：
  刹车 → 后退 → 向一侧转出 → 直行绕过 → 反向转回 → 低速找线
```

- `AVOID`：默认右绕，依次执行 `刹车 → 后退 → 右转出 → 直行 → 左转回 → 找线`。
  连续 3 次绕不过去会锁定停车（`CAR_AVOID_MAX_ATTEMPTS`）。
- `SEARCH`：按最后一次偏差方向原地慢转找线。
- `STOP`：停车；如果线重新出现可自动恢复（`CAR_AUTO_RESUME=1`）。
- `FAULT`：电机驱动板 STBY 拉低，必须断电重启。

---

## 6. 串口遥测

默认 200ms 一行，115200 8N1：

```text
S=LINE D=25.3 U=1 M=18 E=+0.12 T=0.22/0.18 V=0.21/0.18 C=1234/1220 A=0.02 F=0
```

| 字段 | 含义 |
|---|---|
| `S` | 状态：INIT/LINE/OBST/SRCH/STOP/FAULT |
| `D` | 超声波距离（cm） |
| `U` | 超声波本次读数是否有效（1=有效） |
| `M` | 八路循迹掩码（十六进制，bit0=PD2 最左） |
| `E` | 循迹偏差（-1~+1） |
| `T` | 左右轮目标速度（m/s） |
| `V` | 左右轮实际速度（m/s） |
| `C` | 左右编码器累计计数 |
| `A` | 驱动板 ADC 输出电压（V） |
| `F` | 故障码 |

比赛/正式运行时可以把 `CAR_TELEMETRY_ENABLE` 改为 0，避免串口打印占用时间。

---

## 7. 常见问题

| 现象 | 排查 |
|---|---|
| 上电不跑 | 看串口是否停在 `INIT`；检查 `CAR_START_DELAY_MS`、电机 STBY |
| 一给速度就报 `F=1` | 编码器方向反了，改 `CAR_ENCODER_*_INVERT` 或对调 A/B |
| 一给速度就报 `F=2` | 编码器没信号/没接，检查 VCC/GND/A/B |
| 车直线画龙 | 降低 `CAR_LINE_KP`，适当加 `CAR_LINE_KD`，或减小 `CAR_LINE_ERROR_FILTER` |
| 弯道冲出线 | 降低 `CAR_LINE_BASE_SPEED_MPS`，加大 `CAR_LINE_KP` |
| 循迹掩码全 0 | `TRACK_ACTIVE_LEVEL` 配反，或传感器没供电 |
| 循迹掩码全 1 | 同上；也可能是车不在线上 |
| 超声波一直 999 或不变 | ECHO 没接 PA0、模块没供电、TRIG 电平不够 |
| 超声波误停车 | 增大 `CAR_OBSTACLE_STOP_CM` 前先确认 `D` 稳定；检查供电和地面反射 |
| 转向修正方向反了 | 把 `TRACK_REVERSE_ORDER` 改 1，或对调电机 A/B 路 |

---

## 8. 目前没有做的部分

- 没有保留 DT7 遥控和 MaixCam 视觉；
- 超声波绕行是固定方向（默认右绕），没有左右避障传感器来判断哪边更空；
- 没有做多传感器融合/里程计；
- 驱动板 ADC 只做电压遥测，没有换算成电流（需要知道板子的采样电阻/增益）。

这些都可以在这个结构上继续加。先按上面的步骤把电机方向、编码器、循迹极性和
超声波距离调通，再动 PID 和速度参数。
