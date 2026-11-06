# 寻迹避障小车（STM32F407）

这是在原 `工创f4小车` 工程基础上复制、裁剪并改写的新车主控固件。

新小车目前做五件事：

1. **两路直流无刷电机行驶控制**：PWM + INA/INB + STBY，编码器速度闭环；
2. **摄像头 UART 循迹**：接收 ERR/ANG，用 ERR 做 PD 转向；车头八路只用于遥测/调试；
3. **HC-SR04 超声波避障**：前方小于阈值时立即刹车停车，不再自动绕行；障碍物移开并稳定后，可按配置恢复循迹；
4. **蜂鸣器提示**：PB0 低电平触发，上电短鸣一声；圆弧弯道进入/退出各短响一声，直角弯道不提示；
5. **128x64 I2C OLED（SSD1306）**：三行同时显示第一圈、第二圈耗时和障碍物次数。

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
│   ├── lap_timer.c/h         ← 圈计时、障碍物计数
│   ├── oled_display.c/h      ← 128x64 OLED 三行显示
│   └── telemetry.c/h         ← 串口遥测
├── device/
│   ├── bldc_motor.c/h        ← 无刷驱动板 PWM+INA/INB+STBY
│   ├── encoder.c/h           ← 正交编码器
│   ├── camera.c/h            ← 摄像头 UART 帧解析（ERR/ANG）
│   ├── track_sensor.c/h      ← 八路数字传感器（仅遥测/调试）
│   ├── ultrasonic.c/h        ← HC-SR04
│   ├── buzzer.c/h            ← 蜂鸣器（PB0，低电平触发）
│   ├── ssd1306.c/h           ← 128x64 OLED 底层驱动
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

### 第 2 步：摄像头 UART 与车头八路传感器

#### 2.0 摄像头循迹（当前转向来源）

摄像头接 **USART1**：

```text
摄像头 VCC → 5V
摄像头 GND → GND
摄像头 TX  → PA10（USART1_RX）
摄像头 RX  → PA9 （USART1_TX，可选）
```

- 波特率 **115200 8N1**
- 帧格式：`0xA5 | ERR(float32 小端) | ANG(float32 小端) | CURV(float32 小端) | 0x5A`，共 14 字节
- 固件使用 `ERR / 1000` 作为横向偏差；`ANG` 已解析保存，`CURV` 用于直角弯计数，二者都不参与转向
- `ERR` 正=线在画面右侧→右转
- 超过 `CAR_CAMERA_TIMEOUT_MS`（默认 200ms）没有有效帧就停车
- 当前转向不再使用车头八路传感器；八路仍保留给遥测/调试

> **注意**：摄像头模式下，前八路丢线不再触发原来的来回找线；
> 现在按直角弯处理，固定向右转约 90°。圈计时改用摄像头 `CURV` 的直角弯尖峰；
> 圆弧蜂鸣提示仍保留，使用摄像头 `CURV` 判断。

#### 2.1 车头八路数字传感器（仅遥测）

当前代码仍按 **8 路数字输出** 读取（PD2~PD9）。如果你用的是数字循迹模块，
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
- 前方放障碍物，距离小于 `CAR_OBSTACLE_STOP_CM` 时车立即刹车停车；
  移开障碍物、距离大于 `CAR_OBSTACLE_CLEAR_CM` 并保持
  `CAR_OBSTACLE_CLEAR_HOLD_MS` 后，如果车仍在线上会自动恢复循迹。

### 第 5 步：OLED 与圈计时

1. **上电前把车放在起点**（第二个右直角弯末端），再上电；
2. 上电约 1.2s 后进入 `LINE`，开始第一圈计时；
3. OLED 应同时显示三行：`L1 --.--`、`L2 --.--`、`OB 0`；
4. 每完成两个右直角弯，第一圈/第二圈时间会更新；
5. 如果显示镜像或倒置，改 `OLED_FLIP_X` / `OLED_FLIP_Y`。

> 圈数由摄像头 `CURV` 的直角弯尖峰计数。如果某个直角弯的 `CURV` 没有超过
> `CAR_CAM_CORNER_ENTER_DEG`，就不会计数，需要根据 HUD/遥测调低阈值。

---

## 4. 主要参数说明

### 循迹

| 参数 | 默认 | 说明 |
|---|---:|---|
| `CAR_LINE_BASE_SPEED_MPS` | 0.20 | 直线基础速度（降速） |
| `CAR_LINE_MIN_SPEED_MPS` | 0.07 | 大偏差时最低基础速度 |
| `CAR_LINE_MAX_SPEED_MPS` | 0.50 | 单轮最大速度 |
| `CAR_LINE_KP` | 0.24 | 偏差比例系数，兼顾转向和稳定 |
| `CAR_LINE_KD` | 0.10 | 偏差微分，抑制左右摆动 |
| `CAR_LINE_TURN_LIMIT_MPS` | 0.28 | 差速转向限幅 |
| `CAR_LINE_ERROR_FILTER` | 0.25 | 偏差低通，越小越平滑但越滞后 |
| `CAR_LINE_ERROR_DEADBAND` | 0.08 | 中心死区，抑制小幅抖动 |
| `CAR_LINE_SLOWDOWN_GAIN` | 0.50 | 偏差大时降速比例 |
| `TRACK_LPF_ALPHA` | 0.15 | 每路 0/1 的一阶低通系数，越小越平滑但越滞后 |
| `CAR_RIGHT_TURNS_ONLY` | 0 | 旧版找线配置；当前不使用 |
| `CAR_START_REQUIRE_LINE` | 1 | 1=上电后从未见过线时先停车，不原地搜索 |
| `CAR_CAMERA_TIMEOUT_MS` | 200 | 摄像头超过该时间没有有效帧就停车 |

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
| `CAR_OBSTACLE_ENABLE` | 1 | 0=关闭超声波避障 |
| `CAR_OBSTACLE_BEEP_MS` | 200 | 检测到障碍物时蜂鸣时间 |
| `CAR_OBSTACLE_STOP_CM` | 10 | 小于该距离立即停车 |
| `CAR_OBSTACLE_CLEAR_CM` | 15 | 大于该距离才允许恢复，形成回差 |
| `CAR_OBSTACLE_CLEAR_HOLD_MS` | 300 | 超过清除距离后需持续的时间 |
| `CAR_OBSTACLE_STOP_LATCH` | 0 | 1=锁存停车，必须复位；0=障碍物移开后自动恢复 |

行为说明：

- 障碍物进入 `CAR_OBSTACLE_STOP_CM`（默认 10cm）以内：立即刹车，状态切到 `STOP`，不后退、不转向。
- 障碍物移开但距离仍小于 `CAR_OBSTACLE_CLEAR_CM`（默认 15cm）：继续停车，防止在阈值附近反复启停。
- 距离大于 `CAR_OBSTACLE_CLEAR_CM` 并保持 `CAR_OBSTACLE_CLEAR_HOLD_MS` 后：如果车仍在线上，自动恢复循迹。
- 如果需要“遇到障碍物就永久停车”，把 `CAR_OBSTACLE_STOP_LATCH` 改为 `1`，之后必须复位/断电才能继续。

> HC-SR04 在 4m 内没有障碍物时可能没有回波，代码按 999cm（前方空旷）处理。
> 串口 `U=0` 表示本次没有有效回波，可用来判断传感器是否真的在工作。

### 蜂鸣器

| 参数 | 默认 | 说明 |
|---|---|---|
| `BUZZER_PORT` / `BUZZER_PIN` | `GPIOB` / `GPIO_PIN_0` | 接 PB0；当前固件没有其他功能占用 PB0 |
| `BUZZER_ACTIVE_LEVEL` | 0 | 0=低电平触发（本车）；1=高电平触发 |
| 上电提示音 | 120ms | `main.c` 里调用 `Buzzer_Beep(120U)`，用于确认接线和极性 |

常用接口（`Users/device/buzzer.h`）：

```c
Buzzer_Init();          // 初始化，默认关闭
Buzzer_On();            // 一直响
Buzzer_Off();           // 关闭
Buzzer_Beep(100U);      // 非阻塞：响 100ms 后自动关闭
Buzzer_Update();        // 主循环周期调用，处理自动关闭
```

> `Buzzer_Beep()` 不阻塞，但必须让 `Buzzer_Update()` 周期运行；主循环 5ms
> 已经调用。不要用 `HAL_Delay()` 做蜂鸣延时，否则会拖慢电机控制。

### 圆弧弯道蜂鸣提示

只对**圆弧弯道**提示，直角弯道不提示。进入圆弧和退出圆弧各短响一声（默认 80ms）；如果一圈只有一个圆弧，一圈共 2 声。

判断逻辑在 `car_control.c`，使用摄像头的 `CURV`：

- `|CURV|` 持续大于 `CAR_CAM_ARC_ENTER_DEG` 达到
  `CAR_CAM_ARC_ENTER_MS` → 进入圆弧，响一声
- `CURV` 经过历史移动平均后，持续小于出弯阈值达到
  `CAR_CAM_ARC_EXIT_MS` → 退出圆弧，响一声
- 出弯阈值至少取 `CAR_CAM_ARC_EXIT_DEG` 与入弯峰值的
  `CAR_CAM_ARC_EXIT_RATIO` 两者中的较大值，避免出弯时曲率没有完全归零而漏响
- `|CURV|` 大于 `CAR_CAM_ARC_MAX_DEG` → 按直角弯处理，复位并静默
  `CAR_ARC_SUPPRESS_MS`，不产生圆弧提示
- 每一圈只允许一段圆弧提示：入弯一声、出弯一声；圆弧中间 `CURV` 抖动
  或出弯后到起点的直角弯不再重复响
- 每次状态切换后会静默 `CAR_ARC_SUPPRESS_MS`，避免起步和找线时误响

| 参数 | 默认 | 说明 |
|---|---:|---|
| `CAR_ARC_BEEP_ENABLE` | 1 | 0=关闭圆弧提示 |
| `CAR_ARC_BEEP_MS` | 80 | 每次提示鸣叫时间 |
| `CAR_ARC_BEEP_MIN_GAP_MS` | 3000 | 任意两声圆弧提示的最小间隔 |
| `CAR_CAM_ARC_ENTER_DEG` | 8 | 进入圆弧的 CURV 阈值 |
| `CAR_CAM_ARC_EXIT_DEG` | 4 | 退出圆弧的 CURV 阈值 |
| `CAR_CAM_ARC_MAX_DEG` | 40 | 超过该值按直角弯处理 |
| `CAR_CAM_ARC_ENTER_MS` | 100 | 进入圆弧确认时间 |
| `CAR_CAM_ARC_EXIT_MS` | 200 | 退出圆弧确认时间 |
| `CAR_CAM_ARC_MIN_MS` | 400 | 入弯后至少这么久才允许出弯 |
| `CAR_CAM_ARC_CURV_FILTER_N` | 20 | CURV 历史移动平均窗口（约 100ms） |
| `CAR_CAM_ARC_EXIT_RATIO` | 0.40 | 出弯阈值相对入弯峰值的最小比例 |
| `CAR_ARC_SUPPRESS_MS` | 400 | 直角/状态切换后的静默时间 |

> 如果圆弧提示偏早/偏晚，先看调试遥测里的 `CURV` 峰值，再调
> `CAR_CAM_ARC_ENTER_DEG` / `CAR_CAM_ARC_EXIT_DEG` 和确认时间。
> 如果圆弧被误判成直角弯，降低 `CAR_CAM_ARC_MAX_DEG` 或提高
> `CAR_CAM_CORNER_ENTER_DEG`。

### 圈计时与 128x64 OLED

起点在第二个右直角弯末端。**上电前先把车放在起点**，上电后车轮真正
开始转动时开始第一圈计时；摄像头 `CURV` 出现直角弯尖峰时累计弯道，
每完成 `CAR_LAP_CORNERS`（默认 2）个右直角弯算一圈，记录前
`CAR_LAP_COUNT`（默认 2）圈。第二圈完成会立即刹车并锁存停车。

- `CURV >= CAR_CAM_CORNER_ENTER_DEG` 持续 `CAR_CAM_CORNER_ENTER_MS`
  → 进入直角弯
- `CURV <= CAR_CAM_CORNER_EXIT_DEG` 持续 `CAR_CAM_CORNER_EXIT_MS`
  → 记一个直角弯完成
- 圆弧通常 `CURV` 较小，不会触发；阈值和时间都可调

障碍物次数按“从正常循迹进入避障 `STOP`”的次数计数，同一个障碍物不会
在停车期间重复累加。

128x64 OLED 三行同时显示，不需要滚动：

```text
L1 12.34s    第一圈耗时（秒，两位小数）
L2 13.57s    第二圈耗时
OB 3         障碍物次数
```

- 接线：`VCC` 按模块要求（常见 3.3V/5V）、`GND=GND`、`SCL=PB10`、`SDA=PB11`
- 默认按 SSD1306、I2C 地址 `0x3C` 驱动；部分模块地址是 `0x3D`
- 显存 1KB，按 16 字节一块、5ms 一块刷新，不会长时间阻塞电机控制
- 显示方向反了改 `OLED_FLIP_X` / `OLED_FLIP_Y`
- 还没跑完的圈显示 `--.--`

| 参数 | 默认 | 说明 |
|---|---:|---|
| `CAR_LAP_CORNERS` | 2 | 每圈经过的右直角弯数量 |
| `CAR_LAP_COUNT` | 2 | 记录前几圈耗时，跑完立即停车 |
| `CAR_LAP_CORNER_DEBOUNCE_MS` | 300 | 同一个弯道重复触发时的防重复计数时间 |
| `CAR_LAP_START_SPEED_MPS` | 0.020 | 第一圈起计的实际轮速阈值 |
| `CAR_LAP_START_HOLD_MS` | 30 | 轮速超过阈值后的确认时间 |
| `CAR_CAM_CORNER_ENTER_DEG` | 45 | CURV 超过该角度认为进入直角弯 |
| `CAR_CAM_CORNER_EXIT_DEG` | 30 | CURV 低于该角度认为退出直角弯（需高于圆弧 CURV） |
| `CAR_CAM_CORNER_ENTER_MS` | 40 | 进入确认时间 |
| `CAR_CAM_CORNER_EXIT_MS` | 100 | 退出确认时间 |
| `OLED_I2C_ADDR` | 0x3C | OLED 的 7 位 I2C 地址 |
| `OLED_SH1106` | 0 | 1=SH1106（1.3 寸常见）；0=SSD1306 |
| `OLED_FLIP_X/Y` | 0 | 显示方向翻转 |

> 如果某个右直角弯的 CURV 峰值没有超过 45°，圈数会少计；看摄像头 HUD 的
> CURV 峰值，把 `CAR_CAM_CORNER_ENTER_DEG` 调到 峰值的一半左右。
> 如果圆弧被误判成直角弯，则把进入阈值调高。
>
> 如果屏幕是 SH1106（1.3 寸模块常见）而不是 SSD1306，把
> `OLED_SH1106` 改成 `1`，驱动会使用 2 像素列偏移并跳过 SSD1306 的寻址命令。

### 速度环

| 参数 | 默认 | 说明 |
|---|---:|---|
| `CAR_SPEED_CLOSED_LOOP` | 1 | 1=编码器闭环；0=开环占空比 |
| `CAR_SPEED_PID_KP` | 0.90 | 速度环 Kp（降低以减小超调） |
| `CAR_SPEED_PID_KI` | 0.25 | 速度环 Ki（降低以抑制积分饱和） |
| `CAR_SPEED_PID_MAX_INTEGRAL` | 0.40 | 积分限幅 |
| `CAR_SPEED_FF_GAIN` | 0.80 | 速度前馈，补偿低速死区/起步占空比 |
| `CAR_SPEED_RAMP_MPS2` | 1.70 | 加减速斜坡，防止起步猛冲 |
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
INIT ──1.2s──> LINE ──障碍<10cm──> STOP ──障碍移开>15cm 并稳定──> LINE
                 │
                 ├──摄像头无有效帧──> 制动
                 └──编码器故障──> FAULT
```

- `LINE`：正常循迹；检测到前方障碍物（`< CAR_OBSTACLE_STOP_CM`）立即刹车并进入 `STOP`。
- `STOP`：停车。若因障碍物触发，障碍物移开（`> CAR_OBSTACLE_CLEAR_CM`
  并保持 `CAR_OBSTACLE_CLEAR_HOLD_MS`）且线仍在，可自动恢复循迹；
  `CAR_OBSTACLE_STOP_LATCH=1` 时则一直停车，直到复位。
- 当前不再对前八路丢线做特殊处理，也不旋转找线；循迹仍只看摄像头
  `ERR`，摄像头无有效帧时直接制动。
- `Car_SearchLoop` 仅保留为空实现，正常流程不会进入。
- `FAULT`：电机驱动板 STBY 拉低，必须断电重启。

---

## 6. 串口屏 / 串口遥测

串口屏接 **USART2：PA2=TX，PA3=RX，115200 8N1**，与调试串口共用。

默认 `CAR_LAP_SCREEN_ENABLE=1`，每 200ms 只发送三行：

```text
L1=12.34
L2=--.--
OB=0
```

| 字段 | 含义 |
|---|---|
| `L1` | 第一圈耗时（秒），未完成显示 `--.--` |
| `L2` | 第二圈耗时（秒），未完成显示 `--.--` |
| `OB` | 障碍物触发次数，与 OLED 第三行一致 |

串口屏如果只能显示固定格式，直接按这三行做三个文本框即可。

需要调试遥测时，把 `CAR_LAP_SCREEN_ENABLE` 改成 `0`，恢复原来的单行调试输出：

```text
S=LINE D=25.3 U=1 M=18 E=+0.12 K=+26.2 T=0.22/0.18 V=0.21/0.18 C=1234/1220 A=0.02 F=0 L1=12.34 L2=--.-- LAP=3.45 N=2 OB=0
```

| 字段 | 含义 |
|---|---|
| `S` | 状态：INIT/LINE/SRCH/STOP/FAULT |
| `D` | 超声波距离（cm） |
| `U` | 超声波本次读数是否有效（1=有效） |
| `M` | 八路循迹掩码（十六进制，bit0=PD2 最左） |
| `E` | 循迹偏差（-1~+1） |
| `K` | 摄像头 CURV，单位度（圆弧约 8~35，直角弯 >45） |
| `T` | 左右轮目标速度（m/s） |
| `V` | 左右轮实际速度（m/s） |
| `C` | 左右编码器累计计数 |
| `A` | 驱动板 ADC 输出电压（V） |
| `F` | 故障码 |
| `L1` | 第一圈耗时（秒），未完成显示 `--.--` |
| `L2` | 第二圈耗时（秒），未完成显示 `--.--` |
| `LAP` | 当前圈已计时时间（秒） |
| `N` | 当前圈号；未发车为 0，完赛为 `CAR_LAP_COUNT` |
| `OB` | 障碍物触发次数，与 OLED 第三行一致 |

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
| 直角弯丢线后转不回来 | 当前版本不做丢线恢复；检查摄像头 `ERR/CURV` 和转向参数 |
| 循迹掩码全 0 | `TRACK_ACTIVE_LEVEL` 配反，或传感器没供电 |
| 循迹掩码全 1 | 同上；也可能是车不在线上 |
| 超声波一直 999 或不变 | ECHO 没接 PA0、模块没供电、TRIG 电平不够 |
| 超声波误停车 | 增大 `CAR_OBSTACLE_STOP_CM` 前先确认 `D` 稳定；检查供电和地面反射 |
| 转向修正方向反了 | 把 `TRACK_REVERSE_ORDER` 改 1，或对调电机 A/B 路 |
| OLED 不亮 | 检查 VCC/GND、SCL=PB10/SDA=PB11；确认模块地址是 0x3C 或改成 0x3D；I2C 需要上拉 |
| OLED 花屏/整体偏移 | 模块可能是 SH1106，把 `OLED_SH1106` 改成 1；或供电/上拉不稳 |
| OLED 文字镜像/倒置 | 改 `OLED_FLIP_X` 或 `OLED_FLIP_Y` |
| 圈数少计/多计 | 看摄像头 HUD 的 `CURV` 峰值；直角弯没超过 `CAR_CAM_CORNER_ENTER_DEG` 就调低，圆弧误计数就调高 |
| 障碍物次数不增加 | 确认 `CAR_OBSTACLE_ENABLE=1`，且 `S=STOP` 是由障碍物触发 |
| 蜂鸣器不响 | 确认 I/O 接 PB0、模块 VCC/GND 已接；用万用表看 PB0 上电后是否为高（不响）、鸣叫时是否为低 |
| 蜂鸣器上电一直响 | 模块实际是高电平触发，把 `BUZZER_ACTIVE_LEVEL` 改成 1 |

---

## 8. 目前没有做的部分

- 没有保留 DT7 遥控和 MaixCam 视觉；
- 超声波只做“检测到障碍物就停车”，不判断左右空间，也不会自动绕行；
- 圈计时依赖摄像头 `CURV` 识别右直角弯，没有独立的起终点横线/光电传感器；
- OLED 驱动默认按 SSD1306 编写，SH1106 通过 `OLED_SH1106=1` 做偏移适配；
- 没有做多传感器融合/里程计；
- 驱动板 ADC 只做电压遥测，没有换算成电流（需要知道板子的采样电阻/增益）。

这些都可以在这个结构上继续加。先按上面的步骤把电机方向、编码器、循迹极性和
超声波距离调通，再动 PID 和速度参数。
