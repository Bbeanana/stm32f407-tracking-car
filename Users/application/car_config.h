#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

/* =========================================================================
 * 小车参数配置
 *
 * 调试顺序建议：
 *   1) CAR_BRINGUP_MODE = 1：架空车轮，确认电机转向 + 编码器符号；
 *   2) CAR_BRINGUP_MODE = 2：把车放在线上，确认八路传感器极性/顺序；
 *   3) CAR_BRINGUP_MODE = 3：八路传感器校准（电机不动，100ms 打印原始/有效掩码）；
 *   4) CAR_BRINGUP_MODE = 0：正常循迹 + 超声波停车。
 * ========================================================================= */

/* 0=正常循迹；1=电机/编码器测试（车轮必须架空）；
 * 2=传感器测试（电机不动）；3=传感器校准（电机不动，打印更快） */
#ifndef CAR_BRINGUP_MODE
#define CAR_BRINGUP_MODE                 0
#endif

/* 控制周期与启动延时 */
#define CAR_CONTROL_PERIOD_S             0.005f
#define CAR_START_DELAY_MS               1200U

/* ------------------------------ 循迹参数 ------------------------------ */
/* 传感器：1=黑线输出高电平；0=黑线输出低电平（常见） */
#define TRACK_ACTIVE_LEVEL               0
/* 1=通道顺序与车头左右相反（例如模块装反了） */
#define TRACK_REVERSE_ORDER              0

#define CAR_LINE_BASE_SPEED_MPS          0.15f
#define CAR_LINE_MIN_SPEED_MPS           0.04f
#define CAR_LINE_MAX_SPEED_MPS           0.34f
#define CAR_LINE_KP                      0.35f
#define CAR_LINE_KD                      0.05f
#define CAR_LINE_TURN_LIMIT_MPS          0.18f
#define CAR_LINE_ERROR_FILTER            0.35f   /* 0~1，越小越平滑 */
#define CAR_LINE_SLOWDOWN_GAIN           0.50f   /* 偏差大时降速比例 */

/* 0=通用左右找线（按最后偏线方向，超时反向）；
 * 1=只有右转弯特化（固定向右，圆弧提示只认右偏）。 */
#define CAR_RIGHT_TURNS_ONLY             0

/* 1=上电后从未检测到线时不进入搜索，直接停车；
 * 避免车没放正/传感器没接线时一上电就原地右转。 */
#define CAR_START_REQUIRE_LINE           1

/* 丢线/直角弯找线：短暂保持上一次偏差，超过时间则原地找线。
 * 底盘是后轮差速 + 前万向轮，原地转向时绕后轴旋转，车头传感器扫过弯后的线。 */
#define CAR_LOST_STOP_MS                 120U
#define CAR_SEARCH_SPEED_MPS             0.10f
#define CAR_SEARCH_TIMEOUT_MS            3400U   /* 左右弯都有的双向找线时间 */
#define CAR_SEARCH_RIGHT_TIMEOUT_MS      2200U   /* 只有右转时单向找线时间 */
#define CAR_SEARCH_PHASE_MS              1200U   /* 双向模式第一段找线时间 */
#define CAR_SEARCH_HINT_MS               800U    /* 最后一次有效偏差的方向有效期 */
#define CAR_SEARCH_HINT_MIN_ERROR        0.10f   /* 记录转向提示的最小 |E| */
#define CAR_SEARCH_DEFAULT_DIR           (+1)    /* 没有方向提示时先向右转 */
#define CAR_AUTO_RESUME                  1

/* ------------------------------- 圈计时 -------------------------------- */
/* 起点在第二个右直角弯末端：每圈经过 2 个右直角弯；记录前 2 圈 */
#define CAR_LAP_CORNERS                  2U
#define CAR_LAP_COUNT                    2U
#define CAR_LAP_CORNER_DEBOUNCE_MS       300U  /* 同一个弯道重复找回线时防重复计数 */

/* ----------------------------- 超声波避障 ----------------------------- */
/* 策略：距离小于 STOP_CM 立即刹车停车，不再后退/转向绕行。 */
/* 当前临时关闭，先纯调速度环/PID；调好后改回 1。 */
#define CAR_OBSTACLE_ENABLE              0
#define CAR_OBSTACLE_STOP_CM             8.0f
#define CAR_OBSTACLE_CLEAR_CM            15.0f   /* 回差，避免边界抖动 */
#define CAR_OBSTACLE_CLEAR_HOLD_MS       300U    /* 超过 CLEAR_CM 后需保持的时间 */

/* 0=障碍物移开且重新看到线后自动恢复循迹；
 * 1=锁存停车，必须复位/断电（适合“遇到障碍物即结束”） */
#define CAR_OBSTACLE_STOP_LATCH          0

/* 说明：HC-SR04 在“前方 4m 内没有障碍物”时本来就可能没有回波，
 * 因此无回波按 999cm（前方空旷）处理，避免障碍物移开后一直停车。
 * 串口 U=0 可用于观察本次读数是否有效。 */

/* -------------------- 圆弧弯道蜂鸣提示（不含直角弯） -------------------- */
/* 判断依据：偏差持续超过阈值 + 压线通道数较少 + 偏差变化不剧烈。
 * 直角弯道通常表现为多路压线、丢线或偏差突变，会抑制提示。 */
#define CAR_ARC_BEEP_ENABLE              1
#define CAR_ARC_BEEP_MS                  80U    /* 进入/退出各短响一声 */

#define CAR_ARC_ENTER_ABS_ERROR          0.25f  /* |E| 超过该值认为进入圆弧候选 */
#define CAR_ARC_EXIT_ABS_ERROR           0.18f  /* |E| 低于该值认为退出圆弧候选 */
#define CAR_ARC_MAX_ABS_ERROR            0.78f  /* 更大偏差按直角/贴边处理 */
#define CAR_ARC_MAX_SENSORS              3U     /* 同时压线超过该路数按直角处理 */

#define CAR_ARC_ENTER_DEBOUNCE_MS        180U   /* 持续这么久才确认进入 */
#define CAR_ARC_EXIT_DEBOUNCE_MS         150U   /* 持续这么久才确认退出 */
#define CAR_ARC_SHARP_DELTA              0.35f  /* 偏差慢滤波差值超过该值视为突变 */
#define CAR_ARC_SUPPRESS_MS              400U   /* 直角/丢线后抑制提示的时间 */

/* --------------------------- 电机与编码器 ----------------------------- */
/* 1=速度闭环（使用编码器）；0=开环占空比 */
#define CAR_SPEED_CLOSED_LOOP            1

/* 电机方向：某轮“前进指令”实际反转时改为 1 */
#define CAR_MOTOR_L_INVERT               0
#define CAR_MOTOR_R_INVERT               0

/* 编码器方向：前进时速度读数为负则改为 1 */
/* 实测：左轮正转时编码器计数为负，故左编码器取反；右编码器待正转验证 */
#define CAR_ENCODER_L_INVERT             1
#define CAR_ENCODER_R_INVERT             0

/* 输出轴一圈计数 = 编码器线数 × 4 × 减速比（务必实测标定） */
#define CAR_ENCODER_CPR                  1560.0f
/* 车轮半径（米），例如 65mm 轮径填 0.0325f */
#define CAR_WHEEL_RADIUS_M               0.0325f
/* 左右轮中心距（米），仅 Motion_SetVelocity 转向解算使用 */
#define CAR_TRACK_WIDTH_M                0.150f

/* 单轮最大线速度与加减速斜坡 */
#define CAR_MAX_WHEEL_SPEED_MPS          0.55f
#define CAR_SPEED_RAMP_MPS2              1.00f   /* 先放缓加速，减小起步超调 */
#define CAR_SPEED_DEADBAND_MPS           0.004f
#define CAR_SPEED_STOP_EPS_MPS           0.020f
#define CAR_WHEEL_TRIM_L                 1.000f
#define CAR_WHEEL_TRIM_R                 1.000f

/* 速度环 PID（5ms 周期，先调 Kp，再少量加 Ki） */
#define CAR_SPEED_PID_KP                 0.90f   /* 降低比例，减小超调 */
#define CAR_SPEED_PID_KI                 0.25f   /* 降低积分，避免积分饱和 */
#define CAR_SPEED_PID_KD                 0.00f
#define CAR_SPEED_PID_MAX_OUT            1.00f
#define CAR_SPEED_PID_MAX_INTEGRAL       0.40f   /* 原 0.70，积分限幅收紧 */

/* 速度前馈：补偿电机/驱动死区。低速时只靠 PID 可能起不来，
 * 前馈按目标速度直接给一个基础占空比，PID 只做小幅修正。 */
#define CAR_SPEED_FF_GAIN                0.80f

/* 驱动板 PWM 占空比范围 */
#define CAR_MOTOR_MIN_DUTY               0.05f
#define CAR_MOTOR_MAX_DUTY               0.95f

/* 闭环安全检测：编码器反接/未接时自动停机，防止飞车 */
#define CAR_FAULT_DETECT_ENABLE          1
#define CAR_FAULT_REVERSE_MS             200U
#define CAR_FAULT_STALL_MS               800U

/* 调试串口遥测：比赛时可关掉 */
#define CAR_TELEMETRY_ENABLE             1
#define CAR_TELEMETRY_PERIOD_MS          200U
#define CAR_TELEMETRY_CAL_PERIOD_MS      100U

#endif /* CAR_CONFIG_H */
