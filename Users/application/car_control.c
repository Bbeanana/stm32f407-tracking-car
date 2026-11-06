#include "car_control.h"

#include <math.h>
#include <stddef.h>

#include "board_config.h"
#include "buzzer.h"
#include "camera.h"
#include "car_config.h"
#include "lap_timer.h"
#include "tim.h"
#include "motion.h"
#include "track_sensor.h"
#include "ultrasonic.h"

static TrackSensor_t s_track;
static Ultrasonic_t s_ultra;

static CarState_t s_state;
static uint32_t s_state_tick;
static uint32_t s_bringup_tick;

/* 因障碍物进入 STOP 时置 1；障碍物移开并稳定后清零 */
static uint8_t s_stop_by_obstacle;
static uint32_t s_obstacle_clear_tick;

static float s_last_error;
static float s_error_filtered;
static uint8_t s_line_seen_count;
static uint8_t s_line_ever_seen;

/* 摄像头 CURV 直角弯检测状态 */
static uint8_t s_cam_corner_active;
static uint8_t s_cam_corner_low_seen;
static uint32_t s_cam_corner_entry_tick;
static uint32_t s_cam_corner_exit_tick;
static uint32_t s_cam_corner_low_tick;

#if !CAR_RIGHT_TURNS_ONLY
/* 左右弯都可能时，记录丢线前最后一次有效偏差的方向 */
static float s_last_turn_hint;
static uint32_t s_last_turn_hint_tick;
#endif
static int8_t s_search_dir;
#if !CAR_RIGHT_TURNS_ONLY
static uint8_t s_search_phase;
static uint32_t s_search_phase_tick;
#endif

/* 圆弧弯道提示状态（摄像头 CURV） */
static uint8_t s_arc_active;
static uint8_t s_arc_stage;   /* 0=本圈未入弯, 1=已入弯, 2=已出弯 */
static uint8_t s_arc_lap;
static uint32_t s_arc_candidate_tick;
static uint32_t s_arc_entry_tick;
static uint32_t s_arc_exit_tick;
static uint8_t s_arc_suppressed;
static uint32_t s_arc_resume_tick;

/**
 * @brief 复位圆弧检测；suppress=1 时在 CAR_ARC_SUPPRESS_MS 内不提示
 */
static void Car_ArcReset(uint8_t suppress, uint32_t now)
{
    s_arc_active = 0U;
    s_arc_candidate_tick = 0U;
    s_arc_entry_tick = 0U;
    s_arc_exit_tick = 0U;

    if (suppress) {
        s_arc_suppressed = 1U;
        s_arc_resume_tick = now + CAR_ARC_SUPPRESS_MS;
    } else {
        s_arc_suppressed = 0U;
        s_arc_resume_tick = 0U;
    }
}

/**
 * @brief 圆弧弯道进出检测（摄像头 CURV）
 *
 * 直道 CURV 接近 0，圆弧一般在 8~35°，直角弯会出现 >45° 的尖峰。
 * 超过 MAX 时按直角弯处理并静默，不产生圆弧蜂鸣。
 */
static void Car_ArcUpdate(uint32_t now)
{
#if CAR_ARC_BEEP_ENABLE
    float abs_curv;

    /* 第一圈真正开始移动前、完赛后都不做圆弧提示。
     * 状态切换时已经复位过，这里只返回，避免反复延长静默时间。 */
    if (!LapTimer_IsRunning()) {
        return;
    }

    /* 每一圈只允许一段圆弧提示：入弯一声、出弯一声。
     * 圆弧中间 CURV 抖动或出弯后的直角弯不再重复响。 */
    {
        uint8_t lap = LapTimer_GetCurrentLap();
        if (lap != s_arc_lap) {
            s_arc_lap = lap;
            s_arc_stage = 0U;
            s_arc_active = 0U;
            s_arc_candidate_tick = 0U;
            s_arc_entry_tick = 0U;
            s_arc_exit_tick = 0U;
        }
    }

    if (s_arc_stage >= 2U) {
        return;
    }

    abs_curv = fabsf(Camera_GetCurvatureDeg());

    /* 直角弯/异常大曲率：复位圆弧状态，避免把直角弯当圆弧 */
    if (abs_curv > CAR_CAM_ARC_MAX_DEG) {
        Car_ArcReset(1U, now);
        return;
    }

    if (s_arc_suppressed) {
        if ((int32_t)(now - s_arc_resume_tick) < 0) {
            return;
        }
        s_arc_suppressed = 0U;
    }

    if (s_arc_stage == 0U) {
        if (abs_curv >= CAR_CAM_ARC_ENTER_DEG) {
            if (s_arc_candidate_tick == 0U) {
                s_arc_candidate_tick = now;
            } else if ((uint32_t)(now - s_arc_candidate_tick) >=
                       CAR_CAM_ARC_ENTER_MS) {
                s_arc_candidate_tick = 0U;
                s_arc_active = 1U;
                s_arc_entry_tick = now;
                s_arc_exit_tick = 0U;
                s_arc_stage = 1U;
                Buzzer_Beep(CAR_ARC_BEEP_MS);
            }
        } else {
            s_arc_candidate_tick = 0U;
        }
    } else if (s_arc_stage == 1U) {
        /* 防止圆弧中间 CURV 短时掉到 0 附近时提前报出弯 */
        if ((uint32_t)(now - s_arc_entry_tick) < CAR_CAM_ARC_MIN_MS) {
            s_arc_exit_tick = 0U;
            return;
        }

        if (abs_curv <= CAR_CAM_ARC_EXIT_DEG) {
            if (s_arc_exit_tick == 0U) {
                s_arc_exit_tick = now;
            } else if ((uint32_t)(now - s_arc_exit_tick) >=
                       CAR_CAM_ARC_EXIT_MS) {
                s_arc_active = 0U;
                s_arc_exit_tick = 0U;
                s_arc_stage = 2U;
                Buzzer_Beep(CAR_ARC_BEEP_MS);
            }
        } else {
            s_arc_exit_tick = 0U;
        }
    }
#else
    (void)now;
#endif
}

static float CarClamp(float value, float min_value, float max_value)
{
    if (value > max_value) {
        return max_value;
    }
    if (value < min_value) {
        return min_value;
    }
    return value;
}

static void Car_EnterState(CarState_t state)
{
    s_state = state;
    s_state_tick = HAL_GetTick();
    s_line_seen_count = 0U;

    /* 状态切换后先静默一段时间，避免把重新找线/起步当成圆弧 */
    Car_ArcReset(1U, s_state_tick);

    if (state == CAR_STATE_SEARCH) {
#if CAR_RIGHT_TURNS_ONLY
        /* 赛道只有右转弯，丢线后固定向右找线 */
        s_search_dir = 1;
#else
        /* 优先朝丢线前最后看到线的方向转；没有有效提示时用默认方向 */
        s_search_phase = 0U;
        s_search_phase_tick = s_state_tick;
        s_search_dir = (CAR_SEARCH_DEFAULT_DIR >= 0) ? 1 : -1;

        if ((uint32_t)(s_state_tick - s_last_turn_hint_tick) <=
                CAR_SEARCH_HINT_MS &&
            fabsf(s_last_turn_hint) >= CAR_SEARCH_HINT_MIN_ERROR) {
            s_search_dir = (s_last_turn_hint >= 0.0f) ? 1 : -1;
        }
#endif
    }

    if (state == CAR_STATE_LINE) {
        /* 进入循迹时重新武装直角弯检测，避免起点残留 CURV 造成误计数 */
        s_cam_corner_active = 0U;
        s_cam_corner_low_seen = 0U;
        s_cam_corner_entry_tick = 0U;
        s_cam_corner_exit_tick = 0U;
        s_cam_corner_low_tick = 0U;

        s_error_filtered = TrackSensor_GetError(&s_track);
        s_last_error = s_error_filtered;

        /* 进入循迹时按中心死区处理，避免第一拍微分项产生踢动 */
        if (fabsf(s_last_error) < CAR_LINE_ERROR_DEADBAND) {
            s_last_error = 0.0f;
        }
    }

    if (state == CAR_STATE_STOP) {
        s_stop_by_obstacle = 0U;
        s_obstacle_clear_tick = s_state_tick;
    }
}

/**
 * @brief 初始化：电机/编码器 + 传感器 + 状态机
 */
void Car_Init(void)
{
    const TrackSensor_Config_t track_cfg = {
        .port = TRACK_PORT,
        .first_pin = TRACK_FIRST_PIN,
        .pin_count = TRACK_PIN_COUNT,
        .active_level = TRACK_ACTIVE_LEVEL,
        .reverse_order = TRACK_REVERSE_ORDER,
    };

    const Ultrasonic_Config_t ultra_cfg = {
        .trig_port = ULTRA_TRIG_PORT,
        .trig_pin = ULTRA_TRIG_PIN,
        .echo_htim = ULTRA_ECHO_TIM,
        .echo_channel = ULTRA_ECHO_CH,
    };

    Motion_Init();
#if CAR_BRINGUP_MODE == 2 || CAR_BRINGUP_MODE == 3
    /* 传感器测试/校准：只读编码器，不驱动电机 */
    Motion_SetEnabled(0U);
#endif
    TrackSensor_Init(&s_track, &track_cfg);
    Ultrasonic_Init(&s_ultra, &ultra_cfg);
    Camera_Init();

    s_state = CAR_STATE_INIT;
    s_state_tick = HAL_GetTick();
    s_bringup_tick = s_state_tick;
    s_stop_by_obstacle = 0U;
    s_obstacle_clear_tick = s_state_tick;
    s_last_error = 0.0f;
    s_error_filtered = 0.0f;
    s_line_seen_count = 0U;
    s_line_ever_seen = 0U;
#if !CAR_RIGHT_TURNS_ONLY
    s_last_turn_hint = 0.0f;
    s_last_turn_hint_tick = 0U;
#endif
    s_search_dir = 1;
#if !CAR_RIGHT_TURNS_ONLY
    s_search_phase = 0U;
    s_search_phase_tick = s_state_tick;
#endif
    Car_ArcReset(1U, s_state_tick);
}

/**
 * @brief 超声波障碍判定
 *
 * 无回波时 Ultrasonic 会把距离置为 999cm（视为前方空旷），
 * 避免障碍物移开后一直停车；串口 U=0 可用于观察本次读数是否有效。
 */
static uint8_t Car_ObstacleDetected(void)
{
#if CAR_OBSTACLE_ENABLE
    return (Ultrasonic_GetDistanceCm(&s_ultra) < CAR_OBSTACLE_STOP_CM) ? 1U : 0U;
#else
    return 0U;
#endif
}

/**
 * @brief 用摄像头 CURV 检测直角弯
 *
 * 圆弧的 CURV 较小；直角弯的远近线角度差会出现明显尖峰。
 * 进入阈值持续一段时间后置位，回落到退出阈值后计一次弯道。
 */
static void Car_CameraCornerUpdate(uint32_t now)
{
    float abs_curv = fabsf(Camera_GetCurvatureDeg());

    if (!s_cam_corner_active) {
        /* 必须先看到一段低 CURV，才允许进入直角弯。
         * 这样起点停在弯道末端、残留高 CURV 时不会多计一个弯。 */
        if (abs_curv <= CAR_CAM_CORNER_EXIT_DEG) {
            if (s_cam_corner_low_tick == 0U) {
                s_cam_corner_low_tick = now;
            } else if ((uint32_t)(now - s_cam_corner_low_tick) >=
                       CAR_CAM_CORNER_EXIT_MS) {
                s_cam_corner_low_seen = 1U;
            }
        } else {
            s_cam_corner_low_tick = 0U;
        }

        if (s_cam_corner_low_seen &&
            abs_curv >= CAR_CAM_CORNER_ENTER_DEG) {
            if (s_cam_corner_entry_tick == 0U) {
                s_cam_corner_entry_tick = now;
            } else if ((uint32_t)(now - s_cam_corner_entry_tick) >=
                       CAR_CAM_CORNER_ENTER_MS) {
                s_cam_corner_active = 1U;
                s_cam_corner_low_seen = 0U;
                s_cam_corner_entry_tick = 0U;
                s_cam_corner_exit_tick = 0U;
                s_cam_corner_low_tick = 0U;
            }
        } else if (abs_curv < CAR_CAM_CORNER_ENTER_DEG) {
            s_cam_corner_entry_tick = 0U;
        }
    } else {
        if (abs_curv <= CAR_CAM_CORNER_EXIT_DEG) {
            if (s_cam_corner_exit_tick == 0U) {
                s_cam_corner_exit_tick = now;
            } else if ((uint32_t)(now - s_cam_corner_exit_tick) >=
                       CAR_CAM_CORNER_EXIT_MS) {
                s_cam_corner_active = 0U;
                s_cam_corner_exit_tick = 0U;
                s_cam_corner_low_seen = 1U;
                s_cam_corner_low_tick = now;
                LapTimer_OnRightAngle();
            }
        } else {
            s_cam_corner_exit_tick = 0U;
        }
    }
}

/**
 * @brief 正常循迹状态
 */
static void Car_LineLoop(void)
{
    uint32_t now = HAL_GetTick();

    /* 两圈完成，锁存停车 */
    if (LapTimer_IsFinished()) {
        Motion_Brake();
        Car_EnterState(CAR_STATE_STOP);
        return;
    }

    if (Car_ObstacleDetected()) {
        /* 检测到障碍物只停车，不再执行后退/转向绕行 */
        Motion_Brake();
        Buzzer_Beep(CAR_OBSTACLE_BEEP_MS);
        Car_EnterState(CAR_STATE_STOP);
        s_stop_by_obstacle = 1U;
        return;
    }

    /* 循迹只使用摄像头 ERR；前八路传感器不参与转向 */
    if (!Camera_IsValid()) {
        Motion_Brake();
        return;
    }

    s_line_ever_seen = 1U;
    float error = Camera_GetError();

    /* 转向只用摄像头 ERR；圆弧进出蜂鸣用摄像头 CURV，不参与转向 */
    Car_ArcUpdate(now);

    /* 摄像头 CURV 直角弯计数；第二圈完成立即停车 */
    Car_CameraCornerUpdate(now);
    if (LapTimer_IsFinished()) {
        Motion_Brake();
        Car_EnterState(CAR_STATE_STOP);
        return;
    }

    s_error_filtered +=
        CAR_LINE_ERROR_FILTER * (error - s_error_filtered);

    float control_error = s_error_filtered;
    if (fabsf(control_error) < CAR_LINE_ERROR_DEADBAND) {
        control_error = 0.0f;   /* 线在中心附近时不做转向，避免数字量抖动 */
    }

    float delta = control_error - s_last_error;
    float turn = CAR_LINE_KP * control_error + CAR_LINE_KD * delta;

    turn = CarClamp(turn, -CAR_LINE_TURN_LIMIT_MPS, CAR_LINE_TURN_LIMIT_MPS);

    float base = CAR_LINE_BASE_SPEED_MPS *
                 (1.0f - CAR_LINE_SLOWDOWN_GAIN * fabsf(s_error_filtered));
    if (base < CAR_LINE_MIN_SPEED_MPS) {
        base = CAR_LINE_MIN_SPEED_MPS;
    }

    float left = base + turn;
    float right = base - turn;

    left = CarClamp(left, -CAR_LINE_MAX_SPEED_MPS, CAR_LINE_MAX_SPEED_MPS);
    right = CarClamp(right, -CAR_LINE_MAX_SPEED_MPS, CAR_LINE_MAX_SPEED_MPS);

    Motion_SetWheelSpeeds(left, right);
    s_last_error = control_error;
}

/**
 * @brief 丢线/直角弯搜索：先按最后偏线方向原地转，超时后反向再找
 */
static void Car_SearchLoop(void)
{
    uint32_t now = HAL_GetTick();

    if (!TrackSensor_IsLost(&s_track)) {
        if (s_line_seen_count < 250U) {
            s_line_seen_count++;
        }
        if (s_line_seen_count >= 3U) {
            Car_EnterState(CAR_STATE_LINE);
            return;
        }
    } else {
        s_line_seen_count = 0U;
    }

#if CAR_RIGHT_TURNS_ONLY
    uint32_t search_timeout = CAR_SEARCH_RIGHT_TIMEOUT_MS;
#else
    uint32_t search_timeout = CAR_SEARCH_TIMEOUT_MS;
#endif

    if ((uint32_t)(now - s_state_tick) > search_timeout) {
        Motion_Brake();
        Car_EnterState(CAR_STATE_STOP);
        return;
    }

#if !CAR_RIGHT_TURNS_ONLY
    /* 第一段没找到线就立即反向，避免方向提示错误时一直转下去 */
    if (s_search_phase == 0U &&
        (uint32_t)(now - s_search_phase_tick) >= CAR_SEARCH_PHASE_MS) {
        s_search_dir = (int8_t)(-s_search_dir);
        s_search_phase = 1U;
        s_search_phase_tick = now;
    }
#endif

    float search = CAR_SEARCH_SPEED_MPS;
    if (s_search_dir > 0) {
        /* 线原先在右侧：左轮前进、右轮后退 → 原地右转 */
        Motion_SetWheelSpeeds(search, -search);
    } else {
        Motion_SetWheelSpeeds(-search, search);
    }
}

/**
 * @brief 停车状态：障碍物移开且线重新出现后可自动恢复（可配置）
 */
static void Car_StopLoop(void)
{
    Motion_Brake();

    /* 跑完两圈后保持停车，不自动恢复 */
    if (LapTimer_IsFinished()) {
        return;
    }

    if (s_stop_by_obstacle) {
#if CAR_OBSTACLE_STOP_LATCH
        /* 锁存停车：必须复位/断电，适合“遇到障碍物即结束”的场景 */
        return;
#else
        uint32_t now = HAL_GetTick();

        /* 回差：降到 CAR_OBSTACLE_CLEAR_CM 以下仍保持停车，
         * 超过该距离并稳定 CAR_OBSTACLE_CLEAR_HOLD_MS 才允许恢复。 */
        if (Ultrasonic_GetDistanceCm(&s_ultra) < CAR_OBSTACLE_CLEAR_CM) {
            s_obstacle_clear_tick = now;
            return;
        }
        if ((uint32_t)(now - s_obstacle_clear_tick) <
            CAR_OBSTACLE_CLEAR_HOLD_MS) {
            return;
        }
        s_stop_by_obstacle = 0U;
#endif
    }

#if CAR_AUTO_RESUME
    /* 摄像头有效就恢复循迹；前八路传感器不参与转向 */
    if (Camera_IsValid()) {
        Car_EnterState(CAR_STATE_LINE);
    }
#endif
}

/**
 * @brief 调试模式 1：架空车轮，正转 2s → 停 2s → 反转 2s → 停 2s
 */
static void Car_BringupMotorLoop(void)
{
    uint32_t now = HAL_GetTick();

    if ((uint32_t)(now - s_bringup_tick) < CAR_START_DELAY_MS) {
        Motion_Brake();
        return;
    }

    uint32_t t = (uint32_t)(now - s_bringup_tick) - CAR_START_DELAY_MS;
    uint32_t phase = (t / 2000U) % 4U;

    switch (phase) {
        case 0U:
            Motion_SetWheelSpeeds(0.12f, 0.12f);
            break;
        case 2U:
            Motion_SetWheelSpeeds(-0.12f, -0.12f);
            break;
        case 1U:
        case 3U:
        default:
            Motion_Brake();
            break;
    }
}

/**
 * @brief 主循环：5ms 调用一次
 */
void Car_Loop(float dt_s)
{
    TrackSensor_Update(&s_track);   /* 只保留给遥测/调试，不参与转向 */
    Ultrasonic_Update(&s_ultra);
    Camera_Update();

    uint32_t now = HAL_GetTick();

    if (Motion_IsFault()) {
        s_state = CAR_STATE_FAULT;
        Motion_Brake();
        Motion_Loop(dt_s);
        return;
    }

#if CAR_BRINGUP_MODE == 1
    Car_BringupMotorLoop();
    Motion_Loop(dt_s);
    return;
#elif CAR_BRINGUP_MODE == 2 || CAR_BRINGUP_MODE == 3
    Motion_Brake();
    Motion_Loop(dt_s);
    return;
#endif

    switch (s_state) {
        case CAR_STATE_INIT:
            Motion_Brake();
            if ((uint32_t)(now - s_state_tick) >= CAR_START_DELAY_MS) {
                Car_EnterState(CAR_STATE_LINE);
            }
            break;

        case CAR_STATE_LINE:
            Car_LineLoop();
            break;

        case CAR_STATE_SEARCH:
            Car_SearchLoop();
            break;

        case CAR_STATE_STOP:
            Car_StopLoop();
            break;

        case CAR_STATE_FAULT:
            Motion_Brake();
            break;

        default:
            Motion_Brake();
            Car_EnterState(CAR_STATE_INIT);
            break;
    }

    Motion_Loop(dt_s);
}

CarState_t Car_GetState(void)
{
    return s_state;
}

const char *Car_GetStateName(void)
{
    switch (s_state) {
        case CAR_STATE_INIT:
            return "INIT";
        case CAR_STATE_LINE:
            return "LINE";
        case CAR_STATE_SEARCH:
            return "SRCH";
        case CAR_STATE_STOP:
            return "STOP";
        case CAR_STATE_FAULT:
            return "FAULT";
        default:
            return "?";
    }
}

uint8_t Car_IsObstacle(void)
{
    return s_stop_by_obstacle;
}

uint8_t Car_GetTrackMask(void)
{
    return TrackSensor_GetMask(&s_track);
}

uint8_t Car_GetTrackRawMask(void)
{
    return TrackSensor_GetRawMask(&s_track);
}

float Car_GetTrackError(void)
{
    return TrackSensor_GetError(&s_track);
}

float Car_GetDistanceCm(void)
{
    return Ultrasonic_GetDistanceCm(&s_ultra);
}

uint8_t Car_GetUltrasonicValid(void)
{
    return Ultrasonic_IsValid(&s_ultra);
}
