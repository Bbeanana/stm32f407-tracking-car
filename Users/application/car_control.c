#include "car_control.h"

#include <math.h>
#include <stddef.h>

#include "board_config.h"
#include "buzzer.h"
#include "car_config.h"
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

/* 圆弧弯道提示状态 */
static uint8_t s_arc_active;
static uint8_t s_arc_candidate;
static int8_t s_arc_candidate_sign;
static uint32_t s_arc_candidate_tick;
static uint32_t s_arc_exit_tick;
static uint8_t s_arc_suppressed;
static uint32_t s_arc_resume_tick;
static float s_arc_error_slow;

static uint8_t Car_CountMaskBits(uint8_t mask)
{
    uint8_t count = 0U;

    while (mask != 0U) {
        count += (uint8_t)(mask & 1U);
        mask >>= 1U;
    }

    return count;
}

/**
 * @brief 复位圆弧检测；suppress=1 时在 CAR_ARC_SUPPRESS_MS 内不提示
 */
static void Car_ArcReset(uint8_t suppress, uint32_t now)
{
    s_arc_active = 0U;
    s_arc_candidate = 0U;
    s_arc_candidate_sign = 0;
    s_arc_candidate_tick = 0U;
    s_arc_exit_tick = 0U;
    s_arc_error_slow = TrackSensor_GetError(&s_track);

    if (suppress) {
        s_arc_suppressed = 1U;
        s_arc_resume_tick = now + CAR_ARC_SUPPRESS_MS;
    } else {
        s_arc_suppressed = 0U;
        s_arc_resume_tick = 0U;
    }
}

/**
 * @brief 圆弧弯道进出检测
 *
 * 只对“偏差持续较大、压线通道数较少、偏差变化不剧烈”的情况提示；
 * 直角弯道常见的多路压线、丢线、偏差突变都会触发复位/抑制。
 */
static void Car_ArcUpdate(uint32_t now, float error, uint8_t lost,
                          uint8_t all_active)
{
#if CAR_ARC_BEEP_ENABLE
    uint8_t count = Car_CountMaskBits(TrackSensor_GetMask(&s_track));
    float abs_error = fabsf(error);
    float error_delta = fabsf(error - s_arc_error_slow);
    uint8_t sharp = 0U;

    s_arc_error_slow += 0.05f * (error - s_arc_error_slow);

    if (lost || all_active || count > CAR_ARC_MAX_SENSORS ||
        abs_error > CAR_ARC_MAX_ABS_ERROR ||
        error_delta > CAR_ARC_SHARP_DELTA) {
        sharp = 1U;
    }

    if (sharp) {
        Car_ArcReset(1U, now);
        return;
    }

    if (s_arc_suppressed) {
        if ((int32_t)(now - s_arc_resume_tick) < 0) {
            return;
        }
        s_arc_suppressed = 0U;
    }

    if (!s_arc_active) {
        if (abs_error >= CAR_ARC_ENTER_ABS_ERROR &&
            abs_error <= CAR_ARC_MAX_ABS_ERROR) {
            int8_t sign = (error >= 0.0f) ? 1 : -1;

            if (!s_arc_candidate || sign != s_arc_candidate_sign) {
                s_arc_candidate = 1U;
                s_arc_candidate_sign = sign;
                s_arc_candidate_tick = now;
            } else if ((uint32_t)(now - s_arc_candidate_tick) >=
                       CAR_ARC_ENTER_DEBOUNCE_MS) {
                s_arc_candidate = 0U;
                s_arc_active = 1U;
                s_arc_exit_tick = 0U;
                Buzzer_Beep(CAR_ARC_BEEP_MS);
            }
        } else {
            s_arc_candidate = 0U;
        }
    } else {
        if (abs_error <= CAR_ARC_EXIT_ABS_ERROR) {
            if (s_arc_exit_tick == 0U) {
                s_arc_exit_tick = now;
            } else if ((uint32_t)(now - s_arc_exit_tick) >=
                       CAR_ARC_EXIT_DEBOUNCE_MS) {
                s_arc_active = 0U;
                s_arc_exit_tick = 0U;
                Buzzer_Beep(CAR_ARC_BEEP_MS);
            }
        } else {
            s_arc_exit_tick = 0U;
        }
    }
#else
    (void)now;
    (void)error;
    (void)lost;
    (void)all_active;
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

    if (state == CAR_STATE_LINE) {
        s_error_filtered = TrackSensor_GetError(&s_track);
        s_last_error = s_error_filtered;
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

    s_state = CAR_STATE_INIT;
    s_state_tick = HAL_GetTick();
    s_bringup_tick = s_state_tick;
    s_stop_by_obstacle = 0U;
    s_obstacle_clear_tick = s_state_tick;
    s_last_error = 0.0f;
    s_error_filtered = 0.0f;
    s_line_seen_count = 0U;
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
 * @brief 正常循迹状态
 */
static void Car_LineLoop(void)
{
    uint32_t now = HAL_GetTick();

    if (Car_ObstacleDetected()) {
        /* 检测到障碍物只停车，不再执行后退/转向绕行 */
        Motion_Brake();
        Car_EnterState(CAR_STATE_STOP);
        s_stop_by_obstacle = 1U;
        return;
    }

    uint8_t track_lost = TrackSensor_IsLost(&s_track);

    if (track_lost) {
        if ((uint32_t)(now - TrackSensor_GetLastSeenTick(&s_track)) >
            CAR_LOST_STOP_MS) {
            Motion_Brake();
            Car_EnterState(CAR_STATE_SEARCH);
            return;
        }
        /* 短暂丢线：继续使用上一次偏差通过小缝隙 */
    } else {
        if (s_line_seen_count < 250U) {
            s_line_seen_count++;
        }
    }

    uint8_t all_active = TrackSensor_IsAllActive(&s_track);
    float error = TrackSensor_GetError(&s_track);
    if (all_active) {
        error = 0.0f;   /* 十字/宽黑线：直行通过 */
    }

    Car_ArcUpdate(now, error, track_lost, all_active);

    s_error_filtered +=
        CAR_LINE_ERROR_FILTER * (error - s_error_filtered);

    float delta = s_error_filtered - s_last_error;
    float turn = CAR_LINE_KP * s_error_filtered + CAR_LINE_KD * delta;

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
    s_last_error = s_error_filtered;
}

/**
 * @brief 丢线搜索：按最后偏差方向原地慢转
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

    if ((uint32_t)(now - s_state_tick) > CAR_SEARCH_TIMEOUT_MS) {
        Motion_Brake();
        Car_EnterState(CAR_STATE_STOP);
        return;
    }

    float search = CAR_SEARCH_SPEED_MPS;
    if (s_last_error >= 0.0f) {
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
    if (!TrackSensor_IsLost(&s_track)) {
        if (s_line_seen_count < 250U) {
            s_line_seen_count++;
        }
        if (s_line_seen_count >= 3U) {
            Car_EnterState(CAR_STATE_LINE);
        }
    } else {
        s_line_seen_count = 0U;
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
    TrackSensor_Update(&s_track);
    Ultrasonic_Update(&s_ultra);

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
