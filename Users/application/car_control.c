#include "car_control.h"

#include <math.h>
#include <stddef.h>

#include "board_config.h"
#include "car_config.h"
#include "tim.h"
#include "motion.h"
#include "track_sensor.h"
#include "ultrasonic.h"

typedef enum {
    AVOID_STEP_BRAKE = 0,
    AVOID_STEP_BACK,
    AVOID_STEP_TURN_OUT,
    AVOID_STEP_DRIVE,
    AVOID_STEP_TURN_BACK,
    AVOID_STEP_REACQUIRE,
} AvoidStep_t;

static TrackSensor_t s_track;
static Ultrasonic_t s_ultra;

static CarState_t s_state;
static uint32_t s_state_tick;
static uint32_t s_bringup_tick;

static AvoidStep_t s_avoid_step;
static uint32_t s_avoid_tick;
static int8_t s_avoid_dir;
static uint8_t s_avoid_attempts;
static uint8_t s_avoid_locked;

static float s_last_error;
static float s_error_filtered;
static uint8_t s_line_seen_count;

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

    if (state == CAR_STATE_LINE) {
        s_error_filtered = TrackSensor_GetError(&s_track);
        s_last_error = s_error_filtered;
    }

    if (state == CAR_STATE_AVOID) {
        s_avoid_step = AVOID_STEP_BRAKE;
        s_avoid_tick = s_state_tick;
        s_avoid_dir = (CAR_AVOID_DIR >= 0) ? 1 : -1;
        if (s_avoid_attempts < 255U) {
            s_avoid_attempts++;
        }
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
    s_avoid_step = AVOID_STEP_BRAKE;
    s_avoid_tick = s_state_tick;
    s_avoid_dir = (CAR_AVOID_DIR >= 0) ? 1 : -1;
    s_avoid_attempts = 0U;
    s_avoid_locked = 0U;
    s_last_error = 0.0f;
    s_error_filtered = 0.0f;
    s_line_seen_count = 0U;
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
        Motion_Brake();

        if (s_avoid_locked || s_avoid_attempts >= CAR_AVOID_MAX_ATTEMPTS) {
            s_avoid_locked = 1U;
            Car_EnterState(CAR_STATE_STOP);
            return;
        }

        Car_EnterState(CAR_STATE_AVOID);
        return;
    }

    if (TrackSensor_IsLost(&s_track)) {
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

    float error = TrackSensor_GetError(&s_track);
    if (TrackSensor_IsAllActive(&s_track)) {
        error = 0.0f;   /* 十字/宽黑线：直行通过 */
    }

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
 * @brief 超声波绕障状态
 *
 * 流程：
 *   刹车 → 后退一点 → 向一侧原地转出 → 直行绕过障碍
 *   → 反向转回 → 低速直行直到重新看到线 → 回到 LINE
 *
 * 只有一个前向超声波，无法判断左右哪边更空，因此默认固定右绕，
 * 可用 CAR_AVOID_DIR 改成 -1 左绕。路线只有直线/圆弧时足够用。
 */
static void Car_AvoidLoop(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = (uint32_t)(now - s_avoid_tick);
    float turn_speed = CAR_AVOID_TURN_SPEED_MPS;

    switch (s_avoid_step) {
        case AVOID_STEP_BRAKE:
            Motion_Brake();
            if (elapsed >= CAR_AVOID_BRAKE_MS) {
                s_avoid_step = AVOID_STEP_BACK;
                s_avoid_tick = now;
            }
            break;

        case AVOID_STEP_BACK:
            Motion_SetWheelSpeeds(-CAR_AVOID_BACK_SPEED_MPS,
                                  -CAR_AVOID_BACK_SPEED_MPS);

            /* 至少退 BACK_MS；如果障碍物仍然太近，继续退到有转向空间，
             * 最多退 BACK_MAX_MS。这样障碍物贴近车头时也能先拉开距离。 */
            if (elapsed >= CAR_AVOID_BACK_MS &&
                (Ultrasonic_GetDistanceCm(&s_ultra) > CAR_AVOID_BACK_CLEAR_CM ||
                 elapsed >= CAR_AVOID_BACK_MAX_MS)) {
                s_avoid_step = AVOID_STEP_TURN_OUT;
                s_avoid_tick = now;
            }
            break;

        case AVOID_STEP_TURN_OUT:
            if (s_avoid_dir > 0) {
                Motion_SetWheelSpeeds(turn_speed, -turn_speed);
            } else {
                Motion_SetWheelSpeeds(-turn_speed, turn_speed);
            }
            if (elapsed >= CAR_AVOID_TURN_MS) {
                s_avoid_step = AVOID_STEP_DRIVE;
                s_avoid_tick = now;
            }
            break;

        case AVOID_STEP_DRIVE:
            Motion_SetWheelSpeeds(CAR_AVOID_DRIVE_SPEED_MPS,
                                  CAR_AVOID_DRIVE_SPEED_MPS);

            if (elapsed >= CAR_AVOID_DRIVE_MAX_MS) {
                /* 绕了很久还没过去：停车保护，避免继续顶障碍物 */
                Motion_Brake();
                Car_EnterState(CAR_STATE_STOP);
                return;
            }

            if (elapsed >= CAR_AVOID_DRIVE_MS &&
                Ultrasonic_GetDistanceCm(&s_ultra) > CAR_AVOID_CLEAR_CM) {
                s_avoid_step = AVOID_STEP_TURN_BACK;
                s_avoid_tick = now;
            }
            break;

        case AVOID_STEP_TURN_BACK:
            if (s_avoid_dir > 0) {
                Motion_SetWheelSpeeds(-turn_speed, turn_speed);
            } else {
                Motion_SetWheelSpeeds(turn_speed, -turn_speed);
            }
            if (elapsed >= CAR_AVOID_TURN_BACK_MS) {
                s_avoid_step = AVOID_STEP_REACQUIRE;
                s_avoid_tick = now;
                s_line_seen_count = 0U;
            }
            break;

        case AVOID_STEP_REACQUIRE: {
            /* 右绕后线在左侧，左轮慢一点、右轮快一点，让车往线的方向回拐；
             * 左绕时相反。这样不是一直直行，更容易重新扫到线。 */
            float speed = CAR_AVOID_REACQUIRE_SPEED_MPS;
            float turn = CAR_AVOID_REACQUIRE_TURN_MPS;
            if (s_avoid_dir > 0) {
                Motion_SetWheelSpeeds(speed - turn, speed + turn);
            } else {
                Motion_SetWheelSpeeds(speed + turn, speed - turn);
            }

            if (!TrackSensor_IsLost(&s_track)) {
                if (s_line_seen_count < 250U) {
                    s_line_seen_count++;
                }
                if (s_line_seen_count >= 3U) {
                    s_avoid_attempts = 0U;
                    s_avoid_locked = 0U;
                    Car_EnterState(CAR_STATE_LINE);
                    return;
                }
            } else {
                s_line_seen_count = 0U;
            }

            if (elapsed >= CAR_AVOID_REACQUIRE_TIMEOUT_MS) {
                Motion_Brake();
                Car_EnterState(CAR_STATE_SEARCH);
            }
            break;
        }

        default:
            Motion_Brake();
            Car_EnterState(CAR_STATE_STOP);
            break;
    }
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
 * @brief 停车状态：线重新出现时可自动恢复（可配置）
 */
static void Car_StopLoop(void)
{
    Motion_Brake();

#if CAR_AUTO_RESUME
    if (!s_avoid_locked && !TrackSensor_IsLost(&s_track)) {
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

        case CAR_STATE_AVOID:
            Car_AvoidLoop();
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
        case CAR_STATE_AVOID:
            return "AVOID";
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
    return (s_state == CAR_STATE_AVOID) ? 1U : 0U;
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
