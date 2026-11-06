#include "motion.h"

#include <math.h>
#include <stddef.h>

#include "board_config.h"
#include "car_config.h"
#include "bldc_motor.h"
#include "encoder.h"
#include "alg_pid.h"
#include "tim.h"

/* ------------------------------ 电机配置 ------------------------------ */
static const BldcMotor_Config_t kMotorLConfig = {
    .htim = MOTOR_L_PWM_TIM,
    .pwm_ch = MOTOR_L_PWM_CH,
    .ina_port = MOTOR_L_INA_PORT,
    .ina_pin = MOTOR_L_INA_PIN,
    .inb_port = MOTOR_L_INB_PORT,
    .inb_pin = MOTOR_L_INB_PIN,
    .invert = CAR_MOTOR_L_INVERT,
    .min_duty = CAR_MOTOR_MIN_DUTY,
    .max_duty = CAR_MOTOR_MAX_DUTY,
};

static const BldcMotor_Config_t kMotorRConfig = {
    .htim = MOTOR_R_PWM_TIM,
    .pwm_ch = MOTOR_R_PWM_CH,
    .ina_port = MOTOR_R_INA_PORT,
    .ina_pin = MOTOR_R_INA_PIN,
    .inb_port = MOTOR_R_INB_PORT,
    .inb_pin = MOTOR_R_INB_PIN,
    .invert = CAR_MOTOR_R_INVERT,
    .min_duty = CAR_MOTOR_MIN_DUTY,
    .max_duty = CAR_MOTOR_MAX_DUTY,
};

/* ------------------------------ 内部状态 ------------------------------ */
static BldcMotor_t s_motor_l;
static BldcMotor_t s_motor_r;
static Encoder_t s_enc_l;
static Encoder_t s_enc_r;
static PID_Controller_t s_pid_l;
static PID_Controller_t s_pid_r;

static float s_cmd_l;
static float s_cmd_r;
static float s_set_l;
static float s_set_r;

static uint8_t s_inited;
static uint8_t s_closed_loop;
static uint8_t s_output_enabled;
static uint8_t s_fault_code;

static uint32_t s_reverse_since[2];
static uint32_t s_stall_since[2];

static float MotionClamp(float value, float min_value, float max_value)
{
    if (value > max_value) {
        return max_value;
    }
    if (value < min_value) {
        return min_value;
    }
    return value;
}

static float MotionRamp(float current, float target, float max_step)
{
    float delta = target - current;

    if (delta > max_step) {
        delta = max_step;
    }
    if (delta < -max_step) {
        delta = -max_step;
    }

    return current + delta;
}

static void MotionResetPid(PID_Controller_t *pid)
{
    pid->err = 0.0f;
    pid->err_sum = 0.0f;
    pid->last_err = 0.0f;
    pid->output = 0.0f;
}

static void MotionClearFaultTimers(void)
{
    s_reverse_since[0] = 0U;
    s_reverse_since[1] = 0U;
    s_stall_since[0] = 0U;
    s_stall_since[1] = 0U;
}

/**
 * @brief 编码器方向/断线检测
 *
 * 情况 1：目标为正，反馈却持续为负 → 编码器 A/B 接反或方向配置反了。
 * 情况 2：输出已经较大，反馈仍接近 0 → 编码器没接/没信号。
 */
static void MotionCheckFault(uint8_t index, float target, float feedback)
{
#if CAR_FAULT_DETECT_ENABLE
    if (!s_closed_loop || s_fault_code != 0U) {
        return;
    }

    uint32_t now = HAL_GetTick();

    if (fabsf(target) > 0.08f && (target * feedback) < -0.02f) {
        if (s_reverse_since[index] == 0U) {
            s_reverse_since[index] = now;
        } else if ((uint32_t)(now - s_reverse_since[index]) >
                   CAR_FAULT_REVERSE_MS) {
            s_fault_code = 1U;   /* 编码器方向反 */
        }
    } else {
        s_reverse_since[index] = 0U;
    }

    if (fabsf(target) > 0.12f && fabsf(feedback) < 0.005f) {
        if (s_stall_since[index] == 0U) {
            s_stall_since[index] = now;
        } else if ((uint32_t)(now - s_stall_since[index]) >
                   CAR_FAULT_STALL_MS) {
            s_fault_code = 2U;   /* 编码器无反馈/电机堵转 */
        }
    } else {
        s_stall_since[index] = 0U;
    }
#else
    (void)index;
    (void)target;
    (void)feedback;
#endif
}

/**
 * @brief 初始化电机、编码器、速度环
 */
void Motion_Init(void)
{
    BldcDriver_Init(MOTOR_STBY_PORT, MOTOR_STBY_PIN,
                    MOTOR_STBY_ACTIVE_LEVEL);
    BldcDriver_Enable(0U);

    BldcMotor_Init(&s_motor_l, &kMotorLConfig);
    BldcMotor_Init(&s_motor_r, &kMotorRConfig);

    Encoder_Init(&s_enc_l, ENC_L_TIM, (int32_t)CAR_ENCODER_CPR,
                 CAR_WHEEL_RADIUS_M, CAR_ENCODER_L_INVERT);
    Encoder_Init(&s_enc_r, ENC_R_TIM, (int32_t)CAR_ENCODER_CPR,
                 CAR_WHEEL_RADIUS_M, CAR_ENCODER_R_INVERT);

    PID_Init(&s_pid_l, CAR_SPEED_PID_KP, CAR_SPEED_PID_KI, CAR_SPEED_PID_KD,
             CAR_SPEED_PID_MAX_OUT, CAR_SPEED_PID_MAX_INTEGRAL);
    PID_Init(&s_pid_r, CAR_SPEED_PID_KP, CAR_SPEED_PID_KI, CAR_SPEED_PID_KD,
             CAR_SPEED_PID_MAX_OUT, CAR_SPEED_PID_MAX_INTEGRAL);

    s_cmd_l = 0.0f;
    s_cmd_r = 0.0f;
    s_set_l = 0.0f;
    s_set_r = 0.0f;
    s_fault_code = 0U;
    s_closed_loop = CAR_SPEED_CLOSED_LOOP ? 1U : 0U;

#if CAR_BRINGUP_MODE == 1
    /* 电机测试模式使用开环，先确认转向和编码器读数，避免方向接反飞车 */
    s_closed_loop = 0U;
#endif

    MotionClearFaultTimers();
    s_inited = 1U;

    Motion_SetEnabled(1U);
}

/**
 * @brief 总输出使能
 *
 * 关闭时仍会更新编码器（方便手转车轮标定 CPR / 检查传感器），
 * 两个电机强制停止；但 STBY 保持使能，因为部分驱动板的
 * OUTPUT 5V 由 STBY 控制，传感器校准需要这路 5V 供电。
 * 真正的故障保护会在 Motion_Loop 里显式拉低 STBY。
 */
void Motion_SetEnabled(uint8_t enabled)
{
    if (enabled) {
        if (s_fault_code == 0U) {
            s_output_enabled = 1U;
            BldcDriver_Enable(1U);
        }
    } else {
        s_output_enabled = 0U;
        s_cmd_l = 0.0f;
        s_cmd_r = 0.0f;
        s_set_l = 0.0f;
        s_set_r = 0.0f;
        BldcMotor_Stop(&s_motor_l);
        BldcMotor_Stop(&s_motor_r);
        MotionResetPid(&s_pid_l);
        MotionResetPid(&s_pid_r);
        /* 不拉低 STBY，保持驱动板 OUTPUT 5V 有电 */
    }
}

uint8_t Motion_IsEnabled(void)
{
    return s_output_enabled;
}

/**
 * @brief 直接设置左右轮目标线速度（m/s）
 */
void Motion_SetWheelSpeeds(float left_mps, float right_mps)
{
    if (!s_inited || !s_output_enabled || s_fault_code != 0U) {
        return;
    }

    left_mps *= CAR_WHEEL_TRIM_L;
    right_mps *= CAR_WHEEL_TRIM_R;

    s_cmd_l = MotionClamp(left_mps, -CAR_MAX_WHEEL_SPEED_MPS,
                          CAR_MAX_WHEEL_SPEED_MPS);
    s_cmd_r = MotionClamp(right_mps, -CAR_MAX_WHEEL_SPEED_MPS,
                          CAR_MAX_WHEEL_SPEED_MPS);
}

/**
 * @brief 按 vx/wz 解算左右轮速度
 */
void Motion_SetVelocity(float vx, float wz)
{
    float half_track = CAR_TRACK_WIDTH_M * 0.5f;
    Motion_SetWheelSpeeds(vx - wz * half_track, vx + wz * half_track);
}

/**
 * @brief 平滑停车
 */
void Motion_Stop(void)
{
    s_cmd_l = 0.0f;
    s_cmd_r = 0.0f;
}

/**
 * @brief 立即制动（障碍物/故障/状态切换用）
 */
void Motion_Brake(void)
{
    s_cmd_l = 0.0f;
    s_cmd_r = 0.0f;
    s_set_l = 0.0f;
    s_set_r = 0.0f;

    BldcMotor_Stop(&s_motor_l);
    BldcMotor_Stop(&s_motor_r);

    MotionResetPid(&s_pid_l);
    MotionResetPid(&s_pid_r);
    MotionClearFaultTimers();
}

/**
 * @brief 周期任务：5ms 调用一次
 */
void Motion_Loop(float dt_s)
{
    if (!s_inited || dt_s <= 0.0f) {
        return;
    }

    Encoder_Update(&s_enc_l, dt_s);
    Encoder_Update(&s_enc_r, dt_s);

    if (!s_output_enabled || s_fault_code != 0U) {
        BldcMotor_Stop(&s_motor_l);
        BldcMotor_Stop(&s_motor_r);
        return;
    }

    /* 速度斜坡 + 死区 */
    float max_step = CAR_SPEED_RAMP_MPS2 * dt_s;
    s_set_l = MotionRamp(s_set_l, s_cmd_l, max_step);
    s_set_r = MotionRamp(s_set_r, s_cmd_r, max_step);

    /* 死区只清“目标速度很小”的情况，不能清正在斜坡上升的设定值；
     * 否则 1.0m/s² × 5ms = 0.005m/s，每一步都会被 0.010m/s 死区清回 0，
     * 电机永远得不到输出。 */
    if (fabsf(s_cmd_l) < CAR_SPEED_DEADBAND_MPS) {
        s_set_l = 0.0f;
    }
    if (fabsf(s_cmd_r) < CAR_SPEED_DEADBAND_MPS) {
        s_set_r = 0.0f;
    }

    float fb_l = Encoder_GetSpeedMps(&s_enc_l);
    float fb_r = Encoder_GetSpeedMps(&s_enc_r);

    if (s_closed_loop) {
        if (s_set_l == 0.0f && fabsf(fb_l) < CAR_SPEED_STOP_EPS_MPS) {
            BldcMotor_Stop(&s_motor_l);
            MotionResetPid(&s_pid_l);
        } else {
            float out_l = PID_Calculate(&s_pid_l, s_set_l, fb_l);
            out_l += (s_set_l / CAR_MAX_WHEEL_SPEED_MPS) * CAR_SPEED_FF_GAIN;
            out_l = MotionClamp(out_l, -CAR_SPEED_PID_MAX_OUT,
                                CAR_SPEED_PID_MAX_OUT);
            BldcMotor_SetOutput(&s_motor_l, out_l);
        }

        if (s_set_r == 0.0f && fabsf(fb_r) < CAR_SPEED_STOP_EPS_MPS) {
            BldcMotor_Stop(&s_motor_r);
            MotionResetPid(&s_pid_r);
        } else {
            float out_r = PID_Calculate(&s_pid_r, s_set_r, fb_r);
            out_r += (s_set_r / CAR_MAX_WHEEL_SPEED_MPS) * CAR_SPEED_FF_GAIN;
            out_r = MotionClamp(out_r, -CAR_SPEED_PID_MAX_OUT,
                                CAR_SPEED_PID_MAX_OUT);
            BldcMotor_SetOutput(&s_motor_r, out_r);
        }

        MotionCheckFault(0U, s_set_l, fb_l);
        MotionCheckFault(1U, s_set_r, fb_r);

        if (s_fault_code != 0U) {
            Motion_Brake();
            BldcDriver_Enable(0U);
        }
    } else {
        float out_l = s_set_l / CAR_MAX_WHEEL_SPEED_MPS;
        float out_r = s_set_r / CAR_MAX_WHEEL_SPEED_MPS;

        out_l = MotionClamp(out_l, -1.0f, 1.0f);
        out_r = MotionClamp(out_r, -1.0f, 1.0f);

        if (out_l == 0.0f) {
            BldcMotor_Stop(&s_motor_l);
        } else {
            BldcMotor_SetOutput(&s_motor_l, out_l);
        }

        if (out_r == 0.0f) {
            BldcMotor_Stop(&s_motor_r);
        } else {
            BldcMotor_SetOutput(&s_motor_r, out_r);
        }
    }
}

float Motion_GetTargetL(void)
{
    return s_set_l;
}

float Motion_GetTargetR(void)
{
    return s_set_r;
}

float Motion_GetSpeedL(void)
{
    return Encoder_GetSpeedMps(&s_enc_l);
}

float Motion_GetSpeedR(void)
{
    return Encoder_GetSpeedMps(&s_enc_r);
}

int32_t Motion_GetCountL(void)
{
    return Encoder_GetTotal(&s_enc_l);
}

int32_t Motion_GetCountR(void)
{
    return Encoder_GetTotal(&s_enc_r);
}

uint8_t Motion_IsStopped(void)
{
    if (s_cmd_l != 0.0f || s_cmd_r != 0.0f) {
        return 0U;
    }
    if (fabsf(Motion_GetSpeedL()) > CAR_SPEED_STOP_EPS_MPS) {
        return 0U;
    }
    if (fabsf(Motion_GetSpeedR()) > CAR_SPEED_STOP_EPS_MPS) {
        return 0U;
    }
    return 1U;
}

uint8_t Motion_IsFault(void)
{
    return (s_fault_code != 0U) ? 1U : 0U;
}

uint8_t Motion_GetFaultCode(void)
{
    return s_fault_code;
}

void Motion_ClearFault(void)
{
    s_fault_code = 0U;
    MotionClearFaultTimers();
    MotionResetPid(&s_pid_l);
    MotionResetPid(&s_pid_r);
    Motion_SetEnabled(1U);
}
