#include "alg_pid.h"

/**
 * @brief PID控制器初始化
 *
 * @param pid PID控制器结构体指针
 * @param kp 比例项系数
 * @param ki 积分项系数
 * @param kd 微分项系数
 * @param max_out 输出限幅
 * @param max_int 积分限幅
 */
void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd, float max_out, float max_int) {
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->max_out = max_out;
    pid->max_integral = max_int;

    // 清空历史数据
    pid->err = 0;
    pid->err_sum = 0;
    pid->last_err = 0;
    pid->output = 0;
}

/**
 * @brief 计算PID输出值
 *
 * @param pid PID控制器结构体指针
 * @param target 目标值
 * @param feedback 输入值(反馈值)
 */
float PID_Calculate(PID_Controller_t *pid, float target, float feedback) {
    pid->ref = target;
    pid->fdb = feedback;
    pid->err = target - feedback;

    // 比例项
    float p_out = pid->Kp * pid->err;

    // 积分项
    pid->err_sum += pid->err;

    // 积分限幅
    if (pid->err_sum > pid->max_integral) pid->err_sum = pid->max_integral;
    if (pid->err_sum < -pid->max_integral) pid->err_sum = -pid->max_integral;
    float i_out = pid->Ki * pid->err_sum;

    // 微分项
    float d_out = pid->Kd * (pid->err - pid->last_err);

    // 总输出
    pid->output = p_out + i_out + d_out;

    // 输出限幅
    if (pid->output > pid->max_out) pid->output = pid->max_out;
    if (pid->output < -pid->max_out) pid->output = -pid->max_out;

    // 更新上次误差
    pid->last_err = pid->err;

    return pid->output;
}
