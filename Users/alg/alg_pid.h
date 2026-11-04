#ifndef ALG_PID_H
#define ALG_PID_H

/**
 * @brief PID控制器结构体
 */
typedef struct {
    float Kp; // 比例项系数
    float Ki; // 积分项系数
    float Kd; // 微分项系数

    float max_out;      // 输出限幅
    float max_integral; // 积分限幅

    float ref;          // 目标值
    float fdb;          // 反馈值
    float err;          // 当前误差
    float err_sum;      // 误差累计
    float last_err;     // 上次误差
    float output;       // 输出值
} PID_Controller_t;

void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd, float max_out, float max_int);
float PID_Calculate(PID_Controller_t *pid, float target, float feedback);

#endif
