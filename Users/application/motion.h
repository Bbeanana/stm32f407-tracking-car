#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>

/**
 * @brief 两路无刷电机差速底盘控制（编码器速度闭环）
 *
 * 坐标约定：
 *   vx：前后线速度（m/s），正 = 前进
 *   wz：旋转角速度（rad/s），正 = 左转（逆时针）
 *
 * 也可以直接给左右轮目标速度 Motion_SetWheelSpeeds()，
 * 循迹应用使用这个接口更直接。
 *
 * 周期：Motion_Loop() 固定 5ms 调用。
 */

void Motion_Init(void);

/* 总输出使能：0=只更新编码器、不驱动电机（传感器调试/急停用） */
void Motion_SetEnabled(uint8_t enabled);
uint8_t Motion_IsEnabled(void);

/* 直接设置左右轮目标线速度（m/s） */
void Motion_SetWheelSpeeds(float left_mps, float right_mps);

/* 按底盘 vx/wz 解算左右轮目标速度（m/s） */
void Motion_SetVelocity(float vx, float wz);

void Motion_Stop(void);       /* 平滑减速到 0 */
void Motion_Brake(void);      /* 立即停止，清 PID */
void Motion_Loop(float dt_s); /* 5ms 周期 */

float Motion_GetTargetL(void);
float Motion_GetTargetR(void);
float Motion_GetSpeedL(void);
float Motion_GetSpeedR(void);
int32_t Motion_GetCountL(void);
int32_t Motion_GetCountR(void);

uint8_t Motion_IsStopped(void);
uint8_t Motion_IsFault(void);
uint8_t Motion_GetFaultCode(void);
void Motion_ClearFault(void);

#endif /* MOTION_H */
