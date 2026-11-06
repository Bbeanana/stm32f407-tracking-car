#ifndef CAMERA_H
#define CAMERA_H

#include <stdint.h>

/**
 * @brief 摄像头 UART 接收（USART1）
 *
 * 帧格式（14 字节）：
 *   A5 | ERR(float32, 小端) | ANG(float32, 小端) | CURV(float32, 小端) | 5A
 *
 * ERR: -1000~+1000，正值表示线在画面右侧。
 * ANG: 航向角（度），正值表示远处线偏右。
 * CURV: 前后两段线倾角差（度），正值表示前方右弯。
 *
 * Camera_Update() 由主循环周期调用；串口接收在 USART1 中断里完成。
 */
void Camera_Init(void);
void Camera_Update(void);

uint8_t Camera_IsValid(void);
float Camera_GetError(void);          /* 已归一化到 -1~+1 */
float Camera_GetAngleDeg(void);       /* 航向角，单位度 */
float Camera_GetCurvatureDeg(void);   /* 曲率角差，单位度 */
uint32_t Camera_GetLastPacketTick(void);

/* 供 USART1_IRQHandler 调用 */
void Camera_UartIrqHandler(void);

#endif /* CAMERA_H */
