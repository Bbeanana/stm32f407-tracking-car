#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdint.h>

#include "main.h"

/**
 * @brief HC-SR04 超声波测距（TIM 输入捕获，非阻塞）
 *
 * 接线：TRIG = 普通 GPIO 输出；ECHO = 定时器输入捕获通道。
 * 本工程默认 TRIG=PA1，ECHO=PA0/TIM5_CH1，TIM5 以 1MHz 计数。
 *
 * 使用：Ultrasonic_Update() 每 5ms 调用一次；距离更新后
 *       Ultrasonic_GetDistanceCm() 读取，Ultrasonic_IsValid() 判断有效。
 */
typedef struct {
    GPIO_TypeDef *trig_port;
    uint16_t trig_pin;
    TIM_HandleTypeDef *echo_htim;
    uint32_t echo_channel;
} Ultrasonic_Config_t;

typedef struct {
    Ultrasonic_Config_t cfg;
    volatile uint32_t rise_cnt;
    volatile uint32_t width_us;
    volatile uint8_t state;       /* 0=空闲，1=等上升沿，2=等下降沿 */
    volatile uint8_t new_data;
    float distance_cm;
    uint8_t valid;
    uint32_t last_trigger_tick;
    uint8_t inited;
} Ultrasonic_t;

void Ultrasonic_Init(Ultrasonic_t *ultra, const Ultrasonic_Config_t *cfg);
void Ultrasonic_Update(Ultrasonic_t *ultra);
uint8_t Ultrasonic_HasNewData(Ultrasonic_t *ultra);
float Ultrasonic_GetDistanceCm(const Ultrasonic_t *ultra);
uint8_t Ultrasonic_IsValid(const Ultrasonic_t *ultra);

#endif /* ULTRASONIC_H */
