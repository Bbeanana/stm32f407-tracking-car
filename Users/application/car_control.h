#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H

#include <stdint.h>

typedef enum {
    CAR_STATE_INIT = 0,
    CAR_STATE_LINE,
    CAR_STATE_SEARCH,
    CAR_STATE_STOP,       /* 停车：含超声波避障停车与丢线超时停车 */
    CAR_STATE_FAULT,
} CarState_t;

void Car_Init(void);
void Car_Loop(float dt_s);

CarState_t Car_GetState(void);
const char *Car_GetStateName(void);
uint8_t Car_IsObstacle(void);

uint8_t Car_GetTrackMask(void);
uint8_t Car_GetTrackRawMask(void);
float Car_GetTrackError(void);
float Car_GetDistanceCm(void);
uint8_t Car_GetUltrasonicValid(void);

#endif /* CAR_CONTROL_H */
