#ifndef CURRENT_SENSE_H
#define CURRENT_SENSE_H

#include <stdint.h>

void CurrentSense_Init(void);
float CurrentSense_ReadVolts(void);
uint16_t CurrentSense_ReadRaw(void);

#endif /* CURRENT_SENSE_H */
