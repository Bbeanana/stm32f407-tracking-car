#ifndef ADC_H
#define ADC_H

#include <stdint.h>

#include "main.h"

void MX_ADC1_Init(void);
uint16_t ADC1_ReadChannel10(void);

#endif /* ADC_H */
