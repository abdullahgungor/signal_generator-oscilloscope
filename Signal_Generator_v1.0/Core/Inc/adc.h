#ifndef __ADC_H
#define __ADC_H

#include "stm32f1xx_hal.h"

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim3;
void MX_ADC1_Init(void);
void startADC(void);
extern uint16_t adcValue[10];

#endif
