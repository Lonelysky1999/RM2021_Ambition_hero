#ifndef ADC_H
#define ADC_H
#include "main.h"

extern void temperature_ADC_init(void);
extern fp32 get_temprate(void);

uint16_t get_ADC(uint8_t ch);
#endif
