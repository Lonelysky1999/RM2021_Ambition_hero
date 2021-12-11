#ifndef LED_TASK_H
#define LED_TASK_H
#include "main.h"

extern void LEDTask(void *pvParameters);
extern void set_offset_cail_is_ok(void);
extern void amb_write_flash(void);
extern uint8_t return_mpu_offset_is_ok(void);
#endif


