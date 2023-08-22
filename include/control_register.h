// register_space.h

#ifndef CONTROL_REGISTER_H
#define CONTROL_REGISTER_H

// #include <stdint.h>
#include "FreeRTOS.h"
#include "semphr.h"

typedef enum {
    ID = 0,
    PWM0,
    PWM1,
    PWM2,
    PWM3,
    PWM4,
    PWM5,
    PWM6,
    PWM7,
    INT_CODE,
    INT_VALUE,
    REG_MAX
} CTRL_REGISTER_T;

void register_space_init(void);
void read_ctrl_register(uint8_t reg, uint16_t * value);
void write_ctrl_register(uint8_t reg, uint16_t * value);

#endif // CONTROL_REGISTER_H