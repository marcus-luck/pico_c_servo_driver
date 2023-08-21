// register_space.h

#ifndef REGISTER_SPACE_H
#define REGISTER_SPACE_H

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

// #define USER_REGISTER_BASE (0x40000000)  // hypothetical base address

void register_space_init(void);
void read_ctrl_register(uint8_t reg, uint16_t value);
void write_ctrl_register(uint8_t reg, uint16_t value);

#endif // REGISTER_SPACE_H