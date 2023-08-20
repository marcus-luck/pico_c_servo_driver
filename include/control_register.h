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
    INT_CODE,
    INT_VALUE,
    REG_MAX
} CTRL_REGISTER_T;

// #define USER_REGISTER_BASE (0x40000000)  // hypothetical base address

void register_space_init(void);
uint16_t read_ctrl_register(CTRL_REGISTER_T reg);
void write_user_register(CTRL_REGISTER_T reg, uint16_t value);

#endif // REGISTER_SPACE_H