// control_register.c

#include "register_space.h"

// Mutexes (Binary Semaphores) for each register
SemaphoreHandle_t register_mutexes[REG_MAX];

void register_space_init(void) {
    for (int i = 0; i < REG_MAX; i++) {
        register_mutexes[i] = xSemaphoreCreateMutex();
        if (register_mutexes[i] == NULL) {
            // Handle error: Mutex creation failed.
            // You could use some assert mechanism or logging here.
        }
    }
}

uint16_t read_ctrl_register(CTRL_REGISTER_T reg) {
    xSemaphoreTake(register_mutexes[reg], portMAX_DELAY);
    uint16_t value = *((volatile uint16_t*)(USER_REGISTER_BASE + reg * sizeof(uint16_t)));
    xSemaphoreGive(register_mutexes[reg]);
    return value;
}

void write_user_register(CTRL_REGISTER_T reg, uint16_t value) {
    xSemaphoreTake(register_mutexes[reg], portMAX_DELAY);
    *((volatile uint16_t*)(USER_REGISTER_BASE + reg * sizeof(uint16_t))) = value;
    xSemaphoreGive(register_mutexes[reg]);
}
