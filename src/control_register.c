// control_register.c

#include "control_register.h"

// Mutexes (Binary Semaphores) for each register
SemaphoreHandle_t register_mutexes[REG_MAX];
uint16_t registers[REG_MAX];

void register_space_init(void) {
    for (int i = 0; i < REG_MAX; i++) {
        registers[i] = 0;
        register_mutexes[i] = xSemaphoreCreateMutex();
        if (register_mutexes[i] == NULL) {
            // Handle error: Mutex creation failed.
            // You could use some assert mechanism or logging here.
            printf("Mutex creation failed.\n");
        }
    }
}

void read_ctrl_register(uint8_t reg, uint16_t value) {
    xSemaphoreTake(register_mutexes[reg], portMAX_DELAY);
    value = registers[reg];
    xSemaphoreGive(register_mutexes[reg]);
}

void write_ctrl_register(uint8_t reg, uint16_t value) {
    if (reg == ID) {
        // Handle error: ID register is read-only.
        // You could use some assert mechanism or logging here.
        printf("ID register is read-only.\n");
        value = 0;
    } else if (reg >= REG_MAX) {
        // Handle error: Register address is out of range.
        // You could use some assert mechanism or logging here.
        printf("Register address is out of range.\n");
        value = 0;
    }
    printf("Writing %d to register %d.\n", value, reg);
    xSemaphoreTake(register_mutexes[reg], portMAX_DELAY);
    registers[reg] = value;
    xSemaphoreGive(register_mutexes[reg]);
}
