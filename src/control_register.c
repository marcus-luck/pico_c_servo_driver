// control_register.c

#include "control_register.h"

// Mutexes (Binary Semaphores) for each register
SemaphoreHandle_t register_mutexes[REG_MAX];
uint16_t registers[REG_MAX];

/**
 * @brief Initializes the control register space and creates mutexes.
 * 
 * Each register is initialized to 0 and a mutex is created for mutual exclusion.
 */
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

/**
 * @brief Reads the value from the specified register.
 * 
 * The function takes the mutex associated with the register, reads its value,
 * and then releases the mutex.
 * 
 * @param reg The register index.
 * @param value Pointer to store the read value.
 */
void read_ctrl_register(uint8_t reg, uint16_t * value) {
    xSemaphoreTake(register_mutexes[reg], portMAX_DELAY);
    *value = registers[reg];
    xSemaphoreGive(register_mutexes[reg]);
}

/** Read ctrl register from an ISR
 * @brief Reads the value from the specified register.
 * 
 * The function takes the mutex associated with the register, reads its value,
 * and then releases the mutex.
 * 
 * @param reg The register index.
 * @param value Pointer to store the read value.
 */
void read_ctrl_register_isr(uint8_t reg, uint16_t * value) {
    xSemaphoreTakeFromISR(register_mutexes[reg], NULL);
    *value = registers[reg];
    xSemaphoreGiveFromISR(register_mutexes[reg], NULL);
}

/**
 * @brief Writes a value to the specified register.
 * 
 * The function checks for read-only registers and valid register indices.
 * If valid, it takes the mutex, writes the value, and releases the mutex.
 * 
 * @param reg The register index.
 * @param value Pointer to the value to be written.
 */
void write_ctrl_register(uint8_t reg, uint16_t *value) {
    if (reg >= REG_MAX) {
        // Handle error: Register address is out of range.
        // You could use some assert mechanism or logging here.
        printf("Register address is out of range.\n");
        * value = 0;
    } else if (reg == ID) {
        // Handle error: ID register is read-only.
        // You could use some assert mechanism or logging here.
        printf("ID register is read-only.\n");
        * value = 0;
    } 
    printf("Writing %u to register %u.\n", *value, reg);
    xSemaphoreTake(register_mutexes[reg], portMAX_DELAY);
    registers[reg] = *value;
    xSemaphoreGive(register_mutexes[reg]);
}
