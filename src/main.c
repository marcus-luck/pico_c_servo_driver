/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Fade an LED between low and high brightness. An interrupt handler updates
// the PWM slice's output level each time the counter wraps.

#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include <stdio.h>
#include "pico/time.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "uart.h"
#include "control_register.h"

#define argh GPIO_FUNC_UART


#define PWM_PIN0     2
#define PWM_PIN1     3
#define PWM_PIN2     4
#define PWM_PIN3     5
#define PWM_PIN4     6
#define PWM_PIN5     7
#define PWM_PIN6     10
#define PWM_PIN7     11

#define LEN_LED_BANK 8
int LED_BANK[] = {
   PWM_PIN0,
   PWM_PIN1,
   PWM_PIN2,
   PWM_PIN3,
   PWM_PIN4,
   PWM_PIN5,
   PWM_PIN6,
   PWM_PIN7
   };


// Setup pin for PWM static
// Lets assume 2:4:2 in 3 banks.
// bank 1: 2, 3,
#define LEN_BANK1 2
#define LEN_BANK2 4
#define LEN_BANK3 2
uint8_t bank1[2] = {PWM_PIN0, PWM_PIN1};
// bank 2: 4, 5, 6, 7
uint8_t bank2[4] = {PWM_PIN2, PWM_PIN3, PWM_PIN4, PWM_PIN5};
// bank 3: 10, 11
uint8_t bank3[2] = {PWM_PIN6, PWM_PIN7};

// divider = Ceil(125000000/(4096*50))/16 = 611/16 = 38.1875
#define CLKDIV (float)38.1875 // 50 Hz
// divider = Ceil(125000000/(4096*10))/16 = 611/16 = 190.75
#define SYNC1 (float)190.75 // 10 Hz

QueueHandle_t pwmQueue;
uint16_t pwm_val = 0;

void vApplicationMallocFailedHook( void )
{
    /* Called if a call to pvPortMalloc() fails because there is insufficient
    free memory available in the FreeRTOS heap.  pvPortMalloc() is called
    internally by FreeRTOS API functions that create tasks, queues, software
    timers, and semaphores.  The size of the FreeRTOS heap is set by the
    configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

    /* Force an assert. */
    configASSERT( ( volatile void * ) NULL );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
    ( void ) pcTaskName;
    ( void ) pxTask;

    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected. */

    /* Force an assert. */
    configASSERT( ( volatile void * ) NULL );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
    volatile size_t xFreeHeapSpace;

    /* This is just a trivial example of an idle hook.  It is called on each
    cycle of the idle task.  It must *NOT* attempt to block.  In this case the
    idle task just queries the amount of FreeRTOS heap that remains.  See the
    memory management section on the http://www.FreeRTOS.org web site for memory
    management options.  If there is a lot of heap memory free then the
    configTOTAL_HEAP_SIZE value in FreeRTOSConfig.h can be reduced to free up
    RAM. */
    xFreeHeapSpace = xPortGetFreeHeapSize();
   printf("FreeHeap: %i\n", xFreeHeapSpace);
    /* Remove compiler warning about xFreeHeapSpace being set but never used. */
    ( void ) xFreeHeapSpace;
}
/*-----------------------------------------------------------*/




void vRunPWMTask() {
   uint16_t ledctrl[2];
   uint16_t pval;
   uint16_t led_id;
   for (;;) {
      if (xQueueReceive(pwmQueue, &ledctrl, portMAX_DELAY) == pdTRUE) {
         led_id = ledctrl[0] + 1;
         pval = ledctrl[1];
         // read_ctrl_register(led_id, &pwm_val);
         printf("Setting LED %i to %i\n", led_id, pval);
         write_ctrl_register(led_id, &pval);
         // if (led_id == 0xFF) {
         //    for (int i=0; i<LEN_LED_BANK; i++) {
         //       pwm_set_gpio_level(LED_BANK[i], pwm_val);
         //    }
         // } else {
         //    pwm_set_gpio_level(LED_BANK[led_id], pwm_val);
         // }
      }
   }
}


void pwm_output_init(int pwm_pin, float clkdiv, int level) {
   gpio_set_function(pwm_pin, GPIO_FUNC_PWM);
   uint slice_num = pwm_gpio_to_slice_num(pwm_pin);
   pwm_config config = pwm_get_default_config();
   pwm_config_set_clkdiv(&config, clkdiv);
   pwm_init(slice_num, &config, true);
}


void pwm_output_off(uint8_t pwm_pin) {

   pwm_set_gpio_level(pwm_pin, 0);
   // Set GPIO_BANK_enable to 0
}

void pwm_output_on(uint8_t pwm_pin) {

   read_ctrl_register_isr(pwm_pin, &pwm_val); /* TODO: This shouldn't be pwm_pin, it should be led_num, starting from 0-> max amount of leds. Then using enum to convert to output pin. */
   pwm_set_gpio_level(pwm_pin, pwm_val);
   // Set GPIO_BANK_enable to 1
}

void pwm_bank_on(uint8_t * bank, int len) {

   for (int i=0; i<len; i++) {
      pwm_output_on(bank[i]);
   }
}

void pwm_bank_off(uint8_t * bank, int len) {
   for (int i=0; i<len; i++) {
      pwm_output_off(bank[i]);
   }
}

/* State machine ************************************************************/
_Bool is_on = 0;

unsigned long time = 0;
unsigned long button_debounce_time = 0;
const int delayTime = 100; // Half a second debounce
const int buttonDelayTime = 500; // Half a second debounce

int led_state = 0;
uint16_t ledc[2] = {0xFF, 0};

volatile TimerHandle_t state_timer;

void state_machine();
void reset_state_machine();
void stop_state_machine();
void start_state_machine();
void create_state_timer();

/** Stop state machine
 * @brief Stop the running statemachine, move the state to OFF
 * 
 */
void stop_state_machine() {
   if (is_on == 0) {
      return;
   };
   // Stop the timer
   // led_state = 4; // STATE_OFF;
   xTimerStop(state_timer, 0);
   // Turn off all LEDs
   led_state = 4;  // STATE_OFF;
   state_machine();
   is_on = 0;
}

/** Start the state machine
 * @brief Start the state machine
 * 
*/
void start_state_machine() {
   if (is_on == 1) {
      return;
   };
   
   // Start the timer
   reset_state_machine();
   if (state_timer != NULL) xTimerStart(state_timer, 0);
   is_on = 1;
}

/** Reset the state machine
 * @brief Reset the state machine
 * 
*/
void reset_state_machine() {
   if (is_on == 1) {
      stop_state_machine();
      led_state = 0; // STATE_LED1;
      start_state_machine();
   } else {
      led_state = 0; // STATE_LED1;
   }
}

void create_state_timer() {
   state_timer = xTimerCreate(
      "Timer",
      pdMS_TO_TICKS(100),
      pdTRUE,
      NULL,
      &state_machine
   );
}

// void gpio_callback(uint gpio, uint32_t events) {
void gpio_callback() {
    if ((to_ms_since_boot(get_absolute_time()) - button_debounce_time) >= buttonDelayTime) {
        // Recommend to not to change the position of this line
        button_debounce_time = to_ms_since_boot(get_absolute_time());
         if (is_on == 1) {
            stop_state_machine();
            // is_on = 0;
         } else {
            start_state_machine();
            // is_on = 1;
         }
         // irq_clear(PWM_IRQ_WRAP);
         // state_machine();
    }

   // irq_clear(PWM_IRQ_WRAP);
}


void state_machine() {
   //  if ((to_ms_since_boot(get_absolute_time())-time) >= delayTime) {
        // Recommend to not to change the position of this line
      //   time = to_ms_since_boot(get_absolute_time());

      if (led_state == 0) {
         // printf("%d: Turning on LED265\n", time);
         pwm_bank_on(bank1, LEN_BANK1);
         led_state = 1;
      } else if (led_state == 1) {
         // printf("Turning on LED280\n");
         pwm_bank_off(bank1, LEN_BANK1);
         pwm_bank_on(bank2, LEN_BANK2);
         led_state = 2;
      } else if (led_state == 2) {
         // printf("Turning on LED365\n");
         pwm_bank_off(bank2, LEN_BANK2);
         pwm_bank_on(bank3, LEN_BANK3);
         led_state = 3;
      } else if (led_state == 3) {
         // printf("Turning off LED\n");
         pwm_bank_off(bank3, LEN_BANK3);

         led_state = 0;
      } else if (led_state == 4) {
         printf("Turning off LED\n");
         // Off state, the statemachine remains off until reset.
         pwm_bank_off(bank1, LEN_BANK1);
         pwm_bank_off(bank2, LEN_BANK2);
         pwm_bank_off(bank3, LEN_BANK3);
         led_state = 4;
      } else {
         printf("Turning off LED\n");
         for(int i=0; i<LEN_LED_BANK; i++){
            pwm_output_off(LED_BANK[i]);
         }
         led_state = 0;
      }
   // }
}

/* END State machine ****************************************************** */
void vParseCommandTask() {
   char cmd[4];

   for (;;) {
      if (cmd_queue_read(&cmd) == pdTRUE) {

         // Parse the command

         _Bool rw = cmd[0] >> 7;
         uint8_t cmd_type = cmd[0] & 0x7f;
         uint8_t addr = cmd[1];

         uint16_t high = (int)cmd[2] << 8;
         uint16_t low = (int)cmd[3];
         uint16_t val = high + low;
         // printf("rw: %i, cmd: %i, addr: %i, val: %i\n", rw, cmd_type, addr, val);

         if (cmd_type == 0) {
            uint16_t ledctrl[2] = {(uint16_t)addr, val};
            uint8_t lednum = 1 + addr;
            // write_ctrl_register(lednum, &val);
            xQueueSendToFront(pwmQueue, &ledctrl, 0);
         } else if (cmd_type == 1) {
            uint8_t pin = val & 0xF;
            pwm_output_on(pin);
         } else if (cmd_type == 2) {
            uint8_t pin = val & 0xF;
            pwm_output_off(pin);
         }
      }
   }
}


// Misc


void main() {
   // Initialize the Queue
   pwmQueue =  xQueueCreate( 12, sizeof( uint16_t[2] ) );
   cmd_queue_init();
   // Set up our UART with the required speed.
   stdio_init_all();
   register_space_init();
   printf("#### Starting program ####\n");

   // measure_freqs();

   init_uart();

   time = to_ms_since_boot(get_absolute_time());
   printf("Setting up pins\n");
   for (int i=0; i<LEN_LED_BANK; i++) {
      gpio_init(LED_BANK[i]);
      gpio_set_dir(LED_BANK[i], GPIO_OUT);
      pwm_output_init(LED_BANK[i], CLKDIV, 0);
   }

   u_int16_t defval = 25000;
   for (int i=0; i<LEN_LED_BANK; i++) {
      write_ctrl_register(LED_BANK[i], &defval);
      pwm_set_gpio_level(LED_BANK[i], 0);
   }

   // Add ISR for push button
   gpio_init(28);
   gpio_set_dir(28, GPIO_IN);
   gpio_set_irq_enabled_with_callback(28, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);

   // Create tasks
   printf("Creating tasks\n");
   create_state_timer();

   xTaskCreate(vRunPWMTask, "Run PWM Task", 256, NULL, 5, NULL);
   xTaskCreate(vParseCommandTask, "Parser Task", 256, NULL, 4, NULL);
   start_state_machine();

   vTaskStartScheduler();
   // The code will neverreach this point unless something is wrong
    while (1)
        tight_loop_contents();
}