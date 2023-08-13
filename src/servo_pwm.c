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
#include <stdio.h>
#include "pico/time.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "uart.h"

#define argh GPIO_FUNC_UART


#define LED_PIN      3
#define PWM_PIN      4
#define PWM_PIN2     5


void vPWMTask() {
    static int fade = 0;
    static bool going_up = true;

   for (;;) {
      if (going_up) {
         ++fade;
         if (fade > 255) {
            fade = 255;
            going_up = false;
         }
      } else {
         --fade;
         if (fade < 0) {
            fade = 0;
            going_up = true;
         }
      }

      pwm_set_gpio_level(PWM_PIN2, fade * fade);

      vTaskDelay(10);
   }
}

void vBlinkTask() {

   for (;;) {

      gpio_put(LED_PIN, 1);

      vTaskDelay(250);

      gpio_put(LED_PIN, 0);

      vTaskDelay(250);

   }
}

QueueHandle_t cmd_queue;
QueueHandle_t pwmQueue;

uint16_t cdiv = 12000;
uint16_t pwm_val = 12000;


void vRunPWMTask() {
   pwm_set_gpio_level(PWM_PIN, cdiv);
   for (;;) {
      if (xQueueReceive(pwmQueue, &pwm_val, 0) == pdTRUE) {
         printf("Received: %d\n", pwm_val);
         pwm_set_gpio_level(PWM_PIN, pwm_val);
      }
      vTaskDelay(10);  
   }
   // while (1)
   //    tight_loop_contents();
}


void pwm_output_init(int pwm_pin, float clkdiv, int level) {
   gpio_set_function(pwm_pin, GPIO_FUNC_PWM);
   uint slice_num = pwm_gpio_to_slice_num(pwm_pin);
   pwm_config config = pwm_get_default_config();
   pwm_config_set_clkdiv(&config, clkdiv);
   pwm_init(slice_num, &config, true);
   pwm_set_gpio_level(pwm_pin, level);
   // pwm_set_enabled(slice_num, true);
}


void pwm_output_off(uint8_t pwm_pin) {
   uint slice_num = pwm_gpio_to_slice_num(pwm_pin);
   pwm_set_gpio_level(pwm_pin, 0);
   // pwm_set_enabled(slice_num, false);
}

void pwm_output_on(uint8_t pwm_pin) {
   uint slice_num = pwm_gpio_to_slice_num(pwm_pin);
   pwm_set_gpio_level(pwm_pin, pwm_val);
   // pwm_set_enabled(slice_num, true);
}

void vParseCommandTask() {
   command_frame_t cmdbuff;
   uint8_t cmd_type;
   uint16_t val;
   for (;;) {
      if (xQueueReceive(cmd_queue, &cmdbuff, 100) == pdTRUE) {
         cmd_type = cmdbuff.cmd_type;
         val = cmdbuff.val;
         if (cmd_type == 0) {
            printf("setting cdiv\n");
            xQueueSendToFront(pwmQueue, &val, 0);
         } else if (cmd_type == 1) {
            uint8_t pin = val & 0xF;
            printf("turning on %i\n", pin);
            pwm_output_on(pin);
         } else if (cmd_type == 2) {
            uint8_t pin = val & 0xF;
            printf("turning off %i\n", pin);
            pwm_output_off(pin);
         }
      }
      vTaskDelay(10);  
   }
      // xQueueSendToFront(xQueue, &val, 0);
}

void main() {
   // Initialize the Queue
   pwmQueue =  xQueueCreate( 1, sizeof( uint16_t ) );
   cmd_queue = xQueueCreate( 1, sizeof( command_frame_t ) );
   // Set up our UART with the required speed.
   stdio_init_all();
   printf("Hello, world!\n");

   init_uart(cmd_queue);

   // task parse commands


   // Setup pin for blink
   gpio_init(LED_PIN);
   gpio_set_dir(LED_PIN, GPIO_OUT);

   int pin_bank[] = {PWM_PIN, PWM_PIN2};

   // Setup pin for PWM static
   // gpio_init(PWM_PIN2);
   pwm_output_init(pin_bank[0], 4.f, 32000);

   // Setup pins for PWM
   pwm_output_init(pin_bank[1], 4.f, 32000);
   // Create tasks
   xTaskCreate(vBlinkTask, "Blink Task", 128, NULL, 2, NULL);
   xTaskCreate(vPWMTask, "PWM Task", 128, NULL, 1, NULL);

   xTaskCreate(vRunPWMTask, "Run PWM Task", 128, NULL, 3, NULL);
   xTaskCreate(vParseCommandTask, "Parser Task", 128, NULL, 4, NULL);

   vTaskStartScheduler();
   // The code will neverreach this point unless something is wrong
    while (1)
        tight_loop_contents();
}