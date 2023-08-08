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


#define LED_PIN      3
#define PWM_PIN      4
#define PWM_PIN2     5

#define UART_ID      uart1
#define BAUD_RATE    115200
#define DATA_BITS    8
#define STOP_BITS    1
#define PARITY       UART_PARITY_NONE

#define UART_TX_PIN  8
#define UART_RX_PIN  9

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

QueueHandle_t xQueue;

uint16_t cdiv = 12000;
uint16_t pwm_val = 12000;


void vRunPWMTask() {
   pwm_set_gpio_level(PWM_PIN, cdiv);
   for (;;) {
      if (xQueueReceive(xQueue, &pwm_val, 0) == pdTRUE) {
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




void print_cmd(char s[], int len) {
   for (int i=0; i<len; i++) {
      printf("%i:", (int)s[i]);
   }
   printf("\n");
}


// RX interrupt handler
void on_uart_rx() {
   static int chars_rxed = 0;
   char cmd[] = {0,0,0,0};
    while (uart_is_readable(UART_ID)) {
      while (chars_rxed < 4) {
         cmd[chars_rxed] = uart_getc(UART_ID);
         chars_rxed++;
      }
      chars_rxed=0;
      print_cmd(cmd, sizeof(cmd));

      uint16_t high = (int)cmd[2] << 8;
      uint16_t low = (int)cmd[3];
      uint16_t val = high + low;
      printf("cdiv: (%i) %i + %i = %i\n", (int)cmd[2], high, low, val);
      xQueueSendToFront(xQueue, &val, 0);
    }
}

void main() {
   // Initialize the Queue

   xQueue =  xQueueCreate( 1, sizeof( uint16_t ) );

   // Set up our UART with the required speed.
   stdio_init_all();
   printf("Hello, world!\n");
   uart_init(UART_ID, BAUD_RATE);
   // Set the TX and RX pins by using the function select on the GPIO
   // Set datasheet for more information on function select
   gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
   gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
   // Set UART flow control CTS/RTS, we don't want these, so turn them off
   uart_set_hw_flow(UART_ID, false, false);
   // Set our data format
   uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);
    // Set up a RX interrupt
    // We need to set up the handler first
    // Select correct interrupt for the UART we are using
    int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;

    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
    irq_set_enabled(UART_IRQ, true);

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(UART_ID, true, false);

   // Setup pin for blink
   gpio_init(LED_PIN);
   gpio_set_dir(LED_PIN, GPIO_OUT);

   // Setup pin for PWM static
   // gpio_init(PWM_PIN2);
   pwm_output_init(PWM_PIN, 4.f, 32000);

   // Setup pins for PWM
   pwm_output_init(PWM_PIN2, 4.f, 32000);
   // Create tasks
   xTaskCreate(vBlinkTask, "Blink Task", 128, NULL, 2, NULL);
   xTaskCreate(vPWMTask, "PWM Task", 128, NULL, 1, NULL);

   xTaskCreate(vRunPWMTask, "Run PWM Task", 128, NULL, 3, NULL);

   vTaskStartScheduler();
   // The code will neverreach this point unless something is wrong
    while (1)
        tight_loop_contents();
}