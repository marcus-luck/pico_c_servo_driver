#include <stdio.h>
#include "pico/stdlib.h"

#include "FreeRTOS.h"
#include "queue.h"

#include "uart.h"


#define QUEUE_LENGTH 10
#define QUEUE_ITEM_SIZE sizeof(char[4])
#define QUEUE_TIMEOUT (TickType_t) 1

static QueueHandle_t cmd_queue = NULL;

// Setup command queue
void cmd_queue_init() {
   cmd_queue = xQueueCreate(QUEUE_LENGTH, QUEUE_ITEM_SIZE);
}

// Send command to queue
BaseType_t cmd_queue_send(void * cmd) {
   // printf("Sending command %d.\n", cmd);
   BaseType_t xHigherPriorityTaskWoken = pdFALSE;
   BaseType_t status = xQueueSendToFrontFromISR(
      cmd_queue, 
      cmd,
      // QUEUE_TIMEOUT
      xHigherPriorityTaskWoken
   );
   return status;
}

// Read command from queue
BaseType_t cmd_queue_read(void * cmd) {
		/* Wait until something arrives in the queue - this task will block
		indefinitely provided INCLUDE_vTaskSuspend is set to 1 in
		FreeRTOSConfig.h. */
    BaseType_t status = xQueueReceive(
      cmd_queue,
      cmd,
      portMAX_DELAY
      // xHigherPriorityTaskWoken
   );
   return status;
}

// Check if queue is empty
BaseType_t cmd_queue_not_empty() {
   return (_Bool) uxQueueMessagesWaiting(cmd_queue) != 0;
}

void read_cmd(char s[], char end) {
   for (int i=0; i<MAX_UART_MESSAGE_LEN; i++) {
      char next = uart_getc(UART_ID);
      if (next == UART_END) {
         break;
      }
      s[i] = next;
   }
}


// RX interrupt handler
void on_uart_rx() {
   char cmd[4];
      while (uart_is_readable(UART_ID)) {
            read_cmd(cmd, UART_END);

            cmd_queue_send(&cmd);
      }
}

void init_uart() {

   uart_init(UART_ID, BAUD_RATE);
   // Set the TX and RX pins by using the function select on the GPIO
   // Set datasheet for more information on function select
   gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
   gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
   // Set UART flow control CTS/RTS, we don't want these, so turn them off
   uart_set_hw_flow(UART_ID, false, false);
   // Set our data format
   uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);

    // Turn off FIFO's - we want to do this character by character
    uart_set_fifo_enabled(UART_ID, false);

    // Set up a RX interrupt
    // We need to set up the handler first
    // Select correct interrupt for the UART we are using
    int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;

    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
    irq_set_enabled(UART_IRQ, true);

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(UART_ID, true, false);
}
