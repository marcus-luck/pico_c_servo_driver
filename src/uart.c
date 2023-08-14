#include <stdio.h>

#include "pico/stdlib.h"
#include "uart.h"

#include "FreeRTOS.h"
#include "queue.h"

#define QUEUE_LENGTH 1
#define QUEUE_ITEM_SIZE sizeof(command_frame_t)
#define QUEUE_TIMEOUT 10

QueueHandle_t cmd_queue;

// Setup command queue
void cmd_queue_init() {
   cmd_queue = xQueueCreate(QUEUE_LENGTH, QUEUE_ITEM_SIZE);
}

// Send command to queue
void cmd_queue_send(command_frame_t * cmd) {
   xQueueSendToBack(cmd_queue, (void *) &cmd, QUEUE_TIMEOUT);
}

// Read command from queue
BaseType_t cmd_queue_read(command_frame_t * cmd) {
    return xQueueReceive(cmd_queue, (void *) &cmd, QUEUE_TIMEOUT);
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
   static int chars_rxed = 0;
   char cmd[] = {0,0,0,0};
   while (1) {
        while (uart_is_readable(UART_ID)) {
            printf("UART Receiving.\n");
            read_cmd(cmd, UART_END);

            // parse command

            _Bool rw = cmd[0] >> 7;
            uint8_t cmd_type = cmd[0] & 0x7f;
            uint8_t addr = cmd[1];

            uint16_t high = (int)cmd[2] << 8;
            uint16_t low = (int)cmd[3];
            uint16_t val = high + low;
            printf("rw: %i, cmd: %i, addr: %i, val: %i\n", rw, cmd_type, addr, val);

            command_frame_t cnd;
            cnd.rw = rw;
            cnd.cmd_type = cmd_type;
            cnd.addr = addr;
            cnd.val = val;

            cmd_queue_send(&cnd);
    }
    vTaskDelay(10);
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
