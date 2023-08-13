#include <stdio.h>

#include "pico/stdlib.h"
#include "uart.h"

#include "FreeRTOS.h"
#include "queue.h"

QueueHandle_t queue;

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

    while (uart_is_readable(UART_ID)) {
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

        xQueueSendToFront(queue, &cnd, 0);
    }
}

void init_uart(QueueHandle_t * q) {

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

    queue = q;

    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
    irq_set_enabled(UART_IRQ, true);

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(UART_ID, true, false);
}