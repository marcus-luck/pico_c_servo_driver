
#define UART_TX_PIN  8
#define UART_RX_PIN  9

#define UART_ID      uart1
#define BAUD_RATE    115200
#define DATA_BITS    8
#define STOP_BITS    1
#define PARITY       UART_PARITY_NONE

#define MAX_UART_MESSAGE_LEN     4
#define UART_END                 (int)'\n'

// Define the command frame
typedef struct CommandFrame {
   _Bool rw;
   uint8_t id;
   uint8_t cmd_type;
   uint8_t addr;
   uint16_t val;
} command_frame_t;
