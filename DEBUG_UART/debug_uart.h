#ifndef DEBUG_UART_DEBUG_UART_H_
#define DEBUG_UART_DEBUG_UART_H_

#define DEBUG_UART_BUF_TX_SIZE 512
#define DEBUG_UART_BUF_TX_MASK (DEBUG_UART_BUF_TX_SIZE-1)

#define DEBUG_UART_BUF_RX_SIZE 512
#define DEBUG_UART_BUF_RX_MASK (DEBUG_UART_BUF_RX_SIZE-1)

#define DEBUG_ENDL			"\r\n"

#define DEBUG_MODULE_NAME0	"COR"		//core of application
#define DEBUG_MODULE_NAME1	"PWR"		//power management
#define DEBUG_MODULE_NAME2	"INT"		//interrupts
#define DEBUG_MODULE_NAME3	"LOR"		//lora communication
#define DEBUG_MODULE_NAME4	"SIG"		//signalling info

typedef enum T_Debug_module{debug_module_CORE = 0, debug_module_POWER_MANEGEMENT, debug_module_INTTERRUPTS, debug_module_LORA, debug_module_SIGNALLING}T_Debug_module;

typedef enum T_Debug_type{debug_type_INFO, debug_type_IMPORTANT, debug_type_WARNING, debug_type_ERROR}T_Debug_type;

void debug_uart_init();
void debug_uart_putc(char c);
void debug_uart_puts(char * str);
void DEBUG_UART_EVENT(char * buf);
void register_debug_uart_event_callback(void (*callback)(char * buf));

void Debug_log( T_Debug_module module, T_Debug_type type, char * msg, uint8_t endl);

#endif /* DEBUG_UART_DEBUG_UART_H_ */
