#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f10x.h"

#include "debug_uart.h"

static void (*debug_uart_event_callback)(char * buf);

void register_debug_uart_event_callback(void (*callback)(char * buf)) {
	debug_uart_event_callback = callback;
}

char debug_uart_buf_tx[DEBUG_UART_BUF_TX_SIZE];
uint32_t debug_uart_head_tx;
uint32_t debug_uart_tail_tx;

char debug_uart_buf_rx[DEBUG_UART_BUF_RX_SIZE];
uint32_t debug_uart_head_rx;
volatile uint32_t debug_uart_tail_rx;

uint32_t debug_uart_ascii_line;

char Debug_modules[5][4] = {
		{DEBUG_MODULE_NAME0},
		{DEBUG_MODULE_NAME1},
		{DEBUG_MODULE_NAME2},
		{DEBUG_MODULE_NAME3},
		{DEBUG_MODULE_NAME4}
};

char Debug_type0[] = "INFO";
char Debug_type1[] = "IMPORTANT";
char Debug_type2[] = "WARNING";
char Debug_type3[] = "ERROR";

char * Debug_types[4]={
		Debug_type0, Debug_type1, Debug_type2, Debug_type3
};

void Debug_log( T_Debug_module module, T_Debug_type type, char * msg, uint8_t endl){
	if(type == debug_type_IMPORTANT)debug_uart_puts("\033[1;32m");
	if(type == debug_type_ERROR)debug_uart_puts("\033[1;31m");
	if(type == debug_type_WARNING)debug_uart_puts("\033[1;93m");
	debug_uart_putc('[');
	debug_uart_puts(Debug_modules[module]);
	debug_uart_puts("] [");
	debug_uart_puts(Debug_types[type]);
	debug_uart_puts("] >");
	debug_uart_puts(msg);
	debug_uart_puts("\033[0m");
	if(endl) debug_uart_puts(DEBUG_ENDL);
}

void debug_uart_init(){
	RCC->APB2ENR |= (RCC_APB2ENR_USART1EN);		//enable Clock for USART1 periph
	USART1->BRR |= (69 << 4) | 1;				//set USART1 baud rate to 57600
	USART1->CR1 &= ~(USART_CR1_M);				//set USART1 working mode to 8/1
	USART1->CR1 |= (USART_CR1_RXNEIE);
	USART1->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;				//enable USART1
}

void debug_uart_putc(char c){
	uint32_t temporary_head;
	temporary_head = (debug_uart_head_tx+1) & DEBUG_UART_BUF_TX_MASK;
	debug_uart_buf_tx[temporary_head] = c;
	debug_uart_head_tx = temporary_head;
	USART1->CR1 |= (USART_CR1_TXEIE);
}

int debug_uart_getc(){
	int data = -1;
	if(debug_uart_tail_rx == debug_uart_head_rx) return data;
	debug_uart_tail_rx = (debug_uart_tail_rx+1) & DEBUG_UART_BUF_RX_MASK;
	data = debug_uart_buf_rx[debug_uart_tail_rx];
	return data;
}

char * debug_uart_gstr(char * buf) {
	int c;
	char * poi = buf;
	if( debug_uart_ascii_line ) {
		while( (c = debug_uart_getc()) ) {
			if( 13 == c || c < 0) break;
			*buf++ = c;
		}
		*buf=0;
		debug_uart_ascii_line--;
	}
	return poi;
}

void DEBUG_UART_EVENT(char * rbuf) {

	if( debug_uart_ascii_line ) {
		if( debug_uart_event_callback ) {
			debug_uart_gstr( rbuf );
			(*debug_uart_event_callback)( rbuf );
		} else {
			debug_uart_head_rx = debug_uart_tail_tx;
		}
	}
}

void debug_uart_puts(char * str){
	for(uint32_t i = 0; i<strlen(str); i++){
		debug_uart_putc(str[i]);
	}
}

void USART1_IRQHandler(void){

	//TRANSMIT COMPLETE
	if(USART1->SR & USART_SR_TXE){
		if(debug_uart_tail_tx != debug_uart_head_tx){
			debug_uart_tail_tx = (debug_uart_tail_tx+1) & DEBUG_UART_BUF_RX_MASK;
			USART1->DR = debug_uart_buf_tx[debug_uart_tail_tx];
		}else{ USART1->CR1 &= ~(USART_CR1_TXEIE); }
		USART1->SR &= ~USART_SR_TXE;				//reset interrupt flag
	}

	//DATA RECEIVED
	if(USART1->SR & USART_SR_RXNE){
		char data = USART1->DR;
		uint32_t temporary_head = (debug_uart_head_rx+1) & DEBUG_UART_BUF_RX_MASK;
		if(temporary_head == debug_uart_tail_rx){
			//overflow
			debug_uart_head_rx = debug_uart_tail_rx;
		}else{
			switch(data){
			case 0: break;
			case 10: break;// break;
			case 13: debug_uart_ascii_line++;break;
			default: debug_uart_head_rx = temporary_head; debug_uart_buf_rx[temporary_head] = data; break;
			}
		}

		USART1->SR &= ~USART_SR_RXNE;				//reset interrupt flag
	}

}
