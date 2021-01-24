#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f10x.h"

#include "USART_Handler.h"

void USART_HandlerRegisterAsciiLineCallback( T_USART_HANDLER *USART_Handler, void (*callback)(char * buf)){
	USART_Handler->ascii_line_event_callback = callback;
}

//baud
void USART_HandlerInit( T_USART_HANDLER *USART_Handler, USART_TypeDef *USART_periph, char* BUF_TX, char* BUF_RX, uint16_t BUF_TX_SIZE, uint16_t BUF_RX_SIZE, uint32_t periph_freq, uint32_t baudrate){

	USART_Handler->USART_periph = USART_periph;
	USART_Handler->BUF_TX_SIZE = BUF_TX_SIZE;
	USART_Handler->BUF_RX_SIZE = BUF_RX_SIZE;
	USART_Handler->BUF_TX_MASK = (BUF_TX_SIZE-1);
	USART_Handler->BUF_RX_MASK = (BUF_RX_SIZE-1);

	USART_Handler->buf_tx = BUF_TX;
	USART_Handler->buf_rx = BUF_RX;

	USART_Handler->USART_periph->BRR = ((periph_freq)/(baudrate * 16))<<4 | (int)((  ((((float)periph_freq)/((float)baudrate * 16))) - (int)((((float)periph_freq)/((float)baudrate * 16)))  ) *16 );

	//	USART_Handler->USART_periph.BRR |= (69 << 4) | 1;				//set USART1 baud rate to 57600
	USART_Handler->USART_periph->CR1 &= ~(USART_CR1_M);				//set USART1 working mode to 8/1
	USART_Handler->USART_periph->CR1 |= (USART_CR1_RXNEIE);
	USART_Handler->USART_periph->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;				//enable USART1
}

void USART_HandlerPutc( T_USART_HANDLER *USART_Handler, char c){
	uint16_t temporary_head;
	temporary_head = (USART_Handler->head_tx+1) & USART_Handler->BUF_TX_MASK;
	USART_Handler->buf_tx[temporary_head] = c;
	USART_Handler->head_tx = temporary_head;
	USART_Handler->USART_periph->CR1 |= (USART_CR1_TXEIE);
}

int usart_getc( T_USART_HANDLER *USART_Handler ){
	int data = -1;
	if(USART_Handler->tail_rx == USART_Handler->head_rx) return data;
	USART_Handler->tail_rx = (USART_Handler->tail_rx+1) & USART_Handler->BUF_RX_MASK;
	data = USART_Handler->buf_rx[USART_Handler->tail_rx];
	return data;
}

char * usart_getstr(T_USART_HANDLER *USART_Handler, char * buf) {
	int c;
	char * poi = buf;
	if( USART_Handler->ascii_line ) {
		while( (c = usart_getc(USART_Handler)) ) {
			if( 13 == c || c < 0) break;
			*buf++ = c;
		}
		*buf=0;
		USART_Handler->ascii_line--;
	}
	return poi;
}

void USART_HandlerMain( T_USART_HANDLER *USART_Handler, char * buf){

	if( USART_Handler->ascii_line ) {
		if( USART_Handler->ascii_line_event_callback ) {
			usart_getstr(USART_Handler, buf );
			(*USART_Handler->ascii_line_event_callback)( buf );
		} else {
			USART_Handler->head_rx = USART_Handler->tail_tx;
		}
	}
}

void USART_HandlerPuts( T_USART_HANDLER *USART_Handler, char * str){
	for(uint32_t i = 0; i<strlen(str); i++){
		USART_HandlerPutc(USART_Handler, str[i]);
	}
}

void USART_HandlerPutint( T_USART_HANDLER *USART_Handler, int value, uint8_t radix){
	char buf[32];
	itoa(value,buf,radix);
	USART_HandlerPuts(USART_Handler, buf);
}

void USART_Handler_IRQ( T_USART_HANDLER *USART_Handler ){
	//TRANSMIT COMPLETE
	if(USART_Handler->USART_periph->SR & USART_SR_TXE){
		if(USART_Handler->tail_tx != USART_Handler->head_tx){
			USART_Handler->tail_tx = (USART_Handler->tail_tx+1) & USART_Handler->BUF_RX_MASK;
			USART_Handler->USART_periph->DR = USART_Handler->buf_tx[USART_Handler->tail_tx];
		}else{ USART_Handler->USART_periph->CR1 &= ~(USART_CR1_TXEIE); }
		USART_Handler->USART_periph->SR &= ~USART_SR_TXE;				//reset interrupt flag
	}

	//DATA RECEIVED
	if(USART_Handler->USART_periph->SR & USART_SR_RXNE){
		char data = USART_Handler->USART_periph->DR;
		uint32_t temporary_head = (USART_Handler->head_rx+1) & USART_Handler->BUF_RX_MASK;
		if(temporary_head == USART_Handler->tail_rx){
			//overflow
			USART_Handler->head_rx = USART_Handler->tail_rx;
		}else{

			switch(data){
			case 0: break;
			case 10: break;// break;
			case 13: USART_Handler->ascii_line++;break;
			default: USART_Handler->head_rx = temporary_head; USART_Handler->buf_rx[temporary_head] = data; break;
			}
		}

		USART_Handler->USART_periph->SR &= ~USART_SR_RXNE;				//reset interrupt flag
	}

}
