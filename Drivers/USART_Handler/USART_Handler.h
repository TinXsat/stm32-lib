#ifndef DRIVERS_USART_HANDLER_USART_HANDLER_H_
#define DRIVERS_USART_HANDLER_USART_HANDLER_H_


typedef struct T_USART_HANDLER{
	USART_TypeDef *USART_periph;

	uint16_t BUF_TX_SIZE;
	uint16_t BUF_RX_SIZE;
	uint16_t BUF_TX_MASK;
	uint16_t BUF_RX_MASK;

	char *buf_tx;
	uint16_t head_tx;
	uint16_t tail_tx;

	char *buf_rx;
	uint16_t head_rx;
	uint16_t tail_rx;

	uint8_t ascii_line;

	void (*ascii_line_event_callback)(char * buf);

}T_USART_HANDLER;

void USART_HandlerInit( T_USART_HANDLER *USART_Handler, USART_TypeDef *USART_periph, char* BUF_TX, char* BUF_RX, uint16_t BUF_TX_SIZE, uint16_t BUF_RX_SIZE, uint32_t periph_freq, uint32_t baudrate);
void USART_HandlerPutc( T_USART_HANDLER *USART_Handler, char c);
void USART_HandlerPuts( T_USART_HANDLER *USART_Handler, char * str);
void USART_HandlerPutint( T_USART_HANDLER *USART_Handler, int value, uint8_t radix);
void USART_HandlerMain( T_USART_HANDLER *USART_Handler, char * buf);
void USART_Handler_IRQ( T_USART_HANDLER *USART_Handler );
void USART_HandlerRegisterAsciiLineCallback( T_USART_HANDLER *USART_Handler, void (*callback)(char * buf));

#endif /* DRIVERS_USART_HANDLER_USART_HANDLER_H_ */
