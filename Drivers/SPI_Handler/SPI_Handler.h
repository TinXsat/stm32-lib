#ifndef DRIVERS_SPI_HANDLER_SPI_HANDLER_H_
#define DRIVERS_SPI_HANDLER_SPI_HANDLER_H_

typedef struct T_SPI_COMMAND{
	uint8_t * data;
	uint16_t len;
}T_SPI_COMMAND;

typedef struct T_SPI_HANDLER{

	T_SPI_COMMAND *out_buffer;
	T_SPI_COMMAND *in_buffer;
	uint16_t data_pointer;
	uint16_t byte_pointer;
	uint16_t data_len;
	uint8_t device;
	uint8_t busy;
	uint8_t only_transmit;

	void (*spi_set_cs_device)(uint8_t dev, uint8_t state);

	void (*spi_end_event_callback)( uint8_t dev );
	void (*spi_error_event_callback)( uint8_t dev );

}T_SPI_HANDLER;

void SPI_HandlerInit(T_SPI_HANDLER * SPI_Handler, void(*callback)(uint8_t dev, uint8_t state), void(*error_event_callback)( uint8_t dev ));
void SPI_HandlerIRQ(T_SPI_HANDLER * SPI_Handler);
void SPI_HandlerTransmit(T_SPI_HANDLER * SPI_Handler, uint8_t device, T_SPI_COMMAND *out_buffer, T_SPI_COMMAND *in_buffer, uint16_t data_len, void (*end_event_callback)( uint8_t dev ));
void SPI_HandlerTransmitRecieve(T_SPI_HANDLER * SPI_Handler, uint8_t device, T_SPI_COMMAND *out_buffer, T_SPI_COMMAND *in_buffer, uint16_t data_len, void (*end_event_callback)( uint8_t dev ));

#endif /* DRIVERS_SPI_HANDLER_SPI_HANDLER_H_ */
