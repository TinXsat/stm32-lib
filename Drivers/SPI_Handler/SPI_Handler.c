#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "stm32f10x.h"

#include "SPI_Handler.h"

void SPI_HandlerInit(T_SPI_HANDLER * SPI_Handler, void(*callback)(uint8_t dev, uint8_t state), void (*error_event_callback)( uint8_t dev )){
	SPI_Handler->spi_set_cs_device = callback;
	SPI_Handler->spi_error_event_callback = error_event_callback;
}

void SPI_HandlerTransmitRecieve(T_SPI_HANDLER * SPI_Handler, uint8_t device, T_SPI_COMMAND *out_buffer, T_SPI_COMMAND *in_buffer, uint16_t data_len, void (*end_event_callback)( uint8_t dev )){
	SPI_Handler->busy = 1;
	SPI_Handler->out_buffer = out_buffer;
	SPI_Handler->in_buffer = in_buffer;
	SPI_Handler->data_len = data_len;
	SPI_Handler->data_pointer = 0;
	SPI_Handler->byte_pointer = 0;

	SPI_Handler->device = device;

	SPI_Handler->spi_end_event_callback = end_event_callback;
	SPI_Handler->spi_set_cs_device(SPI_Handler->device, 0);

	SPI2->DR = SPI_Handler->out_buffer[0].data[0];
	SPI2->CR2 |= SPI_CR2_RXNEIE | SPI_CR2_TXEIE;
}

void SPI_HandlerTransmit(T_SPI_HANDLER * SPI_Handler, uint8_t device, T_SPI_COMMAND *out_buffer, T_SPI_COMMAND *in_buffer, uint16_t data_len, void (*end_event_callback)( uint8_t dev )){
	SPI_Handler->busy = 1;
	SPI_Handler->only_transmit = 1;
	SPI_Handler->out_buffer = out_buffer;
	SPI_Handler->in_buffer = in_buffer;
	SPI_Handler->data_len = data_len;
	SPI_Handler->data_pointer = 0;
	SPI_Handler->byte_pointer = 0;

	SPI_Handler->device = device;

	SPI_Handler->spi_end_event_callback = end_event_callback;
	SPI_Handler->spi_set_cs_device(SPI_Handler->device, 0);

	SPI2->DR = SPI_Handler->out_buffer[0].data[0];
	SPI2->CR2 |= SPI_CR2_RXNEIE | SPI_CR2_TXEIE;
}

void SPI_HandlerIRQ( T_SPI_HANDLER * SPI_Handler ){

	if(SPI2->SR & SPI_SR_RXNE){
		if(!SPI_Handler->only_transmit){
			SPI_Handler->in_buffer[SPI_Handler->data_pointer].data[SPI_Handler->byte_pointer] = SPI2->DR;
		}else{
			uint8_t asd = SPI2->DR;
		}
	}

	if(SPI2->SR & SPI_SR_TXE){

		SPI_Handler->byte_pointer++;

		if(SPI_Handler->byte_pointer == SPI_Handler->out_buffer[SPI_Handler->data_pointer].len){
			SPI_Handler->byte_pointer = 0;
			if(SPI_Handler->data_pointer+1 < SPI_Handler->data_len){
				SPI_Handler->data_pointer++;
				//send byte after time
				SPI_Handler->spi_set_cs_device(SPI_Handler->device, 1);
				SPI_Handler->spi_set_cs_device(SPI_Handler->device, 0);
				SPI2->DR = SPI_Handler->out_buffer[SPI_Handler->data_pointer].data[SPI_Handler->byte_pointer];
			}else{
				SPI_Handler->spi_set_cs_device(SPI_Handler->device, 1);
				SPI2->CR2 &= ~(SPI_CR2_RXNEIE | SPI_CR2_TXEIE);
				if(SPI_Handler->spi_end_event_callback!=0)SPI_Handler->spi_end_event_callback(SPI_Handler->device);
				SPI_Handler->busy = 0;
			}
		}else{
			SPI2->DR = SPI_Handler->out_buffer[SPI_Handler->data_pointer].data[SPI_Handler->byte_pointer];
		}

	}

}

uint8_t SPI_HandlerIsBusy(T_SPI_HANDLER * SPI_Handler){
	return SPI_Handler->busy;
}
