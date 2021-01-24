#ifndef COMMON_H_
#define COMMON_H_

#include <Drivers/I2C_Handler/I2C_Handler.h>
#include <Drivers/USART_Handler/USART_Handler.h>
#include <Drivers/SPI_Handler/SPI_Handler.h>
#include <GPS_Handler/GPS_Handler.h>

extern T_I2C_HANDLER I2C1_Handler;
extern T_I2C_HANDLER I2C2_Handler;

extern T_USART_HANDLER USART1_Handler;
extern T_USART_HANDLER USART3_Handler;

extern T_GPS_DATA GPS_Data;
extern T_GPS_HANDLER GPS_Handler;

void Hardware_clock_init();
void Hardware_io_init();

#endif /* COMMON_H_ */
