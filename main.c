#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "stm32f10x.h"

#include "common.h"

#include "Sensors/Sensors.h"

#include "Lora_llib/SX1278.h"
#include "Lora_llib/SX1278_hw.h"

#include "Drivers/I2C_Handler/I2C_Handler.h"
#include "Drivers/USART_Handler/USART_Handler.h"
#include "Drivers/SPI_Handler/SPI_Handler.h"

#include "GPS_Handler/GPS_Handler.h"

#include "MPU6050_registers.h"

#include "Misc/Status_Led/Status_Led.h"

char debug_uart_tx_buf[512];
char debug_uart_rx_buf[512];

char gps_uart_tx_buf[512];
char gps_uart_rx_buf[512];

char debug_uart_framebuf[100];

//just simple delay for demonstration
void LoopDelay(volatile uint32_t n) {
	while(n > 0) n--;
}

void debug_uart_str_event(char * str){
	//USART_HandlerPuts( &USART1_Handler, str);
	//USART_HandlerPuts( &USART1_Handler, "\r\n");
	USART_HandlerPuts(&USART1_Handler, str);
	USART_HandlerPuts(&USART1_Handler, "\r\n");
}

void gps_uart_str_event(char * str){
	GPS_HandlerAsciiLine(&GPS_Handler, str);
}

void gps_received_event(T_GPS_DATA* data){
	USART_HandlerPuts(&USART1_Handler, "time:");
	USART_HandlerPutint(&USART1_Handler, data->time_hh,10);
	USART_HandlerPutc(&USART1_Handler, ':');
	USART_HandlerPutint(&USART1_Handler, data->time_mm,10);
	USART_HandlerPutc(&USART1_Handler, ':');
	USART_HandlerPutint(&USART1_Handler, data->time_ss,10);
	USART_HandlerPuts(&USART1_Handler, " latitude:");
	char str[30];
	gcvt(data->latitude_decimald, 8, str);
	USART_HandlerPuts(&USART1_Handler, str);
	USART_HandlerPuts(&USART1_Handler, " longitude:");
	gcvt(data->longitude_decimald, 8, str);
	USART_HandlerPuts(&USART1_Handler, str);
	USART_HandlerPuts(&USART1_Handler, "\r\n");
}

uint16_t buf[30];
uint16_t buf2[30];

T_I2C_DEVICE I2C1_sensors[5];
T_I2C_DEVICE I2C2_sensors[5];

uint8_t s = 0;

char buffer[512];

int message;
int message_length;

void i2c_enumerated(){

	uint8_t bmp280_counter = 0, ina219_counter = 0;

	for(uint8_t i = 0; i<5; i++){
		if(I2C1_sensors[i].responded == 1){
			USART_HandlerPuts( &USART1_Handler, "found sensor: ");


			if(I2C1_sensors[i].slave_address == 0x77 || I2C1_sensors[i].slave_address == 0x76){
				if(I2C1_sensors[i].D0_value == 0x58){
					I2C1_sensors[i].type = 1;
					BMP280_sensors[bmp280_counter].physical_device = &I2C1_sensors[i];
					bmp280_counter++;
				}else I2C1_sensors[i].type = 2;
				I2C1_sensors[i].is_16_bit = 0;
			}else if((I2C1_sensors[i].slave_address & 0xf0) == 0x40){
				I2C1_sensors[i].type = 3;
				I2C1_sensors[i].is_16_bit = 1;

				INA219_sensors[ina219_counter].physical_device = &I2C1_sensors[i];
				ina219_counter++;
			}

			if(I2C1_sensors[i].type == 1) USART_HandlerPuts( &USART1_Handler, "BMP280");
			else if(I2C1_sensors[i].type == 2) USART_HandlerPuts( &USART1_Handler, "BMP388");
			else if(I2C1_sensors[i].type == 3) USART_HandlerPuts( &USART1_Handler, "INA219");
			USART_HandlerPuts( &USART1_Handler, " on address: ");

			char chr[5];
			itoa(I2C1_sensors[i].slave_address,chr,16);
			USART_HandlerPuts( &USART1_Handler, chr);

			USART_HandlerPuts( &USART1_Handler, "\r\n");

		}
	}

	Sensors_init1(BMP280_sensors, bmp280_counter, INA219_sensors, ina219_counter);
	Sensors_Aquire(1);
}

#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43

uint16_t i2c_device_configuration_mpu6050[] = {
		PWR_MGMT_1_REG, 0,	//oversampling x16, sleep
		SMPLRT_DIV_REG, 0x07,	//filter x4, 0,5 stb time, no spi
		ACCEL_CONFIG_REG, 0x00,	//oversampling x16, normal mode
		GYRO_CONFIG_REG, 0x00
};

void mpu_read( T_I2C_DEVICE * i2c_device, uint16_t *data ){
//    int16_t x = (int16_t) (data[0] << 8 | data[1])/131.0;
//    int16_t y = (int16_t) (data[2] << 8 | data[3])/131.0;
//    int16_t z = (int16_t) (data[4] << 8 | data[5])/131.0;
//
//	char str10[10];
//	itoa(x,str10,10);
//	USART_HandlerPuts( &USART1_Handler, str10);
//	USART_HandlerPuts( &USART1_Handler, "     ");
//	itoa(y,str10,10);
//	USART_HandlerPuts( &USART1_Handler, str10);
//	USART_HandlerPuts( &USART1_Handler, "     ");
//	itoa(z,str10,10);
//	USART_HandlerPuts( &USART1_Handler, str10);
//	USART_HandlerPuts( &USART1_Handler, "\r\n");
//
//	I2C_ReadSensorAdd(&I2C2_sensors[0], GYRO_XOUT_H_REG, 6);
//	I2C_ReadSensor(&I2C2_Handler, mpu_read);

	int16_t x = (int16_t) (data[0] << 8 | data[1]);
	int16_t y = (int16_t) (data[2] << 8 | data[3]);
	int16_t z = (int16_t) (data[4] << 8 | data[5]);

    float ax = x / 163.840;
    float ay = y / 163.840;
    float az = z / 163.840;

    	char str10[10];
    	itoa(ax,str10,10);
    	USART_HandlerPuts( &USART1_Handler, str10);
    	USART_HandlerPuts( &USART1_Handler, "     ");
    	itoa(ay,str10,10);
    	USART_HandlerPuts( &USART1_Handler, str10);
    	USART_HandlerPuts( &USART1_Handler, "     ");
    	itoa(az,str10,10);
    	USART_HandlerPuts( &USART1_Handler, str10);
    	USART_HandlerPuts( &USART1_Handler, "\r\n");

	I2C_ReadSensorAdd(&I2C2_sensors[0], ACCEL_XOUT_H_REG, 6);
	I2C_ReadSensor(&I2C2_Handler, mpu_read);

//
//	int16_t temp = (int16_t) (data[0] << 8 | data[1]);
//	float Temperature = (float) ((int16_t) temp / (float) 340.0 + (float) 36.53);
//
//		char str10[10];
//		itoa(Temperature,str10,10);
//		USART_HandlerPuts( &USART1_Handler, str10);
//		USART_HandlerPuts( &USART1_Handler, "\r\n");
//
//	    	I2C_ReadSensorAdd(&I2C2_sensors[0], TEMP_OUT_H_REG, 2);
//	    	I2C_ReadSensor(&I2C2_Handler, mpu_read);
}

void mpu_configured(){
	USART_HandlerPuts( &USART1_Handler, "mpu//configured");

	I2C_ReadSensorAdd(&I2C2_sensors[0], ACCEL_XOUT_H_REG, 6);
	I2C_ReadSensor(&I2C2_Handler, mpu_read);
}

void i2c2_enumerated(){

	for(uint8_t i = 0; i<5; i++){
		if(I2C2_sensors[i].responded == 1){

			char str10[10];
			itoa(I2C2_sensors[i].slave_address,str10,16);
			USART_HandlerPuts( &USART1_Handler, "found mpu6050!");
			USART_HandlerPuts( &USART1_Handler, str10);

			USART_HandlerPuts( &USART1_Handler, "\r\n");

			I2C_WriteConfgurationAdd(&I2C2_sensors[i], i2c_device_configuration_mpu6050, 4);
			I2C_WriteConfguration(&I2C2_Handler, mpu_configured);

		}
	}

}

void I2c_init(){
	I2C1->CR1 &= ~(I2C_CR1_PE);

	I2C1->CR2 |= 0x20;							//freq of cpu
	I2C1->CCR |= 0x80;

	I2C1->CR2 |= I2C_CR2_ITEVTEN;				//enable interrupts status
	I2C1->CR2 |= I2C_CR2_ITERREN;				//enable interrupts error
	I2C1->CR2 |= I2C_CR2_ITBUFEN;				//enable interrupts buffer

	I2C1->CR1 |= (I2C_CR1_PE);					//Peripheral enabled
}

void I2c1_error(T_I2C_DEVICE* device, T_I2C_ERROR_CODE error_code){
	USART_HandlerPuts( &USART1_Handler, "I2C error detected! While access to: ");
	if(device->type == 1) USART_HandlerPuts( &USART1_Handler, "BMP280");
	else if(device->type == 2) USART_HandlerPuts( &USART1_Handler, "BMP388");
	else if(device->type == 3) USART_HandlerPuts( &USART1_Handler, "INA219");
	USART_HandlerPuts( &USART1_Handler, " on address: ");
	char str[10];
	itoa(device->slave_address,str,16);
	USART_HandlerPuts( &USART1_Handler, str);
	USART_HandlerPuts( &USART1_Handler, " Reason: ");
	itoa(error_code,str,16);
	USART_HandlerPuts( &USART1_Handler, str);
	USART_HandlerPuts( &USART1_Handler, "\r\n");

	Sensors.bmp280_initstate = no_init;

}

SX1278_hw_t lora_hw;
SX1278_t lora;

void spi_cs( uint8_t device, uint8_t state){
	if(device == 0){
		if(state == 1){
			GPIOC->BSRR = GPIO_BSRR_BS6;
		}
		if(state == 0){
			GPIOC->BSRR = GPIO_BSRR_BR6;
		}
	}
}

void spi_sucessfully(uint8_t dev){

}

void spi_error(uint8_t dev){

}

int main(void){
	//clock is 64mhz

	Hardware_clock_init();
	Hardware_io_init();

	I2C1->CR2 |= 0x20;							//freq of cpu
	I2C1->CCR |= 0x80;

	I2C1->CR2 |= I2C_CR2_ITEVTEN;				//enable interrupts status
	I2C1->CR2 |= I2C_CR2_ITERREN;				//enable interrupts error
	I2C1->CR2 |= I2C_CR2_ITBUFEN;				//enable interrupts buffer

	I2C1->CR1 |= (I2C_CR1_PE);					//Peripheral enabled

	TIM6->CR1 |= TIM_CR1_URS;
	TIM6->DIER |= TIM_DIER_UIE;
	TIM6->PSC = 100;
	TIM6->ARR = 32;

	TIM6->CR1 |= TIM_CR1_CEN;

	//spi config
	SPI2->CR1 |= ( SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR | SPI_CR1_BR_1 );	//set internal ss pin to high, master mode, and baud to /8
	SPI2->CR1 |= SPI_CR1_SPE;	//enable spi

	SX1278_hw_init(&lora_hw);

	I2C_Handler_init(&I2C1_Handler, I2C1_sensors, I2C1, buf,I2c1_error);
	I2C_Handler_init(&I2C2_Handler, I2C2_sensors, I2C2, buf2,I2c1_error);

	USART_HandlerInit(&USART1_Handler, USART1,debug_uart_tx_buf,debug_uart_rx_buf, 512, 512, 64000000, 57600);
	USART_HandlerRegisterAsciiLineCallback(&USART1_Handler, debug_uart_str_event);

	USART_HandlerInit(&USART3_Handler, USART3,gps_uart_tx_buf, gps_uart_rx_buf, 512, 512, 32000000, 9600);
	USART_HandlerRegisterAsciiLineCallback(&USART3_Handler, gps_uart_str_event);

	GPS_HandlerInit(&GPS_Handler, &USART3_Handler, gps_received_event);

	NVIC_EnableIRQ(USART1_IRQn);
	NVIC_EnableIRQ(USART3_IRQn);
	NVIC_EnableIRQ(I2C1_ER_IRQn);
	NVIC_EnableIRQ(I2C1_EV_IRQn);
	NVIC_EnableIRQ(I2C2_ER_IRQn);
	NVIC_EnableIRQ(I2C2_EV_IRQn);
	NVIC_EnableIRQ(TIM6_IRQn);

	I2C_EnumerateDevices(&I2C1_Handler, 0x40, 0x78, i2c_enumerated);
	I2C_EnumerateDevices(&I2C2_Handler, 0x68, 0x70, i2c2_enumerated);


	lora.hw = &lora_hw;

	SX1278_init(&lora, 433000000, SX1278_POWER_20DBM, SX1278_LORA_SF_12,
		SX1278_LORA_BW_125KHZ, SX1278_LORA_CR_4_8, SX1278_LORA_CRC_EN, 10);

	SX1278_LoRaEntryTx(&lora, 16, 2000);
	//SX1278_LoRaEntryRx(&lora, 16, 2000);
//
	USART_HandlerPuts( &USART1_Handler, "lora initialized!");

	GPIOC->BSRR = GPIO_BSRR_BS8;
	LoopDelay(500000/2);
	GPIOC->BSRR = GPIO_BSRR_BR8;

	int ret = 0;
	char str[10];

	buffer[0] = 'F';
	buffer[6] = 0;

	uint8_t asd[10];

	NVIC_EnableIRQ(SPI2_IRQn);

	SPI_HandlerInit(&SPI2_Handler, spi_cs, spi_error);

	StatusLed_Init();
	StatusLed_setColorRGB(65535/2,0,0);
	StatusLed_setColorRGB2(0,0,65535/10);
	StatusLed_setBlink2(200,1000,5);
	StatusLed_setBlink(1000,6000,1000,3);


	while (1){

//		buforek[0] = 0x4B;
//		buforek[1] = 0x00;
//		buforek[2] = 0x00;
//		buforek[3] = 0x00;
//		out_buf[0].data = buforek;
//		out_buf[0].len = 2;
//		out_buf[1].data = buforek;
//		out_buf[1].len = 2;
//		in_buf[0].data = asd;
//		in_buf[1].data = asd;
//		SPI_HandlerTransmitRecieve(&SPI2_Handler, 0, out_buf, in_buf, 2, spi_sucessfully);
//
//		USART_HandlerPuts( &USART1_Handler, "Sending package...\r\n");
//
//		message_length = 5;
//		ret = SX1278_LoRaEntryTx(&lora, message_length, 2000);
//		itoa(ret, str, 16);
		//USART_HandlerPuts( &USART1_Handler, "Entry: ");
		//USART_HandlerPuts( &USART1_Handler, str);
		//USART_HandlerPuts( &USART1_Handler, "\r\n");

		//USART_HandlerPuts( &USART1_Handler, buffer);
		//USART_HandlerPuts( &USART1_Handler, "\r\n");

//		ret = SX1278_LoRaTxPacket(&lora, (uint8_t*) buffer,
//				message_length, 2000);
//		message += 1;

		//USART_HandlerPuts( &USART1_Handler, "Transmission: ");
//		itoa(ret, str, 16);
		//USART_HandlerPuts( &USART1_Handler, str);
		//USART_HandlerPuts( &USART1_Handler, "\r\n");
		//USART_HandlerPuts( &USART1_Handler, "Package sent...\r\n");

//		if(ret == 1)GPIOC->BSRR = GPIO_BSRR_BS8;

		//USART_HandlerPuts( &USART1_Handler, "Slave ...\r\n");
		//USART_HandlerPuts( &USART1_Handler, "Receiving package...\r\n");

//		ret = SX1278_LoRaRxPacket(&lora);
//		if(ret != 0){
//			//USART_HandlerPuts( &USART1_Handler, "Received: ");
//			//itoa(ret, str, 16);
//			//USART_HandlerPuts( &USART1_Handler, str);
//			//USART_HandlerPuts( &USART1_Handler, "\r\n");
//			if (ret > 0) {
//				SX1278_read(&lora, (uint8_t*) buffer, ret);
//				GPIOC->ODR ^= GPIO_ODR_ODR8;
//				printf("Content: ");
//				USART_HandlerPuts( &USART1_Handler, buffer);
//				USART_HandlerPuts( &USART1_Handler, ", RSSI:");
//				ret = SX1278_RSSI_LoRa(&lora);
//				itoa(ret, str, 10);
//				USART_HandlerPuts( &USART1_Handler, str);
//				USART_HandlerPuts( &USART1_Handler, ", SNR: ");
//				ret = SX1278_SNR_LoRa(&lora);
//				itoa(ret, str, 10);
//				USART_HandlerPuts( &USART1_Handler, str);
//				USART_HandlerPuts( &USART1_Handler, "\r\n");
//			}
//			USART_HandlerPuts( &USART1_Handler, "Package received ...\r\n");
//		}
//

		GPIOC->BSRR = GPIO_BSRR_BS8;
		//LoopDelay(100000/2);
		GPIOC->BSRR = GPIO_BSRR_BR8;
		//LoopDelay(100000/2);

		BMP280_TimerIRQ();
		//USART_HandlerPuts( &USART1_Handler, "m");
		USART_HandlerMain(&USART1_Handler, debug_uart_framebuf);
		USART_HandlerMain(&USART3_Handler, debug_uart_framebuf);
		//DEBUG_UART_EVENT(debug_uart_framebuf);

	}
}

uint8_t recieved_data[10];

void TIM6_IRQHandler(void){
	TIM6->SR &= ~TIM_SR_UIF;
	static uint32_t counter;
	if(counter < 10){
		counter++;
	}else{
		counter = 0;
		StatusLed_HandlerMain();
	}
	I2C_Handler_Main(&I2C1_Handler);
	I2C_Handler_Main(&I2C2_Handler);
}
