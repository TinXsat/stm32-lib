#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "stm32f10x.h"

#include "I2C_Handler/I2C_Handler.h"

#include "DEBUG_UART/debug_uart.h"

char debug_uart_framebuf[100];

//just simple delay for demonstration
void LoopDelay(volatile uint32_t n) {
	while(n > 0) n--;
}

void debug_uart_str_event(char * str){
	debug_uart_puts(str);
	debug_uart_puts("\r\n");
}

uint16_t buf[30];

T_I2C_DEVICE I2C1_sensors[5];
T_I2C_HANDLER I2C1_Handler;

uint8_t s = 0;

//#define dig_T1 BMP280_sensors[s].calibration_data[0]
//#define dig_T2 BMP280_sensors[s].calibration_data[1]
//#define dig_T3 BMP280_sensors[s].calibration_data[2]
//#define dig_P1 BMP280_sensors[s].calibration_data[3]
//#define dig_P2 BMP280_sensors[s].calibration_data[4]
//#define dig_P3 BMP280_sensors[s].calibration_data[5]
//#define dig_P4 BMP280_sensors[s].calibration_data[6]
//#define dig_P5 BMP280_sensors[s].calibration_data[7]
//#define dig_P6 BMP280_sensors[s].calibration_data[8]
//#define dig_P7 BMP280_sensors[s].calibration_data[9]
//#define dig_P8 BMP280_sensors[s].calibration_data[10]
//#define dig_P9 BMP280_sensors[s].calibration_data[11]

uint16_t dig_T1;
int16_t dig_T2;
int16_t dig_T3;
uint16_t dig_P1;
int16_t dig_P2;
int16_t dig_P3;
int16_t dig_P4;
int16_t dig_P5;
int16_t dig_P6;
int16_t dig_P7;
int16_t dig_P8;
int16_t dig_P9;

//#define dig_T1 27504
//#define dig_T2 26435
//#define dig_T3 -1000
//#define dig_P1 36477
//#define dig_P2 -10685
//#define dig_P3 3024
//#define dig_P4 2855
//#define dig_P5 140
//#define dig_P6 -7
//#define dig_P7 15500
//#define dig_P8 -14600
//#define dig_P9 6000

typedef struct T_BMP280{
	T_I2C_DEVICE physical_device;
	uint32_t calibration_data[13];
	int32_t pressure_raw;
	int32_t temperature_raw;
	uint8_t caliration_downloaded;
}T_BMP280;

T_BMP280 BMP280_sensors[2];

//reg_addr,val
uint16_t i2c_device_configuration_bmp288_0[] = {
		0xF4, 0xFC,	//oversampling x16, sleep
		0xF5, 0x08,	//filter x4, 0,5 stb time, no spi
		0xF4, 0xFF	//oversampling x16, normal mode
};

volatile uint8_t state = 0;

int32_t t_fine;

int32_t bmp280_compensate_T_int32(int32_t adc_T){
	int32_t var1, var2, T;
	var1 = ((((adc_T>>3) -((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
	var2 = (((((adc_T>>4) -((int32_t)dig_T1)) * ((adc_T>>4)- ((int32_t)dig_T1))) >> 12) *((int32_t)dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}


//uint32_t bmp280_compensate_P_int32(int32_t adc_P){
//	int32_t var1, var2;
//	uint32_t p;
//	var1 = (((int32_t)t_fine)>>1) - (int32_t)64000;
//	var2 = (((var1>>2) * (var1>>2)) >> 11 ) * ((int32_t)dig_P6);
//	var2 = var2 + ((var1*((int32_t)dig_P5))<<1);
//	var2 = (var2>>2)+(((int32_t)dig_P4)<<16);
//	var1 = (((dig_P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((int32_t)dig_P2) * var1)>>1))>>18;
//	var1 =((((32768+var1))*((int32_t)dig_P1))>>15);
//	if (var1 == 0)
//	{
//	return 0; // avoid exception caused by division by zero
//	}
//	p = (((uint32_t)(((int32_t)1048576)-adc_P)-(var2>>12)))*3125;
//	if (p < 0x80000000)
//	{
//	p = (p << 1) / ((uint32_t)var1);
//	}
//	else
//	{
//	p = (p / (uint32_t)var1) * 2;
//	}
//	var1 = (((int32_t)dig_P9) * ((int32_t)(((p>>3) * (p>>3))>>13)))>>12;
//	var2 = (((int32_t)(p>>2)) * ((int32_t)dig_P8))>>13;
//	p = (uint32_t)((int32_t)p + ((var1 + var2 + dig_P7) >> 4));
//	return p;
//}

uint32_t bmp280_compensate_P_int64(int32_t adc_P)
{
  int64_t var1, var2, p;
  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)dig_P6;
  var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
  var2 = var2 + (((int64_t)dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;
  if (var1 == 0)
  {
    return 0; // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)dig_P9) * (p >> 13) * (p>>13)) >> 25;
  var2 = (((int64_t)dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
  return (uint32_t)p;
}

void i2c_read(T_I2C_DEVICE *device, uint16_t * data){

	debug_uart_puts("sucessfully read data\r\n");

	if(state < 2){
		//read calib.

		for(uint8_t i = 0; i<13; i++){
			switch(i){
			case 0:
				dig_T1	= ((I2C1_Handler.data[(i*2)+1]&0x00ff)<<8) | (I2C1_Handler.data[(i*2)]&0x00ff);
			break;
			case 1:
				dig_T2	= ((I2C1_Handler.data[(i*2)+1]&0x00ff)<<8) | (I2C1_Handler.data[(i*2)]&0x00ff);
			break;
			case 2:
				dig_T3	= ((I2C1_Handler.data[(i*2)+1]&0x00ff)<<8) | (I2C1_Handler.data[(i*2)]&0x00ff);
			break;
			case 3:
				dig_P1	= ((I2C1_Handler.data[(i*2)+1]&0x00ff)<<8) | (I2C1_Handler.data[(i*2)]&0x00ff);
			break;
			case 4:
				dig_P2	= ((I2C1_Handler.data[(i*2)+1]&0x00ff)<<8) | (I2C1_Handler.data[(i*2)]&0x00ff);
			break;
			case 5:
				dig_P3	= ((I2C1_Handler.data[(i*2)+1]&0x00ff)<<8) | (I2C1_Handler.data[(i*2)]&0x00ff);
			break;
			case 6:
				dig_P4	= ((I2C1_Handler.data[(i*2)+1]&0x00ff)<<8) | (I2C1_Handler.data[(i*2)]&0x00ff);
			break;
			case 7:
				dig_P5	= ((I2C1_Handler.data[(i*2)+1]&0x00ff)<<8) | (I2C1_Handler.data[(i*2)]&0x00ff);
			break;
			case 8:
				dig_P6	= ((I2C1_Handler.data[(i*2)+1]&0x00ff)<<8) | (I2C1_Handler.data[(i*2)]&0x00ff);
			break;
			case 9:
				dig_P7	= ((I2C1_Handler.data[(i*2)+1]&0x00ff)<<8) | (I2C1_Handler.data[(i*2)]&0x00ff);
			break;
			case 10:
				dig_P8	= ((I2C1_Handler.data[(i*2)+1]&0x00ff)<<8) | (I2C1_Handler.data[(i*2)]&0x00ff);
			break;
			case 11:
				dig_P9	= ((I2C1_Handler.data[(i*2)+1]&0x00ff)<<8) | (I2C1_Handler.data[(i*2)]&0x00ff);
			break;
			}
			char str[10];
			uint32_t asd	= ((I2C1_Handler.data[(i*2)+1]&0x00ff)<<8) | (I2C1_Handler.data[(i*2)]&0x00ff);
			itoa(asd, str,10);
			debug_uart_puts(str);
			debug_uart_puts("\r\n");
		}

		state++;
	}else{

		s = 0;

		BMP280_sensors[0].pressure_raw=0;
		BMP280_sensors[0].temperature_raw=0;

		//
			BMP280_sensors[0].temperature_raw = ((int32_t)I2C1_Handler.data[3]<<12) + ((int32_t)I2C1_Handler.data[4]<<4) + ((int32_t)I2C1_Handler.data[5]>>4);
			BMP280_sensors[0].pressure_raw = ((int32_t)I2C1_Handler.data[0]<<12) + ((int32_t)I2C1_Handler.data[1]<<4) + ((int32_t)I2C1_Handler.data[2]>>4);

			uint32_t temp = bmp280_compensate_T_int32(BMP280_sensors[0].temperature_raw);
			uint32_t press = bmp280_compensate_P_int64(BMP280_sensors[0].pressure_raw);

		char str[10];

		itoa(BMP280_sensors[0].pressure_raw, str,16);
		debug_uart_puts(str);
		debug_uart_puts("\r\n");

		itoa(press/256, str,10);
		debug_uart_puts(str);
		debug_uart_puts("\r\n");

		itoa(temp, str,10);
		debug_uart_puts(str);
		debug_uart_puts("\r\n");
	}
//	char str[10];
//	itoa(I2C1_Handler.data[0], str,16);
//	debug_uart_puts(str);

}

void i2c_configurated(){
	debug_uart_puts("sucessfully wrote data\r\n");
//	for(uint8_t i = 0; i<5; i++){
//		if(I2C1_sensors[i].responded == 1 && I2C1_sensors[i].type == 3){
//
//			//I2C_ReadSensorAdd(&I2C1_sensors[i],0x00,2);
//			debug_uart_puts("added ina219\r\n");
//			break;
//		}
//	}

	for(uint8_t i = 0; i<5; i++){
		if(I2C1_sensors[i].responded == 1 && I2C1_sensors[i].type == 1){
			if(state < 2){
				I2C_ReadSensorAdd(&I2C1_sensors[i],0x88,26);
				debug_uart_puts("read_calib\r\n");
			}else{
				I2C_ReadSensorAdd(&I2C1_sensors[i],0xF7,6);
				debug_uart_puts("read_data\r\n");
			}

			break;
		}
	}

	I2C_ReadSensor(&I2C1_Handler,i2c_read);
}
#define RAND_MAX 0xffff;

void i2c_enumerated(){

	//TIM3->CCR3 = rand();

	for(uint8_t i = 0; i<5; i++){
		if(I2C1_sensors[i].responded == 1){
			debug_uart_puts("found sensor: ");

			if(I2C1_sensors[i].slave_address == 0x77 || I2C1_sensors[i].slave_address == 0x76){
				if(I2C1_sensors[i].D0_value == 0x58) I2C1_sensors[i].type = 1;
				else I2C1_sensors[i].type = 2;
				I2C1_sensors[i].is_16_bit = 0;
			}else if((I2C1_sensors[i].slave_address & 0xf0) == 0x40){
				I2C1_sensors[i].type = 3;
				I2C1_sensors[i].is_16_bit = 1;
			}

			if(I2C1_sensors[i].type == 1) debug_uart_puts("BMP280");
			else if(I2C1_sensors[i].type == 2) debug_uart_puts("BMP388");
			else if(I2C1_sensors[i].type == 3) debug_uart_puts("INA219");
			debug_uart_puts(" on address: ");

			char chr[5];
			itoa(I2C1_sensors[i].slave_address,chr,16);
			debug_uart_puts(chr);

			debug_uart_puts("\r\n");

			//I2C1_sensors[i].responded = 0;
		}
	}

	for(uint8_t i = 0; i<5; i++){
		if(I2C1_sensors[i].responded == 1 && I2C1_sensors[i].type == 1){

			I2C_WriteConfgurationAdd(&I2C1_sensors[i],i2c_device_configuration_bmp288_0,3);

		}
	}

	I2C_WriteConfguration(&I2C1_Handler, i2c_configurated);

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

int main(void){

	//processor starts up with pll configured for 72Mhz. (we will change it in future to make it fail-save) Let's change that for now to 36Mhz

	RCC->CR |= (RCC_CR_HSION);					//turn on HSI oscillator (8Mhz)
	while(!(RCC->CR & RCC_CR_HSIRDY));			//wait until oscillator starts

	RCC->CFGR &= ~(RCC_CFGR_SW);				//set SYSCLK to HSI (for the PLL configuration)
	while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);	//wait until it switches

	RCC->CR &= ~(RCC_CR_PLLON);					//turn off PLL (to be able to change parameters)

	RCC->CFGR &= ~(RCC_CFGR_PLLSRC);			//select PLL input as HSI/2
	RCC->CFGR |= (RCC_CFGR_PLLMULL16);			//set PLL multiplier to x9

	RCC->CR |= RCC_CR_PLLON;					//turn PLL back on
	while(!(RCC->CR & RCC_CR_PLLRDY));			//wait until it locks

	RCC->CFGR = (RCC->CFGR | RCC_CFGR_SW_1) | (RCC->CFGR & ~(RCC_CFGR_SW_0)); 	//set SYSCLK to PLL
	while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);	//wait until it switches

	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;			//divide by 2 for abp1 pheriphals (max 36mhz)

	RCC->CFGR |= (RCC_CFGR_MCO_SYSCLK);			//set MCO to output sysclk

	RCC->APB2ENR |= (RCC_APB2ENR_IOPCEN);		//enable Clock for GPIOC periph
	RCC->APB2ENR |= (RCC_APB2ENR_IOPAEN);		//enable Clock for GPIOA periph
	RCC->APB2ENR |= (RCC_APB2ENR_IOPBEN);		//enable Clock for GPIOB periph

	RCC->APB1ENR |= (RCC_APB1ENR_I2C1EN);		//enable Clock for i2c1 periph

	RCC->APB1ENR |= (1<<4);		//enable Clock for tim2 periph

	GPIOA->CRH |= GPIO_CRH_MODE8;				//set PA8 as output with drive strenght to 50Mhz
	GPIOA->CRH = ((GPIOA->CRH | GPIO_CRH_CNF8_1) & ~(GPIO_CRH_CNF8_0));	//set PA8 to push-pull alternate function

	//rx and tx debug
	GPIOA->CRH |= GPIO_CRH_MODE9;				//set PA9 as output with drive strenght to 50Mhz
	GPIOA->CRH = ((GPIOA->CRH | GPIO_CRH_CNF9_1) & ~(GPIO_CRH_CNF9_0));	//set PA9 to push-pull alternate function
	GPIOA->CRH &= ~(GPIO_CRH_MODE10);				//set PA10 as output with drive strenght to 50Mhz
	GPIOA->CRH = ((GPIOA->CRH | GPIO_CRH_CNF10_1) & ~(GPIO_CRH_CNF10_0));	//set PA10 to push-pull alternate function

	//led
	GPIOC->CRH |= GPIO_CRH_MODE8_0;				//set PC8 as output with drive strenght to 10Mhz
	GPIOC->CRH &= ~(GPIO_CRH_CNF8_0 | GPIO_CRH_CNF8_1);	//set PC8 to push-pull normal operation

	//i2c bmp280
	GPIOB->CRL |= GPIO_CRL_MODE6;				//set PB6 as output with drive strenght to 50Mhz
	GPIOB->CRL |= GPIO_CRL_CNF6;				//set PB6 to open-drain alternate function
	GPIOB->CRL |= GPIO_CRL_MODE7;				//set PB7 as output with drive strenght to 50Mhz
	GPIOB->CRL |= GPIO_CRL_CNF7;				//set PB7 to open-drain alternate function

	//////////////////i2c
	I2C1->CR2 |= 0x20;							//freq of cpu
	I2C1->CCR |= 0x80;

	I2C1->CR2 |= I2C_CR2_ITEVTEN;				//enable interrupts status
	I2C1->CR2 |= I2C_CR2_ITERREN;				//enable interrupts error
	I2C1->CR2 |= I2C_CR2_ITBUFEN;				//enable interrupts buffer

	I2C1->CR1 |= (I2C_CR1_PE);					//Peripheral enabled
	//////////////////i2c

	TIM6->CR1 |= TIM_CR1_URS;
	TIM6->DIER |= TIM_DIER_UIE;
	TIM6->PSC = 100;
	TIM6->ARR = 32;

	TIM6->CR1 |= TIM_CR1_CEN;

	///pwm

	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

   // / PWM /
//	GPIOC->CRH |= GPIO_CRH_MODE8_0;                //set PC8 as output
//	GPIOC->CRH |= GPIO_CRH_CNF8_1;
//	GPIOC->CRH &= ~GPIO_CRH_CNF8_0;

	GPIOB->CRL |= GPIO_CRL_MODE0_0;                //set PB0 as output
	GPIOB->CRL |= GPIO_CRL_CNF0_1;
	GPIOB->CRL &= ~GPIO_CRL_CNF0_0;

   // / Select PWM2 output mode/
	TIM3->CCMR2 |= TIM_CCMR2_OC3M_0 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;

	TIM3->CCR3 = 6000;

	//AFIO->MAPR |= AFIO_MAPR_TIM3_REMAP_0 | AFIO_MAPR_TIM3_REMAP_1; //Set full-remap

	TIM3->CCER |= TIM_CCER_CC3E;/// Start the timer counter /
	TIM3->CR1 |= TIM_CR1_CEN;

	//pwm

	I2C_Handler_init(&I2C1_Handler, I2C1_sensors, I2C1, buf);

	debug_uart_init();
	register_debug_uart_event_callback(debug_uart_str_event);
	NVIC_EnableIRQ(USART1_IRQn);
	NVIC_EnableIRQ(I2C1_ER_IRQn);
	NVIC_EnableIRQ(I2C1_EV_IRQn);
	NVIC_EnableIRQ(TIM6_IRQn);

	I2C_EnumerateDevices(&I2C1_Handler, 0x40, 0x78, i2c_enumerated);

	while (1){


//		for(uint8_t i = 0; i<15; i++){
//			char str[10];
//			itoa(bufforek[i], str, 16);
//			debug_uart_puts(str);
//			debug_uart_puts("\r\n");
//		}
//
//		for(uint8_t i = 0; i<15; i++){
//			bufforek[i] = 0x0f;
//		}

		//I2C_EnumerateDevices(&I2C1_Handler, 0x40, 0x78, i2c_enumerated);

		//I2C_read(0x76,0xD0,1);

		LoopDelay(10000000);
		for(uint8_t i = 0; i<5; i++){
			if(I2C1_sensors[i].responded == 1 && I2C1_sensors[i].type == 1){
				if(state < 2){
					I2C_ReadSensorAdd(&I2C1_sensors[i],0x88,26);
					debug_uart_puts("read_calib\r\n");
				}else{
					I2C_ReadSensorAdd(&I2C1_sensors[i],0xF7,6);
					debug_uart_puts("read_data\r\n");
				}

				break;
			}
		}

		I2C_ReadSensor(&I2C1_Handler,i2c_read);
		DEBUG_UART_EVENT(debug_uart_framebuf);

	}
}

uint8_t recieved_data[10];

void TIM6_IRQHandler(void){
	TIM6->SR &= ~TIM_SR_UIF;

	I2C_Handler_Main(&I2C1_Handler);
}

void I2C1_ER_IRQHandler(void){

	I2C_Handler_ERIRQ( &I2C1_Handler );

	if(I2C1_Handler.mode != 1 )debug_uart_puts("err");
}

void I2C1_EV_IRQHandler(void){

	I2C_Handler_EVIRQ( &I2C1_Handler );
}
