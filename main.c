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

uint16_t buf[10];

T_I2C_DEVICE I2C1_sensors[5];
T_I2C_HANDLER I2C1_Handler;

//reg_addr,val
uint16_t i2c_device_configuration_bmp288_0[] = {
		0xF4, 0xFC,	//oversampling x16, sleep
		0xF5, 0x08,	//filter x4, 0,5 stb time, no spi
		0xF4, 0xFF	//oversampling x16, normal mode
};

void i2c_read(){

	debug_uart_puts("sucessfully read data\r\n");

	char str[10];
	itoa(I2C1_Handler.data[0], str,16);
	debug_uart_puts(str);

}

void i2c_configurated(){
	debug_uart_puts("sucessfully wrote data\r\n");
	for(uint8_t i = 0; i<5; i++){
		if(I2C1_sensors[i].responded == 1 && I2C1_sensors[i].type == 3){

			I2C_ReadSensorAdd(&I2C1_sensors[i],0x00,1);
			debug_uart_puts("added ina219\r\n");
			break;
		}
	}

	for(uint8_t i = 0; i<5; i++){
		if(I2C1_sensors[i].responded == 1 && I2C1_sensors[i].type == 1){

			I2C_ReadSensorAdd(&I2C1_sensors[i],0xD0,1);
			debug_uart_puts("added bmp280\r\n");
			break;
		}
	}

	I2C_ReadSensor(&I2C1_Handler,i2c_read);
}

void i2c_enumerated(){

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

	I2C_Handler_init(&I2C1_Handler, I2C1_sensors, I2C1, buf);

	debug_uart_init();
	register_debug_uart_event_callback(debug_uart_str_event);
	NVIC_EnableIRQ(USART1_IRQn);
	NVIC_EnableIRQ(I2C1_ER_IRQn);
	NVIC_EnableIRQ(I2C1_EV_IRQn);
	NVIC_EnableIRQ(TIM6_IRQn);

	while (1){

		I2C_EnumerateDevices(&I2C1_Handler, 0x40, 0x78, i2c_enumerated);

		//I2C_read(0x76,0xD0,1);

		LoopDelay(10000000);
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
