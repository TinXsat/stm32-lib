#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "stm32f10x.h"

#include "common.h"

T_I2C_HANDLER I2C1_Handler;
T_I2C_HANDLER I2C2_Handler;

T_USART_HANDLER USART1_Handler;
T_USART_HANDLER USART3_Handler;

T_GPS_DATA GPS_Data;
T_GPS_HANDLER GPS_Handler;

T_SPI_COMMAND out_buf[15];
T_SPI_COMMAND in_buf[15];
T_SPI_HANDLER SPI2_Handler;

void Hardware_clock_init(){

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
	RCC->APB1ENR |= (RCC_APB1ENR_I2C2EN);		//enable Clock for i2c1 periph
	RCC->APB1ENR |= (RCC_APB1ENR_SPI2EN);		//enable Clock for SPI2 periph

	RCC->APB1ENR |= (1<<4);		//enable Clock for tim2 periph

	RCC->APB2ENR |= (RCC_APB2ENR_USART1EN);		//enable Clock for USART1 periph
	RCC->APB1ENR |= (RCC_APB1ENR_USART3EN);		//enable Clock for USART3 periph

	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
}

void Hardware_io_init(){

	//SYSCLK output
	GPIOA->CRH |= GPIO_CRH_MODE8;				//set PA8 as output with drive strenght to 50Mhz
	GPIOA->CRH = ((GPIOA->CRH | GPIO_CRH_CNF8_1) & ~(GPIO_CRH_CNF8_0));	//set PA8 to push-pull alternate function

	//Debug_uart
	GPIOA->CRH |= GPIO_CRH_MODE9;				//set PA9 as output with drive strenght to 50Mhz
	GPIOA->CRH = ((GPIOA->CRH | GPIO_CRH_CNF9_1) & ~(GPIO_CRH_CNF9_0));	//set PA9 to push-pull alternate function
	GPIOA->CRH &= ~(GPIO_CRH_MODE10);				//set PA10 as output with drive strenght to 50Mhz
	GPIOA->CRH = ((GPIOA->CRH | GPIO_CRH_CNF10_1) & ~(GPIO_CRH_CNF10_0));	//set PA10 to push-pull alternate function

	//Gps_uart
	GPIOC->CRH |= GPIO_CRH_MODE10;				//set PA9 as output with drive strenght to 50Mhz
	GPIOC->CRH = ((GPIOC->CRH | GPIO_CRH_CNF10_1) & ~(GPIO_CRH_CNF10_0));	//set PA9 to push-pull alternate function
	GPIOC->CRH &= ~(GPIO_CRH_MODE11);				//set PA10 as output with drive strenght to 50Mhz
	GPIOC->CRH = ((GPIOC->CRH | GPIO_CRH_CNF11_1) & ~(GPIO_CRH_CNF11_0));	//set PA10 to push-pull alternate function

	//debug_led2
	GPIOC->CRH |= GPIO_CRH_MODE8_0;				//set PC8 as output with drive strenght to 10Mhz
	GPIOC->CRH &= ~(GPIO_CRH_CNF8_0 | GPIO_CRH_CNF8_1);	//set PC8 to push-pull normal operation

	//debug_led1
	GPIOC->CRH |= GPIO_CRH_MODE9_0;				//set PC9 as output with drive strenght to 10Mhz
	GPIOC->CRH &= ~(GPIO_CRH_CNF9_0 | GPIO_CRH_CNF9_1);	//set PC9 to push-pull normal operation

	//System I2C
	GPIOB->CRL |= GPIO_CRL_MODE6;				//set PB6 as output with drive strenght to 50Mhz
	GPIOB->CRL |= GPIO_CRL_CNF6;				//set PB6 to open-drain alternate function
	GPIOB->CRL |= GPIO_CRL_MODE7;				//set PB7 as output with drive strenght to 50Mhz
	GPIOB->CRL |= GPIO_CRL_CNF7;				//set PB7 to open-drain alternate function

	//I2C mpu6050
	GPIOB->CRH |= GPIO_CRH_MODE10;				//set PB10 as output with drive strenght to 50Mhz
	GPIOB->CRH |= GPIO_CRH_CNF10;				//set PB10 to open-drain alternate function
	GPIOB->CRH |= GPIO_CRH_MODE11;				//set PB11 as output with drive strenght to 50Mhz
	GPIOB->CRH |= GPIO_CRH_CNF11;				//set PB11 to open-drain alternate function

	//Status_led
	GPIOB->CRL |= GPIO_CRL_MODE0_0;
	GPIOB->CRL |= GPIO_CRL_CNF0_1;
	GPIOB->CRL &= ~GPIO_CRL_CNF0_0;
	GPIOA->CRL |= GPIO_CRL_MODE6_0;
	GPIOA->CRL |= GPIO_CRL_CNF6_1;
	GPIOA->CRL &= ~GPIO_CRL_CNF6_0;
	GPIOA->CRL |= GPIO_CRL_MODE7_0;
	GPIOA->CRL |= GPIO_CRL_CNF7_1;
	GPIOA->CRL &= ~GPIO_CRL_CNF7_0;

	//lora spi
	GPIOB->CRH |= GPIO_CRH_MODE15 | GPIO_CRH_MODE14 | GPIO_CRH_MODE13;	//output 50Mhz
	GPIOB->CRH = ((GPIOB->CRH | GPIO_CRH_CNF15_1 | GPIO_CRH_CNF14_1 | GPIO_CRH_CNF13_1) & ~((GPIO_CRH_CNF15_0) | (GPIO_CRH_CNF14_0) | (GPIO_CRH_CNF13_0)));	//alternate funcion push-pull
	GPIOC->CRL |= GPIO_CRL_MODE6_0;
	GPIOC->CRL &= ~(GPIO_CRL_CNF6);
	GPIOC->CRL |= GPIO_CRL_MODE7_0;
	GPIOC->CRL &= ~(GPIO_CRL_CNF7);
	GPIOC->BSRR = GPIO_BSRR_BS7;
	GPIOB->CRL &= ~(GPIO_CRL_MODE2);
	GPIOB->CRL &= ~(GPIO_CRL_CNF2_0);
	GPIOB->CRL |= (GPIO_CRL_CNF2_1);
	GPIOB->ODR |= (GPIO_ODR_ODR2);





	AFIO->MAPR |= AFIO_MAPR_USART3_REMAP_0; //Set half-remap
}

void I2C1_ER_IRQHandler(void){

	I2C_Handler_ERIRQ( &I2C1_Handler );

}

void I2C1_EV_IRQHandler(void){

	I2C_Handler_EVIRQ( &I2C1_Handler );
}

void I2C2_ER_IRQHandler(void){

	I2C_Handler_ERIRQ( &I2C2_Handler );

}

void I2C2_EV_IRQHandler(void){

	I2C_Handler_EVIRQ( &I2C2_Handler );

}
void USART1_IRQHandler(void){
	USART_Handler_IRQ(&USART1_Handler);
}

void USART3_IRQHandler(void){
	USART_Handler_IRQ(&USART3_Handler);
}

void SPI2_IRQHandler(void){
	SPI_HandlerIRQ(&SPI2_Handler);
}
