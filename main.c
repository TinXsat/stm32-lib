#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "stm32f10x.h"

#include "DEBUG_UART/debug_uart.h"

static void (*i2c1_tx_event_callback)(uint8_t message_id);

void register_i2c_tx_event_callback(void (*callback)(uint8_t message_id)) {
	i2c1_tx_event_callback = callback;
}

static void (*i2c1_rx_event_callback)(uint8_t message_id, uint8_t* data);

void register_i2c_rx_event_callback(void (*callback)(uint8_t message_id, uint8_t* data)) {
	i2c1_rx_event_callback = callback;
}

char debug_uart_framebuf[100];

//just simple delay for demonstration
void LoopDelay(volatile uint32_t n) {
	while(n > 0) n--;
}

void debug_uart_str_event(char * str){
	debug_uart_puts(str);
	debug_uart_puts("\r\n");
}

uint8_t error_flag;
uint8_t event_flag;

typedef struct I2C_Handler{
	uint8_t* data;
	uint8_t data_len;
	uint8_t data_pointer;
	uint8_t reg_address;
	uint8_t slave_address;
	uint8_t transmit;
	uint8_t restart;
	uint8_t message_id;
	uint8_t tx_flag;
}I2C_Handler;

I2C_Handler I2C1_Handler;

uint8_t I2C_write( uint8_t slave_address, uint8_t * data, uint8_t data_len){
	I2C1_Handler.data = data;
	I2C1_Handler.data_len = data_len;
	I2C1_Handler.slave_address = slave_address;
	I2C1_Handler.transmit = 1;
	I2C1_Handler.restart = 0;
	I2C1_Handler.data_pointer = 0;
	I2C1_Handler.message_id++;
	I2C1->CR1 |= I2C_CR1_START;
	return I2C1_Handler.message_id;
}

uint8_t I2C_read( uint8_t slave_address, uint8_t reg_address, uint8_t data_len){
	I2C1_Handler.data_len = data_len;
	I2C1_Handler.slave_address = slave_address;
	I2C1_Handler.reg_address = reg_address;
	I2C1_Handler.transmit = 0;
	I2C1_Handler.restart = 0;
	I2C1_Handler.data_pointer = 0;
	I2C1_Handler.message_id++;
	I2C1->CR1 |= I2C_CR1_START;
	return I2C1_Handler.message_id;
}

void i2c_tx_event( uint8_t message_id ){
	Debug_log(debug_module_CORE,debug_type_IMPORTANT,"got it!",1);
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

	I2C1->CR2 |= 0x20;							//freq of cpu
	I2C1->CCR |= 0x80;

	I2C1->CR2 |= I2C_CR2_ITEVTEN;				//enable interrupts status
	I2C1->CR2 |= I2C_CR2_ITERREN;				//enable interrupts error
	I2C1->CR2 |= I2C_CR2_ITBUFEN;				//enable interrupts buffer

	I2C1->CR1 |= (I2C_CR1_PE);					//Peripheral enabled

	register_i2c_tx_event_callback(i2c_tx_event);

	debug_uart_init();
	register_debug_uart_event_callback(debug_uart_str_event);
	NVIC_EnableIRQ(USART1_IRQn);
	NVIC_EnableIRQ(I2C1_ER_IRQn);
	NVIC_EnableIRQ(I2C1_EV_IRQn);

	while (1){

		if(I2C1_Handler.tx_flag){
			if(i2c1_tx_event_callback != 0)i2c1_tx_event_callback(I2C1_Handler.tx_flag);
			I2C1_Handler.tx_flag = 0;
		}

		uint8_t asd[3] = {0xF4, 0xaa,0x0f};
		//I2C_read(0x76, 0xF7,6);
		I2C_write(0x76, asd, 2);


//		I2C1->CR1 |= I2C_CR1_START;
//		while(!(I2C1->SR1 & I2C_SR1_SB));
//
//		I2C1->DR = 0x76<<1;
//		while(!(I2C1->SR1 & I2C_SR1_ADDR));
//		uint32_t asd = I2C1->SR2;
//
//		I2C1->DR = 0xFB;
//
//		//I2C1->CR1 |= I2C_CR1_STOP;
//
//		if(I2C1->SR1 & I2C_SR1_AF){
//			GPIOC->BSRR |= GPIO_BSRR_BR8;
//		}else{
//			GPIOC->BSRR |= GPIO_BSRR_BS8;
//		}
//
//		//LoopDelay(1000);
//
//		I2C1->CR1 |= I2C_CR1_START;
//		while(!(I2C1->SR1 & I2C_SR1_SB));
//
//		I2C1->DR = (0x76<<1)+1;
//		while(!(I2C1->SR1 & I2C_SR1_ADDR));
//		asd = I2C1->SR2;
//
//		I2C1->CR1 &= ~I2C_CR1_ACK;
//
//		//I2C1->DR = 0xD0;
//
//		I2C1->CR1 |= I2C_CR1_STOP;
//
//		while(!(I2C1->SR1 & I2C_SR1_RXNE));
//
////		if(I2C1->DR != 0x58){
////			GPIOC->BSRR |= GPIO_BSRR_BR8;
////		}else{
////			GPIOC->BSRR |= GPIO_BSRR_BS8;
////		}
//		char buf[6];
//		itoa(I2C1->DR, buf, 10);
//		debug_uart_puts(buf);

//		char message[] = "green text";
//		Debug_log(debug_module_CORE,debug_type_IMPORTANT,message,1);
//		Debug_log(debug_module_CORE,debug_type_ERROR,message,1);
//		Debug_log(debug_module_CORE,debug_type_WARNING,message,1);
		LoopDelay(10000000);
		DEBUG_UART_EVENT(debug_uart_framebuf);

	}
}

void I2C1_ER_IRQHandler(void){

		debug_uart_puts("eer");
}

uint8_t recieved_data[10];

void I2C1_EV_IRQHandler(void){

	if(I2C1->SR1 & I2C_SR1_SB){
		if(!I2C1_Handler.restart) I2C1->DR = I2C1_Handler.slave_address<<1;
		else I2C1->DR = (I2C1_Handler.slave_address<<1) + 1;
	}

	if(I2C1->SR1 & I2C_SR1_ADDR){
		uint32_t asd = I2C1->SR2;
		if(I2C1_Handler.transmit)I2C1->DR = I2C1_Handler.data[0];
		else{
			if(!I2C1_Handler.restart){
				I2C1->DR = I2C1_Handler.reg_address;
				I2C1_Handler.restart = 1;
			}
		}

		if(I2C1_Handler.data_len == 1 && !I2C1_Handler.transmit && I2C1_Handler.restart == 1){
			I2C1->CR1 &= ~I2C_CR1_ACK;
			I2C1->CR1 |= I2C_CR1_STOP;
		}
	}

	if(I2C1_Handler.transmit==0 && I2C1_Handler.restart==1 &&(I2C1->SR1 & I2C_SR1_RXNE)){

		I2C1_Handler.data_pointer++;

		if(I2C1->DR != 0xaa){
			GPIOC->BSRR |= GPIO_BSRR_BR8;
		}else{
			GPIOC->BSRR |= GPIO_BSRR_BS8;
		}
		if(I2C1_Handler.data_len-1 == I2C1_Handler.data_pointer && !I2C1_Handler.transmit){
			I2C1->CR1 &= ~I2C_CR1_ACK;
			I2C1->CR1 |= I2C_CR1_STOP;
		}else{
			I2C1->CR1 |= I2C_CR1_ACK;
		}
	}

	if(I2C1->SR1 & I2C_SR1_TXE){

		if(I2C1_Handler.transmit){
			if(I2C1_Handler.data_pointer == I2C1_Handler.data_len-1){
				I2C1->CR1 |= I2C_CR1_STOP;
				I2C1_Handler.tx_flag = 1;
			}
			I2C1_Handler.data_pointer++;
			I2C1->DR = I2C1_Handler.data[I2C1_Handler.data_pointer];
		}else{
			if(I2C1_Handler.restart==1){
				I2C1->CR1 |= I2C_CR1_START;
			}
		}
	}

}
