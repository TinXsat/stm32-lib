#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "stm32f10x.h"

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

uint8_t buf[5];
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
	uint8_t rx_flag;
	uint8_t error_flag;

	uint8_t mode; //0=nothing 1=enumerate
	uint8_t enumeration_address;
	uint8_t enumeration_start_address;
	uint8_t enumeration_stop_address;
	void (*i2c1_found_device_event_callback)(uint8_t slave_address);
	uint8_t found_enumerated;

}I2C_Handler;

I2C_Handler I2C1_Handler;

void I2C1_EnumerateDevices( uint8_t start_addr, uint8_t stop_addr, void (*callback)(uint8_t slave_address)){
	I2C1_Handler.mode = 1;	//go to enumerate mode
	I2C1_Handler.enumeration_start_address = start_addr;
	I2C1_Handler.enumeration_stop_address = stop_addr;
	I2C1_Handler.i2c1_found_device_event_callback = callback;
}

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
	I2C1_Handler.data=buf;
	I2C1->CR1 |= I2C_CR1_START;
	return I2C1_Handler.message_id;
}

void found_device(uint8_t slave_address){
//	I2C_read(0x76, 0xD0, 2);
//	GPIOC->ODR ^= GPIO_ODR_ODR8;
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

	debug_uart_init();
	register_debug_uart_event_callback(debug_uart_str_event);
	NVIC_EnableIRQ(USART1_IRQn);
	NVIC_EnableIRQ(I2C1_ER_IRQn);
	NVIC_EnableIRQ(I2C1_EV_IRQn);
	NVIC_EnableIRQ(TIM6_IRQn);

	while (1){

		I2C1_EnumerateDevices(0x70, 0x78, found_device);

		//I2C_read(0x76,0xD0,1);

		LoopDelay(10000000);
		DEBUG_UART_EVENT(debug_uart_framebuf);

	}
}

uint8_t recieved_data[10];

typedef struct T_I2C_SENSOR{
	uint8_t responded;
	uint8_t slave_address;
	uint8_t type;	//0-unknown 1-bmp280 2-bmp388 3-ina219
}T_I2C_SENSOR;

T_I2C_SENSOR I2C1_sensors[5];

void TIM6_IRQHandler(void){
	TIM6->SR &= ~TIM_SR_UIF;

	static uint8_t old_mode;

	if(I2C1_Handler.mode == 1){	//if we are enumerating
		if(old_mode != I2C1_Handler.mode){ //first time
			I2C1_Handler.enumeration_address = I2C1_Handler.enumeration_start_address;
			I2C_write(I2C1_Handler.enumeration_address, 0, 0);
		}
		if(I2C1_Handler.tx_flag || I2C1_Handler.error_flag){
			if(I2C1_Handler.enumeration_address > I2C1_Handler.enumeration_stop_address){
				I2C1_Handler.tx_flag = I2C1_Handler.error_flag = 0;
				I2C1_Handler.mode = 0;
				I2C1->CR1 |= I2C_CR1_STOP;

				for(uint8_t i = 0; i<5; i++){
					if(I2C1_sensors[i].responded == 1){
						debug_uart_puts("found sensor: ");
						if(I2C1_sensors[i].type == 1) debug_uart_puts("BMP388");
						debug_uart_puts(" on address: ");
						char chr[5];
						itoa(I2C1_sensors[i].slave_address,chr,16);
						debug_uart_puts(chr);
						debug_uart_puts("\r\n");
						I2C1_Handler.found_enumerated=0;
						I2C1_sensors[i].responded = 0;
					}
				}

			}
		}
		if(I2C1_Handler.tx_flag){			//ack
			I2C1_Handler.tx_flag = 0;

			if(I2C1_Handler.enumeration_address == 0x76 || I2C1_Handler.enumeration_address == 0x77){
				//bmp280 or bmp388
				I2C_read(I2C1_Handler.enumeration_address, 0xD0, 1);
			}else if((I2C1_Handler.enumeration_address & 0xf0) == 0x40){
				//ina
				I2C1_sensors[I2C1_Handler.found_enumerated].responded = 1;
				I2C1_sensors[I2C1_Handler.found_enumerated].slave_address = I2C1_Handler.enumeration_address;
				I2C1_sensors[I2C1_Handler.found_enumerated].type = 3;
				I2C1_Handler.found_enumerated++;

				I2C1_Handler.enumeration_address++;
				uint8_t data[2] = {0xF4, 0xaa};
				I2C_write(I2C1_Handler.enumeration_address, data, 1);
			}else{
				I2C1_Handler.enumeration_address++;
				uint8_t data[2] = {0xF4, 0xaa};
				I2C_write(I2C1_Handler.enumeration_address, data, 1);
			}
		}
		if(I2C1_Handler.error_flag){		//nack
			I2C1_Handler.enumeration_address++;
			I2C1_Handler.error_flag = 0;
			uint8_t asd[3] = {0xF4, 0xaa,0x0f};
			I2C_write(I2C1_Handler.enumeration_address, asd, 1);
		}
		if(I2C1_Handler.rx_flag){			//register val
			I2C1_Handler.rx_flag = 0;

			I2C1_sensors[I2C1_Handler.found_enumerated].responded = 1;
			I2C1_sensors[I2C1_Handler.found_enumerated].slave_address = I2C1_Handler.enumeration_address;
			if(I2C1_Handler.data[0] == 0x58)I2C1_sensors[I2C1_Handler.found_enumerated].type = 1;
			else I2C1_sensors[I2C1_Handler.found_enumerated].type = 2;
			I2C1_Handler.found_enumerated++;
			uint8_t data[2] = {0xF4, 0xaa};
			I2C1_Handler.enumeration_address++;
			I2C_write(I2C1_Handler.enumeration_address, data, 1);

		}
	}
	old_mode = I2C1_Handler.mode;

}

void I2C1_ER_IRQHandler(void){
	if(I2C1->SR1 & I2C_SR1_AF){
		I2C1->SR1 &= ~I2C_SR1_AF;
	}
	if(I2C1->SR1 & I2C_SR1_BERR){
		I2C1->SR1 &= ~I2C_SR1_BERR;
		I2C1->CR1 |= I2C_CR1_SWRST;
		I2C1->CR1 &= ~I2C_CR1_SWRST;
		I2c_init();
	}
	if(I2C1->SR1 & I2C_SR1_ARLO){
		I2C1->SR1 &= ~I2C_SR1_ARLO;
		I2C1->CR1 |= I2C_CR1_SWRST;
		I2C1->CR1 &= ~I2C_CR1_SWRST;
		I2c_init();
	}
	if(I2C1->SR1 & I2C_SR1_OVR){
		I2C1->SR1 &= ~I2C_SR1_OVR;
		I2C1->CR1 |= I2C_CR1_SWRST;
		I2C1->CR1 &= ~I2C_CR1_SWRST;
		I2c_init();
	}
	if(I2C1->SR1 & I2C_SR1_PECERR){
		I2C1->SR1 &= ~I2C_SR1_PECERR;
		I2C1->CR1 |= I2C_CR1_SWRST;
		I2C1->CR1 &= ~I2C_CR1_SWRST;
		I2c_init();
	}
	I2C1_Handler.error_flag = 1;
	debug_uart_puts("er");
}

void I2C1_EV_IRQHandler(void){

	if(I2C1->SR1 & I2C_SR1_SB){
		if(!I2C1_Handler.restart) I2C1->DR = I2C1_Handler.slave_address<<1;
		else I2C1->DR = (I2C1_Handler.slave_address<<1) + 1;
	}

	if(I2C1->SR1 & I2C_SR1_ADDR){
		uint32_t asd = I2C1->SR2;
		if(I2C1_Handler.transmit){
			if(I2C1_Handler.data_pointer == I2C1_Handler.data_len-1){
				I2C1->CR1 |= I2C_CR1_STOP;
				I2C1_Handler.tx_flag = 1;
			}
			I2C1->DR = I2C1_Handler.data[0];
		}else{

			if(I2C1_Handler.data_len == 1 && I2C1_Handler.restart == 1){
				I2C1->CR1 &= ~I2C_CR1_ACK;
				I2C1->CR1 |= I2C_CR1_STOP;
			}

			if(!I2C1_Handler.restart){
				I2C1->DR = I2C1_Handler.reg_address;
				I2C1_Handler.restart = 1;
			}
		}
	}

	if(I2C1_Handler.transmit==0 && I2C1_Handler.restart==1 &&(I2C1->SR1 & I2C_SR1_RXNE)){

		I2C1_Handler.data[I2C1_Handler.data_pointer]= I2C1->DR;
		I2C1_Handler.data_pointer++;

		if(I2C1_Handler.data_len == I2C1_Handler.data_pointer && !I2C1_Handler.transmit){
			I2C1->CR1 &= ~I2C_CR1_ACK;
			I2C1->CR1 |= I2C_CR1_STOP;
			I2C1_Handler.rx_flag = 1;
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
