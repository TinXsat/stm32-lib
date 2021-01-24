#include <string.h>
#include "stm32f10x.h"

#include "SX1278_hw.h"


uint8_t SPI_transmit_recieve( uint8_t byte ){
	SPI2->DR = byte;
	while(!(SPI2->SR & SPI_SR_RXNE));
	return SPI2->DR;
}

void SX1278_hw_init(SX1278_hw_t *hw) {
	SX1278_hw_SetNSS(hw, 1);
	//HAL_GPIO_WritePin(hw->reset.port, hw->reset.pin, GPIO_PIN_SET);
}

void SX1278_hw_SetNSS(SX1278_hw_t *hw, int value) {
	if(value == 1){
		GPIOC->BSRR = GPIO_BSRR_BS6;
	}else if(value == 0){
		GPIOC->BSRR = GPIO_BSRR_BR6;
	}
}

void SX1278_hw_Reset(SX1278_hw_t *hw) {
	SX1278_hw_SetNSS(hw, 1);
	//HAL_GPIO_WritePin(hw->reset.port, hw->reset.pin, GPIO_PIN_RESET);

	SX1278_hw_DelayMs(1);

	//HAL_GPIO_WritePin(hw->reset.port, hw->reset.pin, GPIO_PIN_SET);

	SX1278_hw_DelayMs(100);
}

void SX1278_hw_SPICommand(SX1278_hw_t *hw, uint8_t cmd) {
	SX1278_hw_SetNSS(hw, 0);
	SPI_transmit_recieve(cmd);
	//while (HAL_SPI_GetState(hw->spi) != HAL_SPI_STATE_READY)
		;
}

uint8_t SX1278_hw_SPIReadByte(SX1278_hw_t *hw) {
	uint8_t txByte = 0x00;
	uint8_t rxByte = 0x00;

	SX1278_hw_SetNSS(hw, 0);
	rxByte = SPI_transmit_recieve(txByte);
	//while (HAL_SPI_GetState(hw->spi) != HAL_SPI_STATE_READY)
	//	;
	return rxByte;
}

void LoopDelay2(volatile uint32_t n) {
	while(n > 0) n--;
}

void SX1278_hw_DelayMs(uint32_t msec) {
	LoopDelay2(10000);
}

int SX1278_hw_GetDIO0(SX1278_hw_t *hw) {
	return (GPIOB->IDR & GPIO_IDR_IDR2)>>2;//(HAL_GPIO_ReadPin(hw->dio0.port, hw->dio0.pin) == GPIO_PIN_SET);
}

