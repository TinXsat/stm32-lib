#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "stm32f10x.h"

#include "I2C_Handler.h"

void I2C_LLInit( T_I2C_HANDLER* I2C_Handler ){
	I2C_Handler->I2C_periph->CR2 |= 0x20;							//freq of cpu
	I2C_Handler->I2C_periph->CCR |= 0x80;

	I2C_Handler->I2C_periph->CR2 |= I2C_CR2_ITEVTEN;				//enable interrupts status
	I2C_Handler->I2C_periph->CR2 |= I2C_CR2_ITERREN;				//enable interrupts error
	I2C_Handler->I2C_periph->CR2 |= I2C_CR2_ITBUFEN;				//enable interrupts buffer

	I2C_Handler->I2C_periph->CR1 |= (I2C_CR1_PE);					//Peripheral enabled
}

void I2C_Handler_init(T_I2C_HANDLER* I2C_Handler, T_I2C_DEVICE* devices_t, I2C_TypeDef* I2C_periph_t, uint16_t* buf, void (*callback)(T_I2C_DEVICE* device, T_I2C_ERROR_CODE error_code)){
	I2C_LLInit(I2C_Handler);
	I2C_Handler->devices = devices_t;
	I2C_Handler->data = buf;
	I2C_Handler->I2C_periph = I2C_periph_t;
	I2C_Handler->i2c1_error_event_callback = callback;
}

void I2C_EnumerateDevices(T_I2C_HANDLER* I2C_Handler, uint8_t start_addr, uint8_t stop_addr, void (*callback)(T_I2C_HANDLER* i2c_handler)){
	I2C_Handler->mode = 1;	//go to enumerate mode
	I2C_Handler->enumeration_start_address = start_addr;
	I2C_Handler->enumeration_stop_address = stop_addr;
	I2C_Handler->i2c1_enumerated_event_callback = callback;
}

void I2C_WriteConfgurationAdd(T_I2C_DEVICE * device, uint16_t * data , uint8_t reg_cnt){
	device->write_configuration = 1;
	device->configuration = data;
	device->configuration_len = reg_cnt;
}

void I2C_WriteConfguration(T_I2C_HANDLER* I2C_Handler, void (*callback)(T_I2C_HANDLER* i2c_handler)){
	I2C_Handler->i2c1_configured_event_callback = callback;
	I2C_Handler->mode = 2;
}

void I2C_ReadSensorAdd( T_I2C_DEVICE * device, uint8_t start_reg , uint8_t reg_cnt ){
	device->read_data = 1;
	device->read_start_reg = start_reg;
	device->read_cnt_reg = reg_cnt;
}

void I2C_ReadSensor(T_I2C_HANDLER* I2C_Handler, void (*callback)(T_I2C_DEVICE* device, uint16_t * data) ){
	I2C_Handler->i2c1_read_event_callback = callback;
	I2C_Handler->mode = 3;
}

void I2C_write(T_I2C_HANDLER* I2C_Handler, uint8_t slave_address, uint16_t * data, uint8_t data_len){
	//I2C_Handler->data = data[0];
	for(uint8_t i = 0; i<data_len; i++){
		I2C_Handler->data[i] = data[i];
	}
	I2C_Handler->data_len = data_len;
	I2C_Handler->slave_address = slave_address;
	I2C_Handler->transmit = 1;
	I2C_Handler->restart = 0;
	I2C_Handler->data_pointer = 0;
	I2C_Handler->I2C_periph->CR1 |= I2C_CR1_START;
}

void I2C_read(T_I2C_HANDLER* I2C_Handler, uint8_t slave_address, uint8_t reg_address, uint8_t data_len){
	I2C_Handler->data_len = data_len;
	I2C_Handler->slave_address = slave_address;
	I2C_Handler->reg_address = reg_address;
	I2C_Handler->transmit = 0;
	I2C_Handler->restart = 0;
	I2C_Handler->data_pointer = 0;
	I2C_Handler->I2C_periph->CR1 |= I2C_CR1_START;
}

void I2C_Handler_Main(T_I2C_HANDLER* I2C_Handler){

	if(I2C_Handler->mode == 1){	//if we are enumerating
		if(I2C_Handler->old_mode != I2C_Handler->mode){ //first time
			for(uint8_t i = 0; i<5; i++){
				I2C_Handler->devices[i].responded = 0;
			}
			I2C_Handler->enumeration_address = I2C_Handler->enumeration_start_address;
			I2C_Handler->found_enumerated=0;
			I2C_write(I2C_Handler, I2C_Handler->enumeration_address, 0, 1);
		}
		if(I2C_Handler->tx_flag || I2C_Handler->error_flag){

			if(I2C_Handler->enumeration_address > I2C_Handler->enumeration_stop_address){
				I2C_Handler->tx_flag = I2C_Handler->error_flag = 0;
				I2C_Handler->mode = 0;
				//////
				I2C_Handler->I2C_periph->CR1 |= I2C_CR1_STOP;

				if(I2C_Handler->i2c1_enumerated_event_callback != 0) I2C_Handler->i2c1_enumerated_event_callback();

			}
		}
		if(I2C_Handler->tx_flag){			//ack
			I2C_Handler->tx_flag = 0;
			I2C_read(I2C_Handler, I2C_Handler->enumeration_address, 0xD0, 1);
		}
		if(I2C_Handler->error_flag){		//nack
			I2C_Handler->enumeration_address++;
			I2C_Handler->error_flag = 0;
			uint16_t asd[3] = {0xF4, 0xaa,0x0f};
			I2C_write(I2C_Handler, I2C_Handler->enumeration_address, asd, 1);
		}
		if(I2C_Handler->rx_flag){			//register val
			I2C_Handler->rx_flag = 0;

			I2C_Handler->devices[I2C_Handler->found_enumerated].responded = 1;
			I2C_Handler->devices[I2C_Handler->found_enumerated].slave_address = I2C_Handler->enumeration_address;
			I2C_Handler->devices[I2C_Handler->found_enumerated].D0_value = I2C_Handler->data[0];
			I2C_Handler->found_enumerated++;

			uint16_t data[2] = {0xF4, 0xaa};
			I2C_Handler->enumeration_address++;
			I2C_write(I2C_Handler, I2C_Handler->enumeration_address, data, 1);

		}
	}

	if(I2C_Handler->mode == 2){
		if(I2C_Handler->old_mode != I2C_Handler->mode){ //first time

			I2C_Handler->now_handled_device = 0;
			I2C_Handler->now_handled_register = 0;

			uint8_t found = 0;
			for(uint8_t i=I2C_Handler->now_handled_device; i<5; i++){
				if(I2C_Handler->devices[i].write_configuration == 1){
					I2C_Handler->now_handled_device = i;
					found = 1;
					break;
				}
			}

			if(!found){
				I2C_Handler->mode = 0;
				I2C_Handler->now_handled_register = 0;
				I2C_Handler->now_handled_device = 0;
			}

			if(I2C_Handler->devices[I2C_Handler->now_handled_device].write_configuration == 1){

				I2C_Handler->data[0] = I2C_Handler->devices[I2C_Handler->now_handled_device].configuration[I2C_Handler->now_handled_register*2];
				if(I2C_Handler->devices[I2C_Handler->now_handled_device].is_16_bit == 1){
					I2C_Handler->data[1] = I2C_Handler->devices[I2C_Handler->now_handled_device].configuration[(I2C_Handler->now_handled_register*2)+1]>>8;
					I2C_Handler->data[2] = (uint8_t)I2C_Handler->devices[I2C_Handler->now_handled_device].configuration[(I2C_Handler->now_handled_register*2)+1];
					I2C_write(I2C_Handler, I2C_Handler->devices[I2C_Handler->now_handled_device].slave_address, I2C_Handler->data,3);
				}else{
					I2C_Handler->data[1] = I2C_Handler->devices[I2C_Handler->now_handled_device].configuration[(I2C_Handler->now_handled_register*2)+1];
					I2C_write(I2C_Handler, I2C_Handler->devices[I2C_Handler->now_handled_device].slave_address, I2C_Handler->data,2);
				}

			}
		}

		if(I2C_Handler->tx_flag){			//ack
			I2C_Handler->tx_flag = 0;
			if(I2C_Handler->devices[I2C_Handler->now_handled_device].configuration_len-1 == I2C_Handler->now_handled_register){
				//sent all regs for thisnsor
				I2C_Handler->devices[I2C_Handler->now_handled_device].write_configuration = 0;

				uint8_t found = 0;
				for(uint8_t i=I2C_Handler->now_handled_device; i<5; i++){
					if(I2C_Handler->devices[i].write_configuration == 1){
						I2C_Handler->now_handled_device = i;
						I2C_Handler->now_handled_register = 0;
						found = 1;
						break;
					}
				}

				if(!found){
					I2C_Handler->mode = 0;
					I2C_Handler->now_handled_register = 0;
					I2C_Handler->now_handled_device = 0;
					if(I2C_Handler->i2c1_configured_event_callback != 0) I2C_Handler->i2c1_configured_event_callback();

				}
			}else{
				I2C_Handler->now_handled_register++;
			}

			if(I2C_Handler->devices[I2C_Handler->now_handled_device].write_configuration == 1){
				I2C_Handler->data[0] = I2C_Handler->devices[I2C_Handler->now_handled_device].configuration[(I2C_Handler->now_handled_register*2)];
				if(I2C_Handler->devices[I2C_Handler->now_handled_device].type == 3){//16 bit access
					I2C_Handler->data[1] = I2C_Handler->devices[I2C_Handler->now_handled_device].configuration[(I2C_Handler->now_handled_register*2)+1]>>8;
				}else{
					I2C_Handler->data[1] = I2C_Handler->devices[I2C_Handler->now_handled_device].configuration[(I2C_Handler->now_handled_register*2)+1];
				}

				if(I2C_Handler->devices[I2C_Handler->now_handled_device].is_16_bit == 1){
					I2C_Handler->data[1] = I2C_Handler->devices[I2C_Handler->now_handled_device].configuration[(I2C_Handler->now_handled_register*2)+1]>>8;
					I2C_Handler->data[2] = (uint8_t)I2C_Handler->devices[I2C_Handler->now_handled_device].configuration[(I2C_Handler->now_handled_register*2)+1];
					I2C_write(I2C_Handler, I2C_Handler->devices[I2C_Handler->now_handled_device].slave_address, I2C_Handler->data,3);
				}else{
					I2C_Handler->data[1] = I2C_Handler->devices[I2C_Handler->now_handled_device].configuration[(I2C_Handler->now_handled_register*2)+1];
					I2C_write(I2C_Handler, I2C_Handler->devices[I2C_Handler->now_handled_device].slave_address, I2C_Handler->data,2);
				}
			}
		}
		if(I2C_Handler->error_flag){		//nack
			I2C_Handler->error_flag= 0;
			I2C_Handler->mode = 0;
			if(I2C_Handler->i2c1_error_event_callback != 0) I2C_Handler->i2c1_error_event_callback(&I2C_Handler->devices[I2C_Handler->now_handled_device],I2C_Handler->error_code);
		}

	}

	if(I2C_Handler->mode == 3){
		static uint16_t aux16data[30];

		if(I2C_Handler->old_mode != I2C_Handler->mode){ //first time
			I2C_Handler->now_handled_device = 0;
			I2C_Handler->now_handled_register = 0;

			uint8_t found = 0;
			for(uint8_t i=I2C_Handler->now_handled_device; i<5; i++){
				if(I2C_Handler->devices[i].read_data == 1){
					I2C_Handler->now_handled_device = i;
					found = 1;
					break;
				}
			}

			if(!found){
				I2C_Handler->mode = 0;
				I2C_Handler->now_handled_register = 0;
				I2C_Handler->now_handled_device = 0;
			}

			if(I2C_Handler->devices[I2C_Handler->now_handled_device].read_data == 1){
				if(I2C_Handler->devices[I2C_Handler->now_handled_device].is_16_bit == 1){
					I2C_read(I2C_Handler, I2C_Handler->devices[I2C_Handler->now_handled_device].slave_address, I2C_Handler->devices[I2C_Handler->now_handled_device].read_start_reg, 2);
				}else{
					I2C_read(I2C_Handler, I2C_Handler->devices[I2C_Handler->now_handled_device].slave_address, I2C_Handler->devices[I2C_Handler->now_handled_device].read_start_reg, I2C_Handler->devices[I2C_Handler->now_handled_device].read_cnt_reg);
				}
			}
		}

		if(I2C_Handler->rx_flag){			//ack
			I2C_Handler->rx_flag = 0;

			if(I2C_Handler->devices[I2C_Handler->now_handled_device].is_16_bit == 0){
				//8bit so go to next sensor
				I2C_Handler->devices[I2C_Handler->now_handled_device].read_data = 0;
				if(I2C_Handler->i2c1_read_event_callback != 0) I2C_Handler->i2c1_read_event_callback(&I2C_Handler->devices[I2C_Handler->now_handled_device],I2C_Handler->data);

				uint8_t found = 0;
				for(uint8_t i=I2C_Handler->now_handled_device; i<5; i++){
					if(I2C_Handler->devices[i].read_data == 1){
						I2C_Handler->now_handled_device = i;
						found = 1;
						break;
					}
				}

				if(!found){
					I2C_Handler->mode = 0;
					I2C_Handler->now_handled_register = 0;
					I2C_Handler->now_handled_device = 0;
				}else{
					if(I2C_Handler->devices[I2C_Handler->now_handled_device].read_data == 1){
						if(I2C_Handler->devices[I2C_Handler->now_handled_device].is_16_bit == 1){
							I2C_read(I2C_Handler, I2C_Handler->devices[I2C_Handler->now_handled_device].slave_address, I2C_Handler->devices[I2C_Handler->now_handled_device].read_start_reg, 2);
						}else{
							I2C_read(I2C_Handler, I2C_Handler->devices[I2C_Handler->now_handled_device].slave_address, I2C_Handler->devices[I2C_Handler->now_handled_device].read_start_reg, I2C_Handler->devices[I2C_Handler->now_handled_device].read_cnt_reg);
						}
					}
				}
			}else{
				//16bit go to next register
				aux16data[I2C_Handler->now_handled_register] = (I2C_Handler->data[0]<<8) + I2C_Handler->data[1];

				if(I2C_Handler->devices[I2C_Handler->now_handled_device].read_cnt_reg-1 == I2C_Handler->now_handled_register){

					for(uint8_t i = 0; i<I2C_Handler->devices[I2C_Handler->now_handled_device].read_cnt_reg+1; i++){
						I2C_Handler->data[i] = aux16data[i];
					}
					I2C_Handler->devices[I2C_Handler->now_handled_device].read_data = 0;
					if(I2C_Handler->i2c1_read_event_callback != 0) I2C_Handler->i2c1_read_event_callback(&I2C_Handler->devices[I2C_Handler->now_handled_device],I2C_Handler->data);

					uint8_t found = 0;
					for(uint8_t i=I2C_Handler->now_handled_device; i<5; i++){
						if(I2C_Handler->devices[i].read_data == 1){
							I2C_Handler->now_handled_device = i;
							found = 1;
							break;
						}
					}

					if(!found){
						I2C_Handler->mode = 0;
						I2C_Handler->now_handled_register = 0;
						I2C_Handler->now_handled_device = 0;
					}else{
						if(I2C_Handler->devices[I2C_Handler->now_handled_device].read_data == 1){
							if(I2C_Handler->devices[I2C_Handler->now_handled_device].is_16_bit == 1){
								I2C_read(I2C_Handler, I2C_Handler->devices[I2C_Handler->now_handled_device].slave_address, I2C_Handler->devices[I2C_Handler->now_handled_device].read_start_reg, 2);
							}else{
								I2C_read(I2C_Handler, I2C_Handler->devices[I2C_Handler->now_handled_device].slave_address, I2C_Handler->devices[I2C_Handler->now_handled_device].read_start_reg, I2C_Handler->devices[I2C_Handler->now_handled_device].read_cnt_reg);
							}
						}
					}

				}else{
					I2C_Handler->now_handled_register++;
					I2C_read(I2C_Handler, I2C_Handler->devices[I2C_Handler->now_handled_device].slave_address,I2C_Handler->devices[I2C_Handler->now_handled_device].read_start_reg+I2C_Handler->now_handled_register, 2);
				}
			}
		}

		if(I2C_Handler->tx_flag){			//ack
			I2C_Handler->tx_flag = 0;
		}
		if(I2C_Handler->error_flag){		//nack
			I2C_Handler->error_flag= 0;
			I2C_Handler->mode = 0;
			if(I2C_Handler->i2c1_error_event_callback != 0) I2C_Handler->i2c1_error_event_callback(&I2C_Handler->devices[I2C_Handler->now_handled_device],I2C_Handler->error_code);
		}

	}

	I2C_Handler->old_mode = I2C_Handler->mode;
}

void I2C_Handler_ERIRQ( T_I2C_HANDLER* I2C_Handler ){

	if(I2C_Handler->I2C_periph->SR1 & I2C_SR1_AF){
		I2C_Handler->I2C_periph->SR1 &= ~I2C_SR1_AF;
		if(I2C_Handler->mode == 3){
			I2C_Handler->I2C_periph->CR1 |= I2C_CR1_STOP;
		}
		I2C_Handler->error_code = 0;
	}
	if(I2C_Handler->I2C_periph->SR1 & I2C_SR1_BERR){
		I2C_Handler->I2C_periph->SR1 &= ~I2C_SR1_BERR;
//		I2C_Handler->I2C_periph->CR1 |= I2C_CR1_SWRST;
//		I2C_Handler->I2C_periph->CR1 &= ~I2C_CR1_SWRST;
		I2C_Handler->error_code = 1;
	}
	if(I2C_Handler->I2C_periph->SR1 & I2C_SR1_ARLO){
		I2C_Handler->I2C_periph->SR1 &= ~I2C_SR1_ARLO;
//		I2C_Handler->I2C_periph->CR1 |= I2C_CR1_SWRST;
//		I2C_Handler->I2C_periph->CR1 &= ~I2C_CR1_SWRST;
		I2C_Handler->error_code = 2;
	}
	if(I2C_Handler->I2C_periph->SR1 & I2C_SR1_OVR){
		I2C_Handler->I2C_periph->SR1 &= ~I2C_SR1_OVR;
//		I2C_Handler->I2C_periph->CR1 |= I2C_CR1_SWRST;
//		I2C_Handler->I2C_periph->CR1 &= ~I2C_CR1_SWRST;
		I2C_Handler->error_code = 3;
	}
	if(I2C_Handler->I2C_periph->SR1 & I2C_SR1_PECERR){
		I2C_Handler->I2C_periph->SR1 &= ~I2C_SR1_PECERR;
//		I2C_Handler->I2C_periph->CR1 |= I2C_CR1_SWRST;
//		I2C_Handler->I2C_periph->CR1 &= ~I2C_CR1_SWRST;
		I2C_Handler->error_code = 4;
	}
	I2C_Handler->error_flag = 1;
}

void I2C_Handler_EVIRQ( T_I2C_HANDLER* I2C_Handler ){

	if(I2C_Handler->I2C_periph->SR1 & I2C_SR1_STOPF){
		I2C_Handler->error_code = 5;
		I2C_Handler->error_flag = 1;
	}

	if(I2C_Handler->I2C_periph->SR1 & I2C_SR1_SB){
		if(!I2C_Handler->restart) I2C_Handler->I2C_periph->DR = I2C_Handler->slave_address<<1;
		else I2C_Handler->I2C_periph->DR = (I2C_Handler->slave_address<<1) + 1;
	}

	if(I2C_Handler->I2C_periph->SR1 & I2C_SR1_ADDR){
		uint32_t asd = I2C_Handler->I2C_periph->SR2;
		if(I2C_Handler->transmit){
			if(I2C_Handler->data_pointer == I2C_Handler->data_len-1){
				I2C_Handler->I2C_periph->CR1 |= I2C_CR1_STOP;
				I2C_Handler->tx_flag = 1;
			}
			I2C_Handler->I2C_periph->DR = I2C_Handler->data[0];
		}else{

			if(I2C_Handler->data_len == 1 && I2C_Handler->restart == 1){
				I2C_Handler->I2C_periph->CR1 &= ~I2C_CR1_ACK;
				I2C_Handler->I2C_periph->CR1 |= I2C_CR1_STOP;
			}else{
				I2C_Handler->I2C_periph->CR1 |= I2C_CR1_ACK;
			}

			if(!I2C_Handler->restart){
				I2C_Handler->I2C_periph->DR = I2C_Handler->reg_address;
				I2C_Handler->restart = 1;
			}
		}
	}

	if(I2C_Handler->transmit==0 && I2C_Handler->restart==1 &&(I2C_Handler->I2C_periph->SR1 & I2C_SR1_RXNE)){

		I2C_Handler->data[I2C_Handler->data_pointer]= I2C_Handler->I2C_periph->DR;
		I2C_Handler->data_pointer++;

		if(I2C_Handler->data_len-1 == I2C_Handler->data_pointer){
			I2C_Handler->I2C_periph->CR1 &= ~I2C_CR1_ACK;
			I2C_Handler->I2C_periph->CR1 |= I2C_CR1_STOP;
		}else if(I2C_Handler->data_len == I2C_Handler->data_pointer){
			I2C_Handler->rx_flag = 1;
		}else{
			I2C_Handler->I2C_periph->CR1 |= I2C_CR1_ACK;
		}

	}

	if(I2C_Handler->I2C_periph->SR1 & I2C_SR1_TXE){

		if(I2C_Handler->transmit){
			if(I2C_Handler->data_pointer > I2C_Handler->data_len-1){
				I2C_Handler->tx_flag = 1;
			}
			if(I2C_Handler->data_pointer <= I2C_Handler->data_len-1){
				if(I2C_Handler->data_pointer == I2C_Handler->data_len-1){
					I2C_Handler->I2C_periph->CR1 |= I2C_CR1_STOP;
				}
				I2C_Handler->data_pointer++;
				I2C_Handler->I2C_periph->DR = I2C_Handler->data[I2C_Handler->data_pointer];
			}
		}else{
			if(I2C_Handler->restart==1){
				I2C_Handler->I2C_periph->CR1 |= I2C_CR1_START;
			}
		}
	}
}
