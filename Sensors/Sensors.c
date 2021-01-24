#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "stm32f10x.h"

#include <common.h>
#include <Drivers/I2C_Handler/I2C_Handler.h>
#include "BMP280/BMP280.h"

#include <Misc/Status_Led/Status_Led.h>

#include "Sensors.h"

T_SENSORS Sensors;
T_SENSORS_DATA Sensors_data;

T_BMP280 BMP280_sensors[2];
T_INA219 INA219_sensors[2];

void Sensors_Aquire( uint8_t true ){
	if(true) Sensors.run = 1; else Sensors.run = 0;
}

void Sensors_init1(T_BMP280 *bmp280_sensors, uint8_t bmp280_number, T_INA219 *ina219_sensors, uint8_t ina219_number){
	Sensors.BMP280_sensors = bmp280_sensors;
	Sensors.bmp280_number = bmp280_number;

	Sensors.INA219_sensors = ina219_sensors;
	Sensors.ina219_number = ina219_number;
}

void bmp280_data_ready(){
	char str[10];

	HsvColor bat_col;
	bat_col.s = 255;
	bat_col.v = 255;

	bat_col.h = map(Sensors_data.voltage[0], 2900, 4200, 0, 85);

	RgbColor bat2_col = HsvToRgb(bat_col);

	StatusLed_setColorRGB(bat2_col.r*255,bat2_col.g*255,bat2_col.b*255);

	USART_HandlerPuts( &USART1_Handler, "voltage0:");
	itoa(Sensors_data.voltage[0], str,10);
	USART_HandlerPuts( &USART1_Handler, str);
	USART_HandlerPuts( &USART1_Handler, "mV ");

	USART_HandlerPuts( &USART1_Handler, "voltage1:");
	itoa(Sensors_data.voltage[1], str,10);
	USART_HandlerPuts( &USART1_Handler, str);
	USART_HandlerPuts( &USART1_Handler, "mV ");

	USART_HandlerPuts( &USART1_Handler, "current0:");
	itoa(Sensors_data.current[0], str,10);
	USART_HandlerPuts( &USART1_Handler, str);
	USART_HandlerPuts( &USART1_Handler, "mA ");

	USART_HandlerPuts( &USART1_Handler, "current1:");
	itoa(Sensors_data.current[1], str,10);
	USART_HandlerPuts( &USART1_Handler, str);
	USART_HandlerPuts( &USART1_Handler, "mA ");

	itoa((Sensors_data.pressure[0]/256)/100, str,10);
	USART_HandlerPuts( &USART1_Handler, str);
	USART_HandlerPuts( &USART1_Handler, ".");
	Sensors_data.pressure[0] = (Sensors_data.pressure[0]/256)%100;
	if(Sensors_data.pressure[0] < 10) USART_HandlerPuts( &USART1_Handler, "0");
	itoa(Sensors_data.pressure[0], str,10);
	USART_HandlerPuts( &USART1_Handler, str);
	USART_HandlerPuts( &USART1_Handler, "hPa  press1:");

	itoa((Sensors_data.pressure[1]/256)/100, str,10);
	USART_HandlerPuts( &USART1_Handler, str);
	USART_HandlerPuts( &USART1_Handler, ".");
	Sensors_data.pressure[1] = (Sensors_data.pressure[1]/256)%100;
	if(Sensors_data.pressure[1] < 10) USART_HandlerPuts( &USART1_Handler, "0");
	itoa(Sensors_data.pressure[1], str,10);
	USART_HandlerPuts( &USART1_Handler, str);
	USART_HandlerPuts( &USART1_Handler, "hPa temp0:");

	itoa(Sensors_data.temperature[0]/100, str,10);
	USART_HandlerPuts( &USART1_Handler, str);
	USART_HandlerPuts( &USART1_Handler, ".");
	Sensors_data.temperature[0] = Sensors_data.temperature[0]%100;
	if(Sensors_data.temperature[0] < 10) USART_HandlerPuts( &USART1_Handler, "0");
	itoa(Sensors_data.temperature[0], str,10);
	USART_HandlerPuts( &USART1_Handler, str);
	USART_HandlerPuts( &USART1_Handler, "dC temp1:");

	itoa(Sensors_data.temperature[1]/100, str,10);
	USART_HandlerPuts( &USART1_Handler, str);
	USART_HandlerPuts( &USART1_Handler, ".");
	Sensors_data.temperature[1] = Sensors_data.temperature[1]%100;
	if(Sensors_data.temperature[1] < 10) USART_HandlerPuts( &USART1_Handler, "0");
	itoa(Sensors_data.temperature[1], str,10);
	USART_HandlerPuts( &USART1_Handler, str);
	USART_HandlerPuts( &USART1_Handler, "dC \r\n");
}

void bmp280_read_event( T_I2C_DEVICE * i2c_device, uint16_t *data ){

	if(i2c_device->type == 1){
		for(uint8_t i = 0; i<Sensors.bmp280_number; i++){
			if(Sensors.BMP280_sensors[i].read){
				Sensors.BMP280_sensors[i].read = 0;

				Sensors.BMP280_sensors[i].pressure_raw=0;
				Sensors.BMP280_sensors[i].temperature_raw=0;

				Sensors.BMP280_sensors[i].temperature_raw = ((int32_t)data[3]<<12) + ((int32_t)data[4]<<4) + ((int32_t)data[5]>>4);
				Sensors.BMP280_sensors[i].pressure_raw = ((int32_t)data[0]<<12) + ((int32_t)data[1]<<4) + ((int32_t)data[2]>>4);

				uint32_t temp = bmp280_compensate_T_int32(&Sensors.BMP280_sensors[i], Sensors.BMP280_sensors[i].temperature_raw);
				uint32_t press = bmp280_compensate_P_int64(&Sensors.BMP280_sensors[i], Sensors.BMP280_sensors[i].pressure_raw);

				Sensors_data.temperature[i] = temp;
				Sensors_data.pressure[i] = press;

				if(Sensors.bmp280_number-1 == i){
					Sensors.bmp280_initstate = init_done;
					bmp280_data_ready();
				}

				break;
			}
		}

	}else{
		for(uint8_t i = 0; i<Sensors.ina219_number; i++){
			if(Sensors.INA219_sensors[i].read){
				Sensors.INA219_sensors[i].read = 0;

				uint32_t vol = (((data[0]>>3)*16)/4000.0f)*1000;
				uint32_t curr = (data[2]*0.000122f)*1000;

				Sensors_data.current[i] = curr;
				Sensors_data.voltage[i] = vol;

				if(Sensors.bmp280_number == 0){
					bmp280_data_ready();
					Sensors.bmp280_initstate = init_done;
				}

//				if(Sensors.ina219_number-1 == i){
//					bmp280_data_ready();
//					USART_HandlerPuts( &USART1_Handler, "AAAA!");
//					Sensors.bmp280_initstate = init_done;
//				}

				break;
			}
		}
	}
	//Sensors.bmp280_initstate = init_done;
}

void bmp280_conf_event( T_I2C_DEVICE * i2c_device, uint16_t *data ){
	switch(Sensors.bmp280_initstate){
	case reading_calibration:
		for(uint8_t i = 0; i<Sensors.bmp280_number; i++){
			if(Sensors.BMP280_sensors[i].read){
				bmp280_callibrationload(&Sensors.BMP280_sensors[i], data);
				Sensors.BMP280_sensors[i].read = 0;
				USART_HandlerPuts( &USART1_Handler, "read!");
				if(Sensors.bmp280_number-1 == i) Sensors.bmp280_initstate = init_done;
				break;
			}
		}
		break;
	}
}

void bmp280_congfigurated_event( T_I2C_HANDLER* i2c_handler ){
	switch(Sensors.bmp280_initstate){
	case configuration:
		for(uint8_t i = 0; i<Sensors.bmp280_number; i++){
			if(Sensors.BMP280_sensors[i].physical_device->responded){
				I2C_ReadSensorAdd(Sensors.BMP280_sensors[i].physical_device, 0x88, 26);
				Sensors.BMP280_sensors[i].read = 1;
				USART_HandlerPuts( &USART1_Handler, "confread!");
			}
		}
		I2C_ReadSensor(&I2C1_Handler, bmp280_conf_event);
		Sensors.bmp280_initstate = reading_calibration;
		break;
	}
}

void BMP280_TimerIRQ(){

	if(Sensors.run){

		switch(Sensors.bmp280_initstate){
		case no_init:
			for(uint8_t i = 0; i<Sensors.bmp280_number; i++){
				if(Sensors.BMP280_sensors[i].physical_device->responded){
					I2C_WriteConfgurationAdd(Sensors.BMP280_sensors[i].physical_device, i2c_device_configuration_bmp288_0, 3);

				}
			}
			for(uint8_t i = 0; i<Sensors.ina219_number; i++){
				if(Sensors.INA219_sensors[i].physical_device->responded){
					I2C_WriteConfgurationAdd(Sensors.INA219_sensors[i].physical_device, i2c_device_configuration_ina219_0, 2);

				}
			}
			Sensors.bmp280_initstate = configuration;
			I2C_WriteConfguration(&I2C1_Handler, bmp280_congfigurated_event);
			break;
		case init_done:

			for(uint8_t i = 0; i<Sensors.bmp280_number; i++){
				if(Sensors.BMP280_sensors[i].physical_device->responded){
					I2C_ReadSensorAdd(Sensors.BMP280_sensors[i].physical_device,0xF7,6);
					Sensors.BMP280_sensors[i].read = 1;
				}
			}
			for(uint8_t i = 0; i<Sensors.ina219_number; i++){
				if(Sensors.INA219_sensors[i].physical_device->responded){
					I2C_ReadSensorAdd(Sensors.INA219_sensors[i].physical_device,0x02,3);
					Sensors.INA219_sensors[i].read = 1;
				}
			}
			Sensors.bmp280_initstate = busy;
			I2C_ReadSensor(&I2C1_Handler, bmp280_read_event);
			break;
		}

		//CONFIGURE INA219
	}


}
