#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "stm32f10x.h"

#include <common.h>
#include <Drivers/I2C_Handler/I2C_Handler.h>
#include <Sensors/Sensors.h>

#include "BMP280.h"

uint16_t i2c_device_configuration_bmp288_0[] = {
		0xF4, 0xFC,	//oversampling x16, sleep
		0xF5, 0x08,	//filter x4, 0,5 stb time, no spi
		0xF4, 0xFF	//oversampling x16, normal mode
};

uint16_t i2c_device_configuration_ina219_0[] = {
		0x00, 0x0447,	//pga /1 16 fsr, adc 12b, adc 12b, bus shunt continous
		0x05, 33573,	//calib val
};

void bmp280_callibrationload(T_BMP280 * BMP280, uint16_t * data){

	for(uint8_t i = 0; i<13; i++){
		switch(i){
			case 0: BMP280->dig_T1	= ((data[(i*2)+1]&0x00ff)<<8) | (data[(i*2)]&0x00ff); break;
			case 1: BMP280->dig_T2	= ((data[(i*2)+1]&0x00ff)<<8) | (data[(i*2)]&0x00ff); break;
			case 2: BMP280->dig_T3	= ((data[(i*2)+1]&0x00ff)<<8) | (data[(i*2)]&0x00ff); break;
			case 3: BMP280->dig_P1	= ((data[(i*2)+1]&0x00ff)<<8) | (data[(i*2)]&0x00ff); break;
			case 4: BMP280->dig_P2	= ((data[(i*2)+1]&0x00ff)<<8) | (data[(i*2)]&0x00ff); break;
			case 5: BMP280->dig_P3	= ((data[(i*2)+1]&0x00ff)<<8) | (data[(i*2)]&0x00ff); break;
			case 6: BMP280->dig_P4	= ((data[(i*2)+1]&0x00ff)<<8) | (data[(i*2)]&0x00ff); break;
			case 7: BMP280->dig_P5	= ((data[(i*2)+1]&0x00ff)<<8) | (data[(i*2)]&0x00ff); break;
			case 8: BMP280->dig_P6	= ((data[(i*2)+1]&0x00ff)<<8) | (data[(i*2)]&0x00ff); break;
			case 9: BMP280->dig_P7	= ((data[(i*2)+1]&0x00ff)<<8) | (data[(i*2)]&0x00ff); break;
			case 10: BMP280->dig_P8	= ((data[(i*2)+1]&0x00ff)<<8) | (data[(i*2)]&0x00ff); break;
			case 11: BMP280->dig_P9	= ((data[(i*2)+1]&0x00ff)<<8) | (data[(i*2)]&0x00ff); break;
		}
	}
}

//returns pressure in 1/100degC
int32_t bmp280_compensate_T_int32(T_BMP280 * BMP280, int32_t adc_T){
	int32_t var1, var2, T;
	var1 = ((((adc_T>>3) -((int32_t)BMP280->dig_T1<<1))) * ((int32_t)BMP280->dig_T2)) >> 11;
	var2 = (((((adc_T>>4) -((int32_t)BMP280->dig_T1)) * ((adc_T>>4)- ((int32_t)BMP280->dig_T1))) >> 12) *((int32_t)BMP280->dig_T3)) >> 14;
	BMP280->t_fine = var1 + var2;
	T = (BMP280->t_fine * 5 + 128) >> 8;
	return T;
}

//returns pressure in 1/256Pa
uint32_t bmp280_compensate_P_int64(T_BMP280 * BMP280, int32_t adc_P){
  int64_t var1, var2, p;
  var1 = ((int64_t)BMP280->t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)BMP280->dig_P6;
  var2 = var2 + ((var1 * (int64_t)BMP280->dig_P5) << 17);
  var2 = var2 + (((int64_t)BMP280->dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)BMP280->dig_P3) >> 8) + ((var1 * (int64_t)BMP280->dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)BMP280->dig_P1) >> 33;
  if (var1 == 0)
  {
    return 0; // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)BMP280->dig_P9) * (p >> 13) * (p>>13)) >> 25;
  var2 = (((int64_t)BMP280->dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)BMP280->dig_P7) << 4);
  return (uint32_t)p;
}
