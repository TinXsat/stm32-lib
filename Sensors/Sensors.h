#ifndef SENSORS_SENSORS_H_
#define SENSORS_SENSORS_H_

#include "BMP280/BMP280.h"

typedef struct T_SENSORS_DATA{
	uint32_t pressure[2];
	uint32_t temperature[2];
	uint32_t altitude[2];
	uint32_t current[2];
	uint32_t voltage[2];
	uint32_t power[2];
}T_SENSORS_DATA;

typedef struct T_SENSORS{
	T_BMP280 * BMP280_sensors;
	uint8_t bmp280_number;
	uint8_t bmp280_actual;
	T_BMP280_INITSTATE bmp280_initstate;

	T_INA219 * INA219_sensors;
	uint8_t ina219_number;
	uint8_t ina219_actual;

	uint8_t run;
	uint8_t data_ready;
}T_SENSORS;

extern T_SENSORS Sensors;
extern T_SENSORS_DATA Sensors_data;
extern T_BMP280 BMP280_sensors[2];
extern T_INA219 INA219_sensors[2];

void Sensors_init1(T_BMP280* bmp280_sensors, uint8_t bmp280_number, T_INA219 *ina219_sensors, uint8_t ina219_number);
void Sensors_Aquire( uint8_t true );
void Sensors_init(T_BMP280 bmp280_sensors, uint8_t bmp280_number, T_INA219 ina219_sensors, uint8_t ina219_number);

void BMP280_TimerIRQ();

#endif /* SENSORS_SENSORS_H_ */
