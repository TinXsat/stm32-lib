#ifndef SENSORS_BMP280_H_
#define SENSORS_BMP280_H_

#include <Drivers/I2C_Handler/I2C_Handler.h>

/*
 * INA219 CONF
 * Rsch = 0,01 Ohm
 * PGA /1
 * CURR_LSB = 4.000A/32768 = 0.000122
 * CALL = 0,04096/(0,000125*0,01 Ohm) = 33573
 *
 */

typedef enum {no_init = 0, configuration, configured, reading_calibration, init_done, busy}T_BMP280_INITSTATE;

typedef struct T_BMP280{
	T_I2C_DEVICE * physical_device;

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

	int32_t t_fine;

	int32_t pressure_raw;
	int32_t temperature_raw;

	uint8_t read;
}T_BMP280;

typedef struct T_INA219{
	T_I2C_DEVICE * physical_device;

	uint32_t calibration_reg;

	uint8_t read;
}T_INA219;

extern uint16_t i2c_device_configuration_bmp288_0[];
extern uint16_t i2c_device_configuration_ina219_0[];

void bmp280_callibrationload(T_BMP280 * BMP280, uint16_t * data);
int32_t bmp280_compensate_T_int32(T_BMP280 * BMP280, int32_t adc_T);
uint32_t bmp280_compensate_P_int64(T_BMP280 * BMP280, int32_t adc_P);

#endif /* SENSORS_BMP280_H_ */
