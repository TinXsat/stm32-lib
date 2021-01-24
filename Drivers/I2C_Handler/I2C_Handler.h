#ifndef I2C_HANDLER_I2C_HANDLER_H_
#define I2C_HANDLER_I2C_HANDLER_H_

typedef enum{I2C_AF_ERROR = 0,I2C_BERR_ERROR,I2C_ARLO_ERROR,I2C_OVR_ERROR,I2C_PACERR_ERROR}T_I2C_ERROR_CODE;

//one i2c device "object"
typedef struct T_I2C_DEVICE{
	uint8_t responded;
	uint8_t slave_address;
	uint8_t D0_value;
	uint8_t type;	//0-unknown 1-bmp280 2-bmp388 3-ina219
	uint8_t write_configuration;
	uint8_t configured;
	uint16_t * configuration;
	uint8_t configuration_len;
	uint8_t is_16_bit;
	uint8_t read_data;
	uint8_t read_start_reg;
	uint8_t read_cnt_reg;
}T_I2C_DEVICE;

//whole state machine variables/callbacks
typedef struct T_I2C_HANDLER{

	//low level used to generate frame: start, stop, restart, send byte, read byte
	uint16_t *data;
	uint8_t data_len;
	uint8_t data_pointer;
	uint8_t reg_address;
	uint8_t slave_address;
	uint8_t transmit;
	uint8_t restart;

	//flags used to connect i2c interrupts with main "thread"
	uint8_t tx_flag;
	uint8_t rx_flag;
	uint8_t error_flag;
	uint8_t mode; //0=nothing 1=enumerate 2=write configuration
	T_I2C_DEVICE* devices;
	I2C_TypeDef* I2C_periph;

	//variables used to enumerate devices
	uint8_t enumeration_address;
	uint8_t enumeration_start_address;
	uint8_t enumeration_stop_address;
	uint8_t found_enumerated;
	void (*i2c1_enumerated_event_callback)();

	//variables used to write/read to/from devices
	uint8_t now_handled_device;
	uint8_t now_handled_register;
	void (*i2c1_configured_event_callback)();

	uint8_t read_start_reg;
	uint8_t read_cnt_reg;
	void (*i2c1_read_event_callback)(T_I2C_DEVICE* device, uint16_t * _data);

	uint8_t error_code;
	void (*i2c1_error_event_callback)(T_I2C_DEVICE* device, T_I2C_ERROR_CODE error_code);

	uint8_t old_mode;

}T_I2C_HANDLER;

void I2C_Handler_init(T_I2C_HANDLER* I2C_Handler, T_I2C_DEVICE* devices_t, I2C_TypeDef* I2C_periph_t, uint16_t* buf, void (*callback)(T_I2C_DEVICE* device, T_I2C_ERROR_CODE error_code));
void I2C_EnumerateDevices(T_I2C_HANDLER* I2C_Handler, uint8_t start_addr, uint8_t stop_addr, void (*callback)(T_I2C_HANDLER* i2c_handler));
void I2C_WriteConfgurationAdd(T_I2C_DEVICE * device, uint16_t * data , uint8_t reg_cnt);
void I2C_WriteConfguration(T_I2C_HANDLER* I2C_Handler, void (*callback)(T_I2C_HANDLER* i2c_handler));
void I2C_ReadSensorAdd( T_I2C_DEVICE * device, uint8_t start_reg , uint8_t reg_cnt );
void I2C_ReadSensor(T_I2C_HANDLER* I2C_Handler, void (*callback)(T_I2C_DEVICE* device, uint16_t * data) );

void I2C_Handler_Main(T_I2C_HANDLER* I2C_Handler);
void I2C_Handler_ERIRQ( T_I2C_HANDLER* I2C_Handler);
void I2C_Handler_EVIRQ( T_I2C_HANDLER* I2C_Handler);

#endif /* I2C_HANDLER_I2C_HANDLER_H_ */
