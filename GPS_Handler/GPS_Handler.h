#ifndef GPS_HANDLER_GPS_HANDLER_H_
#define GPS_HANDLER_GPS_HANDLER_H_

typedef struct T_GPS_DATA{

	uint8_t fix_mode;

	uint8_t time_hh;
	uint8_t time_mm;
	uint8_t time_ss;

	uint8_t sattelites_no;

	float ground_speed;

	double latitude_decimald;
	double longitude_decimald;
	float altitude;
	float geoid_wgs84_elipsoid;

}T_GPS_DATA;

typedef struct T_GPS_HANDLER{
	T_USART_HANDLER *USART_Handler;

	T_GPS_DATA GPS_Data;

	void (*gps_fixed_event_callback)(T_GPS_DATA* data);
}T_GPS_HANDLER;

void GPS_HandlerInit(T_GPS_HANDLER * GPS_Handler, T_USART_HANDLER *USART_Handler, void (*callback)(T_GPS_DATA* data));
void GPS_HandlerAsciiLine(T_GPS_HANDLER * GPS_Handler, char * buf);

#endif /* GPS_HANDLER_GPS_HANDLER_H_ */
