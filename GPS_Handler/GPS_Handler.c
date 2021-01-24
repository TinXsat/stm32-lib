#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "stm32f10x.h"

#include <Drivers/USART_Handler/USART_Handler.h>

#include "GPS_Handler.h"

void gps_parse_gpgga(T_GPS_HANDLER * GPS_Handler, char * buf );
void gps_parse_gprma(T_GPS_HANDLER * GPS_Handler, char * buf );

void GPS_HandlerInit(T_GPS_HANDLER * GPS_Handler, T_USART_HANDLER *USART_Handler, void (*callback)(T_GPS_DATA* data)){
	GPS_Handler->USART_Handler = USART_Handler;
	GPS_Handler->gps_fixed_event_callback = callback;
}

void GPS_HandlerAsciiLine(T_GPS_HANDLER * GPS_Handler, char * buf){
	if(buf[0] == '$'){
		if(strstr(buf, "GPGGA")){
			gps_parse_gpgga(GPS_Handler, buf);
		}
		if(strstr(buf, "GPRMA")){
			gps_parse_gprma(GPS_Handler, buf);
		}
	}
}

void gps_parse_gpgga(T_GPS_HANDLER * GPS_Handler, char * buf ){
	char *start, *rest;
	int i = 0;
	start = strtok_r(buf,",;",&rest);
	while(start){
		char *dpoint = 0;
		uint16_t whole_degrees;
		double minutes_rest;
		switch(i){
		case 1:
			GPS_Handler->GPS_Data.time_hh = (start[0]-'0')*10+(start[1]-'0');
			GPS_Handler->GPS_Data.time_mm = (start[2]-'0')*10+(start[3]-'0');
			GPS_Handler->GPS_Data.time_ss = (start[4]-'0')*10+(start[5]-'0');
			break;
		case 2:
			dpoint= strchr(start, '.');
			whole_degrees = (start[0]-'0')*10 + (start[1]-'0');
			if(dpoint - start == 5) whole_degrees = whole_degrees*10 + (start[2]-'0');
			minutes_rest = (atof(dpoint-2))/60;
			GPS_Handler->GPS_Data.latitude_decimald = whole_degrees + minutes_rest;
			break;
		case 4:
			dpoint= strchr(start, '.');
			whole_degrees = (start[0]-'0')*10 + (start[1]-'0');
			if(dpoint - start == 5) whole_degrees = whole_degrees*10 + (start[2]-'0');
			minutes_rest = (atof(dpoint-2))/60;
			GPS_Handler->GPS_Data.longitude_decimald = whole_degrees + minutes_rest;
			break;
		case 6:
			GPS_Handler->GPS_Data.fix_mode = (start[0]-'0');
			break;
		case 7:
			GPS_Handler->GPS_Data.sattelites_no = (start[0]-'0');
			break;
		}
		i++;
		start = strtok_r(NULL,",;",&rest);
	}
	if(GPS_Handler->GPS_Data.fix_mode == 1)GPS_Handler->gps_fixed_event_callback(&GPS_Handler->GPS_Data);
}

void gps_parse_gprma(T_GPS_HANDLER * GPS_Handler,  char * buf ){

}
