#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "stm32f10x.h"

#include "Status_Led.h"

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

RgbColor HsvToRgb(HsvColor hsv)
{
    RgbColor rgb;
    unsigned char region, remainder, p, q, t;

    if (hsv.s == 0)
    {
        rgb.r = hsv.v;
        rgb.g = hsv.v;
        rgb.b = hsv.v;
        return rgb;
    }

    region = hsv.h / 43;
    remainder = (hsv.h - (region * 43)) * 6;

    p = (hsv.v * (255 - hsv.s)) >> 8;
    q = (hsv.v * (255 - ((hsv.s * remainder) >> 8))) >> 8;
    t = (hsv.v * (255 - ((hsv.s * (255 - remainder)) >> 8))) >> 8;

    switch (region)
    {
        case 0:
            rgb.r = hsv.v; rgb.g = t; rgb.b = p;
            break;
        case 1:
            rgb.r = q; rgb.g = hsv.v; rgb.b = p;
            break;
        case 2:
            rgb.r = p; rgb.g = hsv.v; rgb.b = t;
            break;
        case 3:
            rgb.r = p; rgb.g = q; rgb.b = hsv.v;
            break;
        case 4:
            rgb.r = t; rgb.g = p; rgb.b = hsv.v;
            break;
        default:
            rgb.r = hsv.v; rgb.g = p; rgb.b = q;
            break;
    }

    return rgb;
}

HsvColor RgbToHsv(RgbColor rgb)
{
    HsvColor hsv;
    unsigned char rgbMin, rgbMax;

    rgbMin = rgb.r < rgb.g ? (rgb.r < rgb.b ? rgb.r : rgb.b) : (rgb.g < rgb.b ? rgb.g : rgb.b);
    rgbMax = rgb.r > rgb.g ? (rgb.r > rgb.b ? rgb.r : rgb.b) : (rgb.g > rgb.b ? rgb.g : rgb.b);

    hsv.v = rgbMax;
    if (hsv.v == 0)
    {
        hsv.h = 0;
        hsv.s = 0;
        return hsv;
    }

    hsv.s = 255 * (long)(rgbMax - rgbMin) / hsv.v;
    if (hsv.s == 0)
    {
        hsv.h = 0;
        return hsv;
    }

    if (rgbMax == rgb.r)
        hsv.h = 0 + 43 * (rgb.g - rgb.b) / (rgbMax - rgbMin);
    else if (rgbMax == rgb.g)
        hsv.h = 85 + 43 * (rgb.b - rgb.r) / (rgbMax - rgbMin);
    else
        hsv.h = 171 + 43 * (rgb.r - rgb.g) / (rgbMax - rgbMin);

    return hsv;
}

typedef struct T_STATUS_LED{
	uint16_t r;
	uint16_t g;
	uint16_t b;

	uint16_t r2;
	uint16_t g2;
	uint16_t b2;

	uint8_t mode;
	uint16_t on_time;
	uint16_t off_time;
	uint16_t fade_time;
	uint8_t cycles;

	uint8_t mode2;
	uint16_t on_time2;
	uint16_t off_time2;
	uint8_t cycles2;

}T_STATUS_LED;

T_STATUS_LED Status_Led;

void StatusLed_Init(){
	TIM3->CR1 |= TIM_CR1_DIR;

	TIM3->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;
	TIM3->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
	TIM3->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;

	TIM3->CCR3 = 0;
	TIM3->CCR2 = 0;
	TIM3->CCR1 = 0;

	TIM3->CCER |= TIM_CCER_CC3E | TIM_CCER_CC2E | TIM_CCER_CC1E;/// Start the timer counter /
	TIM3->CR1 |= TIM_CR1_CEN;
}

void StatusLed_setColorRGB( uint16_t r, uint16_t g, uint16_t b){
	Status_Led.r = r;
	Status_Led.g = g;
	Status_Led.b = b;
}

void StatusLed_setColorHEX( uint32_t color ){
	Status_Led.r = (color & 0xff0000) >> 16;
	Status_Led.g = (color & 0x00ff00) >> 8;
	Status_Led.b = (color & 0x0000ff);
}

void StatusLed_setBlink( uint16_t on_time, uint16_t off_time, uint16_t fade_time, uint8_t cycles){
	if(cycles == 0) Status_Led.mode = 1; else Status_Led.mode = 2;
	Status_Led.on_time = on_time;
	Status_Led.off_time = off_time;
	Status_Led.fade_time = fade_time;
	Status_Led.cycles = cycles;
}

void StatusLed_setStable(){
	Status_Led.mode = 0;
}

void StatusLed_setColorRGB2( uint16_t r, uint16_t g, uint16_t b){
	Status_Led.r2 = r;
	Status_Led.g2 = g;
	Status_Led.b2 = b;
}

void StatusLed_setColorHEX2( uint32_t color ){
	Status_Led.r2 = (color & 0xff0000) >> 16;
	Status_Led.g2 = (color & 0x00ff00) >> 8;
	Status_Led.b2 = (color & 0x0000ff);
}

void StatusLed_setBlink2( uint16_t on_time, uint16_t off_time, uint8_t cycles){
	if(cycles == 0) Status_Led.mode2 = 1; else Status_Led.mode2 = 2;
	Status_Led.on_time2 = on_time;
	Status_Led.off_time2 = off_time;
	Status_Led.cycles2 = cycles;
}

void StatusLed_setStable2(){
	Status_Led.mode2 = 0;
}

void StatusLed_HandlerMain(){

	static uint16_t statusled_timer1, statusled_timer2;
	uint16_t r = 0,g = 0,b = 0;

		if(Status_Led.mode == 1 || (Status_Led.mode == 2 && Status_Led.cycles > 0)){

			if(statusled_timer1 < Status_Led.fade_time){
				b = map(statusled_timer1, 0, Status_Led.fade_time, 0, Status_Led.b);
				g = map(statusled_timer1, 0, Status_Led.fade_time, 0, Status_Led.g);
				r = map(statusled_timer1, 0, Status_Led.fade_time, 0, Status_Led.r);
			}else if(statusled_timer1 < (Status_Led.fade_time + Status_Led.on_time)){
				r = Status_Led.r;
				g = Status_Led.g;
				b = Status_Led.b;
			}else if(statusled_timer1 < ((Status_Led.fade_time*2) + Status_Led.on_time)){
				b = map(statusled_timer1-(Status_Led.fade_time + Status_Led.on_time), Status_Led.fade_time, 0, 0, Status_Led.b);
				g = map(statusled_timer1-(Status_Led.fade_time + Status_Led.on_time), Status_Led.fade_time, 0, 0, Status_Led.g);
				r = map(statusled_timer1-(Status_Led.fade_time + Status_Led.on_time), Status_Led.fade_time, 0, 0, Status_Led.r);
			}else{
				r = 0;
				g = 0;
				b = 0;
			}

			if(statusled_timer1 < (Status_Led.fade_time*2 + Status_Led.on_time + Status_Led.off_time) ){
				statusled_timer1++;
			}else{
				statusled_timer1 = 0;
			    if(Status_Led.cycles > 0) Status_Led.cycles--;
			}
		}

		if(Status_Led.mode2 == 1 || (Status_Led.mode2 == 2 && Status_Led.cycles2 > 0)){

			if(statusled_timer2 < Status_Led.on_time2){
				if(((uint32_t)(r + Status_Led.r2)) < 65535) r += Status_Led.r2; else r= 65535;
				if(((uint32_t)(g + Status_Led.g2)) < 65535) g += Status_Led.g2; else g= 65535;
				if(((uint32_t)(b + Status_Led.b2)) < 65535) b += Status_Led.b2; else b= 65535;
			}

			if(statusled_timer2 < (Status_Led.on_time2 + Status_Led.off_time2) ){
				statusled_timer2++;
			}else{
				statusled_timer2 = 0;
				if(Status_Led.cycles2 > 0) Status_Led.cycles2--;
			}
		}

		TIM3->CCR1 = r;
		TIM3->CCR2 = g;
		TIM3->CCR3 = b;

}




