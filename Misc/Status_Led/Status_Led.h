#ifndef MISC_STATUS_LED_STATUS_LED_H_
#define MISC_STATUS_LED_STATUS_LED_H_

typedef struct RgbColor
{
    unsigned char r;
    unsigned char g;
    unsigned char b;
} RgbColor;

typedef struct HsvColor
{
    unsigned char h;
    unsigned char s;
    unsigned char v;
} HsvColor;

RgbColor HsvToRgb(HsvColor hsv);
HsvColor RgbToHsv(RgbColor rgb);

long map(long x, long in_min, long in_max, long out_min, long out_max);

void StatusLed_Init();
void StatusLed_HandlerMain();
void StatusLed_setColorRGB( uint16_t r, uint16_t g, uint16_t b);
void StatusLed_setColorHEX( uint32_t color );
void StatusLed_setBlink( uint16_t on_time, uint16_t off_time, uint16_t fade_time, uint8_t cycles);
void StatusLed_setStable();
void StatusLed_setColorRGB2( uint16_t r, uint16_t g, uint16_t b);
void StatusLed_setColorHEX2( uint32_t color );
void StatusLed_setBlink2( uint16_t on_time, uint16_t off_time, uint8_t cycles);
void StatusLed_setStable2();

#endif /* MISC_STATUS_LED_STATUS_LED_H_ */
