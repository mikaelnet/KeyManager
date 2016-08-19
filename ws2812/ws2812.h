/*
 * ws2812.h
 *
 * Created: 2016-08-19 21:58:30
 *  Author: mikael
 */ 


#ifndef WS2812_H_
#define WS2812_H_

#include <avr/io.h>

#define ws2812_port B     // Data port
#define ws2812_pin  4     // Data out pin

struct cRGB  { uint8_t g; uint8_t r; uint8_t b; };

void ws2812_setleds     (struct cRGB  *ledarray, uint16_t number_of_leds);
void ws2812_setleds_pin (struct cRGB  *ledarray, uint16_t number_of_leds, uint8_t pinmask);

#endif /* WS2812_H_ */