#ifndef __LEDRING12_H__
#define __LEDRING12_H__

#include <stdint.h>
#include <stdbool.h>

#define NBR_LEDS  12

extern uint8_t ledringmem[NBR_LEDS * 2];

void ai_showDistance(float AbstandInMeter, uint8_t red, uint8_t yellow, uint8_t blue);

#endif //__LEDRING12_H__
