/*
 * LED.h
 *
 *  Created on: Sep 18, 2024
 *      Author: user
 */

#ifndef INC_LED_H_
#define INC_LED_H_

#include <stdint.h>

#define NUM_LEDS 63
#define DEGREES_PER_LED (360.0f / (float)NUM_LEDS)

#define LED_BIT_1 125
#define LED_BIT_0 62
#define LED_BREAK 0

#define MAX_COLOR_VALUE 255
#define HUE_MAX 360

#define PACK_RGB(r, g, b)  (((g) << 16) | ((r) << 8) | (b))

typedef struct {
    uint32_t fData[NUM_LEDS * 24+48];  // 24 bits per LED + LED break

    int rainbowOffset;
    int isRed;
    int isBlue;
} LED_Struct;

void LED_Init(LED_Struct *led);
void LED_SetColor(LED_Struct *led, uint32_t ledIndex, uint8_t red, uint8_t green, uint8_t blue);
void LED_SetPackedColor(LED_Struct *led, uint32_t ledIndex, uint32_t packedRGB);
void LED_Clear(LED_Struct *led);
void HueToRGB(uint16_t hue, uint8_t *r, uint8_t *g, uint8_t *b);
void LED_RainbowRoll(LED_Struct *led, uint16_t offset);
void LED_Chunk(LED_Struct *led, uint32_t ledIndex, uint32_t endIndex, uint8_t red, uint8_t green, uint8_t blue);
void LED_SetHeading(LED_Struct *led, float heading, uint8_t red, uint8_t green, uint8_t blue, uint8_t bufferSize);
void LED_ShowRobotOrientation(LED_Struct *led, float heading, uint8_t red, uint8_t green, uint8_t blue);

#endif /* INC_LED_H_ */
