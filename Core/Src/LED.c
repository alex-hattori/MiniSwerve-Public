/*
 * LED.c
 *
 *  Created on: Sep 18, 2024
 *      Author: user
 */

#include "LED.h"
#include <math.h>

// Initialize all LEDs to off (black)
void LED_Init(LED_Struct *led) {
    LED_Clear(led);  // Set all LEDs to black
    for (int i = NUM_LEDS * 24; i < (NUM_LEDS * 24 + 48); i++) {
		led->fData[i] = LED_BREAK;  // Set the LED break signal
	}
}

// Set an LED color using individual RGB values
void LED_SetColor(LED_Struct *led, uint32_t ledIndex, uint8_t red, uint8_t green, uint8_t blue) {
    if (ledIndex >= NUM_LEDS) return;  // Ensure we don't go out of bounds
    uint32_t packedRGB = PACK_RGB(red, green, blue);
    LED_SetPackedColor(led, ledIndex, packedRGB);
}

// Set an LED color using a packed RGB value
void LED_SetPackedColor(LED_Struct *led, uint32_t ledIndex, uint32_t packedRGB) {
    // Ensure we don't go out of bounds
    if (ledIndex >= NUM_LEDS) return;

    // Convert packed RGB value into timing values
    for (int bit = 23; bit >= 0; bit--) {
        led->fData[ledIndex * 24 + (23 - bit)] = (packedRGB & (1 << bit)) ? LED_BIT_1 : LED_BIT_0;
    }
}

// Clear all LEDs (set to black)
void LED_Clear(LED_Struct *led) {
    for (uint32_t i = 0; i < NUM_LEDS; i++) {
        LED_SetColor(led, i, 0, 0, 0);  // Set each LED to black
    }
}

// Function to convert a hue value to an RGB color
void HueToRGB(uint16_t hue, uint8_t *r, uint8_t *g, uint8_t *b) {
    float hf = hue / 60.0; // Hue divided by 60
    int i = (int)floor(hf); // Integer part of hf
    float f = hf - i; // Fractional part of hf
    float p = 0;
    float q = 1.0 - f;
    float t = f;

    switch (i) {
        case 0: *r = MAX_COLOR_VALUE; *g = t * MAX_COLOR_VALUE; *b = p * MAX_COLOR_VALUE; break;
        case 1: *r = q * MAX_COLOR_VALUE; *g = MAX_COLOR_VALUE; *b = p * MAX_COLOR_VALUE; break;
        case 2: *r = p * MAX_COLOR_VALUE; *g = MAX_COLOR_VALUE; *b = t * MAX_COLOR_VALUE; break;
        case 3: *r = p * MAX_COLOR_VALUE; *g = q * MAX_COLOR_VALUE; *b = MAX_COLOR_VALUE; break;
        case 4: *r = t * MAX_COLOR_VALUE; *g = p * MAX_COLOR_VALUE; *b = MAX_COLOR_VALUE; break;
        case 5: *r = MAX_COLOR_VALUE; *g = p * MAX_COLOR_VALUE; *b = q * MAX_COLOR_VALUE; break;
    }
}

// Function to update the LEDs with a rolling rainbow effect
void LED_RainbowRoll(LED_Struct *led, uint16_t offset) {
    for (uint16_t i = 0; i < NUM_LEDS; i++) {
        uint16_t hue = (i * (HUE_MAX / NUM_LEDS) + offset) % HUE_MAX;  // Calculate hue with offset
        uint8_t r, g, b;
        HueToRGB(hue, &r, &g, &b);  // Convert hue to RGB

        LED_SetColor(led, i, r, g, b);  // Set the color of the current LED
    }
}

void LED_Chunk(LED_Struct *led, uint32_t ledIndex, uint32_t endIndex, uint8_t red, uint8_t green, uint8_t blue){
	for(int i = ledIndex; i<endIndex; i++){
		LED_SetColor(led,i,red,green,blue);
	}
}

void LED_SetHeading(LED_Struct *led, float heading, uint8_t red, uint8_t green, uint8_t blue, uint8_t bufferSize) {
    // Normalize heading to be within 0 to 360 degrees
    if (heading < 0) {
        heading += 360;
    } else if (heading >= 360) {
        heading -= 360;
    }

    // Calculate the corresponding LED index
    uint16_t ledIndex = (uint16_t)(heading / DEGREES_PER_LED);

    // Set the color for the main LED and the buffer LEDs
    for (int i = -bufferSize; i <= bufferSize; i++) {
        int currentLED = ledIndex + i;

        // Wrap around the index if it goes out of bounds
        if (currentLED < 0) {
            currentLED += NUM_LEDS;  // Wrap to the end
        } else if (currentLED >= NUM_LEDS) {
            currentLED -= NUM_LEDS;  // Wrap to the beginning
        }

        // Set the color of the current LED
        LED_SetColor(led, currentLED, red, green, blue);
    }
}

void LED_ShowRobotOrientation(LED_Struct *led, float heading, uint8_t red, uint8_t green, uint8_t blue) {
	LED_Chunk(led,0,NUM_LEDS,red,green,blue);
	LED_SetHeading(led,heading,0,255,0,3);
    LED_SetHeading(led,heading+180.0f,255,255,0,5);
}

