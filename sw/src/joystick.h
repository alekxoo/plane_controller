// joystick.h
#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <stdint.h>

// Global variables for debugging
extern volatile uint32_t g_left_x;    // Left joystick X value
extern volatile uint32_t g_left_y;    // Left joystick Y value
extern volatile uint32_t g_left_sel;  // Left joystick Select value
extern volatile uint32_t g_right_x;   // Right joystick X value
extern volatile uint32_t g_right_y;   // Right joystick Y value
extern volatile uint32_t g_right_sel; // Right joystick Select value

// Initialize both joysticks
void JoystickInit(void);

// Read both values at once for each joystick
void Joystick_Read();

void joystick_handler(void);

// Read select buttons
uint32_t JoystickLeft_Sel(void);
uint32_t JoystickRight_Sel(void);

#endif
