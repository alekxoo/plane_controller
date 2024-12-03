/*
 * screen_controller.c
 *
 *  Created on: Nov 29, 2024
 *      Author: alekxoo
 */

#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"
#include "../inc/Texas.h"
#include "../inc/PLL.h"
#include "../inc/ST7735.h"
#include "fifo_format.h"
#include "../inc/UART.h"
#include "screen_controller.h"

void display_init(void)
{
    ST7735_InitR(INITR_REDTAB);
    ST7735_SetRotation(1);
}

void DrawPlaneIcon(int16_t xpos, int16_t ypos, uint16_t color) {
    // Body
    ST7735_DrawLine(xpos+10, ypos, xpos+40, ypos, color);        // Main body
    ST7735_DrawLine(xpos+40, ypos, xpos+45, ypos+3, color);      // Nose (top)
    ST7735_DrawLine(xpos+45, ypos+3, xpos+40, ypos+6, color);    // Nose (bottom)
    ST7735_DrawLine(xpos+40, ypos+6, xpos+10, ypos+6, color);    // Bottom body
    ST7735_DrawLine(xpos+10, ypos+6, xpos+8, ypos+3, color);     // Tail end
    ST7735_DrawLine(xpos+8, ypos+3, xpos+10, ypos, color);       // Back to top

    // Wings
    ST7735_DrawLine(xpos+20, ypos-10, xpos+30, ypos, color);     // Left wing
    ST7735_DrawLine(xpos+20, ypos+16, xpos+30, ypos+6, color);   // Right wing

    // Tail
    ST7735_DrawLine(xpos+10, ypos, xpos+5, ypos-5, color);       // Top tail
    ST7735_DrawLine(xpos+10, ypos+6, xpos+5, ypos+11, color);    // Bottom tail
}

void DrawStartupScreen(void) {
    // Clear screen
    ST7735_FillScreen(ST7735_BLACK);

    // Draw loading dots animation
    char loading[] = "Loading";
    int16_t dots = 0;

    for(int i = 0; i < 3; i++) {  // Do animation 3 times
        for(dots = 0; dots <= 3; dots++) {
            ST7735_FillRect(0, 0, 160, 128, ST7735_BLACK);  // Clear screen

            // Draw title
            ST7735_SetTextColor(ST7735_CYAN);
            ST7735_SetCursor(4, 2);
            ST7735_OutString("RC PLANE CONTROLLER");

            // Draw loading text with variable dots
            ST7735_SetCursor(6, 6);
            ST7735_OutString(loading);
            for(int j = 0; j < dots; j++) {
                ST7735_OutString(".");
            }

            // Draw moving plane
            DrawPlaneIcon(20 + (i * 20) + (dots * 10), 40, ST7735_WHITE);

            // Add delay
            for(volatile int d = 0; d < 1000000; d++) {}
        }
    }

    // Final splash screen
    ST7735_FillScreen(ST7735_BLACK);
    DrawPlaneIcon(60, 40, ST7735_WHITE);  // Center plane
    ST7735_SetTextColor(ST7735_GREEN);
    ST7735_SetCursor(3, 3);
    ST7735_OutString("SYSTEM READY");

    // Delay before showing main screen
    for(volatile int d = 0; d < 2000000; d++) {}

    // Clear for main display
    ST7735_FillScreen(ST7735_BLACK);
}
