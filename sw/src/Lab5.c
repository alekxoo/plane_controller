// ----------------------------------------------------------------------------
// 
// File name:     Lab5.c
//
// ----------------------------------------------------------------------------
//
// Description:   This lab is designed to be used on the new Lab Board 
// Author:        Mark McDermott and Jon Valvano
// Orig gen date: August 4, 2024
// 
// Goal of this lab: Audio Communication
//

// ----------------------------------------------------------------------------
// Hardware/software assigned to common
// main initialization initializes all modules
//   PD2, Timer5A, ADC1, UART0 implements TExaSdisplay
// ----------------------------------------------------------------------------
// Hardware/software assigned to transmitter
//   UART0, possible source of input data (conflicts with TExaSdisplay)
//   PC7-PC4 Port_C_Init, possible source of input data
//   Timer0A periodic interrupt to generate input
//   Fifo1 linkage from Input to Encoder
//   Timer1A periodic interrupt create a sequence of frequencies
//   Fifo2 linkage from  Encoder to DAC
//   SSI1 PD3 PD1 PD0 TLV5616 DAC output
//   SysTick ISR output to DAC
// ----------------------------------------------------------------------------
// Hardware/software assigned to reciever
//   Timer2A ADC0 samples sound
//   Fifo3 linkage from  ADC to Decoder
//   main-loop runs decoder
//   PE3 PF3, PF2, PF1, LEDs
//   PA2-PA7, SSI0, ST7735R

#define CR 0x0D   // or 13 in decimal, '\r' in character notation
#define LF 0x0A   // or 10 in decimal, '\n' in character notation

#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"
#include "../inc/Texas.h"
#include "../inc/PLL.h"
#include "../inc/ST7735.h"
#include "fifo_format.h"
#include "../inc/UART.h"
#include "joystick.h"
#include "../inc/Timer1A.h"
#include "../inc/Timer2A.h"
#include "../inc/Timer3A.h"
#include "screen_controller.h"
#include "HC12.h"

void DisableInterrupts(void);           // Disable interrupts
void EnableInterrupts(void);            // Enable interrupts
void WaitForInterrupt(void);

char receivedChar;
uint32_t data[4];

// Function to control Power LED
void Power_LED_On(void) {
    GPIO_PORTD_DATA_R |= 0x08;          // Set PD3 (Power LED)
}

void Power_LED_Off(void) {
    GPIO_PORTD_DATA_R &= ~0x08;         // Clear PD3 (Power LED)
}

// Function to control Receive LED
void Receive_LED_On(void) {
    GPIO_PORTD_DATA_R |= 0x80;          // Set PD7 (Receive LED)
}

void Receive_LED_Off(void) {
    GPIO_PORTD_DATA_R &= ~0x80;         // Clear PD7 (Receive LED)
}

//debug code
int main(void){
      DisableInterrupts();
      PLL_Init(Bus80MHz);               // set system clock to 80 MHz
      PortB_Init();
      PortD_Init();
      PortF_Init();
      Power_LED_On();
      JoystickInit();
      display_init();
      HC12_Init();

      Timer1A_Init(&joystick_handler, 400000, 1);
      Timer2A_Init(&send_data, 1600000, 2); // 800,000 cycles = 10ms period = 100 Hz
      Timer3A_Init(&receive, 400000, 5); // 40,000,000 cycles = 4s period = 4 Hz
      DrawStartupScreen();

      EnableInterrupts();
      while(1) {
              ST7735_SetCursor(0,0);
              ST7735_OutString("Left Y:");
              ST7735_SetCursor(0,1);
              ST7735_OutString("Right X:");
              ST7735_SetCursor(0,2);
              ST7735_OutString("Right Y:");
              ST7735_SetCursor(0,5);
              ST7735_OutString("v-ref: ");


              // Update display
              ST7735_SetCursor(8,0);
              ST7735_OutUDec(g_left_y);
              ST7735_OutString("   ");

              ST7735_SetCursor(8,1);
              ST7735_OutUDec(g_right_x);
              ST7735_OutString("   ");

              ST7735_SetCursor(8,2);
              ST7735_OutUDec(g_right_y);
              ST7735_OutString("   ");

              ST7735_SetCursor(8,5);
              ST7735_OutUDec(g_value);
              ST7735_OutString("   ");

              if (g_value < 3075)
              {
                  ST7735_SetCursor(0,6);
                  ST7735_OutString("Battery Low!!!");
                  ST7735_SetCursor(0,7);
                  ST7735_OutString("Return Flight!!!");

              }
              else
              {
                  ST7735_SetCursor(0,6);
                  ST7735_OutString("Battery Good!!!");
                  ST7735_SetCursor(0,7);
                  ST7735_OutString("Keep Flying!!!");
              }
//          WaitForInterrupt();
      }
}



