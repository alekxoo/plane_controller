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

#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"
#include "../inc/Texas.h"
#include "../inc/PLL.h"
#include "../inc/ST7735.h"
#include "button.h"
#include "fifo_format.h"
#include "decoder.h"
#include "DAC.h"
#include "../inc/dsp.h"


void DisableInterrupts(void);           // Disable interrupts
void EnableInterrupts(void);            // Enable interrupts
Fifo inputFifo;

int main(void){
  DisableInterrupts();
  PLL_Init(Bus80MHz);    // bus clock at 80 MHz
  Fifo_Init(&inputFifo);  // Initialize the FIFO
  PortF_Init();
  SSI_init();
  DAC_Init();
  EnableInterrupts();
  while(1){
      uint32_t symbol = Decoder_Process();

      // Handle the decoded symbol
      switch(symbol) {
          case 1:
              // Handle Symbol 1
              GPIO_PORTF_DATA_R = (GPIO_PORTF_DATA_R & ~0x0E) | 0x02; // Red LED
              break;
          case 2:
              // Handle Symbol 2
              GPIO_PORTF_DATA_R = (GPIO_PORTF_DATA_R & ~0x0E) | 0x04; // Blue LED
              break;
          case 3:
              // Handle Symbol 3
              GPIO_PORTF_DATA_R = (GPIO_PORTF_DATA_R & ~0x0E) | 0x08; // Green LED
              break;
          default:
              // No signal or unknown symbol
              GPIO_PORTF_DATA_R &= ~0x0E; // All LEDs off
              break;
      }
  }
}


