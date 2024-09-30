// Receiver.c
// Lab 5
// Programs to implement receiver functionality   
// ECE445L Fall 2024
//    Jonathan W. Valvano 8/30/24

// ----------------------------------------------------------------------------
// Hardware/software assigned to receiver
//   Timer2A ADC0 samples sound
//   Fifo3 linkage from  ADC to Decoder
//   main-loop runs decoder
//   PE3 PF3, PF2, PF1, LEDs
//   PA2-PA7, SSI0, ST7735R

#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"
#include "../inc/dsp.h"
#include "../inc/fifo.h"
// write this

