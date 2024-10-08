/*
 * encoder.c
 *
 *  Created on: Oct 8, 2024
 *      Author: alekxoo
 */


#include "encoder.h"
#include "fifo_format.h"
#include "DAC.h"

extern Fifo inputFifo;

#define FREQ_SYMBOL0 0     // No sound
#define FREQ_SYMBOL1 330   // 330 Hz for SW1
#define FREQ_SYMBOL2 1000  // 1000 Hz for SW2
#define FREQ_SYMBOL3 660   // 660 Hz for both buttons

static uint8_t currentSymbol = 0;

uint16_t Encoder_Process(void) {
    uint8_t input;
    if (Fifo_Get(&inputFifo, &input)) {
        currentSymbol = input;
    }

    switch (currentSymbol) {
        case 1:
            return FREQ_SYMBOL1;
        case 2:
            return FREQ_SYMBOL2;
        case 3:
            return FREQ_SYMBOL3;
        default:
            return FREQ_SYMBOL0;
    }
}
