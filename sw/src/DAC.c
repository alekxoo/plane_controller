/*
 * DAC.c
 *
 *  Created on: Oct 8, 2024
 *      Author: alekxoo
 */


#include <stdint.h>
#include "DAC.h"
#include "encoder.h"
#include "../inc/tm4c123gh6pm.h"
#include "../inc/SysTick.h"


const uint16_t Wave[64] = {
  2048, 2244, 2438, 2628, 2814, 2990, 3160, 3316, 3462, 3596, 3710,
  3812, 3896, 3958, 4002, 4030, 4048, 4030, 4002, 3958, 3896, 3812,
  3710, 3596, 3462, 3316, 3160, 2990, 2814, 2628, 2438, 2244, 2048,
  1852, 1658, 1468, 1282, 1108,  938,  784,  628,  494,  380,  278,
   186,  116,   54,   10,    0,   10,   54,  116,  186,  278,  380,
   494,  628,  784,  938, 1108, 1282, 1468, 1658, 1852
};

#define SYSTEM_CLOCK 80000000  // Assuming 80 MHz system clock
#define WAVE_SAMPLES 64        // Number of samples per wave cycle
#define DAC_SAMPLE_RATE 44100 // Define your desired sample rate
#define OUTPUT_DURATION_MS 50  // 50 ms output duration
#define OUTPUT_DURATION_SAMPLES (OUTPUT_DURATION_MS * DAC_SAMPLE_RATE / 1000)

volatile uint32_t current_frequency = 0;
volatile uint8_t DAC_OutputEnabled = 0;
volatile uint32_t sample_counter = 0;
volatile uint32_t wave_index = 0;

// Add this function to initialize Port D
void PortD_Init(void) {
    SYSCTL_RCGCGPIO_R |= 0x08;      // Enable Port D clock
    while((SYSCTL_PRGPIO_R & 0x08) == 0) {}; // Wait for clock to stabilize
    GPIO_PORTD_DIR_R |= 0x03;       // Make PD0 and PD1 outputs
    GPIO_PORTD_DEN_R |= 0x03;       // Enable digital I/O on PD0 and PD1
}


void SSI_init(void);
static void Timer0A_Init(uint32_t period);

void DAC_Init(void) {
    SSI_init();
    Timer0A_Init(DAC_SAMPLE_RATE); // Assuming 80MHz system clock
    PortD_Init();  // Initialize Port D for LED indicators
}

void SSI_init(void){
    SYSCTL_RCGCSSI_R |= 0x01;       // activate SSI0
    SYSCTL_RCGCGPIO_R |= 0x01;      // activate port A
    while((SYSCTL_PRGPIO_R&0x01) == 0){};// ready?
    // Configure PA2,3,5 as SSI
    GPIO_PORTA_AFSEL_R |= 0x2C; // enable alt function PA2,3,5
    GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R&0xFF0F00FF)+0x00202200;
    GPIO_PORTA_DEN_R |= 0x04; //PA2
    GPIO_PORTA_DEN_R |= 0x08; //PA3
    GPIO_PORTA_DEN_R |= 0x20; //PA5
    GPIO_PORTA_AMSEL_R &= ~0x2C;

    // Disable SSI before configuration
    SSI0_CR1_R &= ~SSI_CR1_SSE;       // Disable SSI
    SSI0_CR1_R &= ~0x04;        // Select master mode
    SSI0_CC_R &= ~0xF;                  // Use system clock
    SSI0_CPSR_R = 8;
    SSI0_CR0_R &= ~0x40;
    SSI0_CR0_R |= 0x80;

    SSI0_CR0_R &= ~0x30;
    SSI0_CR0_R |= 0xF;
    SSI0_CR1_R |= SSI_CR1_SSE;
}

void DAC_Out(uint16_t code) {
    while((SSI0_SR_R & SSI_SR_TNF) == 0){}; // SSI Transmit FIFO Not Full
    GPIO_PORTA_DATA_R &= ~0x08; // Set PA3 (FSS) low to start communication
    SSI0_DR_R = code;                  // data out
}


// When the timer counts down to zero form this value, it triggers an interrupt and reloads it for next cycle
// interrupt frequency = system clock frequemcy / period
static void Timer0A_Init(uint32_t frequency) {
    uint32_t period = SYSTEM_CLOCK / frequency;
    SYSCTL_RCGCTIMER_R |= 0x01;        // Enable clock for Timer0
    TIMER0_CTL_R = 0x00000000;         // Disable Timer0 during setup
    TIMER0_CFG_R = 0x00000000;         // Configure Timer0 as 32-bit timer
    TIMER0_TAMR_R = 0x00000002;        // Set Timer0 to periodic mode
    TIMER0_TAILR_R = period - 1;       // Set timer reload value for desired period
    TIMER0_IMR_R = 0x00000001;         // Enable timeout interrupt for Timer0A
    TIMER0_CTL_R |= 0x00000001;        // Enable Timer0A

    NVIC_PRI4_R = (NVIC_PRI4_R&0x00FFFFFF)|0x80000000; // Priority 4
    NVIC_EN0_R = 1 << 19;              // Enable interrupt 19 (Timer0A) in NVIC
}


void set_frequency(uint32_t frequency) {
    current_frequency = frequency;
    uint32_t period = SYSTEM_CLOCK / (frequency * WAVE_SAMPLES);

    TIMER0_CTL_R = 0x00000000;    // Disable Timer0A during reconfiguration
    TIMER0_TAILR_R = period - 1;  // Set new reload value
    TIMER0_CTL_R |= 0x00000001;   // Re-enable Timer0A
}

void DAC_Start(void) {
    DAC_OutputEnabled = 1;
    sample_counter = 0;
    wave_index = 0;
}

void DAC_Stop(void) {
    TIMER0_CTL_R &= ~0x00000001;  // Disable Timer0A
}

void Timer0A_Handler(void) {
    TIMER0_ICR_R = TIMER_ICR_TATOCINT;  // Clear the timer interrupt flag

    if (DAC_OutputEnabled) {
        if (sample_counter == 0) {
            // Get new frequency from encoder
            current_frequency = Encoder_Process();
            if (current_frequency == 0) {
                DAC_OutputEnabled = 0;
                DAC_Out(2048);  // Output middle voltage when no sound
                GPIO_PORTD_DATA_R &= ~0x03;  // Turn off both LEDs
                return;
            }
            wave_index = 0;  // Reset wave index for new frequency

            // Set LED indicators based on frequency
            if (current_frequency == 330) {
                GPIO_PORTD_DATA_R = (GPIO_PORTD_DATA_R & ~0x03) | 0x01;  // Turn on PD0, turn off PD1
            } else if (current_frequency == 1000) {
                GPIO_PORTD_DATA_R = (GPIO_PORTD_DATA_R & ~0x03) | 0x02;  // Turn on PD1, turn off PD0
            } else if (current_frequency == 660) {
                GPIO_PORTD_DATA_R |= 0x03;  // Turn on both PD0 and PD1
            } else {
                GPIO_PORTD_DATA_R &= ~0x03;  // Turn off both LEDs for other frequencies
            }
        }

        // Output the current sample
        DAC_Out(Wave[wave_index]);

        // Update wave index for next sample
        wave_index = (wave_index + 1) % WAVE_SAMPLES;

        sample_counter++;
        if (sample_counter >= OUTPUT_DURATION_SAMPLES) {
            sample_counter = 0;
            // Immediately try to get the next frequency
            current_frequency = Encoder_Process();
            if (current_frequency == 0) {
                DAC_OutputEnabled = 0;
                DAC_Out(2048);  // Output middle voltage when no sound
                GPIO_PORTD_DATA_R &= ~0x03;  // Turn off both LEDs
            }
        }
    } else {
        GPIO_PORTD_DATA_R &= ~0x03;  // Turn off both LEDs when output is disabled
    }
}


void DAC_EnableOutput(uint8_t enable) {
    DAC_OutputEnabled = enable;
}

