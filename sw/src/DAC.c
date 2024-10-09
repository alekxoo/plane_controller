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
    1024, 1122, 1219, 1314, 1407, 1495, 1580, 1658, 1731, 1798, 1855,
    1906, 1948, 1979, 2001, 2015, 2024, 2015, 2001, 1979, 1948, 1906,
    1855, 1798, 1731, 1658, 1580, 1495, 1407, 1314, 1219, 1122, 1024,
    926,  829,  734,  641,  554,  469,  392,  314,  247,  190,  139,
     93,   58,   27,    5,    0,    5,   27,   58,   93,  139,  190,
    247,  314,  392,  469,  554,  641,  734,  829,  926
};


#define SYSTEM_CLOCK 80000000  // Assuming 80 MHz system clock
#define WAVE_SAMPLES 64        // Number of samples per wave cycle
#define DAC_SAMPLE_RATE 44100 // Define your desired sample rate
#define OUTPUT_DURATION_MS 500  // 5 sec output duration
#define OUTPUT_DURATION_SAMPLES (DAC_SAMPLE_RATE * OUTPUT_DURATION_MS / 1000)

volatile uint32_t current_frequency = 0;
volatile uint8_t DAC_OutputEnabled = 0;
volatile uint32_t sample_counter = 0;
volatile uint32_t wave_index = 0;
volatile uint32_t increment_value = 1; // New variable for frequency control

// Add this function to initialize Port D
void PortD_Init(void) {
    SYSCTL_RCGCGPIO_R |= 0x08;      // Enable Port D clock
    while((SYSCTL_PRGPIO_R & 0x08) == 0) {}; // Wait for clock to stabilize
    GPIO_PORTD_DIR_R |= 0x03;       // Make PD0 and PD1 outputs
    GPIO_PORTD_DEN_R |= 0x03;       // Enable digital I/O on PD0 and PD1
}


void SSI_init(void);
static void Timer0A_Init(uint32_t frequency);

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
    // Calculate the increment value based on the desired frequency
    increment_value = (uint32_t)((float)frequency * WAVE_SAMPLES / DAC_SAMPLE_RATE * 65536);
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
        if (sample_counter < OUTPUT_DURATION_SAMPLES) {
            // Output the current sample
            DAC_Out(Wave[wave_index >> 10]); // Use the upper 6 bits of wave_index

            // Update wave index for next sample using the increment value
            wave_index = (wave_index + increment_value) & 0xFFFF;

            sample_counter++;
        } else {
            // Stop output after OUTPUT_DURATION_MS
            DAC_OutputEnabled = 0;
            DAC_Out(1024);  // Output middle voltage when no sound (adjusted for 11-bit)
            GPIO_PORTD_DATA_R &= ~0x03;  // Turn off both LEDs

            // Get new frequency from encoder for next cycle
            current_frequency = Encoder_Process();
            set_frequency(current_frequency); // Update the frequency

            // Reset counters for next cycle
            sample_counter = 0;
            wave_index = 0;
        }
    } else {
        // Check if we should start a new cycle
        if (current_frequency != 0) {
            DAC_OutputEnabled = 1;
            sample_counter = 0;
            wave_index = 0;
            set_frequency(current_frequency); // Ensure frequency is set correctly

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
        } else {
            GPIO_PORTD_DATA_R &= ~0x03;  // Turn off both LEDs when output is disabled
            DAC_Out(1024);  // Output middle voltage when disabled (adjusted for 11-bit)
        }
    }
}

void DAC_EnableOutput(uint8_t enable) {
    DAC_OutputEnabled = enable;
}

