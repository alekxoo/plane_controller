// joystick.c
#include <stdint.h>
#include <stdbool.h>
#include "../inc/tm4c123gh6pm.h"
#include "joystick.h"

// Define global variables
volatile uint32_t g_left_x = 0;
volatile uint32_t g_left_y = 0;
volatile uint32_t g_right_x = 0;
volatile uint32_t g_right_y = 0;
volatile uint32_t g_left_sel = 0;
volatile uint32_t g_right_sel = 0;

void JoystickInit(void) {
    volatile uint32_t delay;
    SYSCTL_RCGCADC_R |= 0x00000001; // 1) activate ADC0
    SYSCTL_RCGCGPIO_R |= 0x18;      // 1) activate clock for Port E and D
    while((SYSCTL_PRGPIO_R&0x10) == 0){};
    while((SYSCTL_PRADC_R&0x0001) != 0x0001){};    // good code, but not yet implemented in simulator
    GPIO_PORTE_DIR_R &= ~0x0B;      // 3) make PE3,1-0 input
    GPIO_PORTD_DIR_R &= ~0x07;      // 3) make PD2,1-0 input
    GPIO_PORTE_AFSEL_R |= 0x03;     // 4) enable alternate function on PE1-0
    GPIO_PORTD_AFSEL_R |= 0x03;     // 4) enable alternate function on PD1-0
    GPIO_PORTE_DEN_R &= ~0x03;      // 5) disable digital I/O on PE1-0
    GPIO_PORTD_DEN_R &= ~0x03;      // 5) disable digital I/O on PD1-0
    GPIO_PORTE_DEN_R |= 0x08;      // 5) enable digital I/O on PE3
    GPIO_PORTD_DEN_R |= 0x04;      // 5) enable digital I/O on PD2
    GPIO_PORTE_PUR_R |= 0x08;      // 5) pull-up resistor on PE3
    GPIO_PORTD_PUR_R |= 0x04;      // 5) pull-up resistor on PD2
//  GPIO_PORTE_PCTL_R = GPIO_PORTE_PCTL_R&0xFFFF0000;
    GPIO_PORTE_AMSEL_R |= 0x03;     // 6) enable analog functionality on PE1-0
    GPIO_PORTD_AMSEL_R |= 0x03;     // 6) enable analog functionality on PD1-0
    ADC0_PC_R &= ~0xF;              // 8) clear max sample rate field
    ADC0_PC_R |= 0x1;               //    configure for 125K samples/sec
    ADC0_SSPRI_R = 0x3210;          // 9) Sequencer 3 is lowest priority
    ADC0_ACTSS_R &= ~0x0004;        // 10) disable sample sequencer 2
    ADC0_EMUX_R &= ~0x0F00;         // 11) seq2 is software trigger
    ADC0_SAC_R = 0x06;              // 32-point average
    ADC0_SSMUX2_R = 0x3276;         // 12) set channels for SS2
    ADC0_SSCTL2_R = 0x6000;         // 13) no TS0 D0 IE0 END0 TS1 D1, yes IE1 END1
    ADC0_IM_R &= ~0x0004;           // 14) disable SS2 interrupts
    ADC0_ACTSS_R |= 0x0004;         // 15) enable sample sequencer 2
}

void Joystick_Read() {
    ADC0_PSSI_R = 0x0004;            // 1) initiate SS2
    while((ADC0_RIS_R&0x04)==0){};   // 2) wait for conversion done
    g_right_x = ADC0_SSFIFO2_R&0xFFF;  // 3A) PD1 result
    g_right_y = ADC0_SSFIFO2_R&0xFFF;  // 3A) PD0 result
    g_left_y = ADC0_SSFIFO2_R&0xFFF;  // 3A) PE1 result
    g_left_x = ADC0_SSFIFO2_R&0xFFF;  // 3B) PE0 result
    g_right_sel = GPIO_PORTD_DATA_R & 0x04;
    g_left_sel = GPIO_PORTE_DATA_R & 0x08;
    ADC0_ISC_R = 0x0004;             // 4) acknowledge completion
}

uint32_t JoystickLeft_Sel(void) {
    return (GPIO_PORTE_DATA_R & 0x08); // Read PE3
}

uint32_t JoystickRight_Sel(void) {
    return (GPIO_PORTD_DATA_R & 0x04); // Read PD2
}

void joystick_handler(void) {
    // Read joystick values
    Joystick_Read();
}

