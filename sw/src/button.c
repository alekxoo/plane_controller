#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"
#include "DAC.h"
#include "encoder.h"
#include "fifo_format.h"

volatile uint8_t SW1_Pressed = 0;
volatile uint8_t SW2_Pressed = 0;
extern Fifo inputFifo;

// Debounce variables
#define DEBOUNCE_TIME 50 // 50ms debounce time
volatile uint32_t last_button_press_time = 0;

void PortF_Init(void) {
    volatile uint32_t delay;
    SYSCTL_RCGCGPIO_R |= 0x20;
    delay = SYSCTL_RCGCGPIO_R;
    GPIO_PORTF_LOCK_R = 0x4C4F434B;
    GPIO_PORTF_CR_R = 0x1F;
    GPIO_PORTF_DIR_R &= ~0x11;  // PF0 and PF4 as inputs
    GPIO_PORTF_DEN_R |= 0x11;   // Digital enable
    GPIO_PORTF_PDR_R |= 0x11;   // Pull-down resistors
    GPIO_PORTF_IS_R &= ~0x11;   // Edge-sensitive
    GPIO_PORTF_IBE_R &= ~0x11;  // Not both edges
    GPIO_PORTF_IEV_R |= 0x11;   // Rising edge event
    GPIO_PORTF_ICR_R = 0x11;    // Clear any prior interrupts
    GPIO_PORTF_IM_R |= 0x11;    // Unmask interrupt
    // Set interrupt priority (0 is highest, 7 is lowest)
    NVIC_PRI7_R = (NVIC_PRI7_R & 0xFF00FFFF) | 0x00400000; // Priority 2
    NVIC_EN0_R = 1 << 30;  // Enable IRQ30 for PORTF

    // Initialize SysTick for debounce timing
    NVIC_ST_CTRL_R = 0;                   // Disable SysTick during setup
    NVIC_ST_RELOAD_R = 16000000 - 1;      // Reload value for 1ms at 16MHz
    NVIC_ST_CURRENT_R = 0;                // Clear current value
    NVIC_ST_CTRL_R = 0x00000005;          // Enable SysTick with core clock
}

// Function to get current time in milliseconds
uint32_t millis(void) {
    return NVIC_ST_CURRENT_R / 16000;  // Assuming 16MHz clock
}

void GPIOPortF_Handler(void) {
    uint32_t status = GPIO_PORTF_RIS_R;
    GPIO_PORTF_ICR_R = status;

    uint32_t current_time = millis();

    // Check if enough time has passed since the last button press
    if (current_time - last_button_press_time >= DEBOUNCE_TIME) {
        uint32_t current_state = GPIO_PORTF_DATA_R & 0x11;
        uint8_t fifo_was_empty = (Fifo_Size(&inputFifo) == 0);

        if (current_state == 0x11) {  // Both buttons are pressed
            Fifo_Put(&inputFifo, 3);  // Encode as 3 for both pressed
        } else if (current_state & 0x10) {  // SW1 pressed
            Fifo_Put(&inputFifo, 1);
        } else if (current_state & 0x01) {  // SW2 pressed
            Fifo_Put(&inputFifo, 2);
        } else {
            Fifo_Put(&inputFifo, 0);  // No button pressed
        }

        // If FIFO was empty before this button press, start the DAC
        if (fifo_was_empty && current_state != 0) {
            DAC_Start();
        }

        // Update the last button press time
        last_button_press_time = current_time;
    }
}
