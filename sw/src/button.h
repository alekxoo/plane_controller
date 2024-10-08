/*
 * button.h
 *
 *  Created on: Oct 8, 2024
 *      Author: alekxoo
 */

#include <stdint.h>


#ifndef BUTTON_H_
#define BUTTON_H_

extern volatile uint8_t SW1_Pressed;
extern volatile uint8_t SW2_Pressed;
extern volatile uint8_t Both_Pressed;


void PortF_Init(void);
void handle_button_presses(void);
void GPIOPortF_Handler(void);


#endif /* BUTTON_H_ */
