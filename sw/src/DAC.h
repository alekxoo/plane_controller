/*
 * DAC.h
 *
 *  Created on: Oct 8, 2024
 *      Author: alekxoo
 */

#ifndef DAC_H_
#define DAC_H_

#include <stdint.h>

void SSI_init(void);
void DAC_Init(void);
void DAC_Out(uint16_t code);
void DAC_Start(void);
void DAC_Stop(void);
void set_frequency(uint32_t frequency);
void DAC_EnableOutput(uint8_t enable);
uint16_t Encoder_GetNextSample(void);
void PortD_Init(void);

#endif /* DAC_H_ */
