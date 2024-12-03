// joystick.h
#ifndef HC12_H
#define HC12_H

#include <stdio.h>
#include <stdint.h>
#include "../inc/PLL.h"
#include "../inc/tm4c123gh6pm.h"
#include "../inc/UART.h"
#include "../inc/Timer1A.h"
#include "../inc/Timer2A.h"
#include "joystick.h"

extern volatile int g_value;

void Clock_Delay(uint32_t ulCount);

void Clock_Delay1ms(uint32_t n);
void UART1_Init9600(void);
void UART1_OutChar(char data);
void UART1_OutString(char *pt);
char UART1_InChar(void);
void UART1_OutUDec(uint32_t n);
int int_to_str(int num, char *str);
void HC12_ReadAllInput(void);
void HC12_Init(void);
void PortB_Init(void);
void PortD_Init(void);
void PortF_Init(void);
void HC12_SetCommandMode(void);
void HC12_SetNormalMode(void);
void send_data(void);
void receive(void);
void manual_cmd_uart(void);

#endif
