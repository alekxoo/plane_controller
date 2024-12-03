#include <stdio.h>
#include <stdint.h>
#include "../inc/PLL.h"
#include "../inc/tm4c123gh6pm.h"
#include "../inc/UART.h"
#include "../inc/Timer1A.h"
#include "../inc/Timer2A.h"
#include "joystick.h"

void HC12_ReadAllInput(void);
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void Clock_Delay1ms(uint32_t n);
void Receive_LED_On();
void Receive_LED_Off();
void HC12_SetNormalMode();
void UART1_OutUDec();

void Clock_Delay(uint32_t ulCount){
  while(ulCount){
    ulCount--;
  }
}

void Clock_Delay1ms(uint32_t n){
  while(n){
    //Clock_Delay(23746);  // 1 msec, tuned at 80 MHz, originally part of LCD module
        Clock_Delay(6565); //~1ms for some reason?
        //Clock_Delay(3321);
    n--;
  }
}

void UART1_Init9600(void) {
    SYSCTL_RCGCUART_R |= 0x02;    // 1. Enable UART1
    SYSCTL_RCGCGPIO_R |= 0x02;    // 2. Enable clock for Port B
    while ((SYSCTL_PRGPIO_R & 0x02) == 0) {};  // Wait until Port B is ready

    UART1_CTL_R &= ~0x01;         // 3. Disable UART1 while configuring
    UART1_IBRD_R = 520;           // 4. Set integer portion of BRD for 9600 baud rate (assuming 16 MHz clock)
    UART1_FBRD_R = 53;            // 5. Set fractional portion of BRD
    UART1_LCRH_R = 0x0070;        // 6. 8-bit, no parity, 1 stop bit, enable FIFO
    UART1_CC_R = 0x0;             // 7. Use system clock as UART clock source
    UART1_CTL_R |= 0x301;         // 8. Enable UART1, TXE, and RXE

    GPIO_PORTB_AFSEL_R |= 0x03;   // 9. Enable alternate function on PB0 (RX), PB1 (TX)
    GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & 0xFFFFFF00) + 0x00000011; // Configure PB0 and PB1 as UART1
    GPIO_PORTB_DEN_R |= 0x03;     // 10. Enable digital I/O on PB0, PB1
    GPIO_PORTB_AMSEL_R &= ~0x03;  // 11. Disable analog functionality on PB0, PB1
}

void UART1_OutChar(char data) {
    while ((UART1_FR_R & 0x20) != 0); // Wait until TXFF is 0 (transmit FIFO not full)
    UART1_DR_R = data;                // Transmit the character
}

void UART1_OutString(char *pt) {
    while (*pt) {                     // Loop until end of string
        UART1_OutChar(*pt);           // Send each character
        pt++;                         // Move to the next character
    }
}

void UART1_OutUDec(uint32_t n){
// This function uses recursion to convert decimal number
//   of unspecified length as an ASCII string
  if(n >= 10){
    UART1_OutUDec(n/10);
    n = n%10;
  }
  UART1_OutChar(n+'0'); /* n is between 0 and 9 */
}

char UART1_InChar(void) {
    while ((UART1_FR_R & 0x10) != 0);  // Wait until RXFE is 0
    return (char)(UART1_DR_R & 0xFF);  // Return received character
}

void HC12_ReadAllInput(void) {
    while ((UART1_FR_R & 0x10) == 0) { // While RXFE is 0 (data available)
        UART1_InChar();  // Read each character to clear the buffer
    }
}

void HC12_Init(void) {
      UART1_Init9600();     // Initialize UART1 for HC-12 communication
      Clock_Delay1ms(40);
      HC12_SetNormalMode();
      Clock_Delay1ms(40);
//    UART1_OutString("AT+B9600\n"); // UART baud rate set to 9600
//    Clock_Delay1ms(50);
//    HC12_ReadAllInput();
//    UART1_OutString("AT+C007\n"); // Channel 7 selected (001 to 100 valid)
//    Clock_Delay1ms(50);
//    HC12_ReadAllInput();
//    UART1_OutString("AT+P*\n");   // Highest power level (1 to 8)
//    Clock_Delay1ms(50);
//    HC12_ReadAllInput();
//    UART1_OutString("AT+RF\n");   // Read FU transmission mode (FU3)
//    Clock_Delay1ms(50);
//    HC12_ReadAllInput();
//    UART1_OutString("AT+V\n");    // Read firmware
//    Clock_Delay1ms(50);
//    HC12_ReadAllInput();
//    Clock_Delay1ms(200);
//    HC12_ReadAllInput();          // Remove any buffered input
}

// Initialization for PB1 (feedback signal if needed)
void PortB_Init(void) {
    SYSCTL_RCGCGPIO_R |= 0x02;          // Enable Port B clock
    while ((SYSCTL_PRGPIO_R & 0x02) == 0);  // Wait for clock to be ready
    GPIO_PORTB_DIR_R |= 0x02;           // Set PB1 as output
    GPIO_PORTB_DIR_R &= ~0x04;          // Set PB2 as input (switch)
    GPIO_PORTB_DEN_R |= 0x06;           // Enable digital I/O on PB1 and PB2
    GPIO_PORTB_PUR_R |= 0x04;           // Enable pull-up on PB2
}

void PortD_Init(void) {
    SYSCTL_RCGCGPIO_R |= 0x08;          // Enable Port D clock
    while((SYSCTL_PRGPIO_R & 0x08) == 0){}  // Wait for clock to be ready

    // Configure PD3, PD6, PD7 as outputs
    GPIO_PORTD_DIR_R |= 0xC8;           // Set PD3,PD6,PD7 as outputs (0xC8 = 0b11001000)
    GPIO_PORTD_DEN_R |= 0xC8;           // Enable digital I/O on PD3,PD6,PD7

    // Initialize all LEDs to off
    GPIO_PORTD_DATA_R &= ~0xC8;         // Clear PD3,PD6,PD7
}

void PortF_Init(void){
    SYSCTL_RCGCGPIO_R |= 0x20;          // Enable Port F clock
    while((SYSCTL_PRGPIO_R & 0x20) == 0);  // Wait for clock to be ready

    // Your existing PF1 configuration
    GPIO_PORTF_DIR_R |= 0x02;            // Set PF1 as output
    GPIO_PORTF_DEN_R |= 0x02;            // Enable digital function on PF1

    // Configure PF4
    GPIO_PORTF_DIR_R |= 0x10;            // Set PF4 as output (0x10 = 0b10000)
    GPIO_PORTF_DEN_R |= 0x10;            // Enable digital function on PF4
}

// Set HC-12 to command mode (SET pin low)
void HC12_SetCommandMode(void){
    GPIO_PORTF_DATA_R &= ~0x10;  // Set PF4 low
    Clock_Delay1ms(40);          // Wait 40ms for HC-12 to enter command mode
}

// Set HC-12 to normal mode (SET pin high)
void HC12_SetNormalMode(void){
    GPIO_PORTF_DATA_R |= 0x10;   // Set PF4 high
    Clock_Delay1ms(80);          // Wait 80ms for HC-12 to enter normal mode
}

int int_to_str(int num, char *str) {
    // Always write exactly 4 digits with leading zeros
    str[0] = '0' + (num / 1000);        // Thousands digit
    str[1] = '0' + ((num / 100) % 10);  // Hundreds digit
    str[2] = '0' + ((num / 10) % 10);   // Tens digit
    str[3] = '0' + (num % 10);          // Ones digit
    str[4] = '\0';                      // Null terminator

    return 4; // Always returns 4 since we're always creating 4-digit strings
}

void send_data(void) {
    // Turn on Transmit LED
    GPIO_PORTD_DATA_R |= 0x40;          // Set PD6 (Transmit LED)

    char dataString[50] = {0};  // Clear the buffer
    int pos = 0;  // Position in buffer

    // Add starting exclamation mark
    dataString[pos++] = '!';

    // Build the complete string with 4-digit numbers
    pos += int_to_str(g_left_y, dataString + pos);
    dataString[pos++] = ',';
    pos += int_to_str(g_right_x, dataString + pos);
    dataString[pos++] = ',';
    pos += int_to_str(g_right_y, dataString + pos);

    // Add ending exclamation mark and newline
    dataString[pos++] = '!';
    dataString[pos++] = '\n';
    dataString[pos] = '\0';

    // Send as one complete message
    UART1_OutString(dataString);

    // Turn off Transmit LED
    GPIO_PORTD_DATA_R &= ~0x40;         // Clear PD6 (Transmit LED)}
}

volatile int g_value = 0;

char buffer[30] = {0};
void receive(void) {
    int bufIndex = 0;

    while ((UART1_FR_R & 0x10) == 0 && bufIndex < 29) {
        char c = UART1_InChar();
        if (c == '\n') {
            buffer[bufIndex] = '\0';
            break;
        }
        buffer[bufIndex++] = c;
        Clock_Delay1ms(1);
    }

    if (bufIndex > 0) {
        // Check first and second-to-last chars for !
        // (last char before \0 should be !)
        if (buffer[0] == '!' && buffer[bufIndex-1] == '!') {
            int value = 0;
            // Look at each character between the !s
            for (int i = 1; i < bufIndex-1; i++) {
                if (buffer[i] >= '0' && buffer[i] <= '9') {
                    value = value * 10 + (buffer[i] - '0');
                } else {
                    return;  // Not a number, ignore message
                }
            }
            g_value = value;
        }
    }
}
void manual_cmd_uart(void)
{
    // Read a command from UART0 (user input)
    char command[100];
    int cmdIndex = 0;
    char c;

    // Read characters from UART0 until newline is received
    do {
            c = UART_InChar(); // Read from UART0
            if (c != '\r' && cmdIndex < 99) { // Ignore carriage return, prevent overflow
                    command[cmdIndex++] = c;
                    UART_OutChar(c); // Echo input on UART0 for feedback
            }
    } while (c != '\r'); // Command ends with Enter key (newline)
    command[cmdIndex] = '\0'; // Null-terminate the command string

    UART_OutString("\nSending to HC-12...\n"); // Debug message
    UART_OutString(command);
    UART_OutChar('\n');      // Send newline to HC-12 for proper parsing
    UART1_OutString(command); // Send command to HC-12 via UART1
    UART1_OutChar('\n');      // Send newline to HC-12 for proper parsing

}

int main1(void) {
        DisableInterrupts();
        PLL_Init(Bus80MHz);           // Set system clock (if needed)
        UART_Init();
        UART1_Init9600();     // Initialize UART1 for HC-12 communication
        PortF_Init();
        HC12_SetCommandMode();
//        HC12_SetNormalMode();

        // 400,000 cycles / 80MHz = 5ms interrupt period -> 200 HZ
        Timer2A_Init(&receive, 400000, 3);
        EnableInterrupts();
//        HC12_Init();

        while(1) {
            manual_cmd_uart();
        }

}
