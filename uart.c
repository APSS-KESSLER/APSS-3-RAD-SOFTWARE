#include "uart.h"
#include <msp430fr2355.h>

void initUart(){
// Configure GPIO for UART
P4SEL0 |= BIT2 | BIT3;
P4SEL1 &= ~(BIT2 | BIT3);

    // Configure UART
    UCA1CTLW0 |= UCSWRST;                      // Put eUSCI in reset
    UCA1CTLW0 |= UCSSEL__SMCLK;                // CLK = SMCLK

    // Baud Rate calculation for 28800 with SMCLK = 1MHz
    // N = 1,000,000 / 28800 = ~34.72
    UCA1BR0 = 2; 
    UCA1BR1 = 0;
    UCA1MCTLW = 0x21 | UCOS16 | UCBRF_1;       

    UCA1CTLW0 &= ~UCSWRST;                     // Initialize eUSCI
    UCA1IE |= UCRXIE;
}
void serialPrintChar(char c){
    while(!(UCA1IFG & UCTXIFG));
    UCA1TXBUF = c;
}

void serialPrintUInt(uint32_t num)
{
    char buffer[10];  // enough for max 5 digits + '\0'
    int i = 0;

    // Handle 0 explicitly
    if (num == 0) {
        buffer[i++] = '0';
    } else {
        while (num > 0 && i < sizeof(buffer)-1) {
            buffer[i++] = (num % 10) + '0';
            num /= 10;
        }
    }

    buffer[i] = '\0';

    // Digits are reversed, so print backwards
    int j = i - 1;
    for (; j >= 0; j--) {
        while (!(UCA1IFG & UCTXIFG));
        UCA1TXBUF = buffer[j];
    }
     while (!(UCA1IFG & UCTXIFG));
        UCA1TXBUF = '\n';
}
