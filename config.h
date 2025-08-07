/******************************************************************************
*
* Author: Chris Lawrence
*
* Summary: Header configuration file for constants and macro
*
******************************************************************************/

#ifndef CONFIG_H
#define CONFIG_H


// Time constants
#define TIMER_1MS_SMCLK               1000              // At 1 MHz SMCLK, 1 clock cycle = 1 µs → 1000 cycles = 1 ms
#define TIMER_1US_SMCLK               1                 // At 1 MHz SMCLK, 1 clock cycle = 1 µs → 1000 cycles = 1 ms


// I2C constants
#define SLAVE_ADDR                    0x00              // TODO: Set I2C slave address (placeholder, not yet assigned)

// Command identifiers for Slave device communication
#define CMD_TYPE_0_SLAVE              0
#define CMD_TYPE_1_SLAVE              1
#define CMD_TYPE_2_SLAVE              2

// Command identifiers for Master device communication
#define CMD_TYPE_0_MASTER             3
#define CMD_TYPE_1_MASTER             4
#define CMD_TYPE_2_MASTER             5

// Data lengths for each command type in bytes
#define TYPE_0_LENGTH                 1
#define TYPE_1_LENGTH                 2
#define TYPE_2_LENGTH                 6

#define MAX_BUFFER_SIZE               20                // Will require changing in future once requirements known

// Signal and LED constants
const int SIGNAL_THRESHOLD = 50;                        // Min threshold to trigger on. See calibration.pdf for conversion to mV
const int RESET_THRESHOLD = 25;


#endif /* CONFIG_H */
