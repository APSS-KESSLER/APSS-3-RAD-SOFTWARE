/******************************************************************************
*
* Author: Chris Lawrence
*
* Summary: Source file for global variable definitions
*
******************************************************************************/

#include "config.h" 

// --- DEFINITIONS of global variables ---
// Volatile time variables
volatile unsigned long interrupt_timer      = 0;
volatile unsigned long total_deadtime       = 0;
volatile unsigned long waiting_t1           = 0;
volatile uint32_t timer_overflows           = 0;

// Non-volatile time variables
unsigned long measurement_deadtime          = 0;
unsigned long measurementT1;
unsigned long measurementT2;
unsigned long time_stamp                    = 0;
unsigned long start_time                    = 0;

// Muon variables
float sipm_voltage                          = 0;
long int count                              = 0;
float last_sipm_voltage                     = 0;
float temperatureC;

// SPI command variables
int CMD_TYPE_0_SLAVE = 0;
int CMD_TYPE_1_SLAVE = 1;
int CMD_TYPE_2_SLAVE = 2;
int CMD_TYPE_0_MASTER = 3;
int CMD_TYPE_1_MASTER = 4;
int CMD_TYPE_2_MASTER = 5;

// Flags and mode indicators
volatile uint8_t waiting_for_interrupt      = 0;
uint8_t SLAVE;
uint8_t MASTER;
uint8_t keep_pulse                          = 0;
