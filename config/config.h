/******************************************************************************
*
* Author: Chris Lawrence
*
* Summary: Header configuration file for constants and macro
*
******************************************************************************/

#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

// --- DECLARATIONS of global variables ---
// Volatile time variables
extern volatile unsigned long interrupt_timer;
extern volatile unsigned long total_deadtime;
extern volatile unsigned long waiting_t1;
extern volatile uint32_t timer_overflows;

// Non-volatile time variables
extern unsigned long measurement_deadtime;
extern unsigned long measurementT1;
extern unsigned long measurementT2;
extern unsigned long time_stamp;
extern unsigned long start_time;

// Muon variables
extern float sipm_voltage;
extern long int count;
extern float last_sipm_voltage;
extern float temperatureC;

// Flags and mode indicators
extern volatile uint8_t waiting_for_interrupt;
extern uint8_t SLAVE;
extern uint8_t MASTER;
extern uint8_t keep_pulse;

extern int CMD_TYPE_0_SLAVE; 
extern int CMD_TYPE_1_SLAVE; 
extern int CMD_TYPE_2_SLAVE;  
extern int CMD_TYPE_0_MASTER;   
extern int CMD_TYPE_1_MASTER;   
extern int CMD_TYPE_2_MASTER;   

// --- CONSTANTS defined using macros ---
#define TIMER_1MS_SMCLK     1000
#define TIMER_1US_SMCLK     1
#define SLAVE_ADDR          0x00
#define TYPE_0_LENGTH       1
#define TYPE_1_LENGTH       2
#define TYPE_2_LENGTH       6
#define MAX_BUFFER_SIZE     20
#define SIGNAL_THRESHOLD    50
#define RESET_THRESHOLD     25


#endif /* CONFIG_H */
