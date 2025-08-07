#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

// States for pin 1.4
typedef enum {
    GPIO_14in,
    GPIO_14out,
    A4,
    UCA0STE
} pin_state14;

// States for pin 1.5
typedef enum {
    GPIO_15in,
    GPIO_15out,
    A5,
    UCA0CLK
} pin_state15;

// States for pin 1.6
typedef enum {
    GPIO_16in,
    GPIO_16out,
    A6,
    UCA0RX,
    MISO
} pin_state16;

// States for pin 1.7
typedef enum {
    GPIO_17in,
    GPIO_17out,
    A7,
    UCA0TX,
    MOSI
} pin_state17;

// Prototype declarations
void configurePinI2C(void);
void configurePin1_4(pin_state14 config);
void configurePin1_5(pin_state15 config);
void configurePin1_6(pin_state16 config);
void configurePin1_7(pin_state17 config);

#endif
