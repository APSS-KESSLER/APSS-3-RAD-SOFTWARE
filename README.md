# Radiation Flight Software

## Project Structure

| Module     | Description                         |
|------------|-------------------------------------|
| adc/       | ADC driver                          |
| config/    | Project Wide Variable definitions   |
| datalog/   | Data storage setup                  |
| i2c/       | I2C comms driver                    |
| pinConfig/ | Pin setup                           |
| spi/       | SPI comms driver                    |
| timer/     | Microsecond timer driver            |
| uart/      | Debug UART interface                |
| main/      | Main control loop                   |



# SPI Interfaces

## OBC SPI Interface (eUSCI_A0)

OBC is SPI master, RAD payload is slave.

| Function | MCU Pin |
|----------|---------|
| SIMO     | P1.7    |
| SOMI     | P1.6    |
| SCLK     | P1.5    |
| STE      | P1.4    |

## Debug SPI Interface (eUSCI_B0)

| Function    | MCU Pin |
|-------------|----------|
| SIMO (MOSI) | P1.3     |
| SOMI (MISO) | P1.2     |
| SCLK        | P1.1     |
| STE / CS    | P1.0     |



