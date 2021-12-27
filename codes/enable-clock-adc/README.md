# Enabling ADC peripheral

## About <a name = "about"></a>

Steps to enable and modify peripheral registers on an MCU.

- Go to memory map address of the MCU and get the register address of RCC
- Set the RCC register to enable the desired peripheral clock
- Go to memory map address of the MCU and get the desired register address of ADC (or other peripheral)
- Modify the correpondent bit to the desired configuration

## Documentation <a name = "steps"></a>

 - [STM32L073RZ - Reference Manual](./docs/rm0367-ultralowpower-stm32l0x3-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
 - [STM32L073RZ - Datasheet](./docs/stm32l073rz.pdf)