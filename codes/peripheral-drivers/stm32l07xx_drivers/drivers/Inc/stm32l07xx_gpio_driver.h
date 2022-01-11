/*
 * stm32l07xx_gpio_driver.h
 *
 *  Created on: Jan 9, 2022
 *      Author: leonardo
 */

#ifndef INC_STM32L07XX_GPIO_DRIVER_H_
#define INC_STM32L07XX_GPIO_DRIVER_H_

#include "stm32l07xx.h"

/*
 * This is the configuration structure for a GPIO pin
 * */
typedef struct {
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfig_t;

/*
 * This is a Handle structure for a GPIO pin
 * */

typedef struct {
	GPIO_RegDef_t *pGPIOx;					/* The base address of GPIO Port to which the pin belongs to */
	GPIO_PinConfig_t GPIO_PinConfig;		/* GPIO pin configuration settings */
} GPIO_Handle_t;

/*************************************************************************************************************
 *											APIs supported by this driver
 ************************************************************************************************************* */

/*
 * Peripheral Clock Setup
 * */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t En);

/*
 * Init and DeInit
 * */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data read and write
 * */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and handling
 * */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t En);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32L07XX_GPIO_DRIVER_H_ */
