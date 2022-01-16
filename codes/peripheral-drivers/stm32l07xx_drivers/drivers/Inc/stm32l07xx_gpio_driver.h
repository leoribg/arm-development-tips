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
	uint8_t GPIO_PinMode;				/*!< Possible values from @GPIO_PIN_MODES >*/
	uint8_t GPIO_PinSpeed;				/*!< Possible values from @GPIO_PIN_OUTPUT_SPEEDS >*/
	uint8_t GPIO_PinPuPdControl;		/*!< Possible values from @GPIO_PIN_PULL_UP_DOWN >*/
	uint8_t GPIO_PinOPType;				/*!< Possible values from @@GPIO_PIN_OUTPUT_TYPES >*/
	uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfig_t;

/*
 * This is a Handle structure for a GPIO pin
 * */

typedef struct {
	GPIO_RegDef_t *pGPIOx;					/* The base address of GPIO Port to which the pin belongs to */
	GPIO_PinConfig_t GPIO_PinConfig;		/* GPIO pin configuration settings */
} GPIO_Handle_t;

/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 * */
#define GPIO_PIN_0					0
#define GPIO_PIN_1					1
#define GPIO_PIN_2					2
#define GPIO_PIN_3					3
#define GPIO_PIN_4					4
#define GPIO_PIN_5					5
#define GPIO_PIN_6					6
#define GPIO_PIN_7					7
#define GPIO_PIN_8					8
#define GPIO_PIN_9					9
#define GPIO_PIN_10					10
#define GPIO_PIN_11					11
#define GPIO_PIN_12					12
#define GPIO_PIN_13					13
#define GPIO_PIN_14					14
#define GPIO_PIN_15					15

/*
 * @GPIO_PIN_MODES
 * GPIO pin modes
 * */
#define GPIO_MODE_INPUT				0
#define GPIO_MODE_OUTPUT			1
#define GPIO_MODE_ALTERNATE			2
#define GPIO_MODE_ANALOG			3
#define GPIO_MODE_IT_FT				4
#define GPIO_MODE_IT_RT				5
#define GPIO_MODE_IT_FR				6

/*
 * @GPIO_PIN_OUTPUT_TYPES
 * GPIO pin output types
 * */
#define GPIO_OP_TYPE_PP				0
#define GPIO_OP_TYPE_OD				1

/*
 * @GPIO_PIN_OUTPUT_SPEEDS
 * GPIO pin output speeds
 * */
#define GPIO_SPEED_LOW				0
#define GPIO_SPEED_MEDIUM			1
#define GPIO_SPEED_FAST				2
#define GPIO_SPEED_HIGH				3

/*
 * @GPIO_PIN_PULL_UP_DOWN
 * GPIO pin pull up and pull down configurations
 * */
#define GPIO_NO_PUPD				0
#define GPIO_PU						1
#define GPIO_PD						2

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
