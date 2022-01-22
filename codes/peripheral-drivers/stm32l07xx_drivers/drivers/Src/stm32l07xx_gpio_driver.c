/*
 * stm32l07xx_gpio_driver.c
 *
 *  Created on: Jan 9, 2022
 *      Author: leonardo
 */

#include "stm32l07xx_gpio_driver.h"

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t En) {
	if (En == ENABLE) {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_EN();
		}
	} else {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_DI();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_DI();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_DI();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_DI();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_DI();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_DI();
		}
	}
}

/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             -
 *
 * @Note              -
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {
	uint32_t temp;

	/* Configure GPIO MODE */
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode
				<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // Multiply by 2 * pinNumber because each pin takes 2 bits of configuration in the register
		pGPIOHandle->pGPIOx->MODER &= ~(0x03 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= temp;
	}
	else {
		/* Configuring the pin as INPUT MODE */
		temp = (GPIO_MODE_INPUT << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // Multiply by 2 * pinNumber because each pin takes 2 bits of configuration in the register
				pGPIOHandle->pGPIOx->MODER &= ~(0x03 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
				pGPIOHandle->pGPIOx->MODER |= temp;
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT) {
			/* Configure the Falling Trigger Selection Register (FTSR) */
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT) {
			/* Configure the Rising Trigger Selection Register (RTSR) */
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FR) {
			/* Configure both Falling and Rising Trigger Selection Register (FTSR and RTSR) */
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		/* Configure the GPIO port selection in SYSCFG_EXTICR */
		uint8_t index = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) / 4; // Because each EXTICR register configures 4 pins
		uint8_t position = ((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) % 4) * 4;
		uint8_t port_code = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);

		SYSCFG_PCLK_EN();

		SYSCFG->EXTICR[index] = port_code << (position);

		/* Enable the EXTI interrupt delivery (IMR) */
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	temp = 0;

	/* Configure GPIO SPEED */
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed
			<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // Multiply by 2 * pinNumber because each pin takes 2 bits of configuration in the register
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x03 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;

	/* Configure GPIO PULL UP/DOWN */
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl
			<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // Multiply by 2 * pinNumber because each pin takes 2 bits of configuration in the register
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x03 << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;

	/* Configure GPIO OUTPUT TYPE */
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType
			<< (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // each pin takes 1 bit of configuration in the register
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x01 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	/* Configure GPIO ALTERNATE FUNCTIONALITY */
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTERNATE) {
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] |= (0x0F << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |=
				(pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2)); // Multiply by 4 * pinNumber because each pin takes 4 bits of configuration in the register

	}
}

/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             -
 *
 * @Note              -
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {
	if (pGPIOx == GPIOA) {
		GPIOA_REG_RESET();
	} else if (pGPIOx == GPIOB) {
		GPIOA_REG_RESET();
	} else if (pGPIOx == GPIOC) {
		GPIOA_REG_RESET();
	} else if (pGPIOx == GPIOD) {
		GPIOA_REG_RESET();
	} else if (pGPIOx == GPIOE) {
		GPIOA_REG_RESET();
	} else if (pGPIOx == GPIOH) {
		GPIOA_REG_RESET();
	}
}

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             -
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - pin number of GPIO
 *
 * @return            -   0 or 1
 *
 * @Note              -
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	uint8_t value;

	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x01);

	return value;
}

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
 *
 * @brief             -
 *
 * @param[in]         - base address of the GPIO peripheral
 *
 * @return            - bit mask of all input pins values
 *
 * @Note              -
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {
	uint16_t value;

	value = (uint16_t)pGPIOx->IDR;

	return value;
}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
 *
 * @brief             -
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - pin number of GPIO
 * @param[in]         - value to be written to the pin
 *
 *
 * @Note              -
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,
		uint8_t Value) {
	if(Value == GPIO_SET) {
		/* Write 1 to the bit position corresponding to the Pin Number */
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else {
		/* Write 0 to the bit position corresponding to the Pin Number */
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
 *
 * @brief             -
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - value to be written to all pins of that portal
 *
 *
 * @Note              -
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value) {
	/* Write the entire Value to Port Output Data Register */
	pGPIOx->ODR = Value;
}

/*********************************************************************
 * @fn      		  - GPIO_ToggleOutputPin
 *
 * @brief             -
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - pin number of GPIO
 *
 *
 * @Note              -
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	/* Toggle the value of corresponding bit position of GPIO Port ODR */
	pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 * IRQ Interrupt Configuration
 * */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t En) {
	if (En) {
		if (IRQNumber <= 31) {
			/* Configure the ISER register (Cortex™-M0 Devices | Generic User Guide - page 110) */
			*NVIC_ISER |= (1 << IRQNumber);
		}
	}
	else {
		if (IRQNumber <= 31) {
			/* Configure the ISER register (Cortex™-M0 Devices | Generic User Guide - page 110) */
			*NVIC_ICER |= (1 << IRQNumber);
		}
	}
}

/*
 * IRQ Interrupt Priority Configuration
 * */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = (IRQNumber % 4) * 8;
	uint8_t shift_amount;

	shift_amount = iprx_section + (8 - NO_PRIO_BITS_IMPLEMENTED); /* (Cortex™-M0 Devices | Generic User Guide - page 113) */

	*(NVIC_IPRO0 + iprx) |= (IRQPriority << shift_amount);

}

/*
 * IRQ Interrupt Handling
 * */
void GPIO_IRQHandling(uint8_t PinNumber) {
	/* Clear the EXTI PR Register corresponding to the PinNumber */
	if(EXTI->PR & ( 1 << PinNumber)) {
		EXTI->PR |= ( 1 << PinNumber);
	}
}
