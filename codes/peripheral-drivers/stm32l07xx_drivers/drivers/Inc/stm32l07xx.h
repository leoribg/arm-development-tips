/*
 * stm32l07xx.h
 *
 *  Created on: Jan 8, 2022
 *      Author: leonardo
 */

#ifndef INC_STM32L07XX_H_
#define INC_STM32L07XX_H_

#include <stdint.h>

/*
 *
 * */
#define FLASH_BASEADDR			0x08000000U			// Section 3.3.1 NVM organization of Reference Manual (page 67)
#define SRAM_BASEADDR			0x20000000U			// Section 2.3 Embedded SRAM of Reference Manual (page 64)
#define ROM_BASEADDR			0x1FF00000U			// Section 3.3.1 NVM organization of Reference Manual (page 67)
#define SRAM					SRAM_BASEADDR

/*
 * AHB and APBx Peripheral base addresses
 * */
#define PERIPH_BASEADDR				0x40000000U 		// Section 2.2.2 Memory map and register boundary addresses of Reference Manual (page 59)
#define	APB1PERIPH_BASEADDR			PERIPH_BASEADDR			// Table 3. STM32L0x3 peripheral register boundary addresses of Reference Manual (page 63)
#define	APB2PERIPH_BASEADDR			0x40010000U			// Table 3. STM32L0x3 peripheral register boundary addresses of Reference Manual (page 61)
#define	AHBPERIPH_BASEADDR			0x40020000U			// Table 3. STM32L0x3 peripheral register boundary addresses of Reference Manual (page 60)
#define	IOPORTPERIPH_BASEADDR		0x50000000U			// Table 3. STM32L0x3 peripheral register boundary addresses of Reference Manual (page 60)

/*
 * Base addresses of IOPORT peripherals
 * */
#define GPIOA_BASEADDR			(IOPORTPERIPH_BASEADDR)
#define GPIOB_BASEADDR			(IOPORTPERIPH_BASEADDR + 0x0400U) 	// Table 3. STM32L0x3 peripheral register boundary addresses of Reference Manual (page 60)
#define GPIOC_BASEADDR			(IOPORTPERIPH_BASEADDR + 0x0800U) 	// Table 3. STM32L0x3 peripheral register boundary addresses of Reference Manual (page 60)
#define GPIOD_BASEADDR			(IOPORTPERIPH_BASEADDR + 0x0C00U) 	// Table 3. STM32L0x3 peripheral register boundary addresses of Reference Manual (page 60)
#define GPIOE_BASEADDR			(IOPORTPERIPH_BASEADDR + 0x1000U) 	// Table 3. STM32L0x3 peripheral register boundary addresses of Reference Manual (page 60)
#define GPIOH_BASEADDR			(IOPORTPERIPH_BASEADDR + 0x1C00U) 	// Table 3. STM32L0x3 peripheral register boundary addresses of Reference Manual (page 60)

/*
 * Base addresses of APB1 peripherals
 * */
#define TIMER2_BASEADDR			(APB1PERIPH_BASEADDR + 0x0000U)	// Table 3. STM32L0x3 peripheral register boundary addresses of Reference Manual (page 63)
#define TIMER3_BASEADDR			(APB1PERIPH_BASEADDR + 0x0400U)	// Table 3. STM32L0x3 peripheral register boundary addresses of Reference Manual (page 63)
#define TIMER6_BASEADDR			(APB1PERIPH_BASEADDR + 0x1000U)	// Table 3. STM32L0x3 peripheral register boundary addresses of Reference Manual (page 63)
#define TIMER7_BASEADDR			(APB1PERIPH_BASEADDR + 0x1400U)	// Table 3. STM32L0x3 peripheral register boundary addresses of Reference Manual (page 63)
#define LCD_BASEADDR			(APB1PERIPH_BASEADDR + 0x2400U)	// Table 3. STM32L0x3 peripheral register boundary addresses of Reference Manual (page 63)
#define RTC_BASEADDR			(APB1PERIPH_BASEADDR + 0x2800U)	// Table 3. STM32L0x3 peripheral register boundary addresses of Reference Manual (page 63)
#define WWDG_BASEADDR			(APB1PERIPH_BASEADDR + 0x2C00U)	// Table 3. STM32L0x3 peripheral register boundary addresses of Reference Manual (page 63)
#define IWDG_BASEADDR			(APB1PERIPH_BASEADDR + 0x3000U)	// Table 3. STM32L0x3 peripheral register boundary addresses of Reference Manual (page 63)
#define SPI2_BASEADDR			(APB1PERIPH_BASEADDR + 0x3800U)	// Table 3. STM32L0x3 peripheral register boundary addresses of Reference Manual (page 63)
#define USART2_BASEADDR			(APB1PERIPH_BASEADDR + 0x4400U)	// Table 3. STM32L0x3 peripheral register boundary addresses of Reference Manual (page 63)
#define LPUART1_BASEADDR		(APB1PERIPH_BASEADDR + 0x4800U)	// Table 3. STM32L0x3 peripheral register boundary addresses of Reference Manual (page 63)
#define USART4_BASEADDR			(APB1PERIPH_BASEADDR + 0x4C00U)	// Table 3. STM32L0x3 peripheral register boundary addresses of Reference Manual (page 63)
#define USART5_BASEADDR			(APB1PERIPH_BASEADDR + 0x5000U)	// Table 3. STM32L0x3 peripheral register boundary addresses of Reference Manual (page 63)
#define I2C1_BASEADDR			(APB1PERIPH_BASEADDR + 0x5400U)	// Table 3. STM32L0x3 peripheral register boundary addresses of Reference Manual (page 63)
#define I2C2_BASEADDR			(APB1PERIPH_BASEADDR + 0x5800U)	// Table 3. STM32L0x3 peripheral register boundary addresses of Reference Manual (page 63)
#define USB_BASEADDR			(APB1PERIPH_BASEADDR + 0x5C00U)	// Table 3. STM32L0x3 peripheral register boundary addresses of Reference Manual (page 62)
#define USB_SRAM_BASEADDR		(APB1PERIPH_BASEADDR + 0x6000U)	// Table 3. STM32L0x3 peripheral register boundary addresses of Reference Manual (page 62)
#define CRS_BASEADDR			(APB1PERIPH_BASEADDR + 0x6C00U)	// Table 3. STM32L0x3 peripheral register boundary addresses of Reference Manual (page 62)
#define PWR_BASEADDR			(APB1PERIPH_BASEADDR + 0x7000U)	// Table 3. STM32L0x3 peripheral register boundary addresses of Reference Manual (page 62)
#define DAC1_2_BASEADDR			(APB1PERIPH_BASEADDR + 0x7400U)	// Table 3. STM32L0x3 peripheral register boundary addresses of Reference Manual (page 62)
#define I2C3_BASEADDR			(APB1PERIPH_BASEADDR + 0x7800U)	// Table 3. STM32L0x3 peripheral register boundary addresses of Reference Manual (page 62)
#define LPTIM1_BASEADDR			(APB1PERIPH_BASEADDR + 0x7c00U)	// Table 3. STM32L0x3 peripheral register boundary addresses of Reference Manual (page 62)

/*
 * Base addresses of APB2 peripherals
 * */
#define SYSCFG_BASEADDR			(APB2PERIPH_BASEADDR + 0x0000U)	// Table 3. STM32L0x3 peripheral register boundary addresses of Reference Manual (page 61)
#define EXTI_BASEADDR			(APB2PERIPH_BASEADDR + 0x0400U)	// Table 3. STM32L0x3 peripheral register boundary addresses of Reference Manual (page 61)
#define TIM21_BASEADDR			(APB2PERIPH_BASEADDR + 0x0800U)	// Table 3. STM32L0x3 peripheral register boundary addresses of Reference Manual (page 61)
#define TIM22_BASEADDR			(APB2PERIPH_BASEADDR + 0x1400U)	// Table 3. STM32L0x3 peripheral register boundary addresses of Reference Manual (page 61)
#define FWW_BASEADDR			(APB2PERIPH_BASEADDR + 0x1C00U)	// Table 3. STM32L0x3 peripheral register boundary addresses of Reference Manual (page 61)
#define ADC1_BASEADDR			(APB2PERIPH_BASEADDR + 0x2400U)	// Table 3. STM32L0x3 peripheral register boundary addresses of Reference Manual (page 61)
#define SPI1_BASEADDR			(APB2PERIPH_BASEADDR + 0x3000U)	// Table 3. STM32L0x3 peripheral register boundary addresses of Reference Manual (page 61)
#define USART1_BASEADDR			(APB2PERIPH_BASEADDR + 0x3800U)	// Table 3. STM32L0x3 peripheral register boundary addresses of Reference Manual (page 61)
#define DBG_BASEADDR			(APB2PERIPH_BASEADDR + 0x5800U)	// Table 3. STM32L0x3 peripheral register boundary addresses of Reference Manual (page 61)

/*
 * Base addresses of AHB peripherals
 * */
#define DMA1_BASEADDR			(AHBPERIPH_BASEADDR + 0x0000U)	// Table 3. STM32L0x3 peripheral register boundary addresses of Reference Manual (page 60)
#define RCC_BASEADDR			(AHBPERIPH_BASEADDR + 0x1000U)	// Table 3. STM32L0x3 peripheral register boundary addresses of Reference Manual (page 60)
#define FLASH_CFG_BASEADDR		(AHBPERIPH_BASEADDR + 0x2000U)	// Table 3. STM32L0x3 peripheral register boundary addresses of Reference Manual (page 60)
#define CRC_BASEADDR			(AHBPERIPH_BASEADDR + 0x3000U)	// Table 3. STM32L0x3 peripheral register boundary addresses of Reference Manual (page 60)
#define TSC_BASEADDR			(AHBPERIPH_BASEADDR + 0x4000U)	// Table 3. STM32L0x3 peripheral register boundary addresses of Reference Manual (page 60)
#define RNG_BASEADDR			(AHBPERIPH_BASEADDR + 0x5000U)	// Table 3. STM32L0x3 peripheral register boundary addresses of Reference Manual (page 60)
#define AES_BASEADDR			(AHBPERIPH_BASEADDR + 0x6000U)	// Table 3. STM32L0x3 peripheral register boundary addresses of Reference Manual (page 60)

/*
 * Peripheral Registers Definition Structures
 * */

typedef struct {
	volatile uint32_t MODER;	/* GPIO port mode register */
	volatile uint32_t OTYPER;	/* GPIO port output type register */
	volatile uint32_t OSPEEDR;	/* GPIO port output speed register */
	volatile uint32_t PUPDR;	/* GPIO port pull-up/pull-down register */
	volatile uint32_t IDR;		/* GPIO port input data register */
	volatile uint32_t ODR;		/* GPIO port output data register */
	volatile uint32_t BSRR;		/* GPIO port bit set/reset register */
	volatile uint32_t LCKR;		/* GPIO port configuration lock register */
	volatile uint32_t AFR[2]; 	/* GPIO alternate function low and high registers */
} GPIO_RegDef_t;

typedef struct {
	volatile uint32_t CR;			/* Clock control register */
	volatile uint32_t ICSCR;		/* Internal clock sources calibration register */
	volatile uint32_t CRRCR;		/* Clock recovery RC register */
	volatile uint32_t CFGR;			/* Clock configuration register */
	volatile uint32_t CIER;			/* Clock interrupt enable register */
	volatile uint32_t CIFR;			/* Clock interrupt flag register */
	volatile uint32_t CICR;			/* Clock interrupt clear register */
	volatile uint32_t IOPRSTR;		/* GPIO reset register */
	volatile uint32_t AHBRSTR; 		/* AHB peripheral reset register */
	volatile uint32_t APB2RSTR; 	/* APB2 peripheral reset register */
	volatile uint32_t APB1RSTR; 	/* APB1 peripheral reset register */
	volatile uint32_t IOPENR; 		/* GPIO clock enable register */
	volatile uint32_t AHBENR; 		/* AHB peripheral clock enable register */
	volatile uint32_t APB2ENR; 		/* APB2 peripheral clock enable register */
	volatile uint32_t APB1ENR; 		/* APB1 peripheral clock enable register */
	volatile uint32_t IOPSMENR;		/* GPIO clock enable in Sleep mode register */
	volatile uint32_t AHBSMENR; 	/* AHB peripheral clock enable in Sleep mode register */
	volatile uint32_t APB2SMENR; 	/* APB2 peripheral clock enable in Sleep mode register */
	volatile uint32_t APB1SMENR; 	/* APB1 peripheral clock enable in Sleep mode register */
	volatile uint32_t CCIPR; 		/* Clock configuration register */
	volatile uint32_t CSR; 			/* Control/status register */
} RCC_RegDef_t;

/*
 * Peripheral Definitions
 * */

#define GPIOA	((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB	((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC	((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD	((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE	((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOH	((GPIO_RegDef_t *)GPIOH_BASEADDR)

#define RCC		((RCC_RegDef_t *)RCC_BASEADDR)

/*
 * Clock Enable MACROS for GPIO Peripherals
 * */

#define	GPIOA_PCLK_EN()		(RCC->IOPENR |= (1 << 0)) /* Set the IOPA EN bit */
#define	GPIOB_PCLK_EN()		(RCC->IOPENR |= (1 << 1)) /* Set the IOPB EN bit */
#define	GPIOC_PCLK_EN()		(RCC->IOPENR |= (1 << 2)) /* Set the IOPC EN bit */
#define	GPIOD_PCLK_EN()		(RCC->IOPENR |= (1 << 3)) /* Set the IOPD EN bit */
#define	GPIOE_PCLK_EN()		(RCC->IOPENR |= (1 << 4)) /* Set the IOPE EN bit */
#define	GPIOH_PCLK_EN()		(RCC->IOPENR |= (1 << 7)) /* Set the IOPH EN bit */

#define	GPIOA_PCLK_DI()		(RCC->IOPENR &= ~(1 << 0)) /* Clear the IOPA EN bit */
#define	GPIOB_PCLK_DI()		(RCC->IOPENR &= ~(1 << 1)) /* Clear the IOPB EN bit */
#define	GPIOC_PCLK_DI()		(RCC->IOPENR &= ~(1 << 2)) /* Clear the IOPC EN bit */
#define	GPIOD_PCLK_DI()		(RCC->IOPENR &= ~(1 << 3)) /* Clear the IOPD EN bit */
#define	GPIOE_PCLK_DI()		(RCC->IOPENR &= ~(1 << 4)) /* Clear the IOPE EN bit */
#define	GPIOH_PCLK_DI()		(RCC->IOPENR &= ~(1 << 7)) /* Clear the IOPH EN bit */

/*
 * Clock Enable MACROS for I2C Peripherals
 * */

#define	I2C1_PCLK_EN()		(RCC->APB1ENR |= (1 << 21)) /* Set the I2C1EN bit */
#define	I2C2_PCLK_EN()		(RCC->APB1ENR |= (1 << 22)) /* Set the I2C2EN bit */
#define	I2C3_PCLK_EN()		(RCC->APB1ENR |= (1 << 30)) /* Set the I2C3EN bit */

#define	I2C1_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 21)) /* Clear the I2C1EN bit */
#define	I2C2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 22)) /* Clear the I2C2EN bit */
#define	I2C3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 30)) /* Clear the I2C3EN bit */

/*
 * Generic MACROS
 * */
#define ENABLE 				1U
#define DISABLE 			0U
#define SET 				ENABLE
#define CLEAR 				DISABLE
#define GPIO_SET 			SET
#define GPIO_CLEAR 			CLEAR


#endif /* INC_STM32L07XX_H_ */
