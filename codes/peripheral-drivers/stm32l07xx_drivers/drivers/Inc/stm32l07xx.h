/*
 * stm32l07xx.h
 *
 *  Created on: Jan 8, 2022
 *      Author: leonardo
 */

#ifndef INC_STM32L07XX_H_
#define INC_STM32L07XX_H_

#include <stdint.h>

#define __weak 						__attribute((weak))
/*
 * Cortex-M0 Specific Registers
 * */
#define NVIC_ISER					(volatile uint32_t *)0xE000E100
#define NVIC_ICER					(volatile uint32_t *)0xE000E180
#define NVIC_ISPR					(volatile uint32_t *)0xE000E200
#define NVIC_ICPR					(volatile uint32_t *)0xE000E280

#define NVIC_IPRO0					(volatile uint32_t *)0xE000E400
#define NVIC_IPRO1					(volatile uint32_t *)0xE000E404
#define NVIC_IPRO2					(volatile uint32_t *)0xE000E408
#define NVIC_IPRO3					(volatile uint32_t *)0xE000E40C
#define NVIC_IPRO4					(volatile uint32_t *)0xE000E410
#define NVIC_IPRO5					(volatile uint32_t *)0xE000E414
#define NVIC_IPRO6					(volatile uint32_t *)0xE000E418
#define NVIC_IPRO7					(volatile uint32_t *)0xE000E41C

#define NO_PRIO_BITS_IMPLEMENTED	(2)

/*
 * Memory Base Addresses
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


typedef struct
{
	volatile uint32_t CR1;        	/*!< SPI control register 1 (SPI_CR1) (not used in I2S mode) Address offset: 0x00 */
	volatile uint32_t CR2;        	/*!< SPI control register 2 (SPI_CR2) Address offset: 0x04 */
	volatile uint32_t SR;         	/*!< SPI status register (SPI_SR) Address offset: 0x08 */
	volatile uint32_t DR;         	/*!< SPI data register (SPI_DR) Address offset: 0x0C */
	volatile uint32_t CRCPR;      	/*!< SPI CRC polynomial register (SPI_CRCPR) (not used in I2S mode) Address offset: 0x10 */
	volatile uint32_t RXCRCR;     	/*!< SPI RX CRC register (SPI_RXCRCR) (not used in I2S mode) Address offset: 0x14 */
	volatile uint32_t TXCRCR;     	/*!< SPI TX CRC register (SPI_TXCRCR) (not used in I2S mode) Address offset: 0x18 */
	volatile uint32_t I2SCFGR;    	/*!< SPI_I2S configuration register (SPI_I2SCFGR) Address offset: 0x1C */
	volatile uint32_t I2SPR;      	/*!< SPI_I2S prescaler register (SPI_I2SPR) Address offset: 0x20 */
} SPI_RegDef_t;

typedef struct
{
	volatile uint32_t CR1;        	/*!< I2C control register 1 (I2C_CR1) Address offset: 0x00 */
	volatile uint32_t CR2;        	/*!< I2C control register 2 (I2C_CR2) Address offset: 0x04 */
	volatile uint32_t OAR1;         /*!< I2C own address 1 register (I2C_OAR1) Address offset: 0x08 */
	volatile uint32_t OAR2;         /*!< I2C own address 2 register (I2C_OAR2) Address offset: 0x0C */
	volatile uint32_t TIMINGR;      /*!< I2C timing register (I2C_TIMINGR) Address offset: 0x10 */
	volatile uint32_t TIMEOUTR;    	/*!< I2C timeout register (I2C_TIMEOUTR) Address offset: 0x14 */
	volatile uint32_t ISR;     		/*!< I2C interrupt and status register (I2C_ISR) Address offset: 0x18 */
	volatile uint32_t ICR;    	 	/*!< I2C interrupt clear register (I2C_ICR) Address offset: 0x1C */
	volatile uint32_t PECR;      	/*!< I2C PEC register (I2C_PECR) Address offset: 0x20 */
	volatile uint32_t RXDR;      	/*!< I2C receive data register (I2C_RXDR) Address offset: 0x24 */
	volatile uint32_t TXDR;      	/*!< I2C transmit data register (I2C_TXDR) Address offset: 0x28 */
} I2C_RegDef_t;

typedef struct
{
	volatile uint32_t CR1;        	/*!< USART control register 1 (USART_CR1) Address offset: 0x00 */
	volatile uint32_t CR2;        	/*!< USART control register 2 (USART_CR2) Address offset: 0x04 */
	volatile uint32_t CR3;         	/*!< USART control register 3 (USART_CR3) Address offset: 0x08 */
	volatile uint32_t BRR;         	/*!< USART baud rate register (USART_BRR) Address offset: 0x0C */
	volatile uint32_t GTPR;      	/*!< USART guard time and prescaler register Address (USART_GTPR) offset: 0x10 */
	volatile uint32_t RTOR;     	/*!< USART receiver timeout register (USART_RTOR) Address offset: 0x14 */
	volatile uint32_t RQR;     		/*!< USART request register (USART_RQR) Address offset: 0x18 */
	volatile uint32_t ISR;    		/*!< USART interrupt and status register (USART_ISR) Address offset: 0x1C */
	volatile uint32_t ICR;      	/*!< USART interrupt flag clear register (USART_ICR) Address offset: 0x20 */
	volatile uint32_t RDR;      	/*!< USART receive data register (USART_RDR) Address offset: 0x24 */
	volatile uint32_t TDR;      	/*!< USART transmit data register (USART_TDR) Address offset: 0x28 */
} USART_RegDef_t;

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

typedef struct {
	volatile uint32_t IMR;			/* EXTI interrupt mask register */
	volatile uint32_t EMR;			/* EXTI event mask register */
	volatile uint32_t RTSR;			/* EXTI rising edge trigger selection register */
	volatile uint32_t FTSR;			/* Falling edge trigger selection register */
	volatile uint32_t SWIER;		/* EXTI software interrupt event register */
	volatile uint32_t PR;			/* EXTI pending register */
} EXTI_RegDef_t;

typedef struct {
	volatile uint32_t CFGR1;		/* SYSCFG memory remap register */
	volatile uint32_t CFGR2;		/* SYSCFG peripheral mode configuration register */
	volatile uint32_t EXTICR[4];	/* SYSCFG external interrupt configuration register */
	volatile uint32_t CFGR3;		/* Reference control and status register */
} SYSCFG_RegDef_t;

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

#define EXTI	((EXTI_RegDef_t *)EXTI_BASEADDR)

#define SYSCFG	((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)

#define SPI1	((SPI_RegDef_t *)SPI1_BASEADDR)
#define SPI2	((SPI_RegDef_t *)SPI2_BASEADDR)

#define I2C1	((I2C_RegDef_t *)I2C1_BASEADDR)
#define I2C2	((I2C_RegDef_t *)I2C2_BASEADDR)
#define I2C3	((I2C_RegDef_t *)I2C3_BASEADDR)

#define USART1	((USART_RegDef_t *)USART1_BASEADDR)
#define USART2	((USART_RegDef_t *)USART2_BASEADDR)
#define USART4	((USART_RegDef_t *)USART4_BASEADDR)
#define USART5	((USART_RegDef_t *)USART5_BASEADDR)

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
 * MACROS for GPIO Reset Peripherals
 * */
#define	GPIOA_REG_RESET()		do{ (RCC->IOPRSTR |= (1 << 0)); (RCC->IOPRSTR &= ~(1 << 0)); } while(0) /* Set and clear the IOPA RST bit */
#define	GPIOB_REG_RESET()		do{ (RCC->IOPRSTR |= (1 << 1)); (RCC->IOPRSTR &= ~(1 << 1)); } while(0) /* Set and clear the IOPB RST bit */
#define	GPIOC_REG_RESET()		do{ (RCC->IOPRSTR |= (1 << 2)); (RCC->IOPRSTR &= ~(1 << 2)); } while(0) /* Set and clear the IOPC RST bit */
#define	GPIOD_REG_RESET()		do{ (RCC->IOPRSTR |= (1 << 3)); (RCC->IOPRSTR &= ~(1 << 3)); } while(0) /* Set and clear the IOPD RST bit */
#define	GPIOE_REG_RESET()		do{ (RCC->IOPRSTR |= (1 << 4)); (RCC->IOPRSTR &= ~(1 << 4)); } while(0) /* Set and clear the IOPE RST bit */
#define	GPIOH_REG_RESET()		do{ (RCC->IOPRSTR |= (1 << 7)); (RCC->IOPRSTR &= ~(1 << 7)); } while(0) /* Set and clear the IOPH RST bit */

/*
 * Returns port code based on given GPIO Base Address
 * */
#define GPIO_BASEADDR_TO_CODE(x)		((x == GPIOA) ? 0 :\
										(x == GPIOB) ? 1 :\
										(x == GPIOC) ? 2 :\
										(x == GPIOD) ? 3 :\
										(x == GPIOE) ? 4 :\
										(x == GPIOH) ? 5 : 0)

/*
 * IRQ Numbers
 * */
/* GPIO */
#define IRQ_NO_EXTI0_1					5		/* Table 55. List of vectors of Reference Manual (page 290) */
#define IRQ_NO_EXTI2_3					6		/* Table 55. List of vectors of Reference Manual (page 290) */
#define IRQ_NO_EXTI4_15					7		/* Table 55. List of vectors of Reference Manual (page 290) */
/* SPI */
#define IRQ_NO_SPI_1					25		/* Table 55. List of vectors of Reference Manual (page 290) SPI1 global interrupt */
#define IRQ_NO_SPI_2					26		/* Table 55. List of vectors of Reference Manual (page 290) SPI2 global interrupt*/

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
 * Clock Enable MACROS for USART Peripherals
 * */

#define	USART1_PCLK_EN()		(RCC->APB2ENR |= (1 << 14)) /* Set the USART1EN bit */
#define	USART2_PCLK_EN()		(RCC->APB1ENR |= (1 << 17)) /* Set the USART2EN bit */
#define	USART4_PCLK_EN()		(RCC->APB1ENR |= (1 << 19)) /* Set the USART4EN bit */
#define	USART5_PCLK_EN()		(RCC->APB1ENR |= (1 << 20)) /* Set the USART5EN bit */

#define	USART1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 14)) /* Clear the USART1EN bit */
#define	USART2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 17)) /* Clear the USART2EN bit */
#define	USART4_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 19)) /* Clear the USART4EN bit */
#define	USART5_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 20)) /* Clear the USART5EN bit */

/*
 * Clock Enable MACROS for SPI Peripherals
 * */

#define	SPI1_PCLK_EN()		(RCC->APB2ENR |= (1 << 12)) /* Set the SPI1EN bit */
#define	SPI2_PCLK_EN()		(RCC->APB1ENR |= (1 << 14)) /* Set the SPI2EN bit */

#define	SPI1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 12)) /* Clear the SPI1EN bit */
#define	SPI2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 14)) /* Clear the SPI2EN bit */

/*
 * MACROS for SPI Reset Peripherals
 * */
#define	SPI1_REG_RESET()		do{ (RCC->APB2RSTR |= (12 << 0)); (RCC->APB2RSTR &= ~(12 << 0)); } while(0) /* Set and clear the SPI1 RST bit */
#define	SPI2_REG_RESET()		do{ (RCC->APB1RSTR |= (14 << 1)); (RCC->APB1RSTR &= ~(14 << 1)); } while(0) /* Set and clear the SPI2 RST bit */

/*
 * Clock Enable MACROS for SYSCFG Peripheral
 * */
#define	SYSCFG_PCLK_EN()		(RCC->APB2ENR |= (1 << 0)) /* Set the SYSCFEN bit */

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
