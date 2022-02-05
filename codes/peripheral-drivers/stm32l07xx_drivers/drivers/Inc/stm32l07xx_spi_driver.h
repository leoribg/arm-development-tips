/*
 * stm32l07xx_spi_driver.h
 *
 *  Created on: Jan 28, 2022
 *      Author: leonardo
 */

#ifndef INC_STM32L07XX_SPI_DRIVER_H_
#define INC_STM32L07XX_SPI_DRIVER_H_

#include "stm32l07xx.h"

/*
 * This is the configuration structure for SPIx peripheral
 * */
typedef struct {
	uint8_t SPI_DeviceMode;				/*!< Master selection >*/
	uint8_t SPI_BusConfig;				/*!< Bidirectional data mode enable >*/
	uint8_t SPI_SclkSpeed;				/*!< Baud rate control >*/
	uint8_t SPI_DFF;					/*!< Data frame format >*/
	uint8_t SPI_CPOL;					/*!< Clock polarity >*/
	uint8_t SPI_CPHA;					/*!< Clock phase >*/
	uint8_t SPI_SSM;					/*!< Software slave management >*/
} SPI_Config_t;

/*
 * This is a Handle structure for SPIx peripheral
 * */

typedef struct {
	SPI_RegDef_t *pSPIx;				/* The base address of the SPIx peripheral */
	SPI_Config_t SPI_Config;			/* SPIx configuration settings */
} SPI_Handle_t;

/*
 * SPI_DeviceMode
 * */
#define SPI_DEVICE_MODE_MASTER				1
#define SPI_DEVICE_MODE_SLAVE				0

/*
 * SPI_BusConfig
 * */
#define SPI_BUS_CONFIG_FD					1
#define SPI_BUS_CONFIG_HD					2
#define SPI_BUS_CONFIG_SIMPLEX_TXONLY		3
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY		4

/*
 * SPI_SclkSpeed
 * */
#define SPI_SCLK_SPEED_DIV2					0
#define SPI_SCLK_SPEED_DIV4					1
#define SPI_SCLK_SPEED_DIV8					2
#define SPI_SCLK_SPEED_DIV16				3
#define SPI_SCLK_SPEED_DIV32				4
#define SPI_SCLK_SPEED_DIV64				5
#define SPI_SCLK_SPEED_DIV128				6
#define SPI_SCLK_SPEED_DIV256				7

/*
 * SPI_DFF
 * */
#define SPI_DFF_8BITS						0
#define SPI_DFF_16BITS						1

/*
 * SPI_CPOL
 * */
#define SPI_CPOL_HIGH						1
#define SPI_CPOL_LOW						0

/*
 * SPI_CPHA
 * */
#define SPI_CPHA_HIGH						1
#define SPI_CPHA_LOW						0

/*
 * SPI_SSM
 * */
#define SPI_SSM_EN							1
#define SPI_SSM_DI							0

/*
 * SPI_CR1 Register
 * */
#define SPI_CR1_CPHA						0
#define SPI_CR1_CPOL						1
#define SPI_CR1_MSTR						2
#define SPI_CR1_BR							3
#define SPI_CR1_SPE							6
#define SPI_CR1_SSI							8
#define SPI_CR1_SSM							9
#define SPI_CR1_RXONLY						10
#define SPI_CR1_DFF							11
#define SPI_CR1_BIDIMODE					15

/*
 * SPI_CR2 Register
 * */
#define SPI_CR2_SSOE						2

/*
 * SPI_SR Register
 * */
#define SPI_SR_RXNE							0
#define SPI_SR_TXE							1
#define SPI_SR_BSY							7

/*************************************************************************************************************
 *											APIs supported by this driver
 ************************************************************************************************************* */

/*
 * Peripheral Clock Setup
 * */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t En);

/*
 * Init and DeInit
 * */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data send and receive
 * */
void SPI_sendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_receiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR Handling
 * */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t En);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

/*
 * Other Peripheral Control APIs
 * */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t En);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t En);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t En);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t Flag);


#endif /* INC_STM32L07XX_SPI_DRIVER_H_ */
