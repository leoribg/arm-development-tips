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
	SPI_Config_t SPI_PinConfig;			/* SPIx configuration settings */
} SPI_Handle_t;

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



#endif /* INC_STM32L07XX_SPI_DRIVER_H_ */
