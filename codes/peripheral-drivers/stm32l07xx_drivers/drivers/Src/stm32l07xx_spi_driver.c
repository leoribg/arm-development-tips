/*
 * stm32l07xx_spi_driver.c
 *
 *  Created on: Jan 28, 2022
 *      Author: leonardo
 */

#include "stm32l07xx_spi_driver.h"

/*********************************************************************
 * @fn      		  - SPI_PeriClockControl
 *
 * @brief             -
 *
 * @param[in]         - Pointer to the SPI register
 * @param[in]         - Enable flag
 *
 * @Note              -
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t En) {
	if (En == ENABLE) {
		if (pSPIx == SPI1) {
			SPI1_PCLK_EN();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_EN();
		}
	} else {
		if (pSPIx == SPI1) {
			SPI1_PCLK_DI();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_DI();
		}
	}
}

/*********************************************************************
 * @fn      		  - SPI_Init
 *
 * @brief             -
 *
 * @param[in]         - SPI handle
 *
 * @Note              -
 */
void SPI_Init(SPI_Handle_t *pSPIHandle) {
	uint32_t temp = 0;

	/* peripheral clock enable */
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	/* Configure the SPI CR1 Register */
	/* SPI_DeviceMode */
	temp  = pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR_BIT;

	/* SPI_BusConfig */
	if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD) {
		/* Clear BIDI MODE*/
		temp &= ~( 1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD) {
		/* Set BIDI MODE*/
		temp |= ( 1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY) {
		/* Clear BIDI MODE*/
		temp &= ~( 1 << SPI_CR1_BIDIMODE);
		/* Set RXONLY */
		temp |= ( 1 << SPI_CR1_RXONLY);
	}

	/* SPI_SclkSpeed */
	temp |= pSPIHandle->SPI_Config.SPI_SclkSpeed << SPI_CR1_BR;

	/* SPI_DFF */
	temp |= pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF;

	/* SPI_CPOL */
	temp |= pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL;

	/* SPI_CPHA */
	temp |= pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA;

	/* SPI_SSM */
	temp |= pSPIHandle->SPI_Config.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 = temp;
}

void SPI_DeInit(SPI_RegDef_t *pSPIx) {
	if (pSPIx == SPI1) {
		SPI1_REG_RESET();
	} else if (pSPIx == SPI2) {
		SPI2_REG_RESET();
	}
}
