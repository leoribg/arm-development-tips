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

}

void SPI_DeInit(SPI_RegDef_t *pSPIx) {
	if (pSPIx == SPI1) {
		SPI1_REG_RESET();
	} else if (pSPIx == SPI2) {
		SPI2_REG_RESET();
	}
}
