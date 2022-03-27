/*
 * stm32l07xx_spi_driver.c
 *
 *  Created on: Jan 28, 2022
 *      Author: leonardo
 */

#include <stddef.h>
#include "stm32l07xx_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

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

	/* Enable the peripheral clock */
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	/* Configure the SPI CR1 Register */
	/* SPI_DeviceMode */
	temp  = pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR;

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

/*********************************************************************
 * @fn      		  - SPI_DeInit
 *
 * @brief             -
 *
 * @param[in]         - Pointer to the SPI register
 *
 * @Note              -
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx) {
	if (pSPIx == SPI1) {
		SPI1_REG_RESET();
	} else if (pSPIx == SPI2) {
		SPI2_REG_RESET();
	}
}

/*********************************************************************
 * @fn      		  - SPI_GetFlagStatus
 *
 * @brief             -
 *
 * @param[in]         - Pointer to the SPI register
 * @param[in]         - Bit field of the flag in SR
 *
 * @Note              -
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t Flag) {
	if(pSPIx->SR & (1 << Flag)) {
		return SET;
	}
	return CLEAR;
}

/*********************************************************************
 * @fn      		  - SPI_sendData
 *
 * @brief             -
 *
 * @param[in]         - Pointer to the SPI register
 * @param[in]         - Pointer to the Tx buffer
 * @param[in]         - Number of bytes to be transmitted
 *
 * @Note              -
 */
void SPI_sendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len) {
	while (Len > 0) {
		/* Wait until TXE is set (TX buffer is empty) */
		while (SPI_GetFlagStatus(pSPIx, SPI_SR_TXE) == CLEAR);

		/* Check DFF */
		if ((pSPIx->CR1 & (1 << SPI_CR1_DFF))) {
			/* 16 bit DFF */
			pSPIx->DR = *((uint16_t*) pTxBuffer);
			Len -= 2;
			(uint16_t*) pTxBuffer++;
		} else {
			/* 8 bit DFF */
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}

/*********************************************************************
 * @fn      		  - SPI_sendDataIT
 *
 * @brief             -
 *
 * @param[in]         - Pointer to the SPI Handle
 * @param[in]         - Pointer to the Tx buffer
 * @param[in]         - Number of bytes to be transmitted
 *
 * @Note              -
 */
uint8_t SPI_sendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len) {
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX) {
		/* Save the Tx buffer address and length information in some global variables */
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;
		/* Mark the SPI as busy in transmission, so no other code can use the peripheral */
		pSPIHandle->TxState = SPI_BUSY_IN_TX;
		/* Enable TXIE control bit to get the interrupt whenever TXE flag is set in SR */
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
		/* Data transmission will be handled by the ISR code */
	}

	/* Data transmission will be handled by the ISR */

	return state;
}

/*********************************************************************
 * @fn      		  - SPI_receiveData
 *
 * @brief             -
 *
 * @param[in]         - Pointer to the SPI register
 * @param[in]         - Pointer to the Rx buffer
 * @param[in]         - Number of bytes to be received
 *
 * @Note              -
 */
void SPI_receiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len) {
	while (Len > 0) {
		/* Wait until RXNE is set (RX buffer is not empty) */
		while (SPI_GetFlagStatus(pSPIx, SPI_SR_RXNE) == CLEAR);

		/* Check DFF */
		if ((pSPIx->CR1 & (1 << SPI_CR1_DFF))) {
			/* 16 bit DFF */
			*((uint16_t*) pRxBuffer) = (uint16_t) pSPIx->DR;
			Len -= 2;
			(uint16_t*) pRxBuffer++;
		} else {
			/* 8 bit DFF */
			 *pRxBuffer = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
}

/*********************************************************************
 * @fn      		  - SPI_receiveDataIT
 *
 * @brief             -
 *
 * @param[in]         - Pointer to the SPI Handle
 * @param[in]         - Pointer to the Rx buffer
 * @param[in]         - Number of bytes to be received
 *
 * @Note              -
 */
uint8_t SPI_receiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len) {
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX) {
		/* Save the Tx buffer address and length information in some global variables */
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;
		/* Mark the SPI as busy in transmission, so no other code can use the peripheral */
		pSPIHandle->RxState = SPI_BUSY_IN_RX;
		/* Enable TXIE control bit to get the interrupt whenever RXE flag is set in SR */
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
		/* Data transmission will be handled by the ISR code */
	}

	/* Data transmission will be handled by the ISR */

	return state;
}

/*********************************************************************
 * @fn      		  - SPI_PeripheralControl
 *
 * @brief             -
 *
 * @param[in]         - Pointer to the SPI register
 * @param[in]         - Enable or disable the peripheral
 *
 * @Note              -
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t En) {
	if(En) {
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/*********************************************************************
 * @fn      		  - SPI_SSIConfig
 *
 * @brief             -
 *
 * @param[in]         - Pointer to the SPI register
 * @param[in]         - Set or Clear the SSI
 *
 * @Note              -
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t En) {
	if(En) {
			pSPIx->CR1 |= (1 << SPI_CR1_SSI);
		}
		else {
			pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
		}
}

/*********************************************************************
 * @fn      		  - SPI_SSOEConfig
 *
 * @brief             -
 *
 * @param[in]         - Pointer to the SPI register
 * @param[in]         - Set or Clear the SSOE
 *
 * @Note              -
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t En) {
	if(En) {
			pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
		}
		else {
			pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
		}
}

/* Static helper functions */
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle) {
	/* Check DFF */
	if ((pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))) {
		/* 16 bit DFF */
		pSPIHandle->pSPIx->DR = *((uint16_t*) pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen -= 2;
		(uint16_t*) pSPIHandle->pTxBuffer++;
	} else {
		/* 8 bit DFF */
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}
	if(pSPIHandle->TxLen == 0) {
		/* Close the communication and inform the application that TX operation is over */
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle) {
	/* Check DFF */
		if ((pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))) {
			/* 16 bit DFF */
		*((uint16_t*) pSPIHandle->pRxBuffer) = (uint16_t) pSPIHandle->pSPIx->DR;
			pSPIHandle->RxLen -= 2;
			(uint16_t*) pSPIHandle->pRxBuffer++;
		} else {
			/* 8 bit DFF */
			 *pSPIHandle->pRxBuffer = pSPIHandle->pSPIx->DR;
			pSPIHandle->RxLen--;
			pSPIHandle->pRxBuffer++;
		}
		if(pSPIHandle->RxLen == 0) {
			/* Close the communication and inform the application that RX operation is over */
			SPI_CloseReception(pSPIHandle);
			SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
		}
}
static void spi_err_interrupt_handle(SPI_Handle_t *pSPIHandle) {
	uint8_t temp;

	/* Clear the OVR flag */
	(void) temp;
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX) {
		SPI_ClearOVRFlag(pSPIHandle->pSPIx);
	}

	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);

}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle) {
	/* Clear TXEIE interrupt */
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle) {
	/* Clear RXNEIE interrupt */
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx) {
	uint8_t temp;

	temp = pSPIx->DR;
	temp = pSPIx->SR;

	(void) temp;
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvt) {

}

/*********************************************************************
 * @fn      		  - SPI_IRQHandling
 *
 * @brief             -
 *
 * @param[in]         - Pointer to the SPI Handle
 *
 * @Note              -
 */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle) {

	uint8_t temp;
	uint8_t temp_2;
	/* Check for TXE in SR*/
	temp = (pSPIHandle->pSPIx->SR & SPI_SR_TXE);
	temp_2 = (pSPIHandle->pSPIx->CR2 & SPI_CR2_TXEIE);

	if(temp && temp_2) {
		/* Handle TXE */
		spi_txe_interrupt_handle(pSPIHandle);
	}

	/* Check RXNE */
	temp = (pSPIHandle->pSPIx->SR & SPI_SR_RXNE);
	temp_2 = (pSPIHandle->pSPIx->CR2 & SPI_CR2_RXNEIE);

	if(temp && temp_2) {
		/* Handle RXNE */
		spi_rxne_interrupt_handle(pSPIHandle);
	}

	/* Check OVR flag */
	temp = (pSPIHandle->pSPIx->SR & SPI_SR_OVR);
	temp_2 = (pSPIHandle->pSPIx->CR2 & SPI_CR2_ERRIE);

	if(temp && temp_2) {
		/* Handle ERR */
		spi_err_interrupt_handle(pSPIHandle);
	}
}
