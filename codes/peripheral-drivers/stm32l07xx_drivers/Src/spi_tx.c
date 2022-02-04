/*
 * spi_tx.c
 *
 *  Created on: Feb 1, 2022
 *      Author: leonardo
 */
#include <stdint.h>
#include <stm32l07xx_gpio_driver.h>
#include <string.h>
#include "stm32l07xx.h"
#include "stm32l07xx_spi_driver.h"

/*
 * PA4	SPI1_NSS
 * PA5 	SPI1_SCLK
 * PA6	SPI1_MISO
 * PA7	SPI1_MOSI
 * */

void SPI_GPIOInits();
void SPI1_init();

int main (void) {
	const char user_data[] = "Hello World";

	SPI_GPIOInits();

	SPI1_init();

	SPI_sendData(SPI1, (uint8_t *)user_data, strlen(user_data));

	return 0;
}

void SPI_GPIOInits() {
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOA;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTERNATE;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 0; 					/* DataSheet page 57 (Table 17. Alternate functions port A)*/
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_4;
	GPIO_Init(&SPIPins);

	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	GPIO_Init(&SPIPins);

	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
	GPIO_Init(&SPIPins);

	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;
	GPIO_Init(&SPIPins);

}

void SPI1_init() {
	SPI_Handle_t SPI1Handle;

	SPI1Handle.pSPIx = SPI1;
	SPI1Handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI1Handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI1Handle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	SPI1Handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI1Handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI1Handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI1Handle.SPI_Config.SPI_SSM = SPI_SSM_EN;

	SPI_Init(&SPI1Handle);
}
