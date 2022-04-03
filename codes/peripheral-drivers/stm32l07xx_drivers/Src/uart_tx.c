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
#include "stm32l07xx_usart_driver.h"

char msg[1024] = "UART Tx testing...\n\r";

USART_Handle_t usart1_handle;

/*
 * PC0	USART 1 RX
 * PC1 	USART 1 TX
 * */

void USART1_GPIOInit(void);
void USART1_Init(void);
void GPIO_ButtonInit(void);
void delay(void);

int main(void) {
	GPIO_ButtonInit();

	USART1_GPIOInit();

	USART1_Init();

	USART_PeripheralControl(USART1, ENABLE);

	while (1) {
		//wait till button is pressed
		while (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_13))
			;

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		USART_SendData(&usart1_handle, (uint8_t*) msg, strlen(msg));

	}

	return 0;
}

void USART1_GPIOInit(void) {
	GPIO_Handle_t usart_gpios;

	usart_gpios.pGPIOx = GPIOA;
	usart_gpios.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTERNATE;
	usart_gpios.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	usart_gpios.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
	usart_gpios.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	usart_gpios.GPIO_PinConfig.GPIO_PinAltFunMode = 4;

	//USART1 TX
	usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_9;
	GPIO_Init(&usart_gpios);

	//USART1 RX
	usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_10;
	GPIO_Init(&usart_gpios);
}

void USART1_Init(void) {
	usart1_handle.pUSARTx = USART1;
	usart1_handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	usart1_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	usart1_handle.USART_Config.USART_DeviceMode = USART_MODE_ONLY_TX;
	usart1_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	usart1_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	usart1_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART_Init(&usart1_handle);
}

void GPIO_ButtonInit(void) {
	GPIO_Handle_t GPIO_Led, GPIO_Button;
	memset(&GPIO_Led, 0x00, sizeof(GPIO_Led));
	memset(&GPIO_Button, 0x00, sizeof(GPIO_Button));

	GPIO_Led.pGPIOx = GPIOA;
	GPIO_Led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	GPIO_Led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	GPIO_Led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIO_Led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIO_Led.pGPIOx, ENABLE);

	GPIO_Init(&GPIO_Led);

	GPIO_Button.pGPIOx = GPIOC;
	GPIO_Button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GPIO_Button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU; /* Because there is a pull up resistor on the board */

	GPIO_PeriClockControl(GPIO_Button.pGPIOx, ENABLE);

	GPIO_Init(&GPIO_Button);
}

void delay(void)
{
	for (int i = 0; i < 50000; i++)
			;
}
