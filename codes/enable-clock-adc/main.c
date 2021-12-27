/**
  ******************************************************************************
  * @file    main.c
  * @author  Auto-generated by STM32CubeIDE
  * @version V1.0
  * @brief   Default main function.
  ******************************************************************************
*/

#include<stdint.h>

#define ADC_BASE_ADDR   			      	0x40012400UL // Page 61 of Reference Manual (Table 3. STM32L0x3 peripheral register boundary addresses)

#define ADC_CR1_REG_OFFSET 			    	0x08UL  // Page 340 of Reference Manual (14.12.3 ADC control register (ADC_CR))

#define ADC_CR1_REG_ADDR  			    	(ADC_BASE_ADDR + ADC_CR1_REG_OFFSET)

#define RCC_BASE_ADDR               		0x40021000UL // Page 60 of Reference Manual (Table 3. STM32L0x3 peripheral register boundary addresses)

#define RCC_APB2_ENR_OFFSET         		0x34UL // Page 207 of Reference Manual (APB2 peripheral clock enable register (RCC_APB2ENR))

#define RCC_APB2_ENR_ADDR           		(RCC_BASE_ADDR + RCC_APB2_ENR_OFFSET)

int main(void)
{
	uint32_t *pAdcCrReg =   (uint32_t*) ADC_CR1_REG_ADDR;
	uint32_t *pRccApb2Enr =  (uint32_t*) RCC_APB2_ENR_ADDR;

	//1.Enable the peripheral clock for ADC
	*pRccApb2Enr |= ( 1 << 9); // Page 207 of Reference Manual (ADC EN)

  	//2. modify the ADC_CR register
	*pAdcCrReg |= ( 1 << 28); // Page 340 of Reference Manual (ADVREGEN)

	for(;;);
}
