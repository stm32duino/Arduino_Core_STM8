/**
  ******************************************************************************
  * @file    Project/STM8L15x_StdPeriph_Template/stm8l15x_it.c
  * @author  MCD Application Team
  * @version V1.6.1
  * @date    30-September-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm8_it.h"

#ifdef __cplusplus
 extern "C" {
#endif


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/

#ifdef _COSMIC_
extern void _stext();         /* startup routine */

#pragma section const {vector}

void (* const @vector _vectab[32])() = {
	_stext,					/* RESET       */
	TRAP_IRQHandler,			/* TRAP        */
  NonHandledInterrupt,		/* Reserved */
  FLASH_IRQHandler,   /*FLASH interrupt */
  DMA1_CHANNEL0_1_IRQHandler,  /*DMA1 channel0 and channel1 interrupt */
  DMA1_CHANNEL2_3_IRQHandler,  /*DMA1 channel2 and channel3 interrupt */
  RTC_CSSLSE_IRQHandler,			/* irq4 - RTC/ CSS on LSE interrupt */
  EXTIE_F_PVD_IRQHandler,    /* irq5 - External IT PORTE/F interrupt /PVD interrupt*/
  EXTIB_G_IRQHandler,      /* irq6 - External IT PORTB / PORTG interrupt */
  EXTID_H_IRQHandler,			/* irq7 - External IT PORTD / PORTH interrupt */
  EXTI0_IRQHandler,			/* irq8 - External IT PIN0 interrupt */
  EXTI1_IRQHandler,			/* irq9 - External IT PIN1 interrupt */
  EXTI2_IRQHandler,			/* irq10 - External IT PIN2 interrupt */
  EXTI3_IRQHandler,		  /* irq11 - External IT PIN3 interrupt */
  EXTI4_IRQHandler,      /* irq12 - External IT PIN4 interrupt */
  EXTI5_IRQHandler,      /* irq13 - External IT PIN5 interrupt */
  EXTI6_IRQHandler,      /* irq14 - External IT PIN6 interrupt */
  EXTI7_IRQHandler,      /* irq15 - External IT PIN7 interrupt */
  LCD_AES_IRQHandler,      /* irq16 - LCD / AES interrupt */
  SWITCH_CSS_BREAK_DAC_IRQHandler,      /* irq17 - CLK switch/CSS interrupt/ TIM1 Break interrupt / DAC */
  ADC1_COMP_IRQHandler,      /* irq18 - ADC1 and Comparator interrupt */
  TIM2_UPD_OVF_TRG_BRK_USART2_TX_IRQHandler,      /* irq19 - TIM2 Update/Overflow/Trigger/Break / USART2 TX interrupt */
  TIM2_CC_USART2_RX_IRQHandler,      /* irq20 - TIM2 Capture/Compare / USART2 RX interrupt */
  TIM3_UPD_OVF_TRG_BRK_USART3_TX_IRQHandler,      /* irq21 - TIM3 Update/Overflow/Trigger/Break / USART3 TX interrupt */
  TIM3_CC_USART3_RX_IRQHandler,			/* irq22 - TIM3 Capture/Compare /USART3 RX interrupt */
  TIM1_UPD_OVF_TRG_COM_IRQHandler,      /* irq23 - TIM1 Update/Overflow/Trigger/Commutation interrupt */
  TIM1_CC_IRQHandler,      /* irq24 - TIM1 Capture/Compare interrupt */
  TIM4_UPD_OVF_TRG_IRQHandler,     /* irq25 - TIM4 Update/Overflow/Trigger interrupt */
  SPI1_IRQHandler,      /* irq26 - SPI1 interrupt */
  USART1_TX_TIM5_UPD_OVF_TRG_BRK_IRQHandler,      /* irq27 - USART1 TX / TIM5 Update/Overflow/Trigger/Break interrupt */
  USART1_RX_TIM5_CC_IRQHandler,      /* irq28 - USART1 RX / TIM1 Capture/Compare interrupt */
  I2C1_SPI2_IRQHandler,
  };
#endif
/**
  * @}
  */
#ifdef __cplusplus
}
#endif


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
