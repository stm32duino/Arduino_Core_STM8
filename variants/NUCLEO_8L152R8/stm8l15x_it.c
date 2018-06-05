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
#include "board.h"

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
/**
  * @brief Dummy interrupt routine
  * @par Parameters:
  * None
  * @retval
  * None
*/
INTERRUPT_HANDLER(NonHandledInterrupt,0)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}
#endif

/**
  * @brief TRAP interrupt routine
  * @par Parameters:
  * None
  * @retval
  * None
*/
INTERRUPT_HANDLER_TRAP(TRAP_IRQHandler)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}
/**
  * @brief FLASH Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(FLASH_IRQHandler,1)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}
/**
  * @brief DMA1 channel0 and channel1 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(DMA1_CHANNEL0_1_IRQHandler,2)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}
/**
  * @brief DMA1 channel2 and channel3 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(DMA1_CHANNEL2_3_IRQHandler,3)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}
/**
  * @brief RTC / CSS_LSE Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(RTC_CSSLSE_IRQHandler,4)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}
/**
  * @brief External IT PORTE/F and PVD Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTIE_F_PVD_IRQHandler,5)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}

/**
  * @brief External IT PORTB / PORTG Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTIB_G_IRQHandler,6)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}

/**
  * @brief External IT PORTD /PORTH Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTID_H_IRQHandler,7)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}

/**
  * @brief External IT PIN0 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI0_IRQHandler,8)
{
  GPIO_EXTI_Callback(EXTI_IT_Pin0);

}

/**
  * @brief External IT PIN1 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI1_IRQHandler,9)
{
  GPIO_EXTI_Callback(EXTI_IT_Pin1);

}

/**
  * @brief External IT PIN2 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI2_IRQHandler,10)
{
  GPIO_EXTI_Callback(EXTI_IT_Pin2);

}

/**
  * @brief External IT PIN3 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI3_IRQHandler,11)
{
  GPIO_EXTI_Callback(EXTI_IT_Pin3);

}

/**
  * @brief External IT PIN4 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI4_IRQHandler,12)
{
  GPIO_EXTI_Callback(EXTI_IT_Pin4);
}

/**
  * @brief External IT PIN5 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI5_IRQHandler,13)
{
  GPIO_EXTI_Callback(EXTI_IT_Pin5);

}

/**
  * @brief External IT PIN6 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI6_IRQHandler,14)
{
  GPIO_EXTI_Callback(EXTI_IT_Pin6);

}

/**
  * @brief External IT PIN7 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI7_IRQHandler,15)
{
  GPIO_EXTI_Callback(EXTI_IT_Pin7);

}
/**
  * @brief LCD /AES Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(LCD_AES_IRQHandler,16)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}
/**
  * @brief CLK switch/CSS/TIM1 break Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(SWITCH_CSS_BREAK_DAC_IRQHandler,17)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}

/**
  * @brief ADC1/Comparator Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(ADC1_COMP_IRQHandler,18)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}

/**
  * @brief TIM2 Update/Overflow/Trigger/Break /USART2 TX Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(TIM2_UPD_OVF_TRG_BRK_USART2_TX_IRQHandler,19)
{
  if(TIM2_GetITStatus(TIM2_IT_Update) == SET) {
    TIM_PeriodElapsedCallback(TIM2_E);
    TIM2_ClearITPendingBit(TIM2_IT_Update);
  }
}

/**
  * @brief Timer2 Capture/Compare / USART2 RX Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(TIM2_CC_USART2_RX_IRQHandler,20)
{
  if(TIM2_GetITStatus(TIM2_IT_CC1) == SET) {
    TIM_OC_DelayElapsedCallback(TIM2_E);
    TIM2_ClearITPendingBit(TIM2_IT_CC1);
  }
#if !defined(NO_HWSERIAL)
  if(USART_GetITStatus(USART2, USART_IT_RXNE) == SET) {
    HAL_UART_RxCpltCallback(USART2);
  }
#endif /* ! NO_HWSERIAL */
}


/**
  * @brief Timer3 Update/Overflow/Trigger/Break Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(TIM3_UPD_OVF_TRG_BRK_USART3_TX_IRQHandler,21)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}
/**
  * @brief Timer3 Capture/Compare /USART3 RX Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(TIM3_CC_USART3_RX_IRQHandler,22)
{
#if !defined(NO_HWSERIAL)
  if(USART_GetITStatus(USART3, USART_IT_RXNE) == SET) {
    HAL_UART_RxCpltCallback(USART3);
  }
#endif /* ! NO_HWSERIAL */
}
/**
  * @brief TIM1 Update/Overflow/Trigger/Commutation Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(TIM1_UPD_OVF_TRG_COM_IRQHandler,23)
{
  if(TIM1_GetITStatus(TIM1_IT_Update) == SET) {
    TIM_PeriodElapsedCallback(TIM1_E);
    TIM1_ClearITPendingBit(TIM1_IT_Update);
  }
}
/**
  * @brief TIM1 Capture/Compare Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(TIM1_CC_IRQHandler,24)
{
  if(TIM1_GetITStatus(TIM1_IT_CC1) == SET) {
    TIM_OC_DelayElapsedCallback(TIM1_E);
    TIM1_ClearITPendingBit(TIM1_IT_CC1);
  }
}

/**
  * @brief TIM4 Update/Overflow/Trigger Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(TIM4_UPD_OVF_TRG_IRQHandler,25)
{
  if(TIM4_GetITStatus(TIM4_IT_Update) == SET) {
    TIM_PeriodElapsedCallback(TIM4_E);
    TIM4_ClearITPendingBit(TIM4_IT_Update);
  }
}
/**
  * @brief SPI1 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(SPI1_IRQHandler,26)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}

/**
  * @brief USART1 TX / TIM5 Update/Overflow/Trigger/Break Interrupt  routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(USART1_TX_TIM5_UPD_OVF_TRG_BRK_IRQHandler,27)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}

/**
  * @brief USART1 RX / Timer5 Capture/Compare Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(USART1_RX_TIM5_CC_IRQHandler,28)
{
#if !defined(NO_HWSERIAL)
  if(USART_GetITStatus(USART1, USART_IT_RXNE) == SET) {
    HAL_UART_RxCpltCallback(USART1);
  }
#endif /* ! NO_HWSERIAL */
}




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
