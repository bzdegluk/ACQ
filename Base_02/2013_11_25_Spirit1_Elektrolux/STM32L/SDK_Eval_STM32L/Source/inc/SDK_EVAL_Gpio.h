/**
 * @file    SDK_EVAL_Gpio.h
 * @author  DiZiC Ltd.
 * @version V3.0.0
 * @date    August 10, 2013
 * @brief   GPIO Configuration used in the Software Development Kit eval board to drive GPIOs.
 * @details
 *
 * This module exports API to manage the GPIO from the micro
 * side.
 * The following example shows how to configure the shutdown pin
 * and the GPIO 3 as an EXTI input.
 *
 * <b>Example:</b>
 * @code
 *
 *   ...
 *
 *   SdkEvalM2SGpioInit(M2S_GPIO_SDN,M2S_MODE_GPIO_OUT);
 *
 *   SdkEvalM2SGpioInit(M2S_GPIO_0,M2S_MODE_EXTI_IN);
 *   SdkEvalM2SGpioInterruptCmd(M2S_GPIO_0,0x0F,0x0F,ENABLE);
 *
 *   ...
 *
 * @endcode
 *
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * THIS SOURCE CODE IS PROTECTED BY A LICENSE.
 * FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED
 * IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
 *
 * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
 */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SDK_EVAL_GPIO_H
#define __SDK_EVAL_GPIO_H


  /* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"


#ifdef __cplusplus
extern "C" {
#endif


/** @addtogroup SDK_EVAL_STM32L
 * @{
 */


/** @addtogroup SDK_EVAL_Gpio            SDK EVAL Gpio
 * @brief GPIO Configuration used in the Development Kit eval board to drive GPIOs.
 * @details See the file <i>@ref SDK_EVAL_Gpio.h</i> for more details.
 * @{
 */


/** @defgroup SDK_EVAL_Gpio_Exported_Types               SDK EVAL Gpio Exported Types
 * @{
 */

/**
 * @brief  Number of MCU GPIO pins used for GPIO.
 */

#define M2S_GPIO_NUMBER    2


/**
 * @brief  MCU GPIO pin enumeration for GPIO
 */
typedef enum
{
  M2S_GPIO_0     = 0x00, /*!< GPIO_0 selected */
  M2S_GPIO_SDN   = 0x01  /*!< GPIO_SDN selected */

}M2SGpioPin;

#define IS_M2S_GPIO_PIN(PIN) (((PIN) == M2S_GPIO_0) || \
                              ((PIN) == M2S_GPIO_SDN))

/**
 * @brief  MCU GPIO pin working mode for GPIO
 */
typedef enum
{
  M2S_MODE_GPIO_IN  = 0x00,   /*!< Work as GPIO input */
  M2S_MODE_EXTI_IN,           /*!< Work as EXTI */
  M2S_MODE_GPIO_OUT,          /*!< Work as GPIO output */


}M2SGpioMode;

#define IS_M2S_GPIO_MODE(MODE) (((MODE) == M2S_MODE_GPIO_IN) || \
                                ((MODE) == M2S_MODE_EXTI_IN) || \
                                ((MODE) == M2S_MODE_GPIO_OUT))


/**
 * @}
 */


/** @defgroup SDK_EVAL_Gpio_Exported_Constants                   SDK EVAL Gpio Exported Constants
 * @{
 */

/** @defgroup M2S_Gpio_0_Define
 * @{
 */

#define M2S_GPIO_0_PORT_V_23                        GPIOB
#define M2S_GPIO_0_PORT_V_D1                        GPIOB

#define M2S_GPIO_0_PIN                         GPIO_Pin_5
#define M2S_GPIO_0_RCC_PORT_V_23                    RCC_AHBPeriph_GPIOB
#define M2S_GPIO_0_RCC_PORT_V_D1                    RCC_AHBPeriph_GPIOB

#define M2S_GPIO_0_SPEED                       GPIO_Speed_40MHz
#define M2S_GPIO_0_PUPD                        GPIO_PuPd_NOPULL
#define M2S_GPIO_0_OTYPE                       GPIO_OType_PP
#define M2S_GPIO_0_EXTI_PORT_SOURCE_V_23            EXTI_PortSourceGPIOB
#define M2S_GPIO_0_EXTI_PORT_SOURCE_V_D1            EXTI_PortSourceGPIOB

#define M2S_GPIO_0_EXTI_PIN_SOURCE             EXTI_PinSource5
#define M2S_GPIO_0_EXTI_LINE                   EXTI_Line5
#define M2S_GPIO_0_EXTI_MODE                   EXTI_Mode_Interrupt
#define M2S_GPIO_0_EXTI_TRIGGER                EXTI_Trigger_Falling
#define M2S_GPIO_0_EXTI_IRQN                   EXTI9_5_IRQn
#define M2S_GPIO_0_EXTI_PREEMPTION_PRIORITY    2
#define M2S_GPIO_0_EXTI_SUB_PRIORITY           2
#define M2S_GPIO_0_EXTI_IRQ_HANDLER            EXTI9_5_IRQHandler

/**
 * @}
 */




/** @defgroup M2S_Gpio_SDN_Define
 * @{
 */

#define M2S_GPIO_SDN_PORT                      GPIOC
#define M2S_GPIO_SDN_PIN                       GPIO_Pin_13
#define M2S_GPIO_SDN_RCC_PORT                  RCC_AHBPeriph_GPIOC
#define M2S_GPIO_SDN_SPEED                     GPIO_Speed_40MHz
#define M2S_GPIO_SDN_PUPD                      GPIO_PuPd_UP
#define M2S_GPIO_SDN_OTYPE                     GPIO_OType_PP

/**
 * @}
 */


/**
 * @}
 */


/** @defgroup SDK_EVAL_Gpio_Exported_Macros                      SDK EVAL Gpio Exported Macros
 * @{
 */


/**
 * @}
 */


/** @defgroup SDK_EVAL_Gpio_Exported_Functions                   SDK EVAL Gpio Exported Functions
 * @{
 */

void SdkEvalM2SGpioInit(M2SGpioPin xGpio, M2SGpioMode xGpioMode);
void SdkEvalM2SGpioInterruptCmd(M2SGpioPin xGpio, uint8_t nPreemption, uint8_t nSubpriority, FunctionalState xNewState);
FlagStatus SdkEvalGpioGetLevel(M2SGpioPin xGpio);
void SdkEvalGpioSetLevel(M2SGpioPin xGpio, FlagStatus xLevel);


/**
 * @}
 */


/**
 * @}
 */


/**
 * @}
 */


#ifdef __cplusplus
}
#endif

#endif

/******************* (C) COPYRIGHT 2013 STMicroelectronics *****END OF FILE****/