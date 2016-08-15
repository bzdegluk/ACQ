/**
 * @file     SDK_EVAL_Gpio.c
 * @author  High End Analog & RF BU - AMS / ART Team IMS-Systems Lab
 * @version V3.0.0
 * @date    August 7, 2012
 * @brief    This file provides all the low level API to manage SDK eval pin to drive GPIOs.
 * @details
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
 * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "SDK_EVAL_Gpio.h"
#include "SDK_EVAL_Config.h"

/** @addtogroup SDK_EVAL_STM32L
 * @{
 */


/** @addtogroup SDK_EVAL_Gpio
 * @{
 */


/** @defgroup SDK_EVAL_Gpio_Private_TypesDefinitions             SDK EVAL Gpio Private Types Definitions
 * @{
 */


/**
 * @}
 */




/** @defgroup SDK_EVAL_Gpio_Private_Defines                      SDK EVAL Gpio Private Defines
 * @{
 */


/**
 * @}
 */



/** @defgroup SDK_EVAL_Gpio_Private_Macros                       SDK EVAL Gpio Private Macros
 * @{
 */


/**
 * @}
 */



/** @defgroup SDK_EVAL_Gpio_Private_Variables                    SDK EVAL Gpio Private Variables
 * @{
 */

/**
 * @brief  M2S GPio Port array
 */

GPIO_TypeDef* vectpxM2SGpioPort[2][M2S_GPIO_NUMBER] = {
  {M2S_GPIO_0_PORT_V_23,M2S_GPIO_SDN_PORT},
  {M2S_GPIO_0_PORT_V_D1,M2S_GPIO_SDN_PORT},
  
};

/**
 * @brief  M2S GPio Pin array
 */
static const uint16_t s_vectnM2SGpioPin[M2S_GPIO_NUMBER] = {
        M2S_GPIO_0_PIN,
        M2S_GPIO_SDN_PIN
};

/**
 * @brief  M2S GPio Clock array
 */
static const uint32_t s_vectlM2SGpioClk[2][M2S_GPIO_NUMBER] = {
  {M2S_GPIO_0_RCC_PORT_V_23,M2S_GPIO_SDN_RCC_PORT},
  {M2S_GPIO_0_RCC_PORT_V_D1,M2S_GPIO_SDN_RCC_PORT},
};


/**
 * @brief  M2S GPio Speed array
 */
static const GPIOSpeed_TypeDef s_vectxM2SGpioSpeed[M2S_GPIO_NUMBER] = {
        M2S_GPIO_0_SPEED,
        M2S_GPIO_SDN_SPEED
};

/**
 * @brief  M2S GPio PuPd array
 */
static const GPIOPuPd_TypeDef s_vectxM2SGpioPuPd[M2S_GPIO_NUMBER] = {
        M2S_GPIO_0_PUPD,
        M2S_GPIO_SDN_PUPD
};

/**
 * @brief  M2S GPio Output Type array
 */
static const GPIOOType_TypeDef s_vectxM2SGpioOType[M2S_GPIO_NUMBER] = {
        M2S_GPIO_0_OTYPE,
        M2S_GPIO_SDN_OTYPE
};

/**
 * @brief  M2S Exti Line array
 */
static const uint32_t s_vectlM2SExtiLine[M2S_GPIO_NUMBER-1] = {
        M2S_GPIO_0_EXTI_LINE
};

/**
 * @brief  M2S Exti Port Source array
 */
static const uint8_t s_vectcM2SGpioExtiPortSource[2][M2S_GPIO_NUMBER-1] = {
  {M2S_GPIO_0_EXTI_PORT_SOURCE_V_23},
  {M2S_GPIO_0_EXTI_PORT_SOURCE_V_D1}
};

/**
 * @brief  M2S Exti Pin Source array
 */
static const uint8_t s_vectcM2SGpioExtiPinSource[M2S_GPIO_NUMBER-1] = {
        M2S_GPIO_0_EXTI_PIN_SOURCE
};

/**
 * @brief  M2S Exti Mode array
 */
static const EXTIMode_TypeDef s_vectxM2sGpioExtiMode[M2S_GPIO_NUMBER-1] = {
        M2S_GPIO_0_EXTI_MODE
};

/**
 * @brief  M2S Exti Trigger array
 */
static const EXTITrigger_TypeDef s_vectxM2SGpioExtiTrigger[M2S_GPIO_NUMBER-1] = {
        M2S_GPIO_0_EXTI_TRIGGER
};

/**
 * @brief  M2S Exti IRQn array
 */
static const uint8_t s_vectcM2SGpioExtiIrqn[M2S_GPIO_NUMBER-1] = {
        M2S_GPIO_0_EXTI_IRQN
};


/**
 * @}
 */



/** @defgroup SDK_EVAL_Gpio_Private_FunctionPrototypes                   SDK EVAL Gpio Private Function Prototypes
 * @{
 */


/**
 * @}
 */



/** @defgroup SDK_EVAL_Gpio_Private_Functions                            SDK EVAL Gpio Private Functions
 * @{
 */

/**
 * @brief  Configures MCU GPIO and EXTI Line for GPIOs.
 * @param  xGpio Specifies the GPIO to be configured.
 *         This parameter can be one of following parameters:
 *         @arg M2S_GPIO_0: GPIO_0
 *         @arg M2S_GPIO_SDN: GPIO_SDN
 * @param  xGpioMode Specifies GPIO mode.
 *         This parameter can be one of following parameters:
 *         @arg M2S_MODE_GPIO_IN: MCU GPIO will be used as simple input.
 *         @arg M2S_MODE_EXTI_IN: MCU GPIO will be connected to EXTI line with interrupt
 *         generation capability.
 *         @arg M2S_MODE_GPIO_OUT: MCU GPIO will be used as simple output.
 * @retval None.
 */
void SdkEvalM2SGpioInit(M2SGpioPin xGpio, M2SGpioMode xGpioMode)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  FunctionalState xExtiLine = DISABLE;
  
  /* Check the parameters */
  assert_param(IS_M2S_GPIO_PIN(xGpio));
  assert_param(IS_M2S_GPIO_MODE(xGpioMode));
  
  
  /* Enables the MCU GPIO Clock */
  RCC_AHBPeriphClockCmd(s_vectlM2SGpioClk[SdkEvalGetVersion()>>1][xGpio], ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  
  /* Configures MCU GPIO */
  if(xGpioMode == M2S_MODE_GPIO_OUT)
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  else
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  
  GPIO_InitStructure.GPIO_PuPd = s_vectxM2SGpioPuPd[xGpio];
  GPIO_InitStructure.GPIO_Speed = s_vectxM2SGpioSpeed[xGpio];
  GPIO_InitStructure.GPIO_OType = s_vectxM2SGpioOType[xGpio];
  GPIO_InitStructure.GPIO_Pin = s_vectnM2SGpioPin[xGpio];
  GPIO_Init(vectpxM2SGpioPort[SdkEvalGetVersion()>>1][xGpio], &GPIO_InitStructure);
  
  /* Configures MCU GPIO EXTI line */
  EXTI_InitStructure.EXTI_Line = s_vectlM2SExtiLine[xGpio];
  EXTI_InitStructure.EXTI_Mode = s_vectxM2sGpioExtiMode[xGpio];
  EXTI_InitStructure.EXTI_Trigger = s_vectxM2SGpioExtiTrigger[xGpio];
  EXTI_InitStructure.EXTI_LineCmd = xExtiLine;
  
  if (xGpioMode == M2S_MODE_EXTI_IN)
  {
    /* Connects EXTI Line to MCU GPIO Pin */
    SYSCFG_EXTILineConfig(s_vectcM2SGpioExtiPortSource[SdkEvalGetVersion()>>1][xGpio], s_vectcM2SGpioExtiPinSource[xGpio]);
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;

  }
  if(xGpio != M2S_GPIO_SDN) {
    EXTI_Init(&EXTI_InitStructure);
  }

}


/**
 * @brief  Enables or disables the interrupt on GPIO .
 * @param  xGpio Specifies the GPIO whose priority shall be changed.
 *         This parameter can be one of following parameters:
 *         @arg M2S_GPIO_0: GPIO_0
 * @param  nPreemption Specifies Preemption Priority.
 * @param  nSubpriority Specifies Subgroup Priority.
 * @param  xNewState Specifies the State.
 *         This parameter can be one of following parameters:
 *         @arg ENABLE: Interrupt is enabled
 *         @arg DISABLE: Interrupt is disabled
 * @retval None.
 */
void SdkEvalM2SGpioInterruptCmd(M2SGpioPin xGpio, uint8_t nPreemption, uint8_t nSubpriority, FunctionalState xNewState)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enables or Disables EXTI Interrupt to the specified priority */
  NVIC_InitStructure.NVIC_IRQChannel = s_vectcM2SGpioExtiIrqn[xGpio];
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = nPreemption;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = nSubpriority;
  NVIC_InitStructure.NVIC_IRQChannelCmd = xNewState;

  NVIC_Init(&NVIC_InitStructure);

}


/**
 * @brief  Returns the level of a specified GPIO.
 * @param  xGpio Specifies the GPIO to be read.
 *         This parameter can be one of following parameters:
 *         @arg M2S_GPIO_0: GPIO_0
 * @retval FlagStatus Level of the GPIO. This parameter can be:
 *         SET or RESET.
 */
FlagStatus SdkEvalGpioGetLevel(M2SGpioPin xGpio)
{
  /* Gets the GPIO level */
  uint16_t nDataPort = GPIO_ReadInputData(vectpxM2SGpioPort[SdkEvalGetVersion()>>1][xGpio]);
  if(nDataPort & s_vectnM2SGpioPin[xGpio])
    return SET;
  else
    return RESET;
  
}

/**
 * @brief  Sets the level of a specified GPIO.
 * @param  xGpio Specifies the GPIO to be set.
 *         This parameter can be one of following parameters:
 *         @arg M2S_GPIO_0: GPIO_0
 * @param  FlagStatus Level of the GPIO. This parameter can be:
 *         SET or RESET.
 * @retval None.
 */
void SdkEvalGpioSetLevel(M2SGpioPin xGpio, FlagStatus xLevel)
{
  /* Sets the GPIO level */
  GPIO_WriteBit(vectpxM2SGpioPort[SdkEvalGetVersion()>>1][xGpio], s_vectnM2SGpioPin[xGpio], (BitAction)xLevel);

  
}

/**
 * @brief  Puts at logic 1 the SDN pin.
 * @param  None.
 * @retval None.
 */
void SdkEvalEnterShutdown(void)
{
  /* Puts high the GPIO connected to shutdown pin */
  GPIO_SetBits(M2S_GPIO_SDN_PORT, M2S_GPIO_SDN_PIN);
}


/**
 * @brief  Put at logic 0 the SDN pin.
 * @param  None.
 * @retval None.
 */
void SdkEvalExitShutdown(void)
{
  /* Puts low the GPIO connected to shutdown pin */
  GPIO_ResetBits(M2S_GPIO_SDN_PORT, M2S_GPIO_SDN_PIN);

  /* Delay to allow the circuit POR, about 700 us */
  for(volatile uint32_t i=0;i<0x1E00;i++);
}

FlagStatus SdkEvalCheckShutdown(void)
{
  return SdkEvalGpioGetLevel(M2S_GPIO_SDN);
}

/**
 * @}
 */



/**
 * @}
 */



/**
 * @}
 */



/******************* (C) COPYRIGHT 2012 STMicroelectronics *****END OF FILE****/
