/**
 * @file    SDK_EVAL_Led.c
 * @author  DiZiC Ltd.
 * @version V3.0.0
 * @date    August 10, 2013
 * @brief   This file provides all the low level API to manage SDK LEDs.
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
 */


/* Includes ------------------------------------------------------------------*/
#include "SDK_EVAL_Led.h"
#include "SDK_EVAL_Config.h"

/** @addtogroup SDK_EVAL_STM32L
 * @{
 */


/** @addtogroup SDK_EVAL_Led
 * @{
 */

/** @defgroup SDK_EVAL_Led_Private_TypesDefinitions             SDK EVAL Led Private Types Definitions
 * @{
 */

/**
 * @}
 */


/** @defgroup SDK_EVAL_Led_Private_Defines                      SDK EVAL Led Private Defines
 * @{
 */

/**
 * @}
 */


/** @defgroup SDK_EVAL_Led_Private_Macros                       SDK EVAL Led Private Macros
 * @{
 */

/**
 * @}
 */


/** @defgroup SDK_EVAL_Led_Private_Variables                    SDK EVAL Led Private Variables
 * @{
 */
GPIO_TypeDef* vectpxGpioPortVersion[3][LEDn] = {
  {SDK_EVAL_V2_LED1_GPIO_PORT, SDK_EVAL_V2_LED2_GPIO_PORT},
  {SDK_EVAL_V3_LED1_GPIO_PORT, SDK_EVAL_V3_LED2_GPIO_PORT},
  {SDK_DONGLE_V1_LED1_GPIO_PORT, SDK_DONGLE_V1_LED2_GPIO_PORT}
 };
 
static const uint16_t s_vectnGpioPinVersion[3][LEDn] = {
  {SDK_EVAL_V2_LED1_PIN, SDK_EVAL_V2_LED2_PIN},
  {SDK_EVAL_V3_LED1_PIN, SDK_EVAL_V3_LED2_PIN},
  {SDK_DONGLE_V1_LED1_PIN, SDK_DONGLE_V1_LED2_PIN}
};

static const uint32_t s_vectlGpioClkVersion[3][LEDn] = {
  {SDK_EVAL_V2_LED1_GPIO_CLK, SDK_EVAL_V2_LED2_GPIO_CLK},
  {SDK_EVAL_V3_LED1_GPIO_CLK, SDK_EVAL_V3_LED2_GPIO_CLK},
  {SDK_DONGLE_V1_LED1_GPIO_CLK, SDK_DONGLE_V1_LED2_GPIO_CLK}
};


static GPIO_TypeDef** vectpxGpioPort;
static uint16_t* s_vectnGpioPin;
static uint32_t* s_vectlGpioClk;

/**
 * @}
 */


/**
 * @defgroup SDK_EVAL_Led_Private_FunctionPrototypes                    SDK EVAL Led Private Function Prototypes
 * @{
 */

/**
 * @}
 */


/**
 * @defgroup SDK_EVAL_Led_Private_Functions                             SDK EVAL Led Private Functions
 * @{
 */


/**
 * @brief  Configures LED GPIO.
 * @param  xLed Specifies the Led to be configured.
 *         This parameter can be one of following parameters:
 *         @arg LED1
 *         @arg LED2
 * @retval None.
 */
void SdkEvalLedInit(SdkEvalLed xLed)
{
  vectpxGpioPort = vectpxGpioPortVersion[SdkEvalGetVersion()];
  s_vectnGpioPin = (uint16_t *)s_vectnGpioPinVersion[SdkEvalGetVersion()];
  s_vectlGpioClk = (uint32_t *)s_vectlGpioClkVersion[SdkEvalGetVersion()];

  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /* Enable the GPIO_LED Clock */
  RCC_AHBPeriphClockCmd(s_vectlGpioClk[xLed], ENABLE);
  
  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = s_vectnGpioPin[xLed];
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_Init(vectpxGpioPort[xLed], &GPIO_InitStructure);
  if(SdkEvalGetVersion() == SDK_EVAL_VERSION_2_1 || SdkEvalGetVersion() == SDK_EVAL_VERSION_3 )
    vectpxGpioPort[xLed]->BSRRL = s_vectnGpioPin[xLed];
  if(SdkEvalGetVersion() == SDK_EVAL_VERSION_D1)
   vectpxGpioPort[xLed]->BSRRL = s_vectnGpioPin[xLed]; //*** Set pin High
}

void SdkEvalLedInitPWM(SdkEvalLed xLed)
{
  vectpxGpioPort = vectpxGpioPortVersion[SdkEvalGetVersion()];
  s_vectnGpioPin = (uint16_t *)s_vectnGpioPinVersion[SdkEvalGetVersion()];
  s_vectlGpioClk = (uint32_t *)s_vectlGpioClkVersion[SdkEvalGetVersion()];

  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /* Enable the GPIO_LED Clock */
  RCC_AHBPeriphClockCmd(s_vectlGpioClk[xLed], ENABLE);
  
  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = s_vectnGpioPin[xLed];
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
   GPIO_Init(vectpxGpioPort[xLed], &GPIO_InitStructure);
  
  GPIO_PinAFConfig(vectpxGpioPort[xLed] ,GPIO_PinSource11 ,GPIO_AF_TIM2);
  if(SdkEvalGetVersion() == SDK_EVAL_VERSION_2_1 || SdkEvalGetVersion() == SDK_EVAL_VERSION_3 )
    vectpxGpioPort[xLed]->BSRRL = s_vectnGpioPin[xLed];
  if(SdkEvalGetVersion() == SDK_EVAL_VERSION_D1)
   vectpxGpioPort[xLed]->BSRRL = s_vectnGpioPin[xLed]; //*** Set pin High
}

/**
 * @brief  Turns selected LED On.
 * @param  xLed Specifies the Led to be set on.
 *         This parameter can be one of following parameters:
 *         @arg LED1
 *         @arg LED2
 * @retval None.
 *
 * BSRR stands for bit set/reset register
 * It is seperated into a high and a low word (each of 16 bit size)
 * A logical 1 in BSRRL will set the pin and a logical 1 in BSRRH will
 * reset the pin. A logical 0 in either register has no effect
 *
 */
void SdkEvalLedOn(SdkEvalLed xLed)
{
  if(SdkEvalGetVersion() == SDK_EVAL_VERSION_2_1 || SdkEvalGetVersion() == SDK_EVAL_VERSION_3)
    vectpxGpioPort[xLed]->BSRRH = s_vectnGpioPin[xLed];
  if(SdkEvalGetVersion() == SDK_EVAL_VERSION_D1)
    vectpxGpioPort[xLed]->BSRRH = s_vectnGpioPin[xLed];      //*** Reset pin
}

/**
 * @brief  Turns selected LED Off.
 * @param  xLed Specifies the Led to be set off.
 *         This parameter can be one of following parameters:
 *         @arg LED1
 *         @arg LED2
 * @retval None.
 */
void SdkEvalLedOff(SdkEvalLed xLed)
{
  if(SdkEvalGetVersion() == SDK_EVAL_VERSION_2_1  || SdkEvalGetVersion() == SDK_EVAL_VERSION_3)
      vectpxGpioPort[xLed]->BSRRL = s_vectnGpioPin[xLed];
  if(SdkEvalGetVersion() == SDK_EVAL_VERSION_D1)
        vectpxGpioPort[xLed]->BSRRL = s_vectnGpioPin[xLed];  //*** Set pin High
}

/**
 * @brief  Toggles the selected LED.
 * @param  xLed Specifies the Led to be toggled.
 *         This parameter can be one of following parameters:
 *         @arg LED1
 *         @arg LED2
 * @retval None.
 */
void SdkEvalLedToggle(SdkEvalLed xLed)
{
  vectpxGpioPort[xLed]->ODR ^= s_vectnGpioPin[xLed];
}

/**
 * @brief  Returns the status of a specified led.
 * @param  xLed Specifies the Led to be read.
 *         This parameter can be one of following parameters:
 *         @arg LED1
 *         @arg LED2
 * @retval FlagStatus return the status of the LED. This parameter can be:
 *         SET or RESET.
 */
FlagStatus SdkEvalLedGetState(SdkEvalLed xLed)
{
  if(vectpxGpioPort[xLed]->IDR & s_vectnGpioPin[xLed])
    return RESET;
  else
    return SET;
}


void LED_Flash(SdkEvalLed xLed)
{
  uint8_t times=3;
  
  while(times)
  {
    SdkEvalLedOn(xLed);
    SdkDelayMs(300);
    SdkEvalLedOff(xLed);
    SdkDelayMs(300);
    times--;
  }
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
