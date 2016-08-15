/**
 * @file    SDK_EVAL_Config.c
 * @author  High End Analog & RF BU - AMS / ART Team IMS-Systems Lab
 * @version V3.0.0
 * @date    August 7, 2012
 * @brief   This file provides all the low level API to manage SDK Version identification.
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
#include "SDK_EVAL_Config.h"

/** @addtogroup SDK_EVAL_STM32L
 * @{
 */


/** @addtogroup SDK_EVAL_Config
 * @{
 */

/** @defgroup SDK_EVAL_Config_Private_TypesDefinitions             SDK EVAL Config Private Types Definitions
 * @{
 */

/**
 * @}
 */


/** @defgroup SDK_EVAL_Config_Private_Defines                      SDK EVAL Config Private Defines
 * @{
 */

/**
 * @}
 */


/** @defgroup SDK_EVAL_Config_Private_Macros                       SDK EVAL Config Private Macros
 * @{
 */

/**
 * @}
 */


/** @defgroup SDK_EVAL_Config_Private_Variables                    SDK EVAL Config Private Variables
 * @{
 */
/**
 * @}
 */


/**
 * @defgroup SDK_EVAL_Config_Private_FunctionPrototypes                    SDK EVAL Config Private Function Prototypes
 * @{
 */

/**
 * @}
 */


/**
 * @defgroup SDK_EVAL_Config_Private_Functions                             SDK EVAL Config Private Functions
 * @{
 */
   
                                      
#define SdkEvalPb6Init                 { RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE); \
                                                    GPIO_Init(GPIOB, &(GPIO_InitTypeDef){GPIO_Pin_6, GPIO_Mode_IN, GPIO_Speed_40MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL}); \
                                                  }

#define SdkEvalGetPb6           {(GPIO_ReadInputData(GPIOB) & GPIO_Pin_6)}

static uint8_t s_SdkEvalVersion = SDK_EVAL_VERSION_3;


/**
 * @brief  Identifies the current motherboard.
 * @param  None.
 * @retval None.
 */
void SdkEvalIdentification(void)
{
  GPIO_InitTypeDef GPIO_InitStructure; 
  
  /* Enables the MCU GPIO Clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE, ENABLE);  
  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  
  GPIOE->ODR |= GPIO_Pin_1;
  
  for(volatile uint8_t i=0;i<1;i++);
  
  if((GPIOE->IDR) & GPIO_Pin_1) {
    s_SdkEvalVersion = SDK_EVAL_VERSION_2_1;
  } else 
  {
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    GPIOD->ODR |= GPIO_Pin_2;
    
    for(volatile uint8_t i=0;i<1;i++);
    
    if((GPIOD->IDR) & GPIO_Pin_2) {
      s_SdkEvalVersion = SDK_EVAL_VERSION_3;
    }
    else {
      s_SdkEvalVersion = SDK_EVAL_VERSION_D1;
    }
    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    GPIOD->ODR &= ~GPIO_Pin_2;
  }
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  
  GPIOE->ODR &= ~GPIO_Pin_1;
}


/**
 * @brief  Returns the version of the current motherboard.
 * @param  None.
 * @retval None.
 */
uint8_t SdkEvalGetVersion(void)
{
 return s_SdkEvalVersion;
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
