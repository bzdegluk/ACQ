/**
* @file    SDK_Common.h
* @author  DiZiC Ltd.
* @version V3.0.0
* @date    August 10, 2013
* @brief   System Initialization and Configuration
* @details
*
* Macro: SDK_SYSTEM_CONFIG() moved to main as sequence of singular calls
*
* Original:
*
#define SDK_SYSTEM_CONFIG()           { NVIC_SetVectorTable( NVIC_VectTab_FLASH, 0x3000 ); \
                                        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC | RCC_AHBPeriph_GPIOD | RCC_AHBPeriph_GPIOE, ENABLE ); \
                                        SdkEvalIdentification(); \
                                        SdkStartSysTick(); \
                                        SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK); \
                                        SdkEvalPmRfSwitchInit(); \
                                        SdkEvalPmRfSwitchToVRf(); \
                                        SdkEvalPmI2CInit(); \
                                        SdkEvalPmADCInit(); \
                                        SdkEvalPmRegulateVRfI(3.3); \
                                        SdkEvalLedInit(LED1); \
                                        SdkEvalLedInit(LED2); \
                                        SdkEvalLedInit(LED3); \
                                        SdkEvalLedInit(LED4); \
                                        SdkEvalLedInit(LED5); \
                                        SdkEvalM2SGpioInit(M2S_GPIO_SDN,M2S_MODE_GPIO_OUT); \
                                        SpiritSpiInit(); \
                                        EepromSpiInitialization(); \
                                      }
*/

#define SDK_SYSTEM_CONFIG()           { NVIC_SetVectorTable( NVIC_VectTab_FLASH, 0x3000 ); \
                                        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC | RCC_AHBPeriph_GPIOD | RCC_AHBPeriph_GPIOE, ENABLE ); \
                                        SdkEvalIdentification(); \
                                        SdkStartSysTick(); \
                                        SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK); \
                                        SdkEvalLedInit(LED1); \
                                        SdkEvalLedInit(LED2); \
                                        SdkEvalM2SGpioInit(M2S_GPIO_SDN,M2S_MODE_GPIO_OUT); \
                                        SpiritSpiInit(); \
                                      }

