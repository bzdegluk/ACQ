/**
* @file    SPIRIT_SDK_Application.c
* @author  DiZiC Ltd.
* @version V3.0.1
* @date    August 10, 2013
* @brief   Identification functions for SPIRIT DK.
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
#include "SPIRIT_SDK_Application.h"
#include "stm32l1xx.h"


/**
* @addtogroup SPIRIT_DK                   SPIRIT DK
* @{
*/


/**
* @defgroup SDK_SPIRIT_MANAGEMENT              SDK SPIRIT Management
* @{
*/


/**
* @brief This flag is used to synchronize the TIM3 ISR with the XtalMeasurement routine.
*/
static volatile FlagStatus s_xTIMChCompareModeRaised = RESET;

/**
* @brief This flag is used to synchronize the TIM3 ISR with the XtalMeasurement routine.
*/
static uint8_t s_RfModuleBand = 0;


#define ENABLE_TCXO()           GPIO_SetBits(GPIOC,GPIO_Pin_2);


/**
* @brief A map that contains the SPIRIT version
*/
const SpiritVersionMap xSpiritVersionMap[] =
{
  /* The Control Board frame handler functions */
  {CUT_2_1v4, SPIRIT_VERSION_2_1},
  {CUT_2_1v3, SPIRIT_VERSION_2_1},
  {CUT_3_0, SPIRIT_VERSION_3_0},
};


/**
* @defgroup SDK_SPIRIT_MANAGEMENT_FUNCTIONS    SDK SPIRIT Management Functions
* @{
*/

/**
* @defgroup IDENTIFICATION_FUNCTIONS      SDK SPIRIT Management Identification Functions
* @{
*/


/**
* @brief This function handles TIM3 global interrupt
* @param None.
* @retval None.
*/
void TIM3_IRQHandler(void)
{
  if(TIM_GetITStatus(TIM3, TIM_IT_CC4))
  {
    /* Set the TIM3 Compare IRQ flag */
    s_xTIMChCompareModeRaised = SET;
    
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);
    
  }
  else if(TIM_GetITStatus(TIM3, TIM_IT_CC2))
  {
    /* Set the TIM3 Compare IRQ flag */
    s_xTIMChCompareModeRaised = SET;
    
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
    
  }
}

/**
* @brief This function handles TIM4 global interrupt
* @param None.
* @retval None.
*/
void TIM4_IRQHandler(void)
{
  if(TIM_GetITStatus(TIM4, TIM_IT_CC4))
  {
    /* Set the TIM4 Compare IRQ flag */
    s_xTIMChCompareModeRaised = SET;
    
    TIM_ClearITPendingBit(TIM4, TIM_IT_CC4);
    
  }
  else if(TIM_GetITStatus(TIM4, TIM_IT_CC2))
  {
    /* Set the TIM4 Compare IRQ flag */
    s_xTIMChCompareModeRaised = SET;
    
    TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);
    
  }
}

/**
* @brief  This function can be used to automatically measure the XTAL frequency making use of the
*         Spirit clock output to pin and an STM32L timer in compare mode.
* @param  None.
* @retval None.
*/
#define N_SAMPLES 20
#define SETTLING_PERIODS 4
#define A 0.4
uint32_t SpiritManagementComputeXtalFrequency(void)
{   
  GPIO_TypeDef *pGpioPeriph;
  TIM_TypeDef *pTimerPeriph;
  
  //*** DiZiC Demo
  //*** SPIRIT1 GPIO_0 connected to PB.05, pin 41. AF: TIM3_CH2
  pTimerPeriph=TIM3;
  pGpioPeriph=GPIOB;
  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  /* GPIOB clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
 
  
  //#warning It is more safe disable all the other interrupt source.
  /* MCU GPIO, NVIC and timer configuration structures */
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  TIM_ICInitTypeDef  TIM_ICInitStructure;
  
  uint32_t lMeasuredXtalFrequency;

  
  /* Instance the variables used to compute the XTAL frequency */
  uint8_t CaptureNumber=0;
  uint16_t IC3ReadValue1=0,IC3ReadValue2=0,Capture=0;
  volatile uint16_t cWtchdg = 0;
  uint32_t TIMFreq=0,lXoFreq=0;
  float fXoFreqRounded;
  
  /* Alternate Function (AF): TIM3 channel 2 pin (PB.05) configuration */
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;
  GPIO_Init(pGpioPeriph, &GPIO_InitStructure);
  GPIO_PinAFConfig(pGpioPeriph, GPIO_PinSource5, GPIO_AF_TIM3);
  
  /* Configure the timer compare channel 2 */
  TIM_ICInitStructure.TIM_Channel     = TIM_Channel_2;
  /* Configure the timer IRQ to be raised on the rising fronts */
  TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising;
  /* Input capture selection setting */
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  /* Input capture prescaler setting. Setting it to TIM_ICPSC_DIV8 makes the IRQ are raised every 8 rising fronts detected by hardware.  */
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV8;
  /* Disable every kind of capture filter */
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  
  /* Timer initialization */
  TIM_ICInit(pTimerPeriph, &TIM_ICInitStructure);
  
  /* TIM enable counter */
  TIM_Cmd(pTimerPeriph, ENABLE);
  
  /* Enable the CC2 Interrupt Request */
  TIM_ITConfig(pTimerPeriph, TIM_IT_CC2, ENABLE);
  
  /* Enable the TIM4 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  if(SpiritGeneralGetSpiritVersion() != SPIRIT_VERSION_2_0) {
    /* Disable the clock divider to measure the max frequency of the clock. */
    uint8_t tmp= 0x29; SpiritSpiWriteRegisters(0xB4, 1, &tmp);
  }
  
  /* Spirit1 side clock configuration */
  SpiritGpioClockOutputInit(&(ClockOutputInit){XO_RATIO_1_192, RCO_RATIO_1, EXTRA_CLOCK_CYCLES_0});
  
  /* Instance the structure used to configure the Spirit clock frequency to be divided by a 192 factor. */
  SpiritGpioInit(&(SGpioInit){SPIRIT_GPIO_0, SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_LP, SPIRIT_GPIO_DIG_OUT_MCU_CLOCK});
  
  SpiritGpioClockOutput(S_ENABLE);
  
  /* measure the frequency and average it on N_SAMPLES. Moreover cycle to wait for same SETTLING_PERIODS */
  for(uint32_t i=0;i<2*(N_SAMPLES+SETTLING_PERIODS);i++) {
    /* block the routine until the TIM CCP2 IRQ is raised */
    while(!s_xTIMChCompareModeRaised && (cWtchdg!=0xFFFF)) {
      cWtchdg++;    
    }
    
    if(cWtchdg==0xFFFF) {
      break;
    }
    else {
      cWtchdg=0;
    }
    
    /* reset the IRQ raised flag */
    s_xTIMChCompareModeRaised = RESET;
    
    /* if the SETTLING PERIODS expired */
    if(i>=SETTLING_PERIODS*2) {
      /* First TIMER capture */
      if(CaptureNumber == 0)
      {
        /* Get the Input Capture value */
        IC3ReadValue1 = TIM_GetCapture2(pTimerPeriph);
        CaptureNumber = 1;
      }
      /* Second TIMER capture */
      else if(CaptureNumber == 1)
      {
        /* Get the Input Capture value */
        IC3ReadValue2 = TIM_GetCapture2(pTimerPeriph);
        
        /* Capture computation */
        if (IC3ReadValue2 > IC3ReadValue1)
        {
          /* If the TIMER didn't overflow between the first and the second capture. Compute it as the difference between the second and the first capture values. */
          Capture = (IC3ReadValue2 - IC3ReadValue1) - 1;
        }
        else
        {
          /* .. else, if overflowed 'roll' the first measure to be complementar of 0xFFFF */
          Capture = ((0xFFFF - IC3ReadValue1) + IC3ReadValue2) - 1;
        }
        
        /* Punctual frequency computation */
        TIMFreq = (uint32_t) SystemCoreClock / Capture;
        
        /* Averaged frequency computation */
        lXoFreq =(uint32_t)(A*(float)lXoFreq+(1.0-A)*(float)TIMFreq);
        
        CaptureNumber = 0;
      }
    }
  }
  
  /* Compute the real frequency in Hertz tanking in account the MCU and Spirit divisions */
  lXoFreq *=(192*8);
  
  /* Disable the output clock */
  SpiritGpioClockOutput(S_DISABLE);
  
  /* TIM enable counter */
  TIM_Cmd(pTimerPeriph, DISABLE);
  
  /* Enable the CC2 Interrupt Request */
  TIM_ITConfig(pTimerPeriph, TIM_IT_CC2, DISABLE);
  
  /* SPIRIT GPIO 0 to the default configuration */
  SpiritGpioSetLevel(SPIRIT_GPIO_0, LOW);
  
  if(SpiritGeneralGetSpiritVersion() != SPIRIT_VERSION_2_0)
  {
    uint8_t tmp= 0x21; SpiritSpiWriteRegisters(0xB4, 1, &tmp);
  }
  
  /* Round the measured frequency to be measured as an integer MHz value */
  fXoFreqRounded = (float)lXoFreq/1e6;
  
  if( fXoFreqRounded-(float)((uint32_t)fXoFreqRounded)>0.5)
  {
    lMeasuredXtalFrequency = (((uint32_t)fXoFreqRounded+1)*1000000);
  }
  else
  {
    lMeasuredXtalFrequency = (((uint32_t)fXoFreqRounded)*1000000);
  }
  
  SdkEvalM2SGpioInit(M2S_GPIO_0, M2S_MODE_GPIO_IN);
  
  SpiritRadioSetXtalFrequency(lMeasuredXtalFrequency);
  
  return lMeasuredXtalFrequency;
}

/* This function is used to detect the pa ext board, due to the unworking measurement algorithm */
//*** DiZiC
//*** uint32_t SpiritManagementComputeXtalFrequencyGpio2(void)
 uint32_t SpiritManagementComputeXtalFrequencyGpio0(void)
{
  
  GPIO_TypeDef *pGpioPeriph;
  TIM_TypeDef *pTimerPeriph;
  
 
    //*** DiZiC Demo. SPIRIT1 GPIO_0 connected to = PB.05, AF: TIM3_CH2, pin 41
    pTimerPeriph=TIM3;
    pGpioPeriph=GPIOB;
    /* TIM2 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    /* GPIOB clock enable */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
 
    
  /* MCU GPIO, NVIC and timer configuration structures */
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  TIM_ICInitTypeDef  TIM_ICInitStructure;
  
  uint32_t lMeasuredXtalFrequency;
  
  /* Instance the variables used to compute the XTAL frequency */
  uint8_t CaptureNumber=0;
  uint16_t IC3ReadValue1=0,IC3ReadValue2=0,Capture=0;
  volatile uint16_t cWtchdg = 0;
  uint32_t TIM3Freq=0,lXoFreq=0;
  float fXoFreqRounded;
  
  
  /* TIM3 channel #2 pin (AF of PB.05) configuration */
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;
  GPIO_Init(pGpioPeriph, &GPIO_InitStructure);
  GPIO_PinAFConfig(pGpioPeriph, GPIO_PinSource5, GPIO_AF_TIM3);
  
  /* Configure the timer TIM3 compare channel 2 */
  TIM_ICInitStructure.TIM_Channel     = TIM_Channel_2;
  /* Configure the timer IRQ to be raised on the rising fronts */
  TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising;
  /* Input capture selection setting */
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  /* Input capture prescaler setting. Setting it to TIM_ICPSC_DIV8 makes the IRQ are raised every 8 rising fronts detected by hardware.  */
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV8;
  /* Disable every kind of capture filter */
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  
  /* Timer initialization */
  TIM_ICInit(pTimerPeriph, &TIM_ICInitStructure);
  
  /* TIM enable counter */
  TIM_Cmd(pTimerPeriph, ENABLE);
  
  /* Enable the CC4 Interrupt Request */
  //*** TIM_ITConfig(pTimerPeriph, TIM_IT_CC4, ENABLE);
    /* Enable the CC2 Interrupt Request */
  TIM_ITConfig(pTimerPeriph, TIM_IT_CC2, ENABLE); //***
  
  /* Enable the TIM3 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Spirit1 side clock configuration */
  SpiritGpioClockOutputInit(&(ClockOutputInit){XO_RATIO_1_192, RCO_RATIO_1, EXTRA_CLOCK_CYCLES_0});
  
  /* Instance the structure used to configure the Spirit clock frequency to be divided by a 192 factor. */
  SpiritGpioInit(&(SGpioInit){SPIRIT_GPIO_0, SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_LP, SPIRIT_GPIO_DIG_OUT_MCU_CLOCK});
  SpiritGpioClockOutput(S_ENABLE);
  
  /* measure the frequency and average it on N_SAMPLES. Moreover cycle to wait for same SETTLING_PERIODS */
  for(uint32_t i=0;i<2*(N_SAMPLES+SETTLING_PERIODS);i++)
  {
    /* block the routine until the TIM3 CCP2 IRQ is raised */
    while(!s_xTIMChCompareModeRaised && (cWtchdg!=0xFFFF))
      cWtchdg++;    
    
    if(cWtchdg==0xFFFF)
      break;
    else
      cWtchdg=0;
    
    /* reset the IRQ raised flag */
    s_xTIMChCompareModeRaised = RESET;
    
    /* if the SETTLING PERIODS expired */
    if(i>=SETTLING_PERIODS*2)
    {
      /* First TIMER3 capture */
      if(CaptureNumber == 0)
      {
        /* Get the Input Capture value */
//***        IC3ReadValue1 = TIM_GetCapture4(pTimerPeriph);
        IC3ReadValue1 = TIM_GetCapture2(pTimerPeriph);
        CaptureNumber = 1;
      }
      /* Second TIMER3 capture */
      else if(CaptureNumber == 1)
      {
        /* Get the Input Capture value */
//***        IC3ReadValue2 = TIM_GetCapture4(pTimerPeriph);
        IC3ReadValue2 = TIM_GetCapture2(pTimerPeriph);
        
        /* Capture computation */
        if (IC3ReadValue2 > IC3ReadValue1)
        {
          /* If the TIMER3 didn't overflow between the first and the second capture. Compute it as the difference between the second and the first capture values. */
          Capture = (IC3ReadValue2 - IC3ReadValue1) - 1;
        }
        else
        {
          /* .. else, if overflowed 'roll' the first measure to be complementar of 0xFFFF */
          Capture = ((0xFFFF - IC3ReadValue1) + IC3ReadValue2) - 1;
        }
        
        /* Punctual frequency computation */
        TIM3Freq = (uint32_t) SystemCoreClock / Capture;
        
        /* Averaged frequency computation */
        lXoFreq =(uint32_t)(A*(float)lXoFreq+(1.0-A)*(float)TIM3Freq);
        
        CaptureNumber = 0;
      }
    }
  }
  
  /* Compute the real frequency in Hertz tanking in account the MCU and Spirit divisions */
  lXoFreq *=(192*8);
  
  /* Disable the output clock */
  SpiritGpioClockOutput(S_DISABLE);
  
  /* TIM enable counter */
  TIM_Cmd(pTimerPeriph, DISABLE);
  
  /* Enable the CC4 Interrupt Request */
//***  TIM_ITConfig(pTimerPeriph, TIM_IT_CC4, DISABLE);
  TIM_ITConfig(pTimerPeriph, TIM_IT_CC2, DISABLE);

  
  /* Restore SPIRIT GPIO 0 to the default configuration */
  SpiritGpioSetLevel(SPIRIT_GPIO_0, LOW);
  
  /* Round the measured frequency to be measured as an integer MHz value */
  fXoFreqRounded = (float)lXoFreq/1e6;
  
  if( fXoFreqRounded-(float)((uint32_t)fXoFreqRounded)>0.5)
  {  
    lMeasuredXtalFrequency = (((uint32_t)fXoFreqRounded+1)*1000000);
  }
  else
  {
    lMeasuredXtalFrequency = (((uint32_t)fXoFreqRounded)*1000000);
  }
  
  SdkEvalM2SGpioInit(M2S_GPIO_0, M2S_MODE_GPIO_IN);
  
  return lMeasuredXtalFrequency;
}


/**
* @brief  Compute the SPIRIT version.
* @param  None.
* @retval SpiritVersion The dentified version of the SPIRIT.
*/
void SpiritManagementComputeSpiritVersion(void)
{
  uint16_t nSpiritVersion = SpiritGeneralGetDevicePartNumber();
  SpiritVersion xSpiritVersion;
  
  for(int i=0; i<CUT_MAX_NO; i++)
  {
    if(nSpiritVersion == xSpiritVersionMap[i].nSpiritVersion)
    {
      xSpiritVersion = xSpiritVersionMap[i].xSpiritVersion;
      break;
    }
  }
  
  uint8_t cXtalFreq = SpiritManagementComputeXtalFrequencyGpio0()/1000000;
  
  /* Check if the digital divider works (cut 2.1) or not (cut 2.0) */
  if(xSpiritVersion == SPIRIT_VERSION_2_1)  
  {
    if(cXtalFreq>=24 && cXtalFreq<=26)
    {
      /* Disable the clock divider to measure the max frequency of the clock. */
      uint8_t tmp= 0x29; SpiritSpiWriteRegisters(0xB4, 1, &tmp);
      
      cXtalFreq = SpiritManagementComputeXtalFrequencyGpio0()/1000000;
      
      if(cXtalFreq>=24 && cXtalFreq<=26)
      {
        xSpiritVersion = SPIRIT_VERSION_2_0;
      }
      else
      {
        xSpiritVersion = SPIRIT_VERSION_2_1;
      }
      
      tmp= 0x21; SpiritSpiWriteRegisters(0xB4, 1, &tmp);
    }
    else
    {
      if(cXtalFreq>=12 && cXtalFreq<=13)
      {
        xSpiritVersion = SPIRIT_VERSION_2_1;
      }
    }  
  }

  if(SdkEvalGetVersion() == SDK_EVAL_VERSION_D1)
    xSpiritVersion = SPIRIT_VERSION_3_0_D1;
  
  SpiritGeneralSetSpiritVersion(xSpiritVersion);
  
}

/**
* @brief  Read the status register.
* @param  None
* @retval Status
*/


void SpiritManagementIdentificationRFBoard(void)
{
  
  SpiritManagementComputeSpiritVersion();
  SpiritManagementComputeXtalFrequency();
  
}

void SpiritManagementSetBand(uint8_t value)
{
  s_RfModuleBand = value;
}

uint8_t SpiritManagementGetBand(void)
{
  return s_RfModuleBand;
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

/**
* @}
*/


/******************* (C) COPYRIGHT 2012 STMicroelectronics *****END OF FILE****/
