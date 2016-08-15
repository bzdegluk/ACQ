/**
 * @file    SDK_BasicGeneric_B.c
 * @author  DiZiC Ltd.
 * @version V3.0.1
 * @date    August 11, 2013
 * @brief   Example of reception of SPIRIT Basic packets.
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
 * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
 */


/* Includes ------------------------------------------------------------------*/
#include "SDK_EVAL_Config.h"
#include "SPIRIT_Config.h"
#include "SDK_Configuration_Common.h"
#include "SPIRIT_SDK_Application.h"
#include "stm32l1xx_pwr.h"
#include "stm32l1xx_rtc.h"
#include "stm32l1xx_dbgmcu.h"

#define USE_VCOM  

#define LED_GREEN     LED2 
#define LED_YELLOW    LED1


#ifdef USE_VCOM
#include "SDK_EVAL_VC_General.h"
#endif


#define EnableInterrupts()   __set_PRIMASK(0);
#define DisableInterrupts()  __set_PRIMASK(1);

__ATTRIBUTES void          __set_PRIMASK( unsigned long );



/**
 * @addtogroup SDK_Examples
 * @{
 */

/**
 * @addtogroup SDK_Basic_Generic        SDK Basic Generic
 * @{
 */

/**
 * @addtogroup SDK_Basic_Generic_B              SDK Basic Generic B
 * @brief Device B configured as a receiver.
 * @details This code explains how to configure a receiver for
 * basic packets.
 *
 * The user can change the Basic packet configuration parameters editing the defines
 * at the beginning of the file.
 * @{
 */

/**
 * @defgroup Basic_Generic_B_Private_TypesDefinitions           Basic Generic B Private TypesDefinitions
 * @{
 */

/**
 *@}
 */


/**
 * @defgroup Basic_Generic_B_Private_Defines                    Basic Generic B Private Defines
 * @{
 */

/*  Addresses configuration parameters  */
#undef MY_ADDRESS
#define MY_ADDRESS                  0x44
#undef DESTINATION_ADDRESS
#define DESTINATION_ADDRESS         0x34

/**
 *@}
 */

_Bool PressButtom = FALSE;


/**
 * @defgroup Basic_Generic_B_Private_Macros                             Basic Generic B Private Macros
 * @{
 */

/**
 *@}
 */

/**
 * @defgroup Basic_Generic_B_Private_Variables                          Basic Generic B Private Variables
 * @{
 */

 /**
  * @brief Radio structure fitting
  */
  SRadioInit xRadioInit = {
    XTAL_OFFSET_PPM,
    BASE_FREQUENCY,
    CHANNEL_SPACE,
    CHANNEL_NUMBER,
    MODULATION_SELECT,
    DATARATE,
    FREQ_DEVIATION,
    BANDWIDTH
  };


/**
 * @brief Packet Basic structure fitting
 */
PktBasicInit xBasicInit={
  PREAMBLE_LENGTH,
  SYNC_LENGTH,
  SYNC_WORD,
  LENGTH_TYPE,
  LENGTH_WIDTH,
  CRC_MODE,
  CONTROL_LENGTH,
  EN_ADDRESS,
  EN_FEC,
  EN_WHITENING
};


/**
 * @brief Address structure fitting
 */
PktBasicAddressesInit xAddressInit={
  EN_FILT_MY_ADDRESS,
  MY_ADDRESS,
  EN_FILT_MULTICAST_ADDRESS,
  MULTICAST_ADDRESS,
  EN_FILT_BROADCAST_ADDRESS,
  BROADCAST_ADDRESS
};



/**
 * @brief GPIO IRQ structure fitting
 */
SGpioInit xGpioIRQ={
  SPIRIT_GPIO_0,
  SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_LP,
  SPIRIT_GPIO_DIG_OUT_IRQ
};


/**
 * @brief IRQ status struct declaration
 */
SpiritIrqs xIrqStatus;


/**
 * @brief Rx buffer declaration: how to store the received data
 */
uint8_t vectcRxBuff[96], cRxData;

/**
 *@}
 */

/**
* @brief Declare the Tx done flag
*/
FlagStatus xTxDoneFlag = RESET;


/**
* @brief IRQ status struct declaration
*/
SpiritIrqs xIrqStatus;



/**
 * @defgroup Basic_Generic_B_Private_FunctionPrototypes                         Basic Generic B Private FunctionPrototypes
 * @{
 */

/**
 *@}
 */
uint8_t vectcTxBuff[20]={0,0,0,0,0,0,70,0,2,10,11,9,0,0,0,0,0,0,0,0};
uint8_t vectcTxBuff2[]={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49};
/**
 * @defgroup Basic_Generic_B_Private_Functions                                  Basic Generic B Private Functions
 * @{
 */

/**
 * @brief  This function handles External interrupt request. In this application it is used
 *         to manage the Spirit IRQ configured to be notified on the Spirit GPIO_3.
 * @param  None
 * @retval None
 */

void M2S_GPIO_0_EXTI_IRQ_HANDLER(void)
{
 
  /* Check the flag status of EXTI line */
  if(EXTI_GetITStatus(M2S_GPIO_0_EXTI_LINE)){
    
    /* Get the IRQ status */
    SpiritIrqGetStatus(&xIrqStatus);
    
     
    // Check the SPIRIT RX_DATA_DISC IRQ flag 
    if(xIrqStatus.IRQ_RX_DATA_DISC)
    {
      
      SdkEvalLedToggle(LED_YELLOW);  
 
    }
    
    /* Check the SPIRIT RX_DATA_READY IRQ flag */
    if(xIrqStatus.IRQ_RX_DATA_READY)
    {
     
      /* Get the RX FIFO size */
      cRxData=SpiritLinearFifoReadNumElementsRxFifo();
      
      /* Read the RX FIFO */
      SpiritSpiReadLinearFifo(cRxData, vectcRxBuff);
      
      /* Flush the RX FIFO */
      SpiritCmdStrobeFlushRxFifo();


      
#ifdef USE_VCOM
      /* print the received data */
      printf("B data received: [");
      for(uint8_t i=0 ; i<cRxData ; i++)
        printf("%d ", vectcRxBuff[i]);
      printf("]\r\n");
#endif
      
      SdkEvalLedToggle(LED_GREEN);

    }
     
    
     if(xIrqStatus.IRQ_TX_DATA_SENT)
    {
      /* set the tx_done_flag to manage the event in the main() */
      xTxDoneFlag = SET;
      
    } 
    
    /* Clear the EXTI line flag */
    EXTI_ClearITPendingBit(M2S_GPIO_0_EXTI_LINE);
    
  }
}


/**
 * @brief  System main function.
 * @param  None
 * @retval None
 */
void main (void)
{
     
  //*** DiZiC SPIRIT1 Demo
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0000);   // Use STM32L1xx_flash.icf
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE );
  SdkEvalIdentification();
  SdkStartSysTick();
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
  SdkEvalLedInit(LED1);
  SdkEvalLedInit(LED2);
  SdkEvalM2SGpioInit(M2S_GPIO_SDN,M2S_MODE_GPIO_OUT);
  SpiritSpiInit();
  WUKPIN1_Init();
  /*********************/

  /* Allow access to RTC Domain */
//   PWR_RTCAccessCmd(ENABLE);

  /* Clear WakeUp flag */
//  PWR_ClearFlag(PWR_FLAG_WU);

  /* Check if the StandBy flag is set */
//  if (PWR_GetFlagStatus(PWR_FLAG_SB) != RESET)
//  {
    /* Clear StandBy flag */
//    PWR_ClearFlag(PWR_FLAG_SB);

    /* Wait for RTC APB registers synchronisation */
//    RTC_WaitForSynchro();
    /* No need to configure the RTC as the RTC config(clock source, enable,
       prescaler,...) are kept after wake-up from STANDBY */
 

//  }
  
   
 
    /* RTC Configuration */
  
    /* Reset RTC Domain */
//    RCC_RTCResetCmd(ENABLE);
//    RCC_RTCResetCmd(DISABLE);

    /* Enable the LSE OSC */
//     RCC_LSICmd(ENABLE);
    
    /* Wait till LSE is ready */
//    while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
//    {}

    /* Select the RTC Clock Source */
//    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);

    /* Enable the RTC Clock */
//    RCC_RTCCLKCmd(ENABLE);


    /* Wait for RTC APB registers synchronisation */
//    RTC_WaitForSynchro();
    
 
   /* RTC domain*/  
//  RTC_WakeUpCmd(DISABLE);
  
//  RTC_ITConfig(RTC_IT_WUT , ENABLE);
//  RTC_WakeUpClockConfig(RTC_WakeUpClock_RTCCLK_Div4); 
//  RTC_SetWakeUpCounter(0xFF00);
  
//  PWR_WakeUpPinCmd(PWR_WakeUpPin_1 , DISABLE);
  
  
  
#ifdef USE_VCOM
  /* VC config */
 // SdkEvalVCInit();
 // while(bDeviceState != CONFIGURED);
#endif

  /* Spirit ON */
  SpiritEnterShutdown();
  SpiritExitShutdown();
  
  
  SpiritManagementIdentificationRFBoard();
  SdkEvalM2SGpioInit(M2S_GPIO_0,M2S_MODE_EXTI_IN);
  
  /* Spirit IRQ config */
  SpiritGpioInit(&xGpioIRQ);
  SdkEvalM2SGpioInterruptCmd(M2S_GPIO_0,0x0F,0x0F,ENABLE);
  
  //*** SdkEvalLedOn(LED1);
  /* Spirit Radio config */
  SpiritRadioInit(&xRadioInit);
  
  /* Spirit Packet config */
  SpiritPktBasicInit(&xBasicInit);
  SpiritPktBasicAddressesInit(&xAddressInit);

  /* Spirit IRQs enable */
  SpiritIrqDeInit(&xIrqStatus);
  SpiritIrq(RX_DATA_DISC,S_ENABLE);
  SpiritIrq(RX_DATA_READY,S_ENABLE);
  SpiritIrq(TX_DATA_SENT , S_ENABLE);

  

  /* payload length config */
  SpiritPktBasicSetPayloadLength(512);

  /* enable SQI check */
  SpiritQiSetSqiThreshold(SQI_TH_0);
  SpiritQiSqiCheck(S_ENABLE);

  /* RX timeout config */
  SpiritTimerSetRxTimeoutMs(200.0);
  SpiritTimerSetRxTimeoutStopCondition(SQI_ABOVE_THRESHOLD);

  /* IRQ registers blanking */
  SpiritIrqClearStatus();

  /* RX command */
  SpiritCmdStrobeRx();
  
  
  
 // PWR_PVDCmd(DISABLE);
 // RCC_MSIRangeConfig(RCC_MSIRange_0);
 // RCC_AdjustMSICalibrationValue(0x00);
 // RCC_MSICmd(DISABLE);
 // RCC_HSICmd(DISABLE);
//  PWR_EnterSTOPMode(PWR_Regulator_LowPower , PWR_STOPEntry_WFI);
//  PWR_UltraLowPowerCmd(ENABLE);
//  PWR_FastWakeUpCmd(ENABLE);  
 
////////////////////////////////////////////////////////////////////////////////
// 	ErrorStatus HSE_Status;
//	RCC_HSEConfig(RCC_HSE_ON);
//	HSE_Status = RCC_WaitForHSEStartUp();
//	FLASH_SetLatency(FLASH_Latency_1);
//	FLASH_PrefetchBufferCmd(ENABLE);
//	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
//	RCC_HCLKConfig(RCC_SYSCLK_Div1);
//	RCC_PLLConfig(RCC_PLLSource_HSE, RCC_PLLMul_12, RCC_PLLDiv_3);
//	RCC_PCLK1Config(RCC_HCLK_Div1);
//	RCC_PCLK2Config(RCC_HCLK_Div1);
 ///////////////////////////////////////////////////////////////////////////////// 
  
  
  /* infinite loop */
  while (1){
    
 printf("123");
    SpiritCmdStrobeRx();
        
   
   if(PressButtom)
    {
      PressButtom = FALSE;
      
      NOPdelay(2000);
    
     // fit the TX FIFO 
      SpiritCmdStrobeFlushTxFifo();
      SpiritSpiWriteLinearFifo(500, vectcTxBuff2);
    
     // send the TX command 
     SpiritCmdStrobeTx();
    
    // wait for TX done 
     SdkEvalLedToggle(LED_GREEN);
     while(!xTxDoneFlag);
     SdkEvalLedToggle(LED_GREEN);
     xTxDoneFlag = RESET; 
     
          // fit the TX FIFO 
      SpiritCmdStrobeFlushTxFifo();
      SpiritSpiWriteLinearFifo(500, vectcTxBuff2);
    
     // send the TX command 
     SpiritCmdStrobeTx();
    
    // wait for TX done 
     SdkEvalLedToggle(LED_GREEN);
     while(!xTxDoneFlag);
     SdkEvalLedToggle(LED_GREEN);
     xTxDoneFlag = RESET; 
     
          // fit the TX FIFO 
      SpiritCmdStrobeFlushTxFifo();
      SpiritSpiWriteLinearFifo(500, vectcTxBuff2);
    
     // send the TX command 
     SpiritCmdStrobeTx();
    
    // wait for TX done 
     SdkEvalLedToggle(LED_GREEN);
     while(!xTxDoneFlag);
     SdkEvalLedToggle(LED_GREEN);
     xTxDoneFlag = RESET; 
     
     
    }
   
      
    
  }

}


void WUKPIN1_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* To configure PA00 WakeUP output */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0  ;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_Init( GPIOA, &GPIO_InitStructure);   
            
  /* Configure EXT1 Line 0 in interrupt mode trigged on Falling edge */
  EXTI_InitStructure.EXTI_Line = EXTI_Line0 ;  // PA0 for User button AND IDD_WakeUP
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  
  /* Enable and set EXTI0 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
    
 
}


void EXTI0_IRQHandler(void)
{
   DisableInterrupts(); 
   PressButtom = TRUE;
   EXTI_ClearITPendingBit(EXTI_Line0);
   EnableInterrupts();
}

void NOPdelay(uint16_t delaytime)
{
  uint16_t ii;
  
  for(ii=0 ; ii < delaytime ; ii++)
  {
     __NOP();__NOP();__NOP();__NOP();__NOP();
     __NOP();__NOP();__NOP();__NOP();__NOP();
     __NOP();__NOP();__NOP();__NOP();__NOP();
     __NOP();__NOP();__NOP();__NOP();__NOP();
  
  }

}







#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number */
     printf("Wrong parameters value: file %s on line %d\r\n", file, line);

  /* Infinite loop */
  while (1)
  {
    printf("Wrong parameters value: file %s on line %d\r\n", file, line);
  }
}
#endif


/**
 *@}
 */

/**
 *@}
 */

/**
 *@}
 */

/**
 *@}
 */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
