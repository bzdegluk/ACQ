/**
* @file    DZ-SB-DISC01.c
* @author  DiZiC Ltd.
* @version V3.0.1
* @date    August 11, 2013
* @brief   Example of transmission of SPIRIT Basic packets.
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
#include "SPIRIT_Commands.h"

#define LED_GREEN     LED2 
#define LED_YELLOW    LED1



#define USE_VCOM  1


#ifdef USE_VCOM
#include "SDK_EVAL_VC_General.h"
#endif



#define EnableInterrupts()   __set_PRIMASK(0);
#define DisableInterrupts()  __set_PRIMASK(1);


__ATTRIBUTES void          __set_PRIMASK( unsigned long );

_Bool PressButtom = FALSE;



/**
* @addtogroup SDK_Examples SDK Examples
* @{
*/


/**
* @addtogroup SDK_Basic_Generic        SDK Basic Generic
* @{
*/

/**
* @addtogroup SDK_Basic_Generic_A                                      SDK Basic Generic A
* @brief Device A configured as a transmitter.
* @details This code explains how to configure and manage
* in the simpler way a transmitter of basic packets.
*
* The user can change the Basic packet configuration parameters editing the defines at the beginning of the file.
* @{
*/

/**
* @defgroup Basic_Generic_A_Private_TypesDefinitions                   Basic Generic A Private TypesDefinitions
* @{
*/

/**
*@}
*/


/**
* @defgroup Basic_Generic_A_Private_Defines                            Basic Generic A Private Defines
* @{
*/

/**
*@}
*/


/**
* @defgroup Basic_Generic_A_Private_Macros                                     Basic Generic A Private Macros
* @{
*/

/**
*@}
*/

/**
* @defgroup Basic_Generic_A_Private_Variables                                  Basic Generic A Private Variables
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
* @brief GPIO structure fitting. Change from GPIO_3 to GPIO_0
*/
SGpioInit xGpioIRQ={
  SPIRIT_GPIO_0,
  SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_LP,
  SPIRIT_GPIO_DIG_OUT_IRQ
};


/**
* @brief Declare the Tx done flag
*/
FlagStatus xTxDoneFlag = RESET;

/**
 * @brief Rx buffer declaration: how to store the received data
 */
uint8_t vectcRxBuff[96], cRxData;


/**
* @brief IRQ status struct declaration
*/
SpiritIrqs xIrqStatus;


/**
* @brief Tx buffer declaration: data to transmit
*/
uint8_t vectcTxBuff[20]={1,1,1,1,2,6,7,8,9,10,11,12,0,14,15,16,0,18,19,1};

uint16_t UARTBuff[6]={0};



/**
*@}
*/

/**
* @defgroup Basic_Generic_A_Private_FunctionPrototypes                                         Basic Generic A Private FunctionPrototypes
* @{
*/

/**
*@}
*/


/**
* @defgroup Basic_Generic_A_Private_Functions                                                  Basic Generic A Private Functions
* @{
*/

/**
* @brief  This function handles External interrupt request (associated with Spirit GPIO 0).
* @param  None
* @retval None
*/

void M2S_GPIO_0_EXTI_IRQ_HANDLER(void)

{

  /* Check the flag status of EXTI line */
  if(EXTI_GetITStatus(M2S_GPIO_0_EXTI_LINE))
  {       
      /* Get the IRQ status */
    SpiritIrqGetStatus(&xIrqStatus);
    // Check the SPIRIT RX_DATA_DISC IRQ flag 
    if(xIrqStatus.IRQ_RX_DATA_DISC )
    {
      SdkEvalLedToggle(LED_YELLOW);

    }
    
    //Check the SPIRIT RX_DATA_READY IRQ flag 
    if(xIrqStatus.IRQ_RX_DATA_READY )
    {
      // Get the RX FIFO size 
      cRxData=SpiritLinearFifoReadNumElementsRxFifo();
      
      
      // Read the RX FIFO 
      SpiritSpiReadLinearFifo(cRxData, vectcRxBuff);
      
      // Flush the RX FIFO 
      SpiritCmdStrobeFlushRxFifo();
      
      
     SdkEvalLedToggle(LED_GREEN);
       
      //RX command - to ensure the device will be ready for the next reception 
      //SpiritCmdStrobeRx();
      
       
#ifdef USE_VCOM
      
      printf("B data received: [");
      for(uint8_t i=0 ; i<cRxData ; i++)
        printf("%d ", vectcRxBuff[i]);
      printf("]\r\n");
#endif
     
     RSSI_TO_UART();
        
        
    }
    
  
     /* Check the SPIRIT TX_DATA_SENT IRQ flag */
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
  
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0000);  // Use STM32L1xx_flash.icf
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC | RCC_AHBPeriph_GPIOD | RCC_AHBPeriph_GPIOE, ENABLE );

  SdkEvalIdentification();
  SdkStartSysTick();
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
  SdkEvalLedInit(LED1);
  SdkEvalLedInit(LED2);
  SdkEvalM2SGpioInit(M2S_GPIO_SDN,M2S_MODE_GPIO_OUT);
  SpiritSpiInit();
  USART1_Init();
  WUKPIN1_Init();

  
#ifdef USE_VCOM 
 SdkEvalVCInit(); 
  while(bDeviceState != CONFIGURED);
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
  SpiritIrq(RX_DATA_DISC, S_ENABLE);
  SpiritIrq(RX_DATA_READY, S_ENABLE);
  SpiritIrq(TX_DATA_SENT, S_ENABLE);
 
  /* payload length config */
  SpiritPktBasicSetPayloadLength(512);

  /* enable SQI check */
  SpiritQiSetSqiThreshold(SQI_TH_0);
  SpiritQiSqiCheck(S_ENABLE);

  /* RX timeout config */
  SpiritTimerSetRxTimeoutMs(1000.0);
  SpiritTimerSetRxTimeoutStopCondition(SQI_ABOVE_THRESHOLD);

  /* IRQ registers blanking */
  SpiritIrqClearStatus();
////////////////////////////////////////////////////////////////////////////////
 	ErrorStatus HSE_Status;
	RCC_HSEConfig(RCC_HSE_ON);
	HSE_Status = RCC_WaitForHSEStartUp();
	FLASH_SetLatency(FLASH_Latency_1);
	FLASH_PrefetchBufferCmd(ENABLE);
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
	RCC_HCLKConfig(RCC_SYSCLK_Div1);
	RCC_PLLConfig(RCC_PLLSource_HSE, RCC_PLLMul_12, RCC_PLLDiv_3);
	RCC_PCLK1Config(RCC_HCLK_Div1);
	RCC_PCLK2Config(RCC_HCLK_Div1);
 ///////////////////////////////////////////////////////////////////////////////// 
  
  while (1)
  {
    
     /* RX command */
    SpiritCmdStrobeRx();
 
   
  if(PressButtom)
      {  
         
         PressButtom = FALSE;      
         SdkDelayMs(1000); //wait Rx timeout
     
        /* fit the TX FIFO */
        SpiritCmdStrobeFlushTxFifo();
        SpiritSpiWriteLinearFifo(20, vectcTxBuff);
         
        /* send the TX command */
        SpiritCmdStrobeTx();

        /* wait for TX done */ 
      
        while(!xTxDoneFlag);
        xTxDoneFlag = RESET;
    
      }
  
   
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
  }
}
#endif

void USART1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART1_InitStruct;
 
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
    GPIO_Init(GPIOA, &GPIO_InitStructure);
     

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9,GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10,GPIO_AF_USART1);  
    
    /* Baud rate 9600, 8-bit data, One stop bit
     * No parity, Do both Rx and Tx, No HW flow control
     */
    USART1_InitStruct.USART_BaudRate = 115200;   
    USART1_InitStruct.USART_WordLength = USART_WordLength_8b;  
    USART1_InitStruct.USART_StopBits = USART_StopBits_1;   
    USART1_InitStruct.USART_Parity = USART_Parity_No ;
    USART1_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART1_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    
    /* Enable USART1 */    
    USART_Cmd(USART1, ENABLE);  
    
    /* Configure USART1 */
    USART_Init(USART1, &USART1_InitStruct);
    
    /* Enable RXNE interrupt */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    
    /* Enable USART1 global interrupt */
   NVIC_EnableIRQ(USART1_IRQn);
    
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


void convert_into_char(uint32_t number, uint16_t *p_tab)
{
  uint16_t units=0, tens=0, hundreds=0, thousands=0, misc=0;
  
  units = (((number%10000)%1000)%100)%10;
  tens = ((((number-units)/10)%1000)%100)%10;
  hundreds = (((number-tens-units)/100))%100%10;
  thousands = ((number-hundreds-tens-units)/1000)%10;
  misc = ((number-thousands-hundreds-tens-units)/10000);
  
  *(p_tab+4) = units + 0x30;
  *(p_tab+3) = tens + 0x30;
  *(p_tab+2) = hundreds + 0x30;
  *(p_tab+1) = thousands + 0x30;
  *(p_tab) = misc + 0x30;

}


void RSSI_TO_UART(void)
{
  float dB_Singal;
  
  dB_Singal = SpiritQiGetRssidBm();
  dB_Singal *= 10;
  if(dB_Singal < 0)dB_Singal *= (-1);
  convert_into_char((uint32_t)dB_Singal, UARTBuff);
  UARTBuff[5] = 'B';
  UARTBuff[4] = 'd';
  UARTBuff[0] = '-';
  if(UARTBuff[1] == '0')
  {
    UARTBuff[1]='-';
    UARTBuff[0]=' ';
  }  
  
  USART_SendData(USART1, 'S');  //Start event
  while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
  for(uint8_t i=0 ;i<6 ;i++)
    {
      USART_SendData(USART1, UARTBuff[i]);
      while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    }
  USART_SendData(USART1, 'E'); //End event 
  
}


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
