/**
* @file    Base_Station.c
* @author  
* @version V0.0.2
* @date    25 November 2013
* @brief   Example of reception (transmission if User Push button pressed)
*          of SPIRIT1 Basic packets.
* 
*/

/* Includes ------------------------------------------------------------------*/
#include "SDK_EVAL_Config.h"
#include "SPIRIT_Config.h"
#include "SDK_Configuration_Common.h"
#include "SPIRIT_SDK_Application.h"
#include "SPIRIT_Commands.h"

#define LED_GREEN     LED2 
#define LED_YELLOW    LED1

#define FALSE 0
#define TRUE !FALSE


//#define USE_VCOM  1
//#ifdef USE_VCOM
//#include "SDK_EVAL_VC_General.h"
//#endif

#define EnableInterrupts()   __set_PRIMASK(0);
#define DisableInterrupts()  __set_PRIMASK(1);

__ATTRIBUTES void          __set_PRIMASK( unsigned long );

_Bool PressButtom = FALSE;

#undef MY_ADDRESS
#define MY_ADDRESS                  0x44
#undef DESTINATION_ADDRESS
#define DESTINATION_ADDRESS         0x34

void USART1_Init(void);
uint8_t jednostki, dziesiatki;

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
uint8_t vectcRxBuff[212], cRxData;


/**
* @brief IRQ status struct declaration
*/
SpiritIrqs xIrqStatus;


/**
* @brief Tx buffer declaration: data to transmit
*/
uint8_t vectcTxBuff[20]={1,1,1,1,2,6,7,8,9,10,11,12,0,14,15,16,0,18,19,1};

uint16_t UARTBuff[6]={0};

void M2S_GPIO_0_EXTI_IRQ_HANDLER(void)
{  
    if(EXTI_GetITStatus(M2S_GPIO_0_EXTI_LINE)) // Check the flag status of EXTI line
  {     
    SpiritIrqGetStatus(&xIrqStatus); // Get the IRQ status
    if(xIrqStatus.IRQ_RX_DATA_DISC)  // Check the SPIRIT1 RX_DATA_DISC IRQ flag 
    {
     SdkEvalLedToggle(LED_YELLOW);  // IRQ: Spirit1 RX data discarded (upon filtering)
    }
    if(xIrqStatus.IRQ_RX_DATA_READY) // Check the SPIRIT1 RX_DATA_READY IRQ Flag 
    {
     cRxData=SpiritLinearFifoReadNumElementsRxFifo();  // Get the RX FIFO size 
     
      SpiritSpiReadLinearFifo(cRxData, vectcRxBuff); // Read the RX FIFO 
      
      SpiritCmdStrobeFlushRxFifo();  // Flush the RX FIFO 
      SdkEvalLedToggle(LED_GREEN);
      
      SpiritCmdStrobeRx(); // RX command - to ensure that Rx device will be ready for the next reception 
      RSSI_TO_UART();  // Send RSSI Value of received packet to Discovery Board
      ///////////////////////////////////
           USART_SendData(USART1, 'Q');
      while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
      
  jednostki = (cRxData%10)+0x30;
  dziesiatki = (cRxData/10)+0x30;
  
      USART_SendData(USART1, dziesiatki);
      while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        USART_SendData(USART1, jednostki);
      while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
  
    for(uint8_t i=0 ;i<cRxData ;i++)
    {
      USART_SendData(USART1, vectcRxBuff[i]);
      while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    }
       //////////////////////////////
    }
    
    if(xIrqStatus.IRQ_TX_DATA_SENT) // Check the SPIRIT TX_DATA_SENT IRQ flag
    {
      xTxDoneFlag = SET;   // Set the Tx_done_flag to manage the event in the main()
    }
    
    EXTI_ClearITPendingBit(M2S_GPIO_0_EXTI_LINE);  // Clear the EXTI line flag
  
  }
  
   
} // end of M2S_GPIO_0_EXTI_IRQ_HANDLER()

void main (void)
{
  
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0000);  // Use STM32L1xx_flash.icf
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | \
                        RCC_AHBPeriph_GPIOC | RCC_AHBPeriph_GPIOD | RCC_AHBPeriph_GPIOE, ENABLE );

  SdkEvalIdentification();
  SdkStartSysTick();
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
  SdkEvalLedInit(LED1);
  SdkEvalLedInit(LED2);
  SdkEvalM2SGpioInit(M2S_GPIO_SDN,M2S_MODE_GPIO_OUT);
  SpiritSpiInit();
  USART1_Init();
  
//#ifdef USE_VCOM 
// SdkEvalVCInit(); 
//  while(bDeviceState != CONFIGURED);
//#endif 
  
  // Spirit ON
  SpiritEnterShutdown();
  SpiritExitShutdown();

  SpiritManagementIdentificationRFBoard();
  SdkEvalM2SGpioInit(M2S_GPIO_0,M2S_MODE_EXTI_IN);
  
  // Spirit IRQ config
  SpiritGpioInit(&xGpioIRQ);
  SdkEvalM2SGpioInterruptCmd(M2S_GPIO_0,0x0F,0x0F,ENABLE);
  
  //*** SdkEvalLedOn(LED1);
  // Spirit Radio config
  SpiritRadioInit(&xRadioInit);
  
  // Spirit Packet config
  SpiritPktBasicInit(&xBasicInit);
  SpiritPktBasicAddressesInit(&xAddressInit);

  // Spirit IRQs enable
  SpiritIrqDeInit(&xIrqStatus);
  SpiritIrq(RX_DATA_DISC, S_ENABLE);
  SpiritIrq(RX_DATA_READY, S_ENABLE);
  SpiritIrq(TX_DATA_SENT, S_ENABLE);

  // Declare Lenght of Payload 
  SpiritPktBasicSetPayloadLength(PAYLOAD_LENGTH);

  // Enable SQI check
  SpiritQiSetSqiThreshold(SQI_TH_0);
  SpiritQiSqiCheck(S_ENABLE);
  
  // RX Timeout Configuration
  SpiritTimerSetRxTimeoutMs(RX_TIMEOUT);
  SpiritTimerSetRxTimeoutStopCondition(SQI_ABOVE_THRESHOLD);

  // IRQ registers blanking
  SpiritIrqClearStatus();
  
  SpiritCmdStrobeRx();
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
  
  // Start Application
  while (1)
  {  
    // Enter endless loop  
    // Rx Command: receive message - if any
    SpiritCmdStrobeRx();  // Receive Message
    
    if(PressButtom)
    {  
      // Send Message
      PressButtom = FALSE;      
      SdkDelayMs(RX_TIMEOUT);  // Wait, ensuring that Rx is able to receive 
     
      // Fill the Tx FIFO
      SpiritCmdStrobeFlushTxFifo();
      SpiritSpiWriteLinearFifo(PAYLOAD_LENGTH, vectcTxBuff);
         
      // Send the Tx start command, inititaing message transmission
      SpiritCmdStrobeTx();
 
      // Wait for Tx done
      while(!xTxDoneFlag);
      xTxDoneFlag = RESET;
    }
  } // End of while(1) endless loop
} // End of main()

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
    
  /* Baud rate 115200, 8-bit data, One stop bit
   * No parity, Do both Rx and Tx, No HW flow control
   */
  USART1_InitStruct.USART_BaudRate = 115200;   
  USART1_InitStruct.USART_WordLength = USART_WordLength_8b;  
  USART1_InitStruct.USART_StopBits = USART_StopBits_1;   
  USART1_InitStruct.USART_Parity = USART_Parity_No ;
  USART1_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART1_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    
  // Enable USART1
  USART_Cmd(USART1, ENABLE);  
    
  // Configure USART1
  USART_Init(USART1, &USART1_InitStruct);
    
  // Enable RXNE Interrupt
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    
  // Enable USART1 Global Interrupt
  NVIC_EnableIRQ(USART1_IRQn);   
} // End of USART1_Init()

// Not used in this version
void WUKPIN1_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  // To configure PA.00 WakeUp output
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0  ;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_Init( GPIOA, &GPIO_InitStructure);   
  
  // Configure EXT1 Line 0 in interrupt mode trigged on Rising edge 
  EXTI_InitStructure.EXTI_Line = EXTI_Line0 ;  // PA0 for User button AND IDD_WakeUP
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  
  // Enable and set EXTI0 Interrupt to the lowest priority
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
} // WUKPIN1_Init()


void EXTI0_IRQHandler(void)
{
   DisableInterrupts();
   PressButtom = TRUE;
   EXTI_ClearITPendingBit(EXTI_Line0);
   EnableInterrupts();
} // end of EXTI0_IRQHandler()


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
} // End of convert_into_char(number, *p_tab)

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
  // Sent RSSI string - between Start and End Event Markers
  USART_SendData(USART1, 'S');  // Start Event Marker
  while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
  for(uint8_t i=0 ;i<6 ;i++)
    {
      USART_SendData(USART1, UARTBuff[i]);
      while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    }
  USART_SendData(USART1, 'E'); // End Event Marker
} // End of RSSI_TO_UART()

#ifdef USE_FULL_ASSERT
/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param file: pointer to the source file name
* @param line: assert_param error line source number
* @retval : None
*/
  void assert_failed(uint8_t* file, uint32_t line)
  {
    // User can add his own implementation to report the file name and line number
    printf("Wrong parameters value: file %s on line %d\r\n", file, line);
  
    // Enter Infinite Loop
    while (1)
    {
      // Endless loop
    }
  } // end of assert_failed(file, line)
#endif  // USE_FULL_ASSERT

/**************** (C) COPYRIGHT 2013 DiZiC Ltd. ********* END OF FILE *********/
