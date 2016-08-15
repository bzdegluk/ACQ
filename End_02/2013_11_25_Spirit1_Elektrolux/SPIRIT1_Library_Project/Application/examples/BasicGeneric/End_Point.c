/**
* @file    End_Poin.c
* @author  
* @version V0.0.2
* @date    25 November 2013
* @brief   Example of data transmission of SPIRIT1 Basic packets.
* 
*/

/* Includes ------------------------------------------------------------------*/
#include "SDK_EVAL_Config.h"
#include "SPIRIT_Config.h"
#include "SDK_Configuration_Common.h"
#include "SPIRIT_SDK_Application.h"
#include "SPIRIT_Commands.h"
#include "ACQ.h"

#define LED_GREEN     LED2 
#define LED_YELLOW    LED1

#define FALSE 0
#define TRUE !FALSE

#define EnableInterrupts()   __set_PRIMASK(0);
#define DisableInterrupts()  __set_PRIMASK(1);

__ATTRIBUTES void          __set_PRIMASK( unsigned long );

#undef MY_ADDRESS
#define MY_ADDRESS                  0x44
#undef DESTINATION_ADDRESS
#define DESTINATION_ADDRESS         0x34

void USART1_Init(void);
void IWDG_Init(void);
void SPIRIT_INIT(void);

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


PktBasicAddressesInit xAddressInit={
  EN_FILT_MY_ADDRESS,
  MY_ADDRESS,
  EN_FILT_MULTICAST_ADDRESS,
  MULTICAST_ADDRESS,
  EN_FILT_BROADCAST_ADDRESS,
  BROADCAST_ADDRESS
};


SGpioInit xGpioIRQ={
  SPIRIT_GPIO_0,
  SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_LP,
  SPIRIT_GPIO_DIG_OUT_IRQ
};

SpiritIrqs xIrqStatus;

uint8_t vectcRxBuff[96], cRxData;

uint8_t vectcTxBuff[20]={0,0,88,88,88,88,70,71,72,73,74,75,76,77,78,79,80,81,82,0};

FlagStatus xTxDoneFlag = RESET;

void M2S_GPIO_0_EXTI_IRQ_HANDLER(void)
{
  if(EXTI_GetFlagStatus(M2S_GPIO_0_EXTI_LINE))
  {
    SpiritIrqGetStatus(&xIrqStatus); // Get the IRQ status
          if(xIrqStatus.IRQ_RX_DATA_DISC)  // Check the SPIRIT1 RX_DATA_DISC IRQ flag 
        {
  //        SdkEvalLedToggle(LED_YELLOW);  // IRQ: Spirit1 RX data discarded (upon filtering)
        }
    // Check the SPIRIT TX_DATA_SENT IRQ flag
    if(xIrqStatus.IRQ_TX_DATA_SENT)
    {
      // Set the xTxDoneFlag to manage processing of this event in the main()  
      xTxDoneFlag = SET;
      SdkEvalLedToggle(LED_GREEN);
    }
    if(xIrqStatus.IRQ_TX_FIFO_ERROR)
    {
      //   
      xTxDoneFlag = SET;
//      SdkEvalLedToggle(LED_GREEN);
    }
    if(xIrqStatus.IRQ_RX_DATA_READY)
    {
      command_size = SpiritLinearFifoReadNumElementsRxFifo();  //get the size of received command
      SpiritSpiReadLinearFifo(command_size, command_buffer);    //read the buffer
      SpiritCmdStrobeFlushRxFifo();     //flush the RX fifo
      SdkEvalLedToggle(LED_YELLOW);
//      SpiritCmdStrobeRx();
      command_pending = 1;
    }
    
    // Clear the EXTI line flag
   
    EXTI_ClearFlag(M2S_GPIO_0_EXTI_LINE);
  }
} // end of M2S_GPIO_0_EXTI_IRQ_HANDLER()
//void USART1_IRQn(void)     //// interupt from uart
//{
//
//}



void main (void)
{
  SPIRIT_INIT();
  while (1)
  {
    if (Spirit_data == Spirit_waiting_window)            // check if there is data ready to be sent
    {
      if(SysyTickCnt > ((endpoint_address * 100) + 50))  // check if window for sending
        {
          send_to_Base_Station();                       // sending to Base Station
        }
    }
    
    if(command_pending == 1)
    {
      apply_command();
    }
      
    IWDG_ReloadCounter();          //przeladowanie IWDG    
 //   SdkEvalLedToggle(LED_GREEN);
      
    if (App == App_connected)
    {
      daas_manage();
    }else
    {
      app_connect();
    }
   }
}  // End of main()
    
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
  } // End of assert_failed(file, line)
#endif  // USE_FULL_ASSERT

void USART1_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART1_InitStruct;
    
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; 
  GPIO_Init(GPIOA, &GPIO_InitStructure);
     
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9,GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10,GPIO_AF_USART1);  
    
  /* Baud rate 115200, 8-bit data, One stop bit
   * No parity, Do both Rx and Tx, No HW flow control
   */
  USART1_InitStruct.USART_BaudRate = 9600;   
  USART1_InitStruct.USART_WordLength = USART_WordLength_9b;  
  USART1_InitStruct.USART_StopBits = USART_StopBits_1;   
  USART1_InitStruct.USART_Parity = USART_Parity_Even;
 // USART1_InitStruct.USART_Parity = USART_Parity_No;
  USART1_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART1_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    
  // Enable USART1
  USART_Cmd(USART1, ENABLE);  
    
  // Configure USART1
  USART_Init(USART1, &USART1_InitStruct);
    
  // Enable RXNE Interrupt
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
    
  // Enable USART1 Global Interrupt
//  USART_ClearITPendingBit(USART1, USART_IT_TC);
  NVIC_EnableIRQ(USART1_IRQn); 
//USART_ClearITPendingBit(USART1, USART_IT_TC);  
} // End of USART1_Init()


void IWDG_Init(void)
{
  //konfigurowanie watchdoga IWDG
IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);    // Zezwolenie na zapis rejestrów IWDG
IWDG_SetPrescaler(IWDG_Prescaler_64);             // IWDG taktowany zegarem: 37kHz/64 = 0.578kHz
IWDG_SetReload(0xFFF);                        // Przepelnienie IWDG po okolo 7s   
IWDG_ReloadCounter();                           // Przeladowanie IWDG
IWDG_Enable();                                  // Wlaczenie IWDG i LSI
}

void SPIRIT_INIT(void)
{
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0000);   // Use STM32L1xx_flash.icf
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC, ENABLE );
  SdkEvalIdentification();
  SdkStartSysTick();
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
  SdkEvalLedInit(LED1);
  SdkEvalLedInit(LED2);
  SdkEvalM2SGpioInit(M2S_GPIO_SDN,M2S_MODE_GPIO_OUT);
  SpiritSpiInit();  
  USART1_Init();
  
  SpiritEnterShutdown();
  SpiritExitShutdown();
  
  SpiritManagementIdentificationRFBoard();
  SdkEvalM2SGpioInit(M2S_GPIO_0, M2S_MODE_EXTI_IN);
  
  SdkEvalM2SGpioInterruptCmd(M2S_GPIO_0,0x0F,0x0F,ENABLE);
  SpiritGpioInit(&xGpioIRQ);
  SpiritRadioInit(&xRadioInit);
  SpiritPktBasicInit(&xBasicInit);
  SpiritPktBasicAddressesInit(&xAddressInit);
  SpiritIrqDeInit(&xIrqStatus);
   SpiritIrq(RX_DATA_DISC, S_ENABLE);
   SpiritIrq(RX_DATA_READY, S_ENABLE);  // testy dla odbioru komend !!!!!
  SpiritIrq(TX_DATA_SENT, S_ENABLE);
  SpiritIrq(TX_FIFO_ERROR, S_ENABLE);
  
  // Declare Length of Payload
  SpiritPktBasicSetPayloadLength(PAYLOAD_LENGTH);
  
  SpiritQiSetSqiThreshold(SQI_TH_0);
  SpiritQiSqiCheck(S_ENABLE); 
  SpiritIrqClearStatus();
  
 // SET_INFINITE_RX_TIMEOUT();
  SpiritTimerSetRxTimeoutMs(RX_TIMEOUT);
  //SpiritTimerSetRxTimeoutStopCondition(TIMEOUT_ALWAYS_STOPPED);
  SpiritTimerSetRxTimeoutStopCondition(SQI_ABOVE_THRESHOLD);
  
  SpiritIrqClearStatus();
  //USART_Conf();
  
  //*** For test purposes only: create unsigned, 16-bit counter
  vectcTxBuff[0] = 0;   // Clear Counter to overwrite initial values
  vectcTxBuff[1] = 0;
  SdkDelayMs(2000);  //wait till the tested board FW active
 // IWDG_Init();      // let the dogs out,
}



















/**************** (C) COPYRIGHT 2013 DiZiC Ltd. ********* END OF FILE *********/