/**
 * @file    SDK_StackLlp_B.c
 * @author  High End Analog & RF BU - AMS / ART Team IMS-Systems Lab
 * @version V3.0.0
 * @date    August 7, 2012
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
#include "SDK_Debug.h"

#ifdef STM8L
#include "STM8L_Common.h"
#define XTAL_FREQUENCY          50000000
#define SPIRIT_VERSION          SPIRIT_VERSION_3_0
#define RANGE_TYPE              RANGE_EXT_NONE       //RANGE_EXT_SKYWORKS
#elif SDK
#define USE_VCOM
#include "SDK_Common.h"
#endif

#ifdef USE_VCOM
#include "SDK_EVAL_VC_General.h"
#endif

//#define PIGGYBACKING

/**
 * @addtogroup SDK_Examples
 * @{
 */

/**
 * @addtogroup SDK_StackLlp                     SDK STack LLP
 * @{
 */

/**
 * @addtogroup SDK_StackLlp_B                   SDK STack LLP B
 * @{
 */

/**
 * @defgroup StackLlp_B_Private_TypesDefinitions                STack LLP B Private TypesDefinitions
 * @{
 */

/**
 *@}
 */


/**
 * @defgroup StackLlp_B_Private_Defines                         STack LLP B Private Defines
 * @{
 */

/*  Packet configuration parameters  */
#undef LENGTH_TYPE
#define LENGTH_TYPE                	PKT_LENGTH_VAR
#undef LENGTH_WIDTH
#define LENGTH_WIDTH                	7

/* LLP configuration parameters */
#define EN_AUTOACK                  	S_ENABLE

#ifdef PIGGYBACKING
#define EN_PIGGYBACKING             	S_ENABLE
#else
#define EN_PIGGYBACKING             	S_DISABLE
#endif

#define MAX_RETRANSMISSIONS         	PKT_DISABLE_RETX

/**
 *@}
 */


/**
 * @defgroup StackLlp_B_Private_Macros                          STack LLP B Private Macros
 * @{
 */

/**
 *@}
 */

/**
 * @defgroup StackLlp_B_Private_Variables                       STack LLP B Private Variables
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

/* Packet Basic structure fitting */
PktStackInit xStackInit={
  PREAMBLE_LENGTH,
  SYNC_LENGTH,
  SYNC_WORD,
  LENGTH_TYPE,
  LENGTH_WIDTH,
  CRC_MODE,
  CONTROL_LENGTH,
  EN_FEC,
  EN_WHITENING
};

/* Address structure fitting */
PktStackAddressesInit xAddressInit={
  EN_FILT_MY_ADDRESS,
  MY_ADDRESS,
  EN_FILT_MULTICAST_ADDRESS,
  MULTICAST_ADDRESS,
  EN_FILT_BROADCAST_ADDRESS,
  BROADCAST_ADDRESS
};

/* LLP structure fitting */
PktStackLlpInit xStackLLPInit ={
  EN_AUTOACK,
  EN_PIGGYBACKING,
  MAX_RETRANSMISSIONS
};

/* GPIO structure fitting */
SGpioInit xGpioIRQ={
  SPIRIT_GPIO_3,
  SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_LP,
  SPIRIT_GPIO_DIG_OUT_IRQ
};


SpiritIrqs xIrqStatus;
uint8_t vectcRxBuff[96], cRxData;

#ifdef PIGGYBACKING
uint8_t vectcTxBuff[]={21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40};
#endif

/**
 *@}
 */

/**
 * @defgroup StackLlp_B_Private_FunctionPrototypes                      STack LLP B Private Function Prototypes
 * @{
 */

/**
 *@}
 */


/**
 * @defgroup StackLlp_B_Private_Functions                               STack LLP B Private Functions
 * @{
 */


/**
* @brief  This function handles External lines 15 to 10 interrupt request.
* @param  None
* @retval None
*/
#ifdef STM8L
INTERRUPT_HANDLER(M2S_GPIO_3_EXTI_IRQ_HANDLER,12)
#elif SDK
void M2S_GPIO_3_EXTI_IRQ_HANDLER(void)
#endif
{
  /* Check the flag status of EXTI line */
  if(EXTI_GetITStatus(M2S_GPIO_3_EXTI_LINE)){

	/* Get the IRQ status */
    SpiritIrqGetStatus(&xIrqStatus);

    /* Check the SPIRIT RX_DATA_DISC IRQ flag */
    if(xIrqStatus.IRQ_RX_DATA_DISC)
    {
      SpiritCmdStrobeRx()
      SdkEvalLedToggle(LED1);
    }

    /* Check the SPIRIT TX_DATA_SENT IRQ flag for the acknoledgement */
    if(xIrqStatus.IRQ_TX_DATA_SENT)
    {
//      SdkEvalLedToggle(LED1);

#ifdef PIGGYBACKING
      SpiritSpiWriteLinearFifo(20, vectcTxBuff);
      printf("Loaded piggybacking data: [");

      for(uint8_t i=0 ; i<20 ; i++)
        printf("%d ", vectcTxBuff[i]);
      printf("]\n\r");
#endif

      SpiritCmdStrobeRx();
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

      /*  A simple way to control if the received data sequence is correct (in this case LED2 will toggle) */
      {
        SpiritBool correct=S_TRUE;

        for(uint8_t i=0 ; i<cRxData ; i++)
          if(vectcRxBuff[i] != i+1)
            correct=S_FALSE;

        if(correct)
          SdkEvalLedToggle(LED2);
      }
#ifdef USE_VCOM
      /* print the received data */
      printf("B data received: [");

      for(uint8_t i=0 ; i<cRxData ; i++)
        printf("%d ", vectcRxBuff[i]);
      printf("]\r\n");

#endif
    }
    
    /* Clear the EXTI line flag */
    EXTI_ClearITPendingBit(M2S_GPIO_3_EXTI_LINE);
  }

}


/**
* @brief  System main function.
* @param  None
* @retval None
*/
void main (void)
{
  SDK_SYSTEM_CONFIG();

#ifdef USE_VCOM 
  SdkEvalVCInit(); 
  while(bDeviceState != CONFIGURED); 
#endif 

  /* Spirit ON */
  SpiritEnterShutdown();
  SpiritExitShutdown();

#ifdef STM8L
  SpiritRadioSetXtalFrequency(XTAL_FREQUENCY);        
  SpiritGeneralSetSpiritVersion(SPIRIT_VERSION);  
  
#elif SDK
  SpiritManagementIdentificationRFBoard();

  /* Initialize the signals to drive the range extender application board */
  SpiritManagementRangeExtInit();  
#endif

  /* Spirit IRQ config */
  SpiritGpioInit(&xGpioIRQ);

  /* Spirit Radio config */
  SpiritRadioInit(&xRadioInit);

  /* Spirit Radio set power */
  SpiritRadioSetPALeveldBm(0,POWER_DBM);
  SpiritRadioSetPALevelMaxIndex(0);
  
  /* Spirit Packet config */
  SpiritPktStackInit(&xStackInit);
  //SpiritPktStackAddressesInit(&xAddressInit);
  SpiritPktStackLlpInit(&xStackLLPInit);

  /* Spirit IRQs enable */
  SpiritIrqDeInit(&xIrqStatus);
  SpiritIrq(RX_DATA_DISC,S_ENABLE);
  SpiritIrq(RX_DATA_READY,S_ENABLE);
  SpiritIrq(TX_DATA_SENT , S_ENABLE);

  EXTI_ClearITPendingBit(M2S_GPIO_3_EXTI_LINE);
  SdkEvalM2SGpioInit(M2S_GPIO_3,M2S_MODE_EXTI_IN);

  /* payload length config */
//  SpiritPktStackSetPayloadLength(20);

  /* enable SQI check */
  SpiritQiSetSqiThreshold(SQI_TH_0);
  SpiritQiSqiCheck(S_ENABLE);

  /* rx timeout config */
  SpiritTimerSetRxTimeoutMs(1000.0);
  //SET_INFINITE_RX_TIMEOUT();
  SpiritTimerSetRxTimeoutStopCondition(SQI_ABOVE_THRESHOLD);

  /* IRQ registers blanking */
  SpiritIrqClearStatus();

#ifdef PIGGYBACKING
  SpiritSpiWriteLinearFifo(20, vectcTxBuff);
#endif

#ifdef STM8L
  enableInterrupts();
#elif SDK
  SdkEvalM2SGpioInterruptCmd(M2S_GPIO_3,0x0F,0x0F,ENABLE);
#endif
  
  /* RX command */
  SpiritCmdStrobeRx();

  while (1){
    SDK_ENTER_DFU();      
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
