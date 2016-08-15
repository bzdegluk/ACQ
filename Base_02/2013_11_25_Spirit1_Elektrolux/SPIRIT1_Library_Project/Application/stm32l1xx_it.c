/**
  ******************************************************************************
  * @file    stm32l1xx_it.c
  * @author  MCD Application Team
  * @version V3.3.0
  * @date    21-March-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_it.h"

extern uint16_t counter;
extern uint8_t firstinterrupt;
extern uint8_t command_buffer[60];
extern int command_size;
extern int command_index;
extern int transfer;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval : None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval : None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval : None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval : None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval : None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval : None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval : None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval : None
  */
void PendSV_Handler(void)
{
}



/*******************************************************************************
* Function Name  : USB_LP_IRQHandler
* Description    : This function handles USB Low Priority interrupts  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USB_LP_IRQHandler(void)
{
 // SdkEvalVCIntServRoutine();  /// bzdegluk

}



/******************************************************************************/
/*                 STM32L15x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32l1xx_lp.s).                                            */
/******************************************************************************/


void USART1_IRQHandler(void)
{

if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)      //received data available in buffor
    {
      command_buffer[command_index] = (USART1->DR & (0x0FF));
      command_index++;
      
      if(command_buffer[0] == 0xC6)
      {      
          if(command_index == 4)
            {
              command_size = command_buffer[3];
            }
      }else
      {
        command_index = 0;
      }
      
      if(command_index == (command_size-1))
      {
       transfer = 1;
      }
      
   /*   if (ACQ_state == ACQ_waiting_answer)
      {
        data_received[count_rec] = (USART1->DR & (0x0FF));
        count_rec++;
        if (count_rec > (size_index - 1))               //check if complet answer received
        {
          count_rec = 0;                                // reset the counter for data received
          ACQ_state = ACQ_waiting_next_window;          // start wait for end of the transmision window
          data_ready_for_parsing = 0;
        }
      } */
    }
if (USART_GetITStatus(USART1, USART_IT_TXE) !=RESET)         //transmission complete
    {
  /*    if (frame_ind == 0)
        {      
      ACQ_state = ACQ_waiting_answer;
      ACQ_wait_answ_timer = 1000;
      // USART_ClearITPendingBit(USART1, USART_IT_TC);
      USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
      }
      else
      {
        USART1->DR = (frame_send_read[frame_ind] & (uint16_t)0x01FF);
        frame_ind++;
        if (frame_ind > (frame_size-1))
          frame_ind = 0;
        
      }   */  
    }
  
 
  
 // USART_ITConfig(USART1, USART_IT_TC, ENABLE);
  
  
}


/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval : None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
