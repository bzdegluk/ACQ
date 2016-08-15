 
#include "stm32l1xx_gpio.h"


void USART_Conf(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
   /* USART Tx as alternate function push-pull */
   GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9; /* PA.09 USART1.TX */
   GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
   GPIO_Init(GPIOA, &GPIO_InitStructure);

   /* USART Rx as input floating */
   GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10; /* PA.10 USART1.RX */
   GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
   GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  
	USART_InitTypeDef USART_InitStructure;

	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);

	 /* Enable the EVAL_COM1 Transmoit interrupt: this interrupt is generated when the
	     EVAL_COM1 transmit data register is empty */
//	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);

	  /* Enable the EVAL_COM1 Receive interrupt: this interrupt is generated when the
	     EVAL_COM1 receive data register is not empty */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	USART_Cmd(USART1, ENABLE);

}

