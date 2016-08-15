/**
  ******************************************************************************
  * @file    main.c
  * @author  DiZiC Ltd.
  * @version V0.0.3
  * @date    24 November 2013
  * @brief   Main program body
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
#include <stdio.h>

// Standard STM32L1xxx driver headers
#include "misc.h"
#include "stm32l1xx_lcd.h"
#include "stm32l1xx_rcc.h"
#include "stm32l1xx_rtc.h"
#include "stm32l1xx_exti.h"
#include "stm32l1xx_pwr.h"
#include "stm32l1xx_syscfg.h"
#include "stm32l1xx_dbgmcu.h"

// Touch Sensing Driver headers
#include "stm32_tsl_api.h"

// Discovery board and specific drivers headers
#include "discovery_functions.h"
#include "stm32l_discovery_lcd.h"


/* Private variables ---------------------------------------------------------*/

static volatile uint32_t TimingDelay;
extern uint8_t t_bar[2];              // LCD bar graph: used for displaying active function
extern bool Auto_test;                // Auto_test activation flag: set by interrupt handler if user button is pressed for a few seconds
extern volatile bool KeyPressed;      // Set by User Button Interrupt Handler. Flag indicating that User Button was pressed
extern bool UserButton;               // Set by interrupt handler to indicate that user button is pressed 

/* Private function prototypes -----------------------------------------------*/

uint16_t UARTBuff_Rx[6]={0};
uint8_t  Step_Rx;
bool     DataReady_Rx;
bool     NoAckEvent;

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */


/*******************************************************************************/
/**
  * @brief main entry point.
  * @par Parameters None
  * @retval void None
  * @par Required preconditions: None
  */
int main(void)
{ 
  
 /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32l1xx_md.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32l1xx.c file
     */ 
  
  // Configure Clocks for Application needs
  RCC_Configuration();
  
  // Set internal voltage regulator to 1.8V 
  PWR_VoltageScalingConfig(PWR_VoltageScaling_Range1);
  
  // Wait Until the Voltage Regulator is ready
  while (PWR_GetFlagStatus(PWR_FLAG_VOS) != RESET) ;
  
  // Initialize GP I/O ports
  Init_GPIOs();
  
  // Initialize USART1 to communicate with U-shaped Board with Spirit1 Module
  USART1_Init();
  
  // Enable General Interrupts
  EnableInterrupts();	
  
  // Initialize Touch Sensing Configuration 
  TSL_Init();
  
  // Initializes the LCD Glass
  LCD_GLASS_Init();
        
  // Reset Keypressed flag used in Interrupt and Scroll Sentence services
  KeyPressed = FALSE;

  // Activate User Button
  UserButton = TRUE;
  
  // Display Initial, Welcome Message
  LCD_GLASS_ScrollSentence("       ** DiZIC Co., Ltd. for Electrolux POLAND **",1,SCROLL_SPEED);
  
  // Reset User Push Buttom KeyPress Flag
  KeyPressed = FALSE; 
  
  // Clear LCD bars
  BAR0_OFF;
  BAR1_OFF;
  BAR2_OFF;
  BAR3_OFF;	
  
  // RTC domain
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
  PWR_RTCAccessCmd(ENABLE);
  RCC_RTCCLKCmd(ENABLE);
  RTC_WakeUpCmd(DISABLE);
  
  RTC_ITConfig(RTC_IT_WUT , ENABLE);
  RTC_WakeUpClockConfig(RTC_WakeUpClock_RTCCLK_Div8); 
  RTC_SetWakeUpCounter(0xFFFE);
  RTC_WakeUpCmd(ENABLE);
  
  // End of Initialization and Intro
  // Start Application
  while (1)  // Enter endless loop
  { 
    if(KeyPressed)
    {
      // IRQ Handler just forced Spirit1 (on U-shaped board) to send message
      KeyPressed = FALSE; // Notify that pressing key is accepted and processed
    }
    // Get data packed from remote tag and display RSSI value on the LCD Glas
    if(DataReady_Rx)
    {
      // Received Packet from remote Tag. Display RSSI Value in dBm
      DataReady_Rx = FALSE;                  // Notify that received messsage is accepted and processed
      LCD_GLASS_DisplayStrDeci(UARTBuff_Rx); // Display RSSI level for message received from external Tag
    }   
  } 
} // End of main()		

/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval None
  */
void RCC_Configuration(void)
{  
  // Enable HSI Clock
  RCC_HSICmd(ENABLE);
  
  // !< Wait till HSI is ready
  while (RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET);

  // Set HSI as sys clock
  RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);
  
  // Set MSI clock range to ~4.194MHz
  RCC_MSIRangeConfig(RCC_MSIRange_6);
  
  // Enable the GPIOs clocks
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC| RCC_AHBPeriph_GPIOD| RCC_AHBPeriph_GPIOE| RCC_AHBPeriph_GPIOH, ENABLE);     

  // Enable comparator, LCD and PWR mngt clocks
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_COMP | RCC_APB1Periph_LCD | RCC_APB1Periph_PWR,ENABLE);
    
  // Enable ADC & SYSCFG clocks
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_SYSCFG | RCC_APB2Periph_USART1 , ENABLE);

  // Allow access to the RTC
  PWR_RTCAccessCmd(ENABLE);

  // Reset RTC Backup Domain
  RCC_RTCResetCmd(ENABLE);
  RCC_RTCResetCmd(DISABLE);

  // LSE Enable
  // RCC_LSEConfig(RCC_LSE_ON);
  RCC_LSICmd(ENABLE);
    
  // Wait until LSE is ready
  while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET);
  
  // RTC Clock Source Selection
  //RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE); // External
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);   // Internal
  
  // Enable the RTC
  RCC_RTCCLKCmd(ENABLE);   
  
  // Disable HSE - no external crystal
  RCC_HSEConfig(RCC_HSE_OFF);
  if(RCC_GetFlagStatus(RCC_FLAG_HSERDY) != RESET )
  {
    /* Stay in infinite loop if HSE is not disabled */
    while(1); 
  }
} // end of RCC_Configuration()

/**
  * @brief  To initialize the I/O ports
  * @caller main
  * @param None
  * @retval None
  */
void Init_GPIOs (void)
{
  // GPIO, EXTI and NVIC Init structure declaration
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  // Configure User Button pin as input
  GPIO_InitStructure.GPIO_Pin = USERBUTTON_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_Init(USERBUTTON_GPIO_PORT, &GPIO_InitStructure);

  // Select User Button pin as input source for EXTI Line
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource0);

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
    
  // Counter enable: GPIO set in output for enable the counter
  GPIO_InitStructure.GPIO_Pin = CTN_CNTEN_GPIO_PIN;
  GPIO_Init( CTN_GPIO_PORT, &GPIO_InitStructure);
  
  // To prepare to start counter
  GPIO_HIGH(CTN_GPIO_PORT,CTN_CNTEN_GPIO_PIN);
      
  // Configure Port A LCD Output pins as alternate function
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_8 | GPIO_Pin_9 |GPIO_Pin_10 |GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init( GPIOA, &GPIO_InitStructure);
  
  // Select LCD alternate function for Port A LCD Output pins
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1,GPIO_AF_LCD) ;
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2,GPIO_AF_LCD) ;
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3,GPIO_AF_LCD) ;
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource8,GPIO_AF_LCD) ;
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9,GPIO_AF_LCD) ;
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10,GPIO_AF_LCD) ;
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource15,GPIO_AF_LCD) ;  
  
  // Configure Port B LCD Output pins as Alternate Function
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_8 | GPIO_Pin_9 \
                                 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init( GPIOB, &GPIO_InitStructure);
  
  // Select LCD alternate function for Port B LCD Output pins
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource3,GPIO_AF_LCD) ;
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource4,GPIO_AF_LCD) ;
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource5,GPIO_AF_LCD) ;
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource8,GPIO_AF_LCD) ;
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource9,GPIO_AF_LCD) ;
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10,GPIO_AF_LCD) ;
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11,GPIO_AF_LCD) ;  
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource12,GPIO_AF_LCD) ;
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource13,GPIO_AF_LCD) ;   
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource14,GPIO_AF_LCD) ;
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource15,GPIO_AF_LCD) ;   
  
  // Configure Port C LCD Output pins as alternate function
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_6 \
                                 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |GPIO_Pin_11 ;                               
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init( GPIOC, &GPIO_InitStructure);  

  // Select LCD alternate function for Port B LCD Output pins
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource0,GPIO_AF_LCD) ;
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource1,GPIO_AF_LCD) ; 
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource2,GPIO_AF_LCD) ;
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource3,GPIO_AF_LCD) ;
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6,GPIO_AF_LCD) ;
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7,GPIO_AF_LCD) ;
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource8,GPIO_AF_LCD) ;
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource9,GPIO_AF_LCD) ;
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource10,GPIO_AF_LCD) ; 
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11,GPIO_AF_LCD) ;  
  
} // end of Init_GPIOs ()

// Initialize USATR1
void USART1_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART1_InitStruct;
    
  // To USART1 Tx and Rx lines: two LEDs are connected as follows:
  // USART1 Rx PB.07 = LED #3, Green
  // USART1 Tx PB.06 = LED #4, Blue
  // These two lines are initialized only once here
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
  GPIO_Init(GPIOB, &GPIO_InitStructure);
 
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6,GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7,GPIO_AF_USART1);  
    
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
  
  // Enable RXNE interrupt
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    
  // Enable USART1 global interrupt
  NVIC_EnableIRQ(USART1_IRQn);
    
} // end of  USART1_Init()


/* USART1 IRQ Handler
 *
 * DataReady_Rx become TRUE when complete RSSI Message from SPIRIT1 Module (assembled
 * on U-shaped board) is received, inclding "E" (End-of-Message) mark
 * UARTBuf_Rx[] contains received RSSI string
 */
void USART1_IRQHandler(void)
{
  // RXNE handler
  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
  {
    if((char)USART_ReceiveData(USART1)=='S')
    {
      Step_Rx=0;
    }  
    else if((char)USART_ReceiveData(USART1)=='E')
    {
      DataReady_Rx=TRUE;
    }  
    else 
    {
      UARTBuff_Rx[Step_Rx] = USART_ReceiveData(USART1);
      Step_Rx++;       
    }        
  }
} // end of USART1_IRQHandler()

/**
  * @brief  Retargets the C library printf function to the USART1.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  // Place your implementation of fputc her
  // e.g. write a character to the USART1
  USART_SendData(USART1, (uint8_t) ch);

  // Loop until the end of transmission
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
  {}

  return ch;
}  // end of PUTCHAR_PROTOTYPE

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in 10 ms units.
  * @retval None
  */
void Delay(uint32_t nTime)
{
  TimingDelay = nTime;
  while(TimingDelay != 0); 
} // end of Delay(nTime)

/**
  * @brief  Decrements the TimingDelay variable. In 10 ms units
  * @caller SysTick interrupt Handler 
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
    TimingDelay--;
} // end of TimingDelay_Decrement()

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* Infinite loop */
  while (1);
}

#endif  // USE_FULL_ASSERT

/**** (C) COPYRIGHT 2011 STMicroelectronics / DiZiC ****** END OF FILE *******/
