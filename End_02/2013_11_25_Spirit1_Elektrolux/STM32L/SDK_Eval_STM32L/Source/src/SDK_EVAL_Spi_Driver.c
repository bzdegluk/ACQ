/**
* @file    SDK_EVAL_V3_SPI_Driver.c
* @author  High End Analog & RF BU - AMS
* @version V3.0.0
* @date    April 01, 2013
* @brief   This file provides all the low level SPI API to access to STS2001 using a software watchdog timer to avoid stuck situation.
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
* <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
*
*/

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"
#include "MCU_Interface.h"
#include "SDK_EVAL_Config.h"

#ifdef FREERTOS
#include "FreeRTOS.h"
#include "semphr.h"

static xSemaphoreHandle xSpiMutex;

#endif

/** @addtogroup SDK_EVAL_STM32L
* @{
*/


/** @addtogroup SDK_EVAL_Spi                    SDK EVAL Spi
 * @brief SPI functions implementation.
 * @details This file implements the SPIRIT1 Library SPI interface functions. Please see the MCU_Interface.h file of Spirit1_Library for more details.
 * @{
 */


/** @defgroup SPI_Private_TypesDefinitions
* @{
*/


/**
* @}
*/


/** @defgroup SPI_Private_Defines
* @{
*/


#define CS_TO_SCLK_DELAY  0x0001
#define CLK_TO_CS_DELAY   0x0001

/** @defgroup SPI_Headers
* @{
*/

#define HEADER_WRITE_MASK     0x00 /*!< Write mask for header byte*/
#define HEADER_READ_MASK      0x01 /*!< Read mask for header byte*/
#define HEADER_ADDRESS_MASK   0x00 /*!< Address mask for header byte*/
#define HEADER_COMMAND_MASK   0x80 /*!< Command mask for header byte*/

#define LINEAR_FIFO_ADDRESS 0xFF  /*!< Linear FIFO address*/


/**
* @}
*/


/** @defgroup SDK_EVAL_Spi_Peripheral_Gpio
 * @{
 */

#define SDK_EVAL_V3_SPI_PERIPH_NB                  SPI1
#define SDK_EVAL_V3_SPI_PERIPH_RCC                 RCC_APB2Periph_SPI1
  

/* Defines for MOSI pin*/
#define SDK_EVAL_V3_SPI_PERIPH_MOSI_PORT           GPIOA
#define SDK_EVAL_V3_SPI_PERIPH_MOSI_PIN            GPIO_Pin_7
#define SDK_EVAL_V3_SPI_PERIPH_MOSI_AF             GPIO_AF_SPI1
#define SDK_EVAL_V3_SPI_PERIPH_MOSI_RCC            RCC_AHBPeriph_GPIOA
#define SDK_EVAL_V3_SPI_PERIPH_MOSI_RCC_SOURCE     GPIO_PinSource7

/* Defines for MISO pin */
#define SDK_EVAL_V3_SPI_PERIPH_MISO_PORT           GPIOA
#define SDK_EVAL_V3_SPI_PERIPH_MISO_PIN            GPIO_Pin_6
#define SDK_EVAL_V3_SPI_PERIPH_MISO_AF             GPIO_AF_SPI1
#define SDK_EVAL_V3_SPI_PERIPH_MISO_RCC            RCC_AHBPeriph_GPIOA
#define SDK_EVAL_V3_SPI_PERIPH_MISO_RCC_SOURCE     GPIO_PinSource6

/* Defines for SCLK pin */
#define SDK_EVAL_V3_SPI_PERIPH_SCLK_PORT           GPIOA
#define SDK_EVAL_V3_SPI_PERIPH_SCLK_PIN            GPIO_Pin_5
#define SDK_EVAL_V3_SPI_PERIPH_SCLK_AF             GPIO_AF_SPI1
#define SDK_EVAL_V3_SPI_PERIPH_SCLK_RCC            RCC_AHBPeriph_GPIOA
#define SDK_EVAL_V3_SPI_PERIPH_SCLK_RCC_SOURCE     GPIO_PinSource5

/* Defines for chip select pin */
#define SDK_EVAL_V3_SPI_PERIPH_CS_PORT             GPIOA
#define SDK_EVAL_V3_SPI_PERIPH_CS_PIN              GPIO_Pin_4
#define SDK_EVAL_V3_SPI_PERIPH_CS_RCC              RCC_AHBPeriph_GPIOA
#define SDK_EVAL_V3_SPI_PERIPH_CS_RCC_SOURCE       GPIO_PinSource4


#define SDK_EVAL_V2_SPI_PERIPH_NB                  SPI1
#define SDK_EVAL_V2_SPI_PERIPH_RCC                 RCC_APB2Periph_SPI1

/* Defines for MOSI pin*/
#define SDK_EVAL_V2_SPI_PERIPH_MOSI_PORT           GPIOE
#define SDK_EVAL_V2_SPI_PERIPH_MOSI_PIN            GPIO_Pin_15
#define SDK_EVAL_V2_SPI_PERIPH_MOSI_AF             GPIO_AF_SPI1
#define SDK_EVAL_V2_SPI_PERIPH_MOSI_RCC            RCC_AHBPeriph_GPIOE
#define SDK_EVAL_V2_SPI_PERIPH_MOSI_RCC_SOURCE     GPIO_PinSource15

/* Defines for MISO pin */
#define SDK_EVAL_V2_SPI_PERIPH_MISO_PORT           GPIOE
#define SDK_EVAL_V2_SPI_PERIPH_MISO_PIN            GPIO_Pin_14
#define SDK_EVAL_V2_SPI_PERIPH_MISO_AF             GPIO_AF_SPI1
#define SDK_EVAL_V2_SPI_PERIPH_MISO_RCC            RCC_AHBPeriph_GPIOE
#define SDK_EVAL_V2_SPI_PERIPH_MISO_RCC_SOURCE     GPIO_PinSource14

/* Defines for SCLK pin */
#define SDK_EVAL_V2_SPI_PERIPH_SCLK_PORT           GPIOE
#define SDK_EVAL_V2_SPI_PERIPH_SCLK_PIN            GPIO_Pin_13
#define SDK_EVAL_V2_SPI_PERIPH_SCLK_AF             GPIO_AF_SPI1
#define SDK_EVAL_V2_SPI_PERIPH_SCLK_RCC            RCC_AHBPeriph_GPIOE
#define SDK_EVAL_V2_SPI_PERIPH_SCLK_RCC_SOURCE     GPIO_PinSource13

/* Defines for chip select pin */
#define SDK_EVAL_V2_SPI_PERIPH_CS_PORT             GPIOE
#define SDK_EVAL_V2_SPI_PERIPH_CS_PIN              GPIO_Pin_12
#define SDK_EVAL_V2_SPI_PERIPH_CS_RCC              RCC_AHBPeriph_GPIOE
#define SDK_EVAL_V2_SPI_PERIPH_CS_RCC_SOURCE       GPIO_PinSource12

    

/**
 * @}
 */

/**
* @}
*/


/** @defgroup SPI_Private_Macros
* @{
*/
#define BUILT_HEADER(add_comm, w_r) (add_comm | w_r)  /*!< macro to build the header byte*/
#define WRITE_HEADER    BUILT_HEADER(HEADER_ADDRESS_MASK, HEADER_WRITE_MASK) /*!< macro to build the write header byte*/
#define READ_HEADER     BUILT_HEADER(HEADER_ADDRESS_MASK, HEADER_READ_MASK)  /*!< macro to build the read header byte*/
#define COMMAND_HEADER  BUILT_HEADER(HEADER_COMMAND_MASK, HEADER_WRITE_MASK) /*!< macro to build the command header byte*/


/**
* @}
*/



/** @defgroup SPI_Private_Variables
* @{
*/

static SPI_TypeDef* s_SpiPortVersion[3] = {SDK_EVAL_V2_SPI_PERIPH_NB, SDK_EVAL_V3_SPI_PERIPH_NB, SDK_EVAL_V3_SPI_PERIPH_NB};
GPIO_TypeDef* s_vectpxSpiCsPortVersion[3] = {SDK_EVAL_V2_SPI_PERIPH_CS_PORT, SDK_EVAL_V3_SPI_PERIPH_CS_PORT, SDK_EVAL_V3_SPI_PERIPH_CS_PORT};
static const uint16_t s_vectpxSpiCsPinVersion[3] = {SDK_EVAL_V2_SPI_PERIPH_CS_PIN, SDK_EVAL_V3_SPI_PERIPH_CS_PIN, SDK_EVAL_V3_SPI_PERIPH_CS_PIN};

static SPI_TypeDef* s_SpiPort;
static GPIO_TypeDef** s_vectpxSpiPort;
static uint16_t* s_vectnSpiPin;


/**
* @}
*/


/** @defgroup SPI_Private_FunctionPrototypes
* @{
*/




#ifdef FREERTOS
#define SPI_ENTER_CRITICAL()           xSemaphoreTake(xSpiMutex, portMAX_DELAY);
#define SPI_EXIT_CRITICAL()            xSemaphoreGive(xSpiMutex);
#else
#define SPI_ENTER_CRITICAL()           __disable_irq()
#define SPI_EXIT_CRITICAL()            __enable_irq()
#endif

/**
* @}
*/


/** @defgroup SPI_Private_Functions
* @{
*/

#define SdkEvalSPICSLow()        {(*s_vectpxSpiPort)->BSRRH = *s_vectnSpiPin;}
#define SdkEvalSPICSHigh()       {(*s_vectpxSpiPort)->BSRRL = *s_vectnSpiPin;}

/**
* @brief  Initializes the SPI for STS2001
* @param  None
* @retval None
*/
void SdkEvalSpiInit(void)
{
  SPI_InitTypeDef SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  s_SpiPort = s_SpiPortVersion[SdkEvalGetVersion()];
  s_vectnSpiPin = (uint16_t *)&s_vectpxSpiCsPinVersion[SdkEvalGetVersion()];
  s_vectpxSpiPort = &s_vectpxSpiCsPortVersion[SdkEvalGetVersion()];
    
  if(!SdkEvalGetVersion())
  {
    /* Enable SPI periph and SCLK, MOSI, MISO and CS GPIO clocks */
    RCC_APB2PeriphClockCmd(SDK_EVAL_V2_SPI_PERIPH_RCC, ENABLE);
    RCC_AHBPeriphClockCmd(SDK_EVAL_V2_SPI_PERIPH_MOSI_RCC | SDK_EVAL_V2_SPI_PERIPH_MISO_RCC | SDK_EVAL_V2_SPI_PERIPH_SCLK_RCC | SDK_EVAL_V2_SPI_PERIPH_CS_RCC, ENABLE);
    
    /* Configure the AF for MOSI, MISO and SCLK GPIO pins*/
    GPIO_PinAFConfig(SDK_EVAL_V2_SPI_PERIPH_MOSI_PORT, SDK_EVAL_V2_SPI_PERIPH_MOSI_RCC_SOURCE, SDK_EVAL_V2_SPI_PERIPH_MOSI_AF);
    GPIO_PinAFConfig(SDK_EVAL_V2_SPI_PERIPH_MISO_PORT, SDK_EVAL_V2_SPI_PERIPH_MISO_RCC_SOURCE, SDK_EVAL_V2_SPI_PERIPH_MISO_AF);
    GPIO_PinAFConfig(SDK_EVAL_V2_SPI_PERIPH_SCLK_PORT, SDK_EVAL_V2_SPI_PERIPH_SCLK_RCC_SOURCE, SDK_EVAL_V2_SPI_PERIPH_SCLK_AF);
    
    /* Configure SPI pins:SCLK, MISO and MOSI */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
    
    GPIO_InitStructure.GPIO_Pin = SDK_EVAL_V2_SPI_PERIPH_SCLK_PIN;
    GPIO_Init(SDK_EVAL_V2_SPI_PERIPH_SCLK_PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = SDK_EVAL_V2_SPI_PERIPH_MISO_PIN;
    GPIO_Init(SDK_EVAL_V2_SPI_PERIPH_MISO_PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = SDK_EVAL_V2_SPI_PERIPH_MOSI_PIN;
    GPIO_Init(SDK_EVAL_V2_SPI_PERIPH_MOSI_PORT, &GPIO_InitStructure);
    
    /* Configure SPI pin: CS */
    GPIO_InitStructure.GPIO_Pin = SDK_EVAL_V2_SPI_PERIPH_CS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
    GPIO_Init(SDK_EVAL_V2_SPI_PERIPH_CS_PORT, &GPIO_InitStructure);
    
  }
  else
  {
    /* Enable SPI periph and SCLK, MOSI, MISO and CS GPIO clocks */
    RCC_AHBPeriphClockCmd(SDK_EVAL_V3_SPI_PERIPH_MOSI_RCC | SDK_EVAL_V3_SPI_PERIPH_MISO_RCC | SDK_EVAL_V3_SPI_PERIPH_SCLK_RCC | SDK_EVAL_V3_SPI_PERIPH_CS_RCC, ENABLE);
    
    /* Configure the AF for MOSI, MISO and SCLK GPIO pins*/
    GPIO_PinAFConfig(SDK_EVAL_V3_SPI_PERIPH_MOSI_PORT, SDK_EVAL_V3_SPI_PERIPH_MOSI_RCC_SOURCE, SDK_EVAL_V3_SPI_PERIPH_MOSI_AF);
    GPIO_PinAFConfig(SDK_EVAL_V3_SPI_PERIPH_MISO_PORT, SDK_EVAL_V3_SPI_PERIPH_MISO_RCC_SOURCE, SDK_EVAL_V3_SPI_PERIPH_MISO_AF);
    GPIO_PinAFConfig(SDK_EVAL_V3_SPI_PERIPH_SCLK_PORT, SDK_EVAL_V3_SPI_PERIPH_SCLK_RCC_SOURCE, SDK_EVAL_V3_SPI_PERIPH_SCLK_AF);
    
    /* Configure SPI pins:SCLK, MISO and MOSI */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
    
    GPIO_InitStructure.GPIO_Pin = SDK_EVAL_V3_SPI_PERIPH_SCLK_PIN;
    GPIO_Init(SDK_EVAL_V3_SPI_PERIPH_SCLK_PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = SDK_EVAL_V3_SPI_PERIPH_MISO_PIN;
    GPIO_Init(SDK_EVAL_V3_SPI_PERIPH_MISO_PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = SDK_EVAL_V3_SPI_PERIPH_MOSI_PIN;
    GPIO_Init(SDK_EVAL_V3_SPI_PERIPH_MOSI_PORT, &GPIO_InitStructure);
    
    /* Configure SPI pin: CS */
    GPIO_InitStructure.GPIO_Pin = SDK_EVAL_V3_SPI_PERIPH_CS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
    GPIO_Init(SDK_EVAL_V3_SPI_PERIPH_CS_PORT, &GPIO_InitStructure);
    
    
    /* Enable SPI periph and SCLK, MOSI, MISO and CS GPIO clocks */
    RCC_APB2PeriphClockCmd(SDK_EVAL_V3_SPI_PERIPH_RCC, ENABLE);     

  }
  
  /* Configure SPI peripheral */
  SPI_DeInit(s_SpiPort);
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(s_SpiPort, &SPI_InitStructure);
  
  SPI_Cmd(s_SpiPort, ENABLE);
  
#ifdef FREERTOS  
  xSpiMutex = xSemaphoreCreateMutex();
  if (!xSpiMutex)
    /* Error in resource creation. */
    for (;;);
  xSemaphoreGive(xSpiMutex);  
#endif
  
  
  SdkEvalSPICSHigh();
}


/**
* @brief  Write single or multiple STS2001 register
* @param  cRegAddress: base register's address to be write
* @param  cNbBytes: number of registers and bytes to be write
* @param  pcBuffer: pointer to the buffer of values have to be written into registers
* @retval STS2001 status
*/
StatusBytes SdkEvalSpiWriteRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer)
{
  uint8_t header[2];
  uint16_t tmpstatus = 0x0000;
  StatusBytes *status=(StatusBytes *)&tmpstatus;
  
  /* Built the header bytes */
  header[0]=WRITE_HEADER;
  header[1]=cRegAddress;
  
  
  SPI_ENTER_CRITICAL();
  
  /* Puts the SPI chip select low to start the transaction */
  SdkEvalSPICSLow();
  
  for(volatile uint16_t i=0;i<CS_TO_SCLK_DELAY;i++);
  
  /* Writes the header bytes and read the STS2001 status bytes */
  for(int i=0; i<2; i++)
  {
    while (SPI_GetFlagStatus(s_SpiPort, SPI_FLAG_TXE) == RESET);
    SPI_SendData(s_SpiPort, header[i]);
    while (SPI_GetFlagStatus(s_SpiPort, SPI_FLAG_RXNE) == RESET);
    tmpstatus += ((uint16_t)(SPI_ReceiveData(s_SpiPort)))<<((1-i)*8);
  }
  
  /* Writes the registers according to the number of bytes */
  for(int index=0; index<cNbBytes; index++)
  {
    while (SPI_GetFlagStatus(s_SpiPort, SPI_FLAG_TXE) == RESET);
    SPI_SendData(s_SpiPort, pcBuffer[index]);   
    while (SPI_GetFlagStatus(s_SpiPort, SPI_FLAG_RXNE) == RESET);
    SPI_ReceiveData(s_SpiPort);
  }
  
  /* To be sure to don't rise the Chip Select before the end of last sending */
  while (SPI_GetFlagStatus(s_SpiPort, SPI_FLAG_TXE) == RESET);
  
  /* Puts the SPI chip select high to end the transaction */
  SdkEvalSPICSHigh();
  
  SPI_EXIT_CRITICAL();
  
  return *status;
  
}

/**
* @brief  Read single or multiple STS2001 register
* @param  cRegAddress: base register's address to be read
* @param  cNbBytes: number of registers and bytes to be read
* @param  pcBuffer: pointer to the buffer of registers' values read
* @retval STS2001 status
*/
StatusBytes SdkEvalSpiReadRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer)
{
  uint16_t tmpstatus = 0x0000;
  StatusBytes *status=(StatusBytes *)&tmpstatus;
  
  uint8_t header[2];
  uint8_t dummy=0xFF;
  
  /* Built the header bytes */
  header[0]=READ_HEADER;
  header[1]=cRegAddress;
  
  SPI_ENTER_CRITICAL();
  
  /* Put the SPI chip select low to start the transaction */
  SdkEvalSPICSLow();
  
  for(volatile uint16_t i=0;i<CS_TO_SCLK_DELAY;i++);
  
  /* Write the header bytes and read the STS2001 status bytes */
  for(int i=0; i<2; i++)
  {
    while (SPI_GetFlagStatus(s_SpiPort, SPI_FLAG_TXE) == RESET);
    SPI_SendData(s_SpiPort, header[i]);
    while (SPI_GetFlagStatus(s_SpiPort, SPI_FLAG_RXNE) == RESET);
    tmpstatus += ((uint16_t)(SPI_ReceiveData(s_SpiPort)))<<((1-i)*8);
  }
  
  /* Read the registers according to the number of bytes */
  for(int index=0; index<cNbBytes; index++)
  {
    while (SPI_GetFlagStatus(s_SpiPort, SPI_FLAG_TXE) == RESET);
    SPI_SendData(s_SpiPort, dummy);
    while (SPI_GetFlagStatus(s_SpiPort, SPI_FLAG_RXNE) == RESET);
    *pcBuffer = SPI_ReceiveData(s_SpiPort);
    pcBuffer++;
  }
  
  while (SPI_GetFlagStatus(s_SpiPort, SPI_FLAG_TXE) == RESET);
  
  /* Put the SPI chip select high to end the transaction */
  SdkEvalSPICSHigh();
  
  SPI_EXIT_CRITICAL();
  
  
  return *status;
  
}

/**
* @brief  Send a command
* @param  cCommandCode: command code to be sent
* @retval STS2001 status
*/
StatusBytes SdkEvalSpiCommandStrobes(uint8_t cCommandCode)
{
  uint8_t header[2];
  uint16_t tmpstatus = 0x0000;
  StatusBytes *status=(StatusBytes *)&tmpstatus;
  
  /* Built the header bytes */
  header[0]=COMMAND_HEADER;
  header[1]=cCommandCode;
  
  
  SPI_ENTER_CRITICAL();
  
  /* Puts the SPI chip select low to start the transaction */
  SdkEvalSPICSLow();
  
  for(volatile uint16_t i=0;i<CS_TO_SCLK_DELAY;i++);
  
  /* Writes the header bytes and read the STS2001 status bytes */
  for(int i=0; i<2; i++)
  {
    while (SPI_GetFlagStatus(s_SpiPort, SPI_FLAG_TXE) == RESET);
    SPI_SendData(s_SpiPort, header[i]);
    while (SPI_GetFlagStatus(s_SpiPort, SPI_FLAG_RXNE) == RESET);
    tmpstatus += ((uint16_t)(SPI_ReceiveData(s_SpiPort)))<<((1-i)*8);
  }
  
  while (SPI_GetFlagStatus(s_SpiPort, SPI_FLAG_TXE) == RESET);
  
  /* Puts the SPI chip select high to end the transaction */
  SdkEvalSPICSHigh();
  
  SPI_EXIT_CRITICAL();
  
  return *status;
  
}


/**
* @brief  Write data into TX FIFO
* @param  cNbBytes: number of bytes to be written into TX FIFO
* @param  pcBuffer: pointer to data to write
* @retval STS2001 status
*/
StatusBytes SdkEvalSpiWriteFifo(uint8_t cNbBytes, uint8_t* pcBuffer)
{
  uint16_t tmpstatus = 0x0000;
  StatusBytes *status=(StatusBytes *)&tmpstatus;

  uint8_t header[2];
  
  /* Built the header bytes */
  header[0]=WRITE_HEADER;
  header[1]=LINEAR_FIFO_ADDRESS;
  
  
  SPI_ENTER_CRITICAL();
  
  /* Put the SPI chip select low to start the transaction */
  SdkEvalSPICSLow();
  
  for(volatile uint16_t i=0;i<CS_TO_SCLK_DELAY;i++);
  
  /* Write the header bytes and read the STS2001 status bytes */
  for(int i=0; i<2; i++)
  {
    while (SPI_GetFlagStatus(s_SpiPort, SPI_FLAG_TXE) == RESET);
    SPI_SendData(s_SpiPort, header[i]);
    while (SPI_GetFlagStatus(s_SpiPort, SPI_FLAG_RXNE) == RESET);
    tmpstatus += ((uint16_t)(SPI_ReceiveData(s_SpiPort)))<<((1-i)*8);
  }
  
  /* Write the data into the FIFO according to the number of bytes */
  for(int index=0; index<cNbBytes; index++)
  {
    while (SPI_GetFlagStatus(s_SpiPort, SPI_FLAG_TXE) == RESET);
    SPI_SendData(s_SpiPort, pcBuffer[index]);
    while (SPI_GetFlagStatus(s_SpiPort, SPI_FLAG_RXNE) == RESET);
    SPI_ReceiveData(s_SpiPort);
  }
  
  /* To be sure to don't rise the Chip Select before the end of last sending */
  while (SPI_GetFlagStatus(s_SpiPort, SPI_FLAG_TXE) == RESET);
  
  /* Put the SPI chip select high to end the transaction */
  SdkEvalSPICSHigh();

  SPI_EXIT_CRITICAL();

  return *status;
  
}


/**
* @brief  Read data from RX FIFO
* @param  cNbBytes: number of bytes to read from RX FIFO
* @param  pcBuffer: pointer to data read from RX FIFO
* @retval STS2001 status
*/
StatusBytes SdkEvalSpiReadFifo(uint8_t cNbBytes, uint8_t* pcBuffer)
{
  uint16_t tmpstatus = 0x0000;
  StatusBytes *status=(StatusBytes *)&tmpstatus;
  
  uint8_t header[2];
  uint8_t dummy=0xFF;
  
  /* Built the header bytes */
  header[0]=READ_HEADER;
  header[1]=LINEAR_FIFO_ADDRESS;
  
  SPI_ENTER_CRITICAL();
  
  /* Put the SPI chip select low to start the transaction */
  SdkEvalSPICSLow();
  
  for(volatile uint16_t i=0;i<CS_TO_SCLK_DELAY;i++);
  
  /* Write the header bytes and read the STS2001 status bytes */
  for(int i=0; i<2; i++)
  {
    while (SPI_GetFlagStatus(s_SpiPort, SPI_FLAG_TXE) == RESET);
    SPI_SendData(s_SpiPort, header[i]);
    while (SPI_GetFlagStatus(s_SpiPort, SPI_FLAG_RXNE) == RESET);
    tmpstatus += ((uint16_t)(SPI_ReceiveData(s_SpiPort)))<<((1-i)*8);
  }
  
  /* Read the data from the FIFO according to the number of bytes */
  for(int index=0; index<cNbBytes; index++)
  {
    while (SPI_GetFlagStatus(s_SpiPort, SPI_FLAG_TXE) == RESET);
    SPI_SendData(s_SpiPort, dummy);
    while (SPI_GetFlagStatus(s_SpiPort, SPI_FLAG_RXNE) == RESET);
    *pcBuffer = SPI_ReceiveData(s_SpiPort);
    pcBuffer++;
  }
  
  /* To be sure to don't rise the Chip Select before the end of last sending */
  while (SPI_GetFlagStatus(s_SpiPort, SPI_FLAG_TXE) == RESET);
  
  /* Put the SPI chip select high to end the transaction */
  SdkEvalSPICSHigh();
  
  SPI_EXIT_CRITICAL();
  
  
  return *status;
  
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



/******************* (C) COPYRIGHT 2013 STMicroelectronics *****END OF FILE****/