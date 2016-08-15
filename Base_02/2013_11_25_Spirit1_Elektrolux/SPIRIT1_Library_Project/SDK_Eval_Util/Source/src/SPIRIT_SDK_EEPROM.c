/**
* @file    SPIRIT_SDK_EEPROM.c
* @author  High End Analog & RF BU - AMS / ART Team IMS-Systems Lab
* @version V3.0.1
* @date    November 19, 2012
* @brief   The service layer of the SPIRIT DK.
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
#include "SPIRIT_SDK_EEPROM.h"
#include "SDK_EVAL_Config.h"

/**
* @addtogroup SPIRIT_DK                   SPIRIT DK
* @{
*/


/**
* @defgroup SDK_EEPROM              SDK SPIRIT Management
* @{
*/


/** @defgroup SPI_Private_Functions
* @{
*/

static SPI_TypeDef* s_EepromSpiPortVersion[3] = {EEPROM_V2_SPI_PERIPH_NB, EEPROM_V3_SPI_PERIPH_NB, EEPROM_V3_SPI_PERIPH_NB};
GPIO_TypeDef* s_vectpxEepromSpiCsPortVersion[3] = {EEPROM_V2_SPI_PERIPH_CS_PORT, EEPROM_V3_SPI_PERIPH_CS_PORT, EEPROM_VD1_SPI_PERIPH_CS_PORT};
static const uint16_t s_vectpxEepromSpiCsPinVersion[3] = {EEPROM_V2_SPI_PERIPH_CS_PIN, EEPROM_V3_SPI_PERIPH_CS_PIN, EEPROM_VD1_SPI_PERIPH_CS_PIN};

static SPI_TypeDef* s_EepromSpiPort;
static GPIO_TypeDef** s_vectpxEepromSpiCsPort;
static uint16_t* s_vectnEepromSpiCsPin;

#define EepromSPICSLow()        {(*s_vectpxEepromSpiCsPort)->BSRRH = *s_vectnEepromSpiCsPin;}
#define EepromSPICSHigh()       {(*s_vectpxEepromSpiCsPort)->BSRRL = *s_vectnEepromSpiCsPin;}


void EepromWriteEnable(void);


/**
* @brief  Initializes the SPI for the EEPROM.
*         SPI, MISO, MOSI and SCLK are the same used for the SPIRIT1.
*         This function can be replaced by EepromCsPinInitialization if
*         SpiritSpiInit is called.
* @param  None
* @retval None
*/
void EepromSpiInitialization(void)
{ 
  SPI_InitTypeDef SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  s_EepromSpiPort = s_EepromSpiPortVersion[SdkEvalGetVersion()];
  s_vectnEepromSpiCsPin = (uint16_t *)&s_vectpxEepromSpiCsPinVersion[SdkEvalGetVersion()];
  s_vectpxEepromSpiCsPort = &s_vectpxEepromSpiCsPortVersion[SdkEvalGetVersion()];
    
  if(SdkEvalGetVersion() == SDK_EVAL_VERSION_2_1) {
    /* Enable SPI periph and SCLK, MOSI, MISO and CS GPIO clocks */
    RCC_APB2PeriphClockCmd(EEPROM_V2_SPI_PERIPH_RCC, ENABLE);
    RCC_AHBPeriphClockCmd(EEPROM_V2_SPI_PERIPH_MOSI_RCC | EEPROM_V2_SPI_PERIPH_MISO_RCC | EEPROM_V2_SPI_PERIPH_SCLK_RCC | EEPROM_V2_SPI_PERIPH_CS_RCC, ENABLE);
    
    /* Configure the AF for MOSI, MISO and SCLK GPIO pins*/
    GPIO_PinAFConfig(EEPROM_V2_SPI_PERIPH_MOSI_PORT, EEPROM_V2_SPI_PERIPH_MOSI_RCC_SOURCE, EEPROM_V2_SPI_PERIPH_MOSI_AF);
    GPIO_PinAFConfig(EEPROM_V2_SPI_PERIPH_MISO_PORT, EEPROM_V2_SPI_PERIPH_MISO_RCC_SOURCE, EEPROM_V2_SPI_PERIPH_MISO_AF);
    GPIO_PinAFConfig(EEPROM_V2_SPI_PERIPH_SCLK_PORT, EEPROM_V2_SPI_PERIPH_SCLK_RCC_SOURCE, EEPROM_V2_SPI_PERIPH_SCLK_AF);
    
    /* Configure SPI pins:SCLK, MISO and MOSI */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
    
    GPIO_InitStructure.GPIO_Pin = EEPROM_V2_SPI_PERIPH_SCLK_PIN;
    GPIO_Init(EEPROM_V2_SPI_PERIPH_SCLK_PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = EEPROM_V2_SPI_PERIPH_MISO_PIN;
    GPIO_Init(EEPROM_V2_SPI_PERIPH_MISO_PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = EEPROM_V2_SPI_PERIPH_MOSI_PIN;
    GPIO_Init(EEPROM_V2_SPI_PERIPH_MOSI_PORT, &GPIO_InitStructure);

  }
  else if(SdkEvalGetVersion() == SDK_EVAL_VERSION_3 || SdkEvalGetVersion() == SDK_EVAL_VERSION_D1) {      
    /* Enable SPI periph and SCLK, MOSI, MISO and CS GPIO clocks */
    RCC_APB1PeriphClockCmd(EEPROM_V3_SPI_PERIPH_RCC, ENABLE);  
    RCC_AHBPeriphClockCmd(EEPROM_V3_SPI_PERIPH_MOSI_RCC | EEPROM_V3_SPI_PERIPH_MISO_RCC | EEPROM_V3_SPI_PERIPH_SCLK_RCC | EEPROM_V3_SPI_PERIPH_CS_RCC, ENABLE);
    
    /* Configure the AF for MOSI, MISO and SCLK GPIO pins*/
    GPIO_PinAFConfig(EEPROM_V3_SPI_PERIPH_MOSI_PORT, EEPROM_V3_SPI_PERIPH_MOSI_RCC_SOURCE, EEPROM_V3_SPI_PERIPH_MOSI_AF);
    GPIO_PinAFConfig(EEPROM_V3_SPI_PERIPH_MISO_PORT, EEPROM_V3_SPI_PERIPH_MISO_RCC_SOURCE, EEPROM_V3_SPI_PERIPH_MISO_AF);
    GPIO_PinAFConfig(EEPROM_V3_SPI_PERIPH_SCLK_PORT, EEPROM_V3_SPI_PERIPH_SCLK_RCC_SOURCE, EEPROM_V3_SPI_PERIPH_SCLK_AF);
    
    /* Configure SPI pins:SCLK, MISO and MOSI */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
    
    GPIO_InitStructure.GPIO_Pin = EEPROM_V3_SPI_PERIPH_SCLK_PIN;
    GPIO_Init(EEPROM_V3_SPI_PERIPH_SCLK_PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = EEPROM_V3_SPI_PERIPH_MISO_PIN;
    GPIO_Init(EEPROM_V3_SPI_PERIPH_MISO_PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = EEPROM_V3_SPI_PERIPH_MOSI_PIN;
    GPIO_Init(EEPROM_V3_SPI_PERIPH_MOSI_PORT, &GPIO_InitStructure);
    
  }
  
  /* Configure SPI pin: CS */
  GPIO_InitStructure.GPIO_Pin = *s_vectnEepromSpiCsPin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_Init(*s_vectpxEepromSpiCsPort, &GPIO_InitStructure);
  
  /* Configure SPI peripheral */
  SPI_DeInit(s_EepromSpiPort);
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(s_EepromSpiPort, &SPI_InitStructure);
  
  SPI_Cmd(s_EepromSpiPort, ENABLE);
  
  EepromSPICSHigh();
  
}

/**
* @brief  Initialization of the CSn pin of the EEPROM.
*         This function is called internally by EepromCsPinInitialization.
* @param  None
* @retval None
*/
void EepromCsPinInitialization(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  s_EepromSpiPort = s_EepromSpiPortVersion[SdkEvalGetVersion()];
  s_vectnEepromSpiCsPin = (uint16_t *)&s_vectpxEepromSpiCsPinVersion[SdkEvalGetVersion()];
  s_vectpxEepromSpiCsPort = &s_vectpxEepromSpiCsPortVersion[SdkEvalGetVersion()];
  
  if(!SdkEvalGetVersion())
  {            
    /* Configure SPI pin: CS */
    GPIO_InitStructure.GPIO_Pin = EEPROM_V2_SPI_PERIPH_CS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
    GPIO_Init(EEPROM_V2_SPI_PERIPH_CS_PORT, &GPIO_InitStructure);
    
    /* Enable CS GPIO clock */
    RCC_AHBPeriphClockCmd(EEPROM_V2_SPI_PERIPH_CS_RCC, ENABLE);
  }
  else
  {
    /* Configure SPI pin: CS */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
    
    if(SdkEvalGetVersion()==SDK_EVAL_VERSION_3)
    {
      GPIO_InitStructure.GPIO_Pin = EEPROM_V3_SPI_PERIPH_CS_PIN;
      GPIO_Init(EEPROM_V3_SPI_PERIPH_CS_PORT, &GPIO_InitStructure);
      RCC_AHBPeriphClockCmd(EEPROM_V3_SPI_PERIPH_CS_RCC, ENABLE);
    }
    else
    {
      GPIO_InitStructure.GPIO_Pin = EEPROM_VD1_SPI_PERIPH_CS_PIN;
      GPIO_Init(EEPROM_VD1_SPI_PERIPH_CS_PORT, &GPIO_InitStructure);
      RCC_AHBPeriphClockCmd(EEPROM_VD1_SPI_PERIPH_CS_RCC, ENABLE);
    }
    
    
  }
  
  /* Put the SPI chip select high to end the transaction */
  EepromSPICSHigh();
}

/**
* @brief  Read the status register.
* @param  None
* @retval Status
*/
uint8_t EepromStatus(void)
{
  uint8_t status;
  uint8_t cmd[] = {EEPROM_CMD_RDSR, 0xFF};
  //  SPI_ENTER_CRITICAL();
  
  /* Put the SPI chip select low to start the transaction */
  EepromSPICSLow();
  
  //  for(volatile uint16_t i=0;i<CS_TO_SCLK_DELAY;i++);
  
  /* Send command */
  for(uint8_t k=0;k<2;k++)
  {
    while (SPI_GetFlagStatus(s_EepromSpiPort, SPI_FLAG_TXE) == RESET);
    SPI_SendData(s_EepromSpiPort, cmd[k]);
    while (SPI_GetFlagStatus(s_EepromSpiPort, SPI_FLAG_RXNE) == RESET);
    status = SPI_ReceiveData(s_EepromSpiPort);
  }
  
  while (SPI_GetFlagStatus(s_EepromSpiPort, SPI_FLAG_TXE) == RESET);
  
  /* Put the SPI chip select high to end the transaction */
  EepromSPICSHigh();
  
  //  SPI_EXIT_CRITICAL();
  return status;
}


/**
* @brief  Set the ERSR status bit.
* @param  None
* @retval Status
*/
uint8_t EepromSetSrwd(void)
{
  uint8_t status;
  uint8_t cmd[] = {EEPROM_CMD_WRSR, EEPROM_STATUS_SRWD};
  //  SPI_ENTER_CRITICAL();
  
  EepromWriteEnable();
  
  /* Put the SPI chip select low to start the transaction */
  EepromSPICSLow();
  
  //  for(volatile uint16_t i=0;i<CS_TO_SCLK_DELAY;i++);
  
  /* Send command */
  for(uint8_t k=0;k<2;k++)
  {
    while (SPI_GetFlagStatus(s_EepromSpiPort, SPI_FLAG_TXE) == RESET);
    SPI_SendData(s_EepromSpiPort, cmd[k]);
    while (SPI_GetFlagStatus(s_EepromSpiPort, SPI_FLAG_RXNE) == RESET);
    status = SPI_ReceiveData(s_EepromSpiPort);
  }
  
  while (SPI_GetFlagStatus(s_EepromSpiPort, SPI_FLAG_TXE) == RESET);
  
  /* Put the SPI chip select high to end the transaction */
  EepromSPICSHigh();
  
  //  SPI_EXIT_CRITICAL();
  return status;
}

/**
* @brief  Reset the ERSR status bit.
* @param  None
* @retval Status
*/
uint8_t EepromResetSrwd(void)
{
  uint8_t status;
  uint8_t cmd[] = {EEPROM_CMD_WRSR, EEPROM_STATUS_SRWD};
  //  SPI_ENTER_CRITICAL();
  
  EepromWriteEnable();
  
  /* Put the SPI chip select low to start the transaction */
  EepromSPICSLow();
  
  //  for(volatile uint16_t i=0;i<CS_TO_SCLK_DELAY;i++);
  
  /* Send command */
  for(uint8_t k=0;k<2;k++)
  {
    while (SPI_GetFlagStatus(s_EepromSpiPort, SPI_FLAG_TXE) == RESET);
    SPI_SendData(s_EepromSpiPort, cmd[k]);
    while (SPI_GetFlagStatus(s_EepromSpiPort, SPI_FLAG_RXNE) == RESET);
    status = SPI_ReceiveData(s_EepromSpiPort);
  }
  
  while (SPI_GetFlagStatus(s_EepromSpiPort, SPI_FLAG_TXE) == RESET);
  
  /* Put the SPI chip select high to end the transaction */
  EepromSPICSHigh();
  
  //  SPI_EXIT_CRITICAL();
  return status;
}

/**
* @brief  Set the internal WEL flag to allow write operation.
* @param  None
* @retval None
*/
void EepromWriteEnable(void)
{
  //  SPI_ENTER_CRITICAL();
  
  /* Put the SPI chip select low to start the transaction */
  EepromSPICSLow();
  
  //  for(volatile uint16_t i=0;i<CS_TO_SCLK_DELAY;i++);
  
  /* Send command */
  while (SPI_GetFlagStatus(s_EepromSpiPort, SPI_FLAG_TXE) == RESET);
  SPI_SendData(s_EepromSpiPort, EEPROM_CMD_WREN);
  while (SPI_GetFlagStatus(s_EepromSpiPort, SPI_FLAG_RXNE) == RESET);
  SPI_ReceiveData(s_EepromSpiPort);
  
  while (SPI_GetFlagStatus(s_EepromSpiPort, SPI_FLAG_TXE) == RESET);
  
  /* Put the SPI chip select high to end the transaction */
  EepromSPICSHigh();
  
  //  SPI_EXIT_CRITICAL();
  
}

/**
* @brief  Wait polling the SPI until the internal WIP flag is RESET.
*         The flag is SET when a write operation is running.
* @param  None
* @retval None
*/
void EepromWaitEndWriteOperation(void)
{
  uint8_t cmd = EEPROM_CMD_RDSR;
  uint8_t dummy = 0xFF;
  uint8_t status;
  //  SPI_ENTER_CRITICAL();
  
  /* Put the SPI chip select low to start the transaction */
  EepromSPICSLow();
  
  //  for(volatile uint16_t i=0;i<CS_TO_SCLK_DELAY;i++);
  
  /* Send command */
  while (SPI_GetFlagStatus(s_EepromSpiPort, SPI_FLAG_TXE) == RESET);
  SPI_SendData(s_EepromSpiPort, cmd);
  while (SPI_GetFlagStatus(s_EepromSpiPort, SPI_FLAG_RXNE) == RESET);
  SPI_ReceiveData(s_EepromSpiPort);
  
  /* Polling on status register */
  do{
    while (SPI_GetFlagStatus(s_EepromSpiPort, SPI_FLAG_TXE) == RESET);
    SPI_SendData(s_EepromSpiPort, dummy);
    while (SPI_GetFlagStatus(s_EepromSpiPort, SPI_FLAG_RXNE) == RESET);
    status = SPI_ReceiveData(s_EepromSpiPort);
  }while(status&EEPROM_STATUS_WIP);
  
  while (SPI_GetFlagStatus(s_EepromSpiPort, SPI_FLAG_TXE) == RESET);
  
  /* Put the SPI chip select high to end the transaction */
  EepromSPICSHigh();
  
  //  SPI_EXIT_CRITICAL();
  
}


/**
* @brief  Read a page of the EEPROM.
*         A page size is 32 bytes.
*         The pages are 256.
*         Page 0 address: 0x0000
*         Page 1 address: 0x0020
*         ...
*         Page 255 address: 0x1FE0
* @param  None
* @retval None
*/
void EepromRead(uint16_t nAddress, uint8_t cNbBytes, uint8_t* pcBuffer)
{
  uint8_t cmd[3];
  cmd[0] = EEPROM_CMD_READ;

  for(uint8_t k=0; k<2; k++) {
    cmd[k+1] = (uint8_t)(nAddress>>((1-k)*8));
  }  
  
  /* Wait the end of a previous write operation */
  EepromWaitEndWriteOperation();
  
  //  SPI_ENTER_CRITICAL();
  
  /* Put the SPI chip select low to start the transaction */
  EepromSPICSLow();
  
  //  for(volatile uint16_t i=0;i<CS_TO_SCLK_DELAY;i++);
  
  /* Write the header bytes and read the SPIRIT status bytes */
  for(uint8_t i=0; i<3; i++) {
    while (SPI_GetFlagStatus(s_EepromSpiPort, SPI_FLAG_TXE) == RESET);
    SPI_SendData(s_EepromSpiPort, cmd[i]);
    while (SPI_GetFlagStatus(s_EepromSpiPort, SPI_FLAG_RXNE) == RESET);
    SPI_ReceiveData(s_EepromSpiPort);
  }
  
  /* Read the registers according to the number of bytes */
  for(int index=0; index<cNbBytes; index++)
  {
    while (SPI_GetFlagStatus(s_EepromSpiPort, SPI_FLAG_TXE) == RESET);
    SPI_SendData(s_EepromSpiPort, 0xFF);
    while (SPI_GetFlagStatus(s_EepromSpiPort, SPI_FLAG_RXNE) == RESET);
    *pcBuffer = SPI_ReceiveData(s_EepromSpiPort);
    pcBuffer++;
  }
  
  while (SPI_GetFlagStatus(s_EepromSpiPort, SPI_FLAG_TXE) == RESET);
  
  /* Put the SPI chip select high to end the transaction */
  EepromSPICSHigh();
  
  //  SPI_EXIT_CRITICAL();
  
}


/**
* @brief  Write a page of the EEPROM.
*         A page size is 32 bytes.
*         The pages are 256.
*         Page 0 address: 0x0000
*         Page 1 address: 0x0020
*         ...
*         Page 255 address: 0x1FE0
*         It is allowed to write only a page for each operation. If the bytes
*         exceed the single page location, the other bytes are written at the 
*         beginning.
* @param  None
* @retval None
*/
void EepromWrite(uint16_t nAddress, uint8_t cNbBytes, uint8_t* pcBuffer)
{
  uint8_t cmd = EEPROM_CMD_WRITE;
  uint8_t address[2];
  
  /* Wait the end of a previous write operation */
  EepromWaitEndWriteOperation();
  
  /* SET the WREN flag */
  EepromWriteEnable();
  
  for(uint8_t k=0; k<2; k++) {
    address[k] = (uint8_t)(nAddress>>((1-k)*8));
  }
  
  //  SPI_ENTER_CRITICAL();
  
  /* Put the SPI chip select low to start the transaction */
  EepromSPICSLow();
  
  //  for(volatile uint16_t i=0;i<CS_TO_SCLK_DELAY;i++);
  
  /* Write the header bytes and read the SPIRIT status bytes */
  while (SPI_GetFlagStatus(s_EepromSpiPort, SPI_FLAG_TXE) == RESET);
  SPI_SendData(s_EepromSpiPort, cmd);
  while (SPI_GetFlagStatus(s_EepromSpiPort, SPI_FLAG_RXNE) == RESET);
  SPI_ReceiveData(s_EepromSpiPort);
  
  for(int i=0; i<2; i++)
  {
    while (SPI_GetFlagStatus(s_EepromSpiPort, SPI_FLAG_TXE) == RESET);
    SPI_SendData(s_EepromSpiPort, address[i]);
    while (SPI_GetFlagStatus(s_EepromSpiPort, SPI_FLAG_RXNE) == RESET);
    SPI_ReceiveData(s_EepromSpiPort);
  }
  
  /* Writes the registers according to the number of bytes */
  for(int index=0; index<cNbBytes; index++)
  {
    while (SPI_GetFlagStatus(s_EepromSpiPort, SPI_FLAG_TXE) == RESET);
    SPI_SendData(s_EepromSpiPort, pcBuffer[index]);   
    while (SPI_GetFlagStatus(s_EepromSpiPort, SPI_FLAG_RXNE) == RESET);
    SPI_ReceiveData(s_EepromSpiPort);
  }
  
  while (SPI_GetFlagStatus(s_EepromSpiPort, SPI_FLAG_TXE) == RESET);
  
  /* Put the SPI chip select high to end the transaction */
  EepromSPICSHigh();
  
  //  SPI_EXIT_CRITICAL();
  
}


/**
* @}
*/

/**
* @}
*/


/******************* (C) COPYRIGHT 2012 STMicroelectronics *****END OF FILE****/
