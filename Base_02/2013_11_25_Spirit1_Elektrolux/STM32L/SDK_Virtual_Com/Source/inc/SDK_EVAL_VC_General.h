/**
 * @file    SDK_EVAL_VC_General.h
 * @author  MSH RF/ART Team IMS-Systems Lab
 * @version V1.0.0
 * @date    August 4, 2011
 * @brief   Header for SDK EVAL Virtual Com Setup & API.
 * @details
 *
 * This module provides API to use the Virtual Com.
 * An initialization function has to be called at the beginning of each program
 * in order to open the communication stream with the serial client.
 * One of the most used function is <i>@ref SdkEvalVCPrintf()</i>
 * based on an overriding of the standard C function <i>vsprintf()</i>.
 * It can be used to print a formatted string on the VCOM channel.
 * Moreover, there are other lower level API used to send a buffer or a char.
 * All the data to be sent are stored in a circular buffer of 512 bytes.
 * Every time the buffer has to be sent, the function <i>SdkEvalVCSendData()</i>
 * is required to be called.
 *
 * <b>Example:</b>
 * @code
 *
 *   uint8_t dummyNumber = 4;
 *
 *   ...
 *
 *   SdkEvalVCInit();
 *
 *   ...
 *
 *   SdkEvalVCPrintf("Hello world %d\n\r", dummyNumber);
 *   SdkEvalVCSendData();
 *
 *   ...
 *
 * @endcode
 *
 *
 * In order to receive data, two variables have been defined.
 * The first is the global input input buffer <i>@ref g_vectcVCRxBuffer</i> and
 * the global variable <i>@ref g_lVCNbBytesReceived</i>, which provides the number
 * of received bytes (it has to be reset or updated by the user every time the
 * received data are read).
 * The following example shows how to receive a string from a serial terminal (like
 * <i>hyperterminal</i> or <i>putty</i>) running on a PC.
 *
 * <b>Example:</b>
 * @code
 *  uint16_t n_from_vcom_data=0;
 *  uint8_t from_vcom[300];
 *
 *  while(1)
 *  {
 *    // control if the VCOM RX buffer is not empty
 *    if(g_lVCNbBytesReceived != 0){
 *      // copy each byte in an user-defined buffer
 *      for(uint16_t i=0 ; i<g_lVCNbBytesReceived ; i++)
 *        from_vcom[i+n_from_vcom_data]=g_vectcVCRxBuffer[i];
 *
 *      // compute the size of the received string
 *      n_from_vcom_data += g_lVCNbBytesReceived;
 *
 *      // free the VCOM RX buffer
 *      g_lVCNbBytesReceived=0;
 *
 *      // if '\r' has been received
 *      if(n_from_vcom_data && (char)from_vcom[n_from_vcom_data-1] == '\r'){
 *        // add the string termination chars
 *        from_vcom[n_from_vcom_data - 1] = '\0';
 *
 *        // exit the loop and return
 *        break;
 *      }
 *    }
 *  }
 *
 * @endcode
 *
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


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SDK_EVAL_VC_GENERAL_H
#define __SDK_EVAL_VC_GENERAL_H


/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"
#include "usb_lib.h"
#include "usb_conf.h"
#include "SDK_EVAL_VC_Desc.h"
#include "SDK_EVAL_VC_Istr.h"
#include "SDK_EVAL_VC_Prop.h"
#include "SDK_EVAL_VC_Pwr.h"



#ifdef __cplusplus
extern "C" {
#endif


/** @defgroup SDK_EVAL_Virtual_Com  SDK EVAL Virtual Com
 * @brief This module provides API to use the Virtual Com. In this way, the user can
 * communicate with the microcontroller using a common serial terminal.
 * @note The ST Virtual Com driver must be installed on the PC.
 * @note Since the STM32L microcontroller is not provided with an unambiguous part number,
 * the USB descriptor is generated by the routine <i>@ref SdkEvalVCResetRandomSerialNumber</i> .
 *
 * @{
 */


/** @defgroup SDK_EVAL_VC_General   SDK EVAL VC General
 * @brief Main functions for Virtual COM Port Device.
 * @details See the file <i>@ref SDK_EVAL_VC_General.h</i> for more details.
 * @{
 */


/** @defgroup SDK_EVAL_VC_General_Exported_Types    SDK EVAL VC General Exported Types
 * @{
 */
#define MASS_MEMORY_START     0x04002000
#define BULK_MAX_PACKET_SIZE  0x00000040

#define VC_TX_BUFFER_DATA_SIZE  1024 // 512 // 250    //512 //2048


/**
 * @brief For STM32L15xx devices it is possible to use the internal USB pullup
 *        controlled by register SYSCFG_PMC (refer to RM0038 reference manual for
 *        more details).
 *        It is also possible to use external pullup (and disable the internal pullup)
 *        by setting the define USB_USE_EXTERNAL_PULLUP in file SDK_EVAL_Virtual_Com.h
 *        and configuring the right pin to be used for the external pull up configuration.
 *        Uncomment the following define to use an external pull up instead of the
 *        integrated STM32L15xx internal pull up. In this case make sure to set up
 *        correctly the external required hardware and the GPIO defines below.
 */

//#define USB_USE_EXTERNAL_PULLUP

#if !defined(USB_USE_EXTERNAL_PULLUP)
#define STM32L15_USB_CONNECT                SYSCFG_USBPuCmd(ENABLE)
#define STM32L15_USB_DISCONNECT             SYSCFG_USBPuCmd(DISABLE)

#elif defined(USB_USE_EXTERNAL_PULLUP)

/* PA10 is chosen just as illustrating example, you should modify the defines
below according to your hardware configuration. */
#define USB_DISCONNECT                      GPIOA
#define USB_DISCONNECT_PIN                  GPIO_Pin_10
#define RCC_AHBPeriph_GPIO_DISCONNECT       RCC_AHBPeriph_GPIOA
#define STM32L15_USB_CONNECT                GPIO_SetBits(USB_DISCONNECT, USB_DISCONNECT_PIN)
#define STM32L15_USB_DISCONNECT             GPIO_ResetBits(USB_DISCONNECT, USB_DISCONNECT_PIN)
#endif /* USB_USE_EXTERNAL_PULLUP */

/**
 * @}
 */


/** @defgroup SDK_EVAL_VC_General_Exported_Constants    SDK EVAL VC General Exported Constants
 * @{
 */

/**
 * @}
 */


/** @defgroup SDK_EVAL_VC_General_Exported_Macros     SDK EVAL VC General Exported Macros
 * @{
 */

/**
 * @}
 */


/** @defgroup SDK_EVAL_VC_General_Exported_Functions    SDK EVAL VC General Exported Functions
 * @{
 */
void SdkEvalVCInit(void);
void SdkEvalVCEnterLowPowerMode(void);
void SdkEvalVCLeaveLowPowerMode(void);
void SdkEvalVCCableConfig(FunctionalState xNewState);
void SdkEvalVCPrintf(const char *str,...);
void SdkEvalVCWriteTxBuffer(uint8_t* pcDataBuffer, uint16_t nNbBytes);
void SdkEvalVCSendData(void);
void SdkEvalVCGetSerialNum(void);
void SdkEvalVCResetCounter(void);
void SdkEvalVCSetCounter(uint32_t lInIndex, uint32_t lOutIndex);

unsigned char __io_getcharNonBlocking(unsigned char *data);
void __io_putchar( char c );
int __io_getchar(void);
void __io_flush( void );

/**
 * @}
 */


/**
 * @}
 */


/**
 * @}
 */



#ifdef __cplusplus
}
#endif

#endif

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
