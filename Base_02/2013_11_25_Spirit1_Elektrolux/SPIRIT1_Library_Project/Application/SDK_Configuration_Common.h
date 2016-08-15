/**
 * @file    SDK_Configuration_Common.h
 * @author  STMicroelectronics / DiZiC Ltd.
 * @version V1.0.1
 * @date    25 November 2013
 * @brief   Example of configuration and management of SPIRIT Basic packets.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SDK_PKT_TEST_COMMON_H
#define __SDK_PKT_TEST_COMMON_H

#define USE_HIGH_BAND

#ifdef __cplusplus
 extern "C" {
#endif

/**
 * @addtogroup SDK_Examples
 * @{
 */

/**
 * @defgroup SDK_BasicPktTest
 * @{
 */

/**
 * @defgroup SDK_BasicPktTest_Common                                    SDK Basic Pkt Test Common
 * @brief Radio and packet parameters definitions.
 * @details These parameters are in common between the device A and B.
 *
 * The user can change the configuration parameters editing these defines.
 * @{
 */

/**
 * @defgroup BasicPktTest_Common_Exported_Types                         Basic Pkt Test Common Exported Types
 * @{
 */

/**
 *@}
 */


/**
 * @defgroup BasicPktTest_Common_Exported_Constants                     Basic Pkt Test Common Exported Constants
 * @{
 */


//  Radio configuration parameters
#define XTAL_OFFSET_PPM             0

#ifdef USE_VERY_LOW_BAND
#define BASE_FREQUENCY              169.0e6
#endif

#ifdef USE_LOW_BAND
#define BASE_FREQUENCY              315.0e6
#endif

#ifdef USE_MIDDLE_BAND
#define BASE_FREQUENCY              433.0e6
#endif

#ifdef USE_HIGH_BAND
//#define BASE_FREQUENCY              868.3e6
// To Center spectrum inside European "g1" Band: 868.0 MHz ... 868.6 MHz
// fcarrier = 868.3 MHz - 1/2 Data Rate = 868.300000 MHz - 0.5*38400 Hz =
// fcarrier = 868.2808 MHz
//#define BASE_FREQUENCY          868280729  // Nearest value to 868280800
#define BASE_FREQUENCY          868280856  // Nearest value to 868280800. Best fit
#endif

#define CHANNEL_SPACE                20e3
#define CHANNEL_NUMBER                  0
#define MODULATION_SELECT             FSK
#define DATARATE                    38433  // Nearest value to 38400
#define FREQ_DEVIATION              19836  // Nearest value to 20e3
#define BANDWIDTH                  102115  // Nearest value to 100 kHz

#define POWER_DBM                   11.6

#define RSSI_THRESHOLD              0x20

// Rx Timeout in ms. Should be reasonambly greater than TX_DELAY
#define RX_TIMEOUT                3000.0    // Rx Timeout in ms
// After packet transmission is finihed, Delay (in ms) to initiate next transmission
#define TX_DELAY                  1000.0    // Time gap between successive transmissions in ms
   
//  Packet configuration parameters
#define PAYLOAD_LENGTH              96     // Length of payload in Bytes
#define PREAMBLE_LENGTH             PKT_PREAMBLE_LENGTH_04BYTES
#define SYNC_LENGTH                 PKT_SYNC_LENGTH_4BYTES
#define SYNC_WORD                   0x1A2635A8
#define LENGTH_TYPE                 PKT_LENGTH_FIX
#define LENGTH_WIDTH                7
#define CRC_MODE                    PKT_CRC_MODE_8BITS
#define CONTROL_LENGTH              PKT_CONTROL_LENGTH_0BYTES
#define EN_ADDRESS                  S_DISABLE
#define EN_FEC                      S_DISABLE
#define EN_WHITENING                S_ENABLE

//  Addresses configuration parameters
#define EN_FILT_MY_ADDRESS          S_DISABLE
#define MY_ADDRESS                  0x34
#define EN_FILT_MULTICAST_ADDRESS   S_DISABLE
#define MULTICAST_ADDRESS           0xEE
#define EN_FILT_BROADCAST_ADDRESS   S_DISABLE
#define BROADCAST_ADDRESS           0xFF
#define DESTINATION_ADDRESS         0x44

/**
 *@}
 */


/**
 * @defgroup BasicPktTest_Common_Exported_Macros                                Basic Pkt Test Common Exported Macros
 * @{
 */

/**
 *@}
 */


/**
 * @defgroup BasicPktTest_Common_Exported_Functions                             Basic Pkt Test Common Exported Functions
 * @{
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

/**
 *@}
 */

#ifdef __cplusplus
}
#endif

#endif

/**** (C) COPYRIGHT 2011-2013 STMicroelectronics / DiZiC ***** END OF FILE ****/
