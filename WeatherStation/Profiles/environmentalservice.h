/**************************************************************************************************
  Filename:       environmentalservice.h
  Revised:        $Date: 2013-08-23 11:45:31 -0700 (Fri, 23 Aug 2013) $
  Revision:       $Revision: 35100 $

  Description:    Environmental Sensing service definitions and prototypes


  Copyright 2012-2013 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

#ifndef ENVIRONMENTALSERVICE_H
#define ENVIRONMENTALSERVICE_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "st_util.h"
  
/*********************************************************************
 * CONSTANTS
 */

// Service UUID
#define ENVIRONMENTAL_SERV_UUID         0x181A
  
#define BAROMETER_DATA_UUID             0x2A6D
#define BAROMETER_CONF_UUID             0xAA41
#define BAROMETER_PERI_UUID             0xAA42
  
#define TEMP_DATA_UUID                  0x2A6E
#define HUMI_DATA_UUID                  0x2A6F
#define TEMP_CONF_UUID                  0xAA43
#define TEMP_PERI_UUID                  0xAA44
  
// #define UV_DATA_UUID                    0x2A76
#define UV_DATA_UUID                    0xAA47
#define UV_CONF_UUID                    0xAA45
#define UV_PERI_UUID                    0xAA46

// Sensor Profile Services bit fields
#define ENVIRONMENTAL_SERVICE           0x00000008

// Length of sensor data in bytes
#define BAROMETER_DATA_LEN              4
#define SHT20_DATA_LEN                  4
#define TEMP_DATA_LEN                   2
#define HUMI_DATA_LEN                   2
#define SI1132_DATA_LEN                 6
#define ALS_DATA_LEN                    4
#define UV_DATA_LEN                     1
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */


  /*********************************************************************
 * API FUNCTIONS
 */

/*
 * Environmental_AddService- Initializes the Sensor GATT Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 */
extern bStatus_t Environmental_AddService( uint32 services );

/*
 * Environmental_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t Environmental_RegisterAppCBs( sensorCBs_t *appCallbacks );

/*
 * Environmental_SetParameter - Set a Sensor GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t Environmental_SetParameter( uint8 param, uint8 len, void *value );

/*
 * Environmental_GetParameter - Get a Sensor GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t Environmental_GetParameter( uint8 param, void *value );


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* ENVIRONMENTALSERVICE_H */
