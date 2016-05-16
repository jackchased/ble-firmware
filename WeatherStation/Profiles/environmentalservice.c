/**************************************************************************************************
  Filename:       environmentalservice.c
  Revised:        $Date: 2013-08-23 11:45:31 -0700 (Fri, 23 Aug 2013) $
  Revision:       $Revision: 35100 $

  Description:    Environmental Sensing Service (ESS)


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

/*********************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "linkdb.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"

#include "environmentalservice.h"
#include "st_util.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/* Service configuration values */
#define SENSOR_SERVICE_UUID      ENVIRONMENTAL_SERV_UUID

#define SENSOR_DATA_UUID         BAROMETER_DATA_UUID
#define SENSOR_CONFIG_UUID       BAROMETER_CONF_UUID
#define SENSOR_PERIOD_UUID       BAROMETER_PERI_UUID
   
#define SENSOR_DATA1_UUID        TEMP_DATA_UUID
#define SENSOR_DATA2_UUID        HUMI_DATA_UUID
#define SENSOR_CONFIG1_UUID      TEMP_CONF_UUID
#define SENSOR_PERIOD1_UUID      TEMP_PERI_UUID

#define SENSOR_DATA3_UUID        UV_DATA_UUID
#define SENSOR_CONFIG2_UUID      UV_CONF_UUID
#define SENSOR_PERIOD2_UUID      UV_PERI_UUID

#define SENSOR_SERVICE           ENVIRONMENTAL_SERVICE
#define SENSOR_DATA_LEN          BAROMETER_DATA_LEN
#define SENSOR_DATA1_LEN         TEMP_DATA_LEN
#define SENSOR_DATA2_LEN         HUMI_DATA_LEN
#define SENSOR_DATA3_LEN         ALS_DATA_LEN

#define SENSOR_DATA_DESCR        "Barom. Data"
#define SENSOR_CONFIG_DESCR      "Barom. Conf."
#define SENSOR_PERIOD_DESCR      "Barom. Period"
   
#define SENSOR_DATA1_DESCR       "Temp. Data"
#define SENSOR_DATA2_DESCR       "Humi. Data"
#define SENSOR_CONFIG1_DESCR     "SHT20. Conf."
#define SENSOR_PERIOD1_DESCR     "SHT20. Period"
   
#define SENSOR_DATA3_DESCR       "UV Data"
#define SENSOR_CONFIG2_DESCR     "UV Conf."
#define SENSOR_PERIOD2_DESCR     "UV Period"
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Service UUID
static CONST uint8 sensorServiceUUID[TI_UUID_SIZE] =
{
  TI_UUID(SENSOR_SERVICE_UUID),
};

// Characteristic UUID: data
static CONST uint8 sensorDataUUID[TI_UUID_SIZE] =
{
  TI_UUID(SENSOR_DATA_UUID),
};

// Characteristic UUID: config
static CONST uint8 sensorCfgUUID[TI_UUID_SIZE] =
{
  TI_UUID(SENSOR_CONFIG_UUID),
};

// Characteristic UUID: period
static CONST uint8 sensorPeriodUUID[TI_UUID_SIZE] =
{
  TI_UUID(SENSOR_PERIOD_UUID),
};


// Characteristic UUID: data1
static CONST uint8 sensorData1UUID[TI_UUID_SIZE] =
{
  TI_UUID(SENSOR_DATA1_UUID),
};

// Characteristic UUID: data2
static CONST uint8 sensorData2UUID[TI_UUID_SIZE] =
{
  TI_UUID(SENSOR_DATA2_UUID),
};

// Characteristic UUID: config1
static CONST uint8 sensorCfg1UUID[TI_UUID_SIZE] =
{
  TI_UUID(SENSOR_CONFIG1_UUID),
};

// Characteristic UUID: period1
static CONST uint8 sensorPeriod1UUID[TI_UUID_SIZE] =
{
  TI_UUID(SENSOR_PERIOD1_UUID),
};



// Characteristic UUID: data3
static CONST uint8 sensorData3UUID[TI_UUID_SIZE] =
{
  TI_UUID(SENSOR_DATA3_UUID),
};

// Characteristic UUID: config2
static CONST uint8 sensorCfg2UUID[TI_UUID_SIZE] =
{
  TI_UUID(SENSOR_CONFIG2_UUID),
};

// Characteristic UUID: period2
static CONST uint8 sensorPeriod2UUID[TI_UUID_SIZE] =
{
  TI_UUID(SENSOR_PERIOD2_UUID),
};
/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static sensorCBs_t *sensor_AppCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// Profile Service attribute
static CONST gattAttrType_t sensorService = { TI_UUID_SIZE, sensorServiceUUID };

// Characteristic Value: data
static uint8 sensorData[SENSOR_DATA_LEN] = { 0, 0, 0, 0};

// Characteristic Properties: data
static uint8 sensorDataProps = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Characteristic Configuration: data
static gattCharCfg_t sensorDataConfig[GATT_MAX_NUM_CONN];

// Characteristic User Description: data
static uint8 sensorDataUserDescr[] = SENSOR_DATA_DESCR;

// Characteristic Properties: configuration
static uint8 sensorCfgProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic Value: configuration
static uint8 sensorCfg = 0;

// Characteristic User Description: configuration
static uint8 sensorCfgUserDescr[] = SENSOR_CONFIG_DESCR; 

// Characteristic Properties: period
static uint8 sensorPeriodProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic Value: period
static uint8 sensorPeriod = SENSOR_MIN_UPDATE_PERIOD / SENSOR_PERIOD_RESOLUTION;

// Characteristic User Description: period
static uint8 sensorPeriodUserDescr[] = SENSOR_PERIOD_DESCR;



// Characteristic Value: data1
static uint8 sensorData1[SENSOR_DATA1_LEN] = { 0, 0};

// Characteristic Properties: data1
static uint8 sensorData1Props = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Characteristic Configuration: data1
static gattCharCfg_t sensorData1Config[GATT_MAX_NUM_CONN];

// Characteristic User Description: data1
static uint8 sensorData1UserDescr[] = SENSOR_DATA1_DESCR;


// Characteristic Value: data2
static uint8 sensorData2[SENSOR_DATA2_LEN] = { 0, 0};

// Characteristic Properties: data2
static uint8 sensorData2Props = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Characteristic Configuration: data2
static gattCharCfg_t sensorData2Config[GATT_MAX_NUM_CONN];

// Characteristic User Description: data2
static uint8 sensorData2UserDescr[] = SENSOR_DATA2_DESCR;


// Characteristic Properties: configuration1
static uint8 sensorCfg1Props = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic Value: configuration1
static uint8 sensorCfg1 = 0;

// Characteristic User Description: configuration1
static uint8 sensorCfg1UserDescr[] = SENSOR_CONFIG1_DESCR; 

// Characteristic Properties: period1
static uint8 sensorPeriod1Props = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic Value: period1
static uint8 sensorPeriod1 = SENSOR_MIN_UPDATE_PERIOD / SENSOR_PERIOD_RESOLUTION;

// Characteristic User Description: period1
static uint8 sensorPeriod1UserDescr[] = SENSOR_PERIOD1_DESCR;



// Characteristic Value: data3
static uint8 sensorData3[SENSOR_DATA3_LEN] = { 0, 0, 0, 0};

// Characteristic Properties: data3
static uint8 sensorData3Props = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Characteristic Configuration: data3
static gattCharCfg_t sensorData3Config[GATT_MAX_NUM_CONN];

// Characteristic User Description: data3
static uint8 sensorData3UserDescr[] = SENSOR_DATA3_DESCR;


// Characteristic Properties: configuration2
static uint8 sensorCfg2Props = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic Value: configuration2
static uint8 sensorCfg2 = 0;

// Characteristic User Description: configuration2
static uint8 sensorCfg2UserDescr[] = SENSOR_CONFIG2_DESCR; 

// Characteristic Properties: period2
static uint8 sensorPeriod2Props = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic Value: period2
static uint8 sensorPeriod2 = SENSOR_MIN_UPDATE_PERIOD / SENSOR_PERIOD_RESOLUTION;

// Characteristic User Description: period2
static uint8 sensorPeriod2UserDescr[] = SENSOR_PERIOD2_DESCR;
/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t sensorAttrTable[] =
{
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&sensorService                   /* pValue */
  },
  
    // Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &sensorDataProps
    },
    
      // Characteristic Value "Data"
      {
        { TI_UUID_SIZE, sensorDataUUID },
        GATT_PERMIT_READ,
        0,
        sensorData
      },

      // Characteristic configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)sensorDataConfig
      },

      // Characteristic User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        sensorDataUserDescr
      },

    // Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &sensorCfgProps
    },

      // Characteristic Value "Configuration"
      {
        { TI_UUID_SIZE, sensorCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        &sensorCfg
      },

      // Characteristic User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        sensorCfgUserDescr
      },
     // Characteristic Declaration "Period"
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &sensorPeriodProps
    },

      // Characteristic Value "Period"
      {
        { TI_UUID_SIZE, sensorPeriodUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        &sensorPeriod
      },

      // Characteristic User Description "Period"
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        sensorPeriodUserDescr
      },



    // Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &sensorData1Props
    },	
      // Characteristic Value "Data1"
      {
        { TI_UUID_SIZE, sensorData1UUID },
        GATT_PERMIT_READ,
        0,
        sensorData1
      },

      // Characteristic configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)sensorData1Config
      },

      // Characteristic User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        sensorData1UserDescr
      },

    // Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &sensorData2Props
    },	
      // Characteristic Value "Data2"
      {
        { TI_UUID_SIZE, sensorData2UUID },
        GATT_PERMIT_READ,
        0,
        sensorData2
      },

      // Characteristic configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)sensorData2Config
      },

      // Characteristic User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        sensorData2UserDescr
      },
	  
	  
    // Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &sensorCfg1Props
    },

      // Characteristic Value "Configuration"
      {
        { TI_UUID_SIZE, sensorCfg1UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        &sensorCfg1
      },

      // Characteristic User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        sensorCfg1UserDescr
      },
     // Characteristic Declaration "Period"
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &sensorPeriod1Props
    },

      // Characteristic Value "Period"
      {
        { TI_UUID_SIZE, sensorPeriod1UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        &sensorPeriod1
      },

      // Characteristic User Description "Period"
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        sensorPeriod1UserDescr
      },
      
      
      
      // Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &sensorData3Props
    },	
      // Characteristic Value "Data3"
      {
        { TI_UUID_SIZE, sensorData3UUID },
        GATT_PERMIT_READ,
        0,
        sensorData3
      },

      // Characteristic configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)sensorData3Config
      },

      // Characteristic User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        sensorData3UserDescr
      },
	  
	  
    // Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &sensorCfg2Props
    },

      // Characteristic Value "Configuration 2"
      {
        { TI_UUID_SIZE, sensorCfg2UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        &sensorCfg2
      },

      // Characteristic User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        sensorCfg2UserDescr
      },
     // Characteristic Declaration "Period 2"
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &sensorPeriod2Props
    },

      // Characteristic Value "Period 2"
      {
        { TI_UUID_SIZE, sensorPeriod2UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        &sensorPeriod2
      },

      // Characteristic User Description "Period 2"
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        sensorPeriod2UserDescr
      },
};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 sensor_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen );
static bStatus_t sensor_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset );
static void sensor_HandleConnStatusCB( uint16 connHandle, uint8 changeType );


/*********************************************************************
 * PROFILE CALLBACKS
 */
// Simple Profile Service Callbacks
static CONST gattServiceCBs_t sensorCBs =
{
  sensor_ReadAttrCB,  // Read callback function pointer
  sensor_WriteAttrCB, // Write callback function pointer
  NULL                // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Environmental_AddService
 *
 * @brief   Initializes the Sensor Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t Environmental_AddService( uint32 services )
{
  uint8 status = SUCCESS;

  // Register with Link DB to receive link status change callback
  VOID linkDB_Register( sensor_HandleConnStatusCB );
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, sensorDataConfig );
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, sensorData1Config );
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, sensorData2Config );
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, sensorData3Config );
  
  if (services & SENSOR_SERVICE )
  {
       // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( sensorAttrTable,
                                          GATT_NUM_ATTRS( sensorAttrTable ),
                                          &sensorCBs );
  }

  return ( status );
}


/*********************************************************************
 * @fn      Environmental_RegisterAppCBs
 *
 * @brief   Registers the application callback function. Only call
 *          this function once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t Environmental_RegisterAppCBs( sensorCBs_t *appCallbacks )
{
  if ( sensor_AppCBs == NULL )
  {
    if ( appCallbacks != NULL )
    {
      sensor_AppCBs = appCallbacks;
    }

    return ( SUCCESS );
  }

  return ( bleAlreadyInRequestedMode );
}

/*********************************************************************
 * @fn      Environmental_SetParameter
 *
 * @brief   Set a parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to right
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t Environmental_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;

  switch ( param )
  {
    case SENSOR_DATA:
    if ( len == SENSOR_DATA_LEN )
    {
      VOID osal_memcpy( sensorData, value, SENSOR_DATA_LEN );
      // See if Notification has been enabled
      GATTServApp_ProcessCharCfg( sensorDataConfig, sensorData, FALSE,
                                 sensorAttrTable, GATT_NUM_ATTRS( sensorAttrTable ),
                                 INVALID_TASK_ID );
    }
    else
    {
      ret = bleInvalidRange;
    }
    break;

    case SENSOR_CONF:
      if ( len == sizeof ( uint8 ) )
      {
        sensorCfg = *((uint8*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case SENSOR_PERI:
      if ( len == sizeof ( uint8 ) )
      {
        sensorPeriod = *((uint8*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;


    case SENSOR_DATA1:
    if ( len == SENSOR_DATA1_LEN )
    {
      VOID osal_memcpy( sensorData1, value, SENSOR_DATA1_LEN );
      // See if Notification has been enabled
      GATTServApp_ProcessCharCfg( sensorData1Config, sensorData1, FALSE,
                                 sensorAttrTable, GATT_NUM_ATTRS( sensorAttrTable ),
                                 INVALID_TASK_ID );
    }
    else
    {
      ret = bleInvalidRange;
    }
    break;

    case SENSOR_DATA2:
    if ( len == SENSOR_DATA2_LEN )
    {
      VOID osal_memcpy( sensorData2, value, SENSOR_DATA2_LEN );
      // See if Notification has been enabled
      GATTServApp_ProcessCharCfg( sensorData2Config, sensorData2, FALSE,
                                 sensorAttrTable, GATT_NUM_ATTRS( sensorAttrTable ),
                                 INVALID_TASK_ID );
    }
    else
    {
      ret = bleInvalidRange;
    }
    break;
	
    case SENSOR_CONF1:
      if ( len == sizeof ( uint8 ) )
      {
        sensorCfg1 = *((uint8*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case SENSOR_PERI1:
      if ( len == sizeof ( uint8 ) )
      {
        sensorPeriod1 = *((uint8*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

      
  case SENSOR_DATA3:
    if ( len == SENSOR_DATA3_LEN )
    {
      VOID osal_memcpy( sensorData3, value, SENSOR_DATA3_LEN );
      // See if Notification has been enabled
      GATTServApp_ProcessCharCfg( sensorData3Config, sensorData3, FALSE,
                                 sensorAttrTable, GATT_NUM_ATTRS( sensorAttrTable ),
                                 INVALID_TASK_ID );
    }
    else
    {
      ret = bleInvalidRange;
    }
    break;
	
    case SENSOR_CONF2:
      if ( len == sizeof ( uint8 ) )
      {
        sensorCfg2 = *((uint8*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case SENSOR_PERI2:
      if ( len == sizeof ( uint8 ) )
      {
        sensorPeriod2 = *((uint8*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
      
      
    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return ( ret );
}

/*********************************************************************
 * @fn      Environmental_GetParameter
 *
 * @brief   Get a Sensor Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t Environmental_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;

  switch ( param )
  {
    case SENSOR_DATA:
      VOID osal_memcpy( value, sensorData, SENSOR_DATA_LEN );
      break;

    case SENSOR_CONF:
      *((uint8*)value) = sensorCfg;
      break;

    case SENSOR_PERI:
      *((uint8*)value) = sensorPeriod;
      break;


    case SENSOR_DATA1:
      VOID osal_memcpy( value, sensorData1, SENSOR_DATA1_LEN );
      break;
	  
    case SENSOR_DATA2:
      VOID osal_memcpy( value, sensorData2, SENSOR_DATA2_LEN );
      break;
	  
    case SENSOR_CONF1:
      *((uint8*)value) = sensorCfg1;
      break;

    case SENSOR_PERI1:
      *((uint8*)value) = sensorPeriod1;
      break;

      
    case SENSOR_DATA3:
      VOID osal_memcpy( value, sensorData3, SENSOR_DATA3_LEN );
      break;
	  
    case SENSOR_CONF2:
      *((uint8*)value) = sensorCfg2;
      break;

    case SENSOR_PERI2:
      *((uint8*)value) = sensorPeriod2;
      break;
      
      
    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return ( ret );
}

/*********************************************************************
 * @fn          sensor_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 *
 * @return      Success or Failure
 */
static uint8 sensor_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen )
{
  uint16 uuid;
  bStatus_t status = SUCCESS;

  // If attribute permissions require authorization to read, return error
  if ( gattPermitAuthorRead( pAttr->permissions ) )
  {
    // Insufficient authorization
    return ( ATT_ERR_INSUFFICIENT_AUTHOR );
  }

  // Make sure it's not a blob operation (no attributes in the profile are long)
  if ( offset > 0 )
  {
    return ( ATT_ERR_ATTR_NOT_LONG );
  }

  if (utilExtractUuid16(pAttr,&uuid) == FAILURE) {
    // Invalid handle
    *pLen = 0;
    return ATT_ERR_INVALID_HANDLE;
  }

  switch ( uuid )
  {
    // No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
    // gattserverapp handles those reads
    case SENSOR_DATA_UUID:
      *pLen = SENSOR_DATA_LEN;
      VOID osal_memcpy( pValue, pAttr->pValue, SENSOR_DATA_LEN );
      break;

    case SENSOR_CONFIG_UUID:
    case SENSOR_PERIOD_UUID:
      *pLen = 1;
      pValue[0] = *pAttr->pValue;
      break;


    case SENSOR_DATA1_UUID:
      *pLen = SENSOR_DATA1_LEN;
      VOID osal_memcpy( pValue, pAttr->pValue, SENSOR_DATA1_LEN );
      break;

    case SENSOR_DATA2_UUID:
      *pLen = SENSOR_DATA2_LEN;
      VOID osal_memcpy( pValue, pAttr->pValue, SENSOR_DATA2_LEN );
      break;
	  
    case SENSOR_CONFIG1_UUID:
    case SENSOR_PERIOD1_UUID:
      *pLen = 1;
      pValue[0] = *pAttr->pValue;
      break;
     
      
    case SENSOR_DATA3_UUID:
      *pLen = SENSOR_DATA3_LEN;
      VOID osal_memcpy( pValue, pAttr->pValue, SENSOR_DATA3_LEN );
      break;
	  
    case SENSOR_CONFIG2_UUID:
    case SENSOR_PERIOD2_UUID:
      *pLen = 1;
      pValue[0] = *pAttr->pValue;
      break;
      
    default:
      *pLen = 0;
      status = ATT_ERR_ATTR_NOT_FOUND;
      break;
    }

  return ( status );
}

/*********************************************************************
* @fn      sensor_WriteAttrCB
*
* @brief   Validate attribute data prior to a write operation
*
* @param   connHandle - connection message was received on
* @param   pAttr - pointer to attribute
* @param   pValue - pointer to data to be written
* @param   len - length of data
* @param   offset - offset of the first octet to be written
*
* @return  Success or Failure
*/
static bStatus_t sensor_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                           uint8 *pValue, uint8 len, uint16 offset )
{
  bStatus_t status = SUCCESS;
  uint8 notifyApp = 0xFF;
  uint16 uuid;

  // If attribute permissions require authorization to write, return error
  if ( gattPermitAuthorWrite( pAttr->permissions ) )
  {
    // Insufficient authorization
    return ( ATT_ERR_INSUFFICIENT_AUTHOR );
  }

  if (utilExtractUuid16(pAttr,&uuid) == FAILURE) {
    // Invalid handle
    return ATT_ERR_INVALID_HANDLE;
  }

  switch ( uuid )
  {
    case SENSOR_DATA_UUID:
    case SENSOR_DATA1_UUID:
    case SENSOR_DATA2_UUID:
    case SENSOR_DATA3_UUID:
      // Should not get here
      break;

    case SENSOR_CONFIG_UUID:
      // Validate the value
      // Make sure it's not a blob oper
      if ( offset == 0 )
      {
        if ( len != 1 )
        {
          status = ATT_ERR_INVALID_VALUE_SIZE;
        }
      }
      else
      {
        status = ATT_ERR_ATTR_NOT_LONG;
      }

      // Write the value
      if ( status == SUCCESS )
      {
        uint8 *pCurValue = (uint8 *)pAttr->pValue;

        *pCurValue = pValue[0];

        if( pAttr->pValue == &sensorCfg )
        {
          notifyApp = SENSOR_CONF;
        }
      }
      break;

    case SENSOR_PERIOD_UUID:
      // Validate the value
      // Make sure it's not a blob oper
      if ( offset == 0 )
      {
        if ( len != 1 )
        {
          status = ATT_ERR_INVALID_VALUE_SIZE;
        }
      }
      else
      {
        status = ATT_ERR_ATTR_NOT_LONG;
      }
      // Write the value
      if ( status == SUCCESS )
      {
        if (pValue[0]>=(SENSOR_MIN_UPDATE_PERIOD/SENSOR_PERIOD_RESOLUTION))
        {

          uint8 *pCurValue = (uint8 *)pAttr->pValue;
          *pCurValue = pValue[0];

          if( pAttr->pValue == &sensorPeriod )
          {
            notifyApp = SENSOR_PERI;
          }
        }
        else
        {
           status = ATT_ERR_INVALID_VALUE;
        }
      }
      break;


    case SENSOR_CONFIG1_UUID:
      // Validate the value
      // Make sure it's not a blob oper
      if ( offset == 0 )
      {
        if ( len != 1 )
        {
          status = ATT_ERR_INVALID_VALUE_SIZE;
        }
      }
      else
      {
        status = ATT_ERR_ATTR_NOT_LONG;
      }

      // Write the value
      if ( status == SUCCESS )
      {
        uint8 *pCurValue = (uint8 *)pAttr->pValue;

        *pCurValue = pValue[0];

        if( pAttr->pValue == &sensorCfg1 )
        {
          notifyApp = SENSOR_CONF1;
        }
      }
      break;

    case SENSOR_PERIOD1_UUID:
      // Validate the value
      // Make sure it's not a blob oper
      if ( offset == 0 )
      {
        if ( len != 1 )
        {
          status = ATT_ERR_INVALID_VALUE_SIZE;
        }
      }
      else
      {
        status = ATT_ERR_ATTR_NOT_LONG;
      }
      // Write the value
      if ( status == SUCCESS )
      {
        if (pValue[0]>=(SENSOR_MIN_UPDATE_PERIOD/SENSOR_PERIOD_RESOLUTION))
        {

          uint8 *pCurValue = (uint8 *)pAttr->pValue;
          *pCurValue = pValue[0];

          if( pAttr->pValue == &sensorPeriod1 )
          {
            notifyApp = SENSOR_PERI1;
          }
        }
        else
        {
           status = ATT_ERR_INVALID_VALUE;
        }
      }
      break;

      
    case SENSOR_CONFIG2_UUID:
      // Validate the value
      // Make sure it's not a blob oper
      if ( offset == 0 )
      {
        if ( len != 1 )
        {
          status = ATT_ERR_INVALID_VALUE_SIZE;
        }
      }
      else
      {
        status = ATT_ERR_ATTR_NOT_LONG;
      }

      // Write the value
      if ( status == SUCCESS )
      {
        uint8 *pCurValue = (uint8 *)pAttr->pValue;

        *pCurValue = pValue[0];

        if( pAttr->pValue == &sensorCfg2 )
        {
          notifyApp = SENSOR_CONF2;
        }
      }
      break;

    case SENSOR_PERIOD2_UUID:
      // Validate the value
      // Make sure it's not a blob oper
      if ( offset == 0 )
      {
        if ( len != 1 )
        {
          status = ATT_ERR_INVALID_VALUE_SIZE;
        }
      }
      else
      {
        status = ATT_ERR_ATTR_NOT_LONG;
      }
      // Write the value
      if ( status == SUCCESS )
      {
        if (pValue[0]>=(SENSOR_MIN_UPDATE_PERIOD/SENSOR_PERIOD_RESOLUTION))
        {

          uint8 *pCurValue = (uint8 *)pAttr->pValue;
          *pCurValue = pValue[0];

          if( pAttr->pValue == &sensorPeriod2 )
          {
            notifyApp = SENSOR_PERI2;
          }
        }
        else
        {
           status = ATT_ERR_INVALID_VALUE;
        }
      }
      break;
      
      
      
    case GATT_CLIENT_CHAR_CFG_UUID:
      status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                              offset, GATT_CLIENT_CFG_NOTIFY );
      break;

    default:
      // Should never get here!
      status = ATT_ERR_ATTR_NOT_FOUND;
      break;
  }

  // If a charactersitic value changed then callback function to notify application of change
  if ( (notifyApp != 0xFF ) && sensor_AppCBs && sensor_AppCBs->pfnSensorChange )
  {
    sensor_AppCBs->pfnSensorChange( notifyApp );
  }

  return ( status );
}

/*********************************************************************
 * @fn          sensor_HandleConnStatusCB
 *
 * @brief       Sensor Profile link status change handler function.
 *
 * @param       connHandle - connection handle
 * @param       changeType - type of change
 *
 * @return      none
 */
static void sensor_HandleConnStatusCB( uint16 connHandle, uint8 changeType )
{
  // Make sure this is not loopback connection
  if ( connHandle != LOOPBACK_CONNHANDLE )
  {
    // Reset Client Char Config if connection has dropped
    if ( ( changeType == LINKDB_STATUS_UPDATE_REMOVED )      ||
         ( ( changeType == LINKDB_STATUS_UPDATE_STATEFLAGS ) &&
           ( !linkDB_Up( connHandle ) ) ) )
    {
      GATTServApp_InitCharCfg( connHandle, sensorDataConfig );
      GATTServApp_InitCharCfg( connHandle, sensorData1Config );
      GATTServApp_InitCharCfg( connHandle, sensorData2Config );
      GATTServApp_InitCharCfg( connHandle, sensorData3Config );
    }
    if ( changeType == LINKDB_STATUS_UPDATE_NEW )
    {
      GATTServApp_WriteCharCfg(connHandle,sensorDataConfig,GATT_CLIENT_CFG_NOTIFY);
      GATTServApp_WriteCharCfg(connHandle,sensorData1Config,GATT_CLIENT_CFG_NOTIFY);
      GATTServApp_WriteCharCfg(connHandle,sensorData2Config,GATT_CLIENT_CFG_NOTIFY);
      GATTServApp_WriteCharCfg(connHandle,sensorData3Config,GATT_CLIENT_CFG_NOTIFY);
    }
  }
}


/*********************************************************************
*********************************************************************/
