/**************************************************************************************************
  Filename:       NineAxisSensor.c
  Revised:        $Date: 2010-08-06 08:56:11 -0700 (Fri, 06 Aug 2010) $
  Revision:       $Revision: 23333 $

  Description:    This file contains the NineAxisSensor sample application
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.

  Copyright 2010 - 2013 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_i2c.h"

#include "gatt.h"
#include "hci.h"

#include "gapgattserver.h"
#include "gattservapp.h"

#include "peripheral.h"

#include "gapbondmgr.h"

#if defined FEATURE_OAD
  #include "oad.h"
  #include "oad_target.h"
#endif

// Services
#include "st_util.h"
#include "devinfoservice.h"
#include "dinservice.h"
#include "ainservice.h"
#include "environmentalservice.h"
#include "gyroservice.h"

// Sensor drivers
#include "NineAxisSensor.h"
#include "hal_sensor.h"

#include "hal_din.h"
#include "hal_mag.h"
#include "hal_acc_gyro.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// How often to perform periodic event
#define AIN_DEFAULT_PERIOD                    2000
#define MAG_DEFAULT_PERIOD                    2000
#define GYRO_DEFAULT_PERIOD                   2000

// What is the advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely

#if defined ( CC2540_MINIDK )
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_LIMITED
#else
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL
#endif  // defined ( CC2540_MINIDK )

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     80

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         8

// Company Identifier: Texas Instruments Inc. (13)
#define TI_COMPANY_ID                         0x000D

#define INVALID_CONNHANDLE                    0xFFFF

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8 NineAxisSensor_TaskID;   // Task ID for internal task/event processing

static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
  // complete name
  0x0F,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  0x4E,   // 'N'
  0x69,   // 'i'
  0x6E,   // 'n'
  0x65,   // 'e'
  0x41,   // 'A'
  0x78,   // 'x'
  0x69,   // 'i'
  0x73,   // 's'
  0x53,   // 'S'
  0x65,   // 'e'
  0x6E,   // 'n'
  0x73,   // 's'
  0x6F,   // 'o'
  0x72,   // 'r'

  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),   // 100ms
  HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
  LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),   // 1s
  HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8 advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED
};

// GAP GATT Attributes
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "Nine Axis Sensor";

// Sensor State Variables
static bool   ainEnabled = FALSE;
static bool   magEnabled = FALSE;
static bool   gyroEnabled = FALSE;

static uint16 sensorAinPeriod = AIN_DEFAULT_PERIOD;
static uint16 sensorMagPeriod = MAG_DEFAULT_PERIOD;
static uint16 sensorGyrPeriod = GYRO_DEFAULT_PERIOD;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void NineAxisSensor_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );

static uint16 readAdcData( uint8 channel );
static void readMagData( void );
static void readGyroData( void );

static void ainChangeCB( uint8 paramID );
static void environmentalChangeCB( uint8 paramID );
static void gyroChangeCB( uint8 paramID );

static void resetSensorSetup( void );
static void resetCharacteristicValue( uint16 servID, uint8 paramID, uint8 value, uint8 paramLen );
static void resetCharacteristicValues( void );

#if defined( CC2540_MINIDK )
static void NineAxisSensor_HandleKeys( uint8 shift, uint8 keys );
#endif

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t NineAxisSensor_PeripheralCBs =
{
  peripheralStateNotificationCB,  // Profile State Change Callbacks
  NULL                            // When a valid RSSI is read from controller (not used by application)
};

// GAP Bond Manager Callbacks
static gapBondCBs_t NineAxisSensor_BondMgrCBs =
{
  NULL,                     // Passcode callback (not used by application)
  NULL                      // Pairing / Bonding state Callback (not used by application)
};

// Simple GATT Profile Callbacks
static sensorCBs_t NineAxisSensor_AinCBs =
{
  ainChangeCB,              // Characteristic value change callback
};

static sensorCBs_t NineAxisSensor_EnvironmentalCBs =
{
  environmentalChangeCB,    // Characteristic value change callback
};

static sensorCBs_t NineAxisSensor_GyroCBs =
{
  gyroChangeCB,             // Characteristic value change callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      NineAxisSensor_Init
 *
 * @brief   Initialization function for the Simple BLE Peripheral App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void NineAxisSensor_Init( uint8 task_id )
{
  NineAxisSensor_TaskID = task_id;

  // Setup the GAP
  VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );
  
  // Setup the GAP Peripheral Role Profile
  {
    #if defined( CC2540_MINIDK )
      // For the CC2540DK-MINI keyfob, device doesn't start advertising until button is pressed
      uint8 initial_advertising_enable = FALSE;
    #else
      // For other hardware platforms, device starts advertising upon initialization
      uint8 initial_advertising_enable = TRUE;
    #endif

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime = 0;
    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );

    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );

    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
  }

  // Set the GAP Characteristics
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );

  // Set advertising interval
  {
    uint16 advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
  }

  // Setup the GAP Bond Manager
  {
    uint32 passkey = 0; // passkey "000000"
    uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8 mitm = TRUE;
    uint8 ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8 bonding = TRUE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
  }

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );            // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
  DevInfo_AddService();                           // Device Information Service
  Din_AddService( GATT_ALL_SERVICES );            // Digital Input Service
  Ain_AddService( GATT_ALL_SERVICES );            // Analog Input Service
  Environmental_AddService( GATT_ALL_SERVICES );  // Environmental Service
  Gyro_AddService( GATT_ALL_SERVICES );           // Gyro Service
  
#if defined FEATURE_OAD
  VOID OADTarget_AddService();                    // OAD Profile
#endif
  
  // Setup the Seensor Profile Characteristic Values
  resetCharacteristicValues();

  // Initialise sensor drivers
  HalDinInit();
  HalMagInit();
  HalAccGyroInit();

  // Register callback with SimpleGATTprofile
  VOID Ain_RegisterAppCBs( &NineAxisSensor_AinCBs );
  VOID Environmental_RegisterAppCBs( &NineAxisSensor_EnvironmentalCBs );
  VOID Gyro_RegisterAppCBs( &NineAxisSensor_GyroCBs );
  
  // Enable clock divide on halt
  // This reduces active current while radio is active and CC254x MCU
  // is halted
  // HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT );

  // Setup a delayed profile startup
  osal_set_event( NineAxisSensor_TaskID, SBP_START_DEVICE_EVT );
  HalLedSet (HAL_LED_1, HAL_LED_MODE_ON);
}

/*********************************************************************
 * @fn      NineAxisSensor_ProcessEvent
 *
 * @brief   Simple BLE Peripheral Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 NineAxisSensor_ProcessEvent( uint8 task_id, uint16 events )
{

  VOID task_id; // OSAL required parameter that isn't used in this function

  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( NineAxisSensor_TaskID )) != NULL )
    {
      NineAxisSensor_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & SBP_START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPRole_StartDevice( &NineAxisSensor_PeripheralCBs );

    // Start Bond Manager
    VOID GAPBondMgr_Register( &NineAxisSensor_BondMgrCBs );

    return ( events ^ SBP_START_DEVICE_EVT );
  }

  //////////////////////////
  //         AIN          //
  //////////////////////////
  if ( events & SBP_READ_AIN_EVT )
  {
    if(ainEnabled)
    {
      uint16 ainData;
      ainData = readAdcData(HAL_ADC_CHANNEL_6);
      Ain_SetParameter(SENSOR_DATA, AIN_DATA_LEN, &ainData);
      
      osal_start_timerEx( NineAxisSensor_TaskID, SBP_READ_AIN_EVT, sensorAinPeriod );
    }
    else
    {
      resetCharacteristicValue( AIN_SERV_UUID, SENSOR_DATA, 0, AIN_DATA_LEN);
      resetCharacteristicValue( AIN_SERV_UUID, SENSOR_CONF, ST_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
    }

    return (events ^ SBP_READ_AIN_EVT);
  }

  //////////////////////////
  //     Magnetometer     //
  //////////////////////////
  if ( events & SBP_MAGNETOMETER_SENSOR_EVT )
  {
    if(magEnabled)
    {
      if (HalMagStatus() == MAG3110_DATA_READY)
      {
        readMagData();
      }
      else if (HalMagStatus() == MAG3110_OFF)
      {
        HalMagTurnOn();
      }

      osal_start_timerEx( NineAxisSensor_TaskID, SBP_MAGNETOMETER_SENSOR_EVT, sensorMagPeriod );
    }
    else
    {
      HalMagTurnOff();
      resetCharacteristicValue( ENVIRONMENTAL_SERV_UUID, SENSOR_DATA, 0, MAGNETOMETER_DATA_LEN);
      resetCharacteristicValue( ENVIRONMENTAL_SERV_UUID, SENSOR_CONF, ST_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
    }

    return (events ^ SBP_MAGNETOMETER_SENSOR_EVT);
  }

  //////////////////////////
  //      Gyroscope       //
  //////////////////////////
  if ( events & SBP_GYROSCOPE_SENSOR_EVT )
  {
    if(gyroEnabled)
    {
      if (HalAccGyroStatus() == LSM6DS3_DATA_READY)
      {
        readGyroData();
      }
      else if (HalAccGyroStatus() == LSM6DS3_OFF)
      {
        HalAccGyroTurnOn();
      }

      osal_start_timerEx( NineAxisSensor_TaskID, SBP_GYROSCOPE_SENSOR_EVT, sensorGyrPeriod );
    }
    else
    {
      HalAccGyroTurnOff();
      resetCharacteristicValue( GYROSCOPE_SERV_UUID, SENSOR_DATA, 0, GYROSCOPE_DATA_LEN);
      resetCharacteristicValue( GYROSCOPE_SERV_UUID, SENSOR_DATA1, 0, ACCELEROMETER_DATA_LEN);
      resetCharacteristicValue( GYROSCOPE_SERV_UUID, SENSOR_CONF, ST_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
    }

    return (events ^ SBP_GYROSCOPE_SENSOR_EVT);
  }

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      NineAxisSensor_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void NineAxisSensor_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
  default:
    // do nothing
    break;
  }
}

/*********************************************************************
 * @fn      resetSensorSetup
 *
 * @brief   Turn off all sensors that are on
 *
 * @param   none
 *
 * @return  none
 */
static void resetSensorSetup (void)
{
  if (ainEnabled)
  {
    ainEnabled = FALSE;
  }

  if (HalMagStatus()!=MAG3110_OFF || magEnabled)
  {
    HalMagTurnOff();
    magEnabled = FALSE;
  }

  if (gyroEnabled)
  {
    HalAccGyroTurnOff();
    gyroEnabled = FALSE;
  }

  // Reset all characteristics values
  resetCharacteristicValues();
}

/*********************************************************************
 * @fn      peripheralStateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void peripheralStateNotificationCB( gaprole_States_t newState )
{
#ifdef PLUS_BROADCASTER
  static uint8 first_conn_flag = 0;
#endif // PLUS_BROADCASTER
  
  
  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        uint8 ownAddress[B_ADDR_LEN];
        uint8 systemId[DEVINFO_SYSTEM_ID_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);
      }
      break;

    case GAPROLE_ADVERTISING:
      break;

    case GAPROLE_CONNECTED:
      break;

    case GAPROLE_CONNECTED_ADV:
      break;      
      
    case GAPROLE_WAITING:
      // Link terminated intentionally: reset all sensors
      resetSensorSetup();
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      break;

    case GAPROLE_ERROR:
      break;

    default:
      break;

  }

  gapProfileState = newState;

#if !defined( CC2540_MINIDK )
  VOID gapProfileState;     // added to prevent compiler warning with
                            // "CC2540 Slave" configurations
#endif


}

/*********************************************************************
 * @fn      readMagData
 *
 * @brief   Read magnetometer data
 *
 * @param   none
 *
 * @return  none
 */
static void readMagData( void )
{
  uint8 mData[MAGNETOMETER_DATA_LEN];
  uint16 x;
  uint16 y;
  uint16 z;

  if (HalMagRead(mData))
  {
    x = BUILD_UINT16( mData[0], mData[1]);
    x = x * 10;
    y = BUILD_UINT16( mData[2], mData[3]);
    y = y * 10;
    z = BUILD_UINT16( mData[4], mData[5]);
    z = z * 10;
    mData[0] = x;
    mData[2] = y;
    mData[4] = z;
    Environmental_SetParameter(SENSOR_DATA, MAGNETOMETER_DATA_LEN, mData);
  }
}

/*********************************************************************
 * @fn      readGyroData
 *
 * @brief   Read gyroscope data
 *
 * @param   none
 *
 * @return  none
 */
static void readGyroData( void )
{
  uint8 tmp[12];
  uint8 gData[GYROSCOPE_DATA_LEN];
  uint8 aData[ACCELEROMETER_DATA_LEN];
  if (HalAccGyroRead(tmp))
  {
    gData[0] = tmp[0];
    gData[1] = tmp[1];
    gData[2] = tmp[2];
    gData[3] = tmp[3];
    gData[4] = tmp[4];
    gData[5] = tmp[5];    
    aData[0] = tmp[6];
    aData[1] = tmp[7];
    aData[2] = tmp[8];
    aData[3] = tmp[9];
    aData[4] = tmp[10];
    aData[5] = tmp[11];
    
    Gyro_SetParameter( SENSOR_DATA, GYROSCOPE_DATA_LEN, gData);    
    Gyro_SetParameter( SENSOR_DATA1, ACCELEROMETER_DATA_LEN, aData);
  }
}

/*********************************************************************
 * @fn      readAdcData
 *
 * @brief   Read ADC channel value
 *
 * @return  none
 */
static uint16 readAdcData( uint8 channel )
{
  uint16 adcVdd3, adcChannel, vdd,  voltage;

  HalAdcSetReference(HAL_ADC_REF_125V);
  adcVdd3 = HalAdcRead(HAL_ADC_CHN_VDD3, HAL_ADC_RESOLUTION_8);
  vdd = (adcVdd3) * 29;  // (1240 * 3 / 127), ADC internal voltage for CC254x is 1.24V

  HalAdcSetReference( HAL_ADC_REF_AVDD );
  adcChannel = HalAdcRead( channel, HAL_ADC_RESOLUTION_8);
  voltage = (uint16)( ((uint32)adcChannel * (uint32)vdd) / 127 );

  return voltage;
}

/*********************************************************************
 * @fn      ainChangeCB
 *
 * @brief   Callback from Ain Service indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void ainChangeCB( uint8 paramID )
{
  uint8 newValue;

  switch (paramID)
  {
    case SENSOR_CONF:
      Ain_GetParameter( SENSOR_CONF, &newValue );

      if ( newValue == ST_CFG_SENSOR_DISABLE )
      {
        if(ainEnabled)
        {
          ainEnabled = FALSE;
          osal_set_event( NineAxisSensor_TaskID, SBP_READ_AIN_EVT);
        }
      }
      else if ( newValue == ST_CFG_SENSOR_ENABLE )
      {
        if(!ainEnabled)
        {
          ainEnabled = TRUE;
          osal_set_event( NineAxisSensor_TaskID, SBP_READ_AIN_EVT);
        }
      }
      break;

    case SENSOR_PERI:
      Ain_GetParameter( SENSOR_PERI, &newValue );
      sensorAinPeriod = newValue*SENSOR_PERIOD_RESOLUTION;
      break;

    default:
      // Should not get here
      break;
  }
}

/*********************************************************************
 * @fn      environmentalChangeCB
 *
 * @brief   Callback from Magnetometer Service indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void environmentalChangeCB( uint8 paramID )
{
  uint8 newValue;

  switch (paramID)
  {
    case SENSOR_CONF:
      Environmental_GetParameter( SENSOR_CONF, &newValue );

      if ( newValue == ST_CFG_SENSOR_DISABLE )
      {
        if(magEnabled)
        {
          magEnabled = FALSE;
          osal_set_event( NineAxisSensor_TaskID, SBP_MAGNETOMETER_SENSOR_EVT);
        }
      }
      else if ( newValue == ST_CFG_SENSOR_ENABLE )
      {
        if(!magEnabled)
        {
          magEnabled = TRUE;
          osal_set_event( NineAxisSensor_TaskID, SBP_MAGNETOMETER_SENSOR_EVT);
        }
      }
      break;

    case SENSOR_PERI:
      Environmental_GetParameter( SENSOR_PERI, &newValue );
      sensorMagPeriod = newValue*SENSOR_PERIOD_RESOLUTION;
      break;

    default:
      // Should not get here
      break;
  }
}

/*********************************************************************
 * @fn      gyroChangeCB
 *
 * @brief   Callback from GyroProfile indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void gyroChangeCB( uint8 paramID )
{
  uint8 newValue;
  
  switch (paramID)
  {
    case SENSOR_CONF:
      Gyro_GetParameter( SENSOR_CONF, &newValue );
      
      if (newValue == ST_CFG_SENSOR_DISABLE)
      {
        if (gyroEnabled)
        {
          gyroEnabled = FALSE;
          osal_set_event( NineAxisSensor_TaskID, SBP_GYROSCOPE_SENSOR_EVT);
        }
      }
      else if ( newValue == ST_CFG_SENSOR_ENABLE )
      {
        if(!gyroEnabled)
        {
          gyroEnabled = TRUE;
          osal_set_event( NineAxisSensor_TaskID, SBP_GYROSCOPE_SENSOR_EVT);
        }
      }
      break;
      
    case SENSOR_PERI:
      Gyro_GetParameter( SENSOR_PERI, &newValue );
      sensorGyrPeriod = newValue*SENSOR_PERIOD_RESOLUTION;
      break;
      
    default:
      // Should not get here
      break;
  }
}

/*********************************************************************
 * @fn      resetCharacteristicValue
 *
 * @brief   Initialize a characteristic value to zero
 *
 * @param   servID - service ID (UUID)
 *
 * @param   paramID - parameter ID of the value is to be cleared
 *
 * @param   vakue - value to initialise with
 *
 * @param   paramLen - length of the parameter
 *
 * @return  none
 */
static void resetCharacteristicValue(uint16 servUuid, uint8 paramID, uint8 value, uint8 paramLen)
{
  uint8* pData = osal_mem_alloc(paramLen);

  if (pData == NULL)
  {
    return;
  }

  osal_memset(pData,value,paramLen);

  switch(servUuid)
  {
    case AIN_SERV_UUID:
      Ain_SetParameter( paramID, paramLen, pData);
      break;

    case ENVIRONMENTAL_SERV_UUID:
      Environmental_SetParameter( paramID, paramLen, pData);
      break;

    case GYROSCOPE_SERV_UUID:
      Gyro_SetParameter( paramID, paramLen, pData);
      break;

    default:
      // Should not get here
      break;
  }

  osal_mem_free(pData);
}

/*********************************************************************
 * @fn      resetCharacteristicValues
 *
 * @brief   Initialize all the characteristic values
 *
 * @return  none
 */
static void resetCharacteristicValues( void )
{
  resetCharacteristicValue( AIN_SERV_UUID, SENSOR_DATA, 0, AIN_DATA_LEN);
  resetCharacteristicValue( AIN_SERV_UUID, SENSOR_CONF, ST_CFG_SENSOR_DISABLE, sizeof( uint8 ));
  resetCharacteristicValue( AIN_SERV_UUID, SENSOR_PERI, AIN_DEFAULT_PERIOD / SENSOR_PERIOD_RESOLUTION, sizeof ( uint8 ));

  resetCharacteristicValue( ENVIRONMENTAL_SERV_UUID, SENSOR_DATA, 0, MAGNETOMETER_DATA_LEN);
  resetCharacteristicValue( ENVIRONMENTAL_SERV_UUID, SENSOR_CONF, ST_CFG_SENSOR_DISABLE, sizeof( uint8 ));
  resetCharacteristicValue( ENVIRONMENTAL_SERV_UUID, SENSOR_PERI, MAG_DEFAULT_PERIOD / SENSOR_PERIOD_RESOLUTION, sizeof ( uint8 ));

  resetCharacteristicValue( GYROSCOPE_SERV_UUID, SENSOR_DATA, 0, GYROSCOPE_DATA_LEN);
  resetCharacteristicValue( GYROSCOPE_SERV_UUID, SENSOR_DATA1, 0, ACCELEROMETER_DATA_LEN);
  resetCharacteristicValue( GYROSCOPE_SERV_UUID, SENSOR_CONF, ST_CFG_SENSOR_DISABLE, sizeof( uint8 ));
  resetCharacteristicValue( GYROSCOPE_SERV_UUID, SENSOR_PERI, GYRO_DEFAULT_PERIOD / SENSOR_PERIOD_RESOLUTION, sizeof ( uint8 ));
}

/*********************************************************************
*********************************************************************/
