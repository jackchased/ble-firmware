/**************************************************************************************************
  Filename:       WeatherStation.c
  Revised:        $Date: 2010-08-06 08:56:11 -0700 (Fri, 06 Aug 2010) $
  Revision:       $Revision: 23333 $

  Description:    This file contains the Simple BLE Peripheral sample application
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

#include "math.h"
#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_i2c.h"

#include "gatt.h"
#include "hci.h"

#include "gapgattserver.h"
#include "gattservapp.h"

#include "peripheral.h"

#include "gapbondmgr.h"

// Services
#include "st_util.h"
#include "devinfoservice.h"
#include "environmentalservice.h"
#include "gasservice.h"
#include "micservice.h"

// Sensor drivers
#include "WeatherStation.h"
#include "hal_sensor.h"

#include "hal_bar.h"
#include "hal_humi.h"
#include "hal_uv.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// How often to perform periodic event
#define BAR_DEFAULT_PERIOD                    2000
#define HUM_DEFAULT_PERIOD                    2000
#define UV_DEFAULT_PERIOD                     2000
#define GAS_DEFAULT_PERIOD                    2000
#define MIC_DEFAULT_PERIOD                    200

// Constants for two-stage reading
#define HUM_FSM_PERIOD                        20

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
static uint8 WeatherStation_TaskID;   // Task ID for internal task/event processing

static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
  // complete name
  0x0F,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  0x57,   // 'W'
  0x65,   // 'e'
  0x61,   // 'a'
  0x74,   // 't'
  0x68,   // 'h'
  0x65,   // 'e'
  0x72,   // 'r'
  0x53,   // 'S'
  0x74,   // 't'
  0x61,   // 'a'
  0x74,   // 't'
  0x69,   // 'i'
  0x6f,   // '0'
  0x6e,   // 'n'

  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),    // 100ms
  HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
  LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),    // 1s
  HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),

  // Tx power level
  0x02,    // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0        // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8 advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,    // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
};

// GAP GATT Attributes
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "Weather Station";

// Sensor State Variables
static bool   barEnabled = FALSE;
static bool   humiEnabled = FALSE;
static bool   uvEnabled = FALSE;
static bool   gasEnabled = FALSE;
static bool   micEnabled = FALSE;

static uint8  humiState = 0;

static uint16 sensorBarPeriod = BAR_DEFAULT_PERIOD;
static uint16 sensorHumPeriod = HUM_DEFAULT_PERIOD;
static uint16 sensorUvPeriod = UV_DEFAULT_PERIOD;
static uint16 sensorGasPeriod = GAS_DEFAULT_PERIOD;
static uint16 sensorMicPeriod = MIC_DEFAULT_PERIOD;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void WeatherStation_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );

static void readBarData( void );
static void readHumData( void );
static void readUvData( void );
static void readMq7Data( void );
static void readMicData( void );
static uint16 readAdcData( uint8 channel );
static uint16 readAdcDataRes14( uint8 channel );

static void environmentalChangeCB( uint8 paramID );
static void gasChangeCB( uint8 paramID );
static void micChangeCB( uint8 paramID );

static void resetSensorSetup( void );
static void resetCharacteristicValue( uint16 servID, uint8 paramID, uint8 value, uint8 paramLen );
static void resetCharacteristicValues( void );

#if defined( CC2540_MINIDK )
static void WeatherStation_HandleKeys( uint8 shift, uint8 keys );
#endif

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t WeatherStation_PeripheralCBs =
{
  peripheralStateNotificationCB,  // Profile State Change Callbacks
  NULL                            // When a valid RSSI is read from controller (not used by application)
};

// GAP Bond Manager Callbacks
static gapBondCBs_t WeatherStation_BondMgrCBs =
{
  NULL,                     // Passcode callback (not used by application)
  NULL                      // Pairing / Bonding state Callback (not used by application)
};

// Simple GATT Profile Callbacks
static sensorCBs_t WeatherStation_EnvironmentalCBs =
{
  environmentalChangeCB,    // Characteristic value change callback
};

static sensorCBs_t WeatherStation_GasCBs =
{
  gasChangeCB,              // Characteristic value change callback
};

static sensorCBs_t WeatherStation_MicCBs =
{
  micChangeCB,              // Characteristic value change callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      WeatherStation_Init
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
void WeatherStation_Init( uint8 task_id )
{
  WeatherStation_TaskID = task_id;

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
  Environmental_AddService( GATT_ALL_SERVICES );
  Gas_AddService( GATT_ALL_SERVICES );
  Mic_AddService( GATT_ALL_SERVICES );

#if defined FEATURE_OAD
  VOID OADTarget_AddService();                    // OAD Profile
#endif

  // Setup the Seensor Profile Characteristic Values
  resetCharacteristicValues();

  // Initialise sensor drivers
  HalBarInit();
  HalHumiInit();
  APCFG |= BV(0);
  APCFG |= BV(4);

  // Register callback with SimpleGATTprofile
  VOID Environmental_RegisterAppCBs( &WeatherStation_EnvironmentalCBs );
  VOID Gas_RegisterAppCBs( &WeatherStation_GasCBs );
  VOID Mic_RegisterAppCBs( &WeatherStation_MicCBs );

  // Enable clock divide on halt
  // This reduces active current while radio is active and CC254x MCU
  // is halted
  // HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT );

  // Setup a delayed profile startup
  // osal_set_event( WeatherStation_TaskID, SBP_START_DEVICE_EVT );
  osal_start_timerEx( WeatherStation_TaskID, SBP_START_DEVICE_EVT, 50 );
}

/*********************************************************************
 * @fn      WeatherStation_ProcessEvent
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
uint16 WeatherStation_ProcessEvent( uint8 task_id, uint16 events )
{

  VOID task_id; // OSAL required parameter that isn't used in this function

  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( WeatherStation_TaskID )) != NULL )
    {
      WeatherStation_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & SBP_START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPRole_StartDevice( &WeatherStation_PeripheralCBs );

    // Start Bond Manager
    VOID GAPBondMgr_Register( &WeatherStation_BondMgrCBs );

    HalUvInit();

    return ( events ^ SBP_START_DEVICE_EVT );
  }

  //////////////////////////
  //      Barometer       //
  //////////////////////////
  if ( events & SBP_BAROMETER_SENSOR_EVT )
  {
    if(barEnabled)
    {
      if (HalBarStatus() == LPS25HB_DATA_READY)
      {
        readBarData();
      }
      else if (HalBarStatus() == LPS25HB_OFF)
      {
        HalBarTurnOn();
      }
      
      osal_start_timerEx( WeatherStation_TaskID, SBP_BAROMETER_SENSOR_EVT, sensorBarPeriod );
    }
    else
    {
      HalBarTurnOff();
      resetCharacteristicValue( ENVIRONMENTAL_SERV_UUID, SENSOR_DATA, 0, BAROMETER_DATA_LEN);
      resetCharacteristicValue( ENVIRONMENTAL_SERV_UUID, SENSOR_CONF, ST_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
    }

    return (events ^ SBP_BAROMETER_SENSOR_EVT);
  }

  //////////////////////////
  //      Humidity        //
  //////////////////////////
  if ( events & SBP_HUMIDITY_SENSOR_EVT )
  {
    if (humiEnabled)
    {
      HalHumiExecMeasurementStep(humiState);
      if (humiState == 2)
      {
        readHumData();
        humiState = 0;
        osal_start_timerEx( WeatherStation_TaskID, SBP_HUMIDITY_SENSOR_EVT, sensorHumPeriod );
      }
      else
      {
        humiState++;
        osal_start_timerEx( WeatherStation_TaskID, SBP_HUMIDITY_SENSOR_EVT, HUM_FSM_PERIOD );
      }
    }
    else
    {
      resetCharacteristicValue( ENVIRONMENTAL_SERV_UUID, SENSOR_DATA1, 0, TEMP_DATA_LEN);
      resetCharacteristicValue( ENVIRONMENTAL_SERV_UUID, SENSOR_DATA2, 0, HUMI_DATA_LEN);
      resetCharacteristicValue( ENVIRONMENTAL_SERV_UUID, SENSOR_CONF1, ST_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
    }

    return (events ^ SBP_HUMIDITY_SENSOR_EVT);
  }

  //////////////////////////
  //         UV           //
  //////////////////////////
  if ( events & SBP_UV_SENSOR_EVT )
  {
    if (uvEnabled)
    {
      readUvData();
      osal_start_timerEx( WeatherStation_TaskID, SBP_UV_SENSOR_EVT,  sensorUvPeriod);
    }
    else
    {
      HalUvTurnOff();
      resetCharacteristicValue( ENVIRONMENTAL_SERV_UUID, SENSOR_DATA3, 0, ALS_DATA_LEN);
      resetCharacteristicValue( ENVIRONMENTAL_SERV_UUID, SENSOR_CONF2, ST_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
    }

    return (events ^ SBP_UV_SENSOR_EVT);
  }

  //////////////////////////
  //         MQ-7         //
  //////////////////////////
  if ( events & SBP_MQ7_SENSOR_EVT )
  {
    if(gasEnabled)
    {
      readMq7Data();
      osal_start_timerEx( WeatherStation_TaskID, SBP_MQ7_SENSOR_EVT, sensorGasPeriod );
    }
    else
    {
      resetCharacteristicValue( GAS_SERV_UUID, SENSOR_DATA, 0, GAS_DATA_LEN);
      resetCharacteristicValue( GAS_SERV_UUID, SENSOR_CONF, ST_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
    }

    return (events ^ SBP_MQ7_SENSOR_EVT);
  }

  //////////////////////////
  //         MIC          //
  //////////////////////////
  if ( events & SBP_MIC_SENSOR_EVT )
  {
    if(micEnabled)
    {
      readMicData();
      osal_start_timerEx( WeatherStation_TaskID, SBP_MIC_SENSOR_EVT, sensorMicPeriod );
    }
    else
    {
      resetCharacteristicValue( MIC_SERV_UUID, SENSOR_DATA, 0, MIC_DATA_LEN);
      resetCharacteristicValue( MIC_SERV_UUID, SENSOR_CONF, ST_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
    }

    return (events ^ SBP_MIC_SENSOR_EVT);
  }
  
  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      WeatherStation_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void WeatherStation_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
  #if defined( CC2540_MINIDK )
    case KEY_CHANGE:
      WeatherStation_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;
  #endif // #if defined( CC2540_MINIDK )

  default:
    // do nothing
    break;
  }
}

#if defined( CC2540_MINIDK )
/*********************************************************************
 * @fn      WeatherStation_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void WeatherStation_HandleKeys( uint8 shift, uint8 keys )
{
  uint8 SK_Keys = 0;

  VOID shift;  // Intentionally unreferenced parameter

  if ( keys & HAL_KEY_SW_1 )
  {
    SK_Keys |= SK_KEY_LEFT;
  }

  if ( keys & HAL_KEY_SW_2 )
  {

    SK_Keys |= SK_KEY_RIGHT;

    // if device is not in a connection, pressing the right key should toggle
    // advertising on and off
    // Note:  If PLUS_BROADCASTER is define this condition is ignored and
    //        Device may advertise during connections as well. 
#ifndef PLUS_BROADCASTER  
    if( gapProfileState != GAPROLE_CONNECTED )
    {
#endif // PLUS_BROADCASTER
      uint8 current_adv_enabled_status;
      uint8 new_adv_enabled_status;

      //Find the current GAP advertisement status
      GAPRole_GetParameter( GAPROLE_ADVERT_ENABLED, &current_adv_enabled_status );

      if( current_adv_enabled_status == FALSE )
      {
        new_adv_enabled_status = TRUE;
      }
      else
      {
        new_adv_enabled_status = FALSE;
      }

      //change the GAP advertisement status to opposite of current status
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status );
#ifndef PLUS_BROADCASTER
    }
#endif // PLUS_BROADCASTER
  }

  // Set the value of the keys state to the Simple Keys Profile;
  // This will send out a notification of the keys state if enabled
  SK_SetParameter( SK_KEY_ATTR, sizeof ( uint8 ), &SK_Keys );
}
#endif // #if defined( CC2540_MINIDK )

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
  if (barEnabled)
  {
    HalBarTurnOff();
    barEnabled = FALSE;
  }

  if (humiEnabled)
  {
    HalHumiInit();
    humiEnabled = FALSE;
  }

  if (uvEnabled)
  {
    HalUvTurnOff();
    uvEnabled = FALSE;
  }

  if (gasEnabled)
  {
    gasEnabled = FALSE;
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

        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          // Display device address
          HalLcdWriteString( bdAddr2Str( ownAddress ),  HAL_LCD_LINE_2 );
          HalLcdWriteString( "Initialized",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    case GAPROLE_ADVERTISING:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Advertising",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    case GAPROLE_CONNECTED:
      {        
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Connected",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
          
#ifdef PLUS_BROADCASTER
        // Only turn advertising on for this state when we first connect
        // otherwise, when we go from connected_advertising back to this state
        // we will be turning advertising back on.
        if ( first_conn_flag == 0 ) 
        {
          uint8 adv_enabled_status = 1;
          GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8), &adv_enabled_status); // Turn on Advertising
          first_conn_flag = 1;
        }
#endif // PLUS_BROADCASTER
      }
      break;

    case GAPROLE_CONNECTED_ADV:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Connected Advertising",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;      
    case GAPROLE_WAITING:
      {
        // Link terminated intentionally: reset all sensors
        resetSensorSetup();
        
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Disconnected",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Timed Out",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
          
#ifdef PLUS_BROADCASTER
        // Reset flag for next connection.
        first_conn_flag = 0;
#endif //#ifdef (PLUS_BROADCASTER)
      }
      break;

    case GAPROLE_ERROR:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Error",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    default:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

  }

  gapProfileState = newState;

#if !defined( CC2540_MINIDK )
  VOID gapProfileState;     // added to prevent compiler warning with
                            // "CC2540 Slave" configurations
#endif


}

/*********************************************************************
 * @fn      readBarData
 *
 * @brief   Read barometer data
 *
 * @param   none
 *
 * @return  none
 */
static void readBarData( void )
{
  uint8 bData[BAROMETER_DATA_LEN];
  uint32 barData;  
    
  if (HalBarRead(bData))
  {
    barData = BUILD_UINT32( bData[0], bData[1], bData[2], 00);
    barData = barData >> 12;
    barData = barData * 1000;
    Environmental_SetParameter(SENSOR_DATA, BAROMETER_DATA_LEN, &barData);
  }
}

/*********************************************************************
 * @fn      readHumData
 *
 * @brief   Read humidity and temperature data
 *
 * @param   none
 *
 * @return  none
 */
static void readHumData(void)
{
  uint8 hData[SHT20_DATA_LEN];
  uint16 temp, hum, tempRel, humRel;
  
  if (HalHumiReadMeasurement(hData))
  {
    // calculate temperature
    temp = BUILD_UINT16(hData[0], hData[1]);
    tempRel = calcTmpRel(temp);
    
    // calculate relative humidity
    hum = BUILD_UINT16(hData[2], hData[3]);  
    humRel = calcHumRel(hum);    
    
    Environmental_SetParameter( SENSOR_DATA1, TEMP_DATA_LEN, &tempRel);
    Environmental_SetParameter( SENSOR_DATA2, HUMI_DATA_LEN, &humRel);
  }
}

/*********************************************************************
 * @fn      readUvData
 *
 * @brief   Read UV data
 *
 * @param   none
 *
 * @return  none
 */
static void readUvData(void)
{
  uint8 uData[SI1132_DATA_LEN];

  // uint16 uv, uvRel;
  uint32 vis, visRel;
  
  if (HalUvRead(uData))
  {
    // calculate visible light
    vis = BUILD_UINT32(uData[0], uData[1], 00, 00);
    visRel = ( vis - 256 ) * 1000 / 282;

/*
    // calculate UV index
    uv = BUILD_UINT16(uData[4], uData[5]);  
    uvRel = uv / 100;
*/
    Environmental_SetParameter( SENSOR_DATA3, ALS_DATA_LEN, &visRel);
  }
}

 /*********************************************************************
 * @fn      readMq7Data
 *
 * @brief   Read MQ-7 sensor
 *
 * @return  none
 */
static void readMq7Data( void )
{
  float RsRoRatio;
  float R_Ethanol = 0.5485;
  float vout_air_init = 390;
  uint16 vout = readAdcData(HAL_ADC_CHANNEL_0);

  uint16 ppm_measured;

  if (vout_air_init >= vout)
  {
    vout_air_init = vout;
    ppm_measured = 0;
  } else {
    RsRoRatio = (vout_air_init / (float)vout) * ((5000.0 - (float)vout) / (5000.0 - vout_air_init));
    
    if (RsRoRatio > 0.25)
    {
      ppm_measured = 0;
    } else {
      ppm_measured = (uint16)pow((10), (1 + ((log10(0.25) - log10(RsRoRatio)) / R_Ethanol)));
    }
  }

  Gas_SetParameter( SENSOR_DATA, GAS_DATA_LEN, &ppm_measured );
}

 /*********************************************************************
 * @fn      readMicData
 *
 * @brief   Read Microphone sensor
 *
 * @return  none
 */
static void readMicData( void )
{
  float Pa;
  uint16 dB_SPL;
  uint16 vout = readAdcDataRes14(HAL_ADC_CHANNEL_4);

  Pa = ( (float)vout * 10 ) / 72254;
  dB_SPL = (uint16)(20 * log10(Pa / 0.00002));

  Mic_SetParameter( SENSOR_DATA, MIC_DATA_LEN, &dB_SPL );
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
  voltage = (adcChannel) * (vdd / 127);

  return voltage;
}

/*********************************************************************
 * @fn      readAdcDataRes14
 *
 * @brief   Read ADC channel value
 *
 * @return  none
 */
static uint16 readAdcDataRes14( uint8 channel )
{
  uint16 adcVdd3, adcChannel, vdd,  voltage;
  
  HalAdcSetReference(HAL_ADC_REF_125V);
  adcVdd3 = HalAdcRead(HAL_ADC_CHN_VDD3, HAL_ADC_RESOLUTION_8);
  vdd = (adcVdd3) * 292;  // (12400 * 3 / 127), ADC internal voltage for CC254x is 1.24V, 1.2400 V 

  HalAdcSetReference( HAL_ADC_REF_AVDD );
  adcChannel = HalAdcRead( channel, HAL_ADC_RESOLUTION_14);
  voltage = (adcChannel) * (vdd / 8191);

  return voltage;
}

/*********************************************************************
 * @fn      environmentalChangeCB
 *
 * @brief   Callback from Environmental Service indicating a value change
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
        if(barEnabled)
        {
          barEnabled = FALSE;
          osal_set_event( WeatherStation_TaskID, SBP_BAROMETER_SENSOR_EVT);
        }
      }
      else if ( newValue == ST_CFG_SENSOR_ENABLE )
      {
        if(!barEnabled)
        {
          barEnabled = TRUE;
          osal_set_event( WeatherStation_TaskID, SBP_BAROMETER_SENSOR_EVT);
        }
      }
      break;

  case SENSOR_PERI:
      Environmental_GetParameter( SENSOR_PERI, &newValue );
      sensorBarPeriod = newValue*SENSOR_PERIOD_RESOLUTION;
      break;

      
  case SENSOR_CONF1:
      Environmental_GetParameter( SENSOR_CONF1, &newValue );

      if ( newValue == ST_CFG_SENSOR_DISABLE )
      {
        if(humiEnabled)
        {
          humiEnabled = FALSE;
          osal_set_event( WeatherStation_TaskID, SBP_HUMIDITY_SENSOR_EVT);
        }
      }
      else if ( newValue == ST_CFG_SENSOR_ENABLE )
      {
        if(!humiEnabled)
        {
          humiEnabled = TRUE;
          humiState = 0;
          osal_set_event( WeatherStation_TaskID, SBP_HUMIDITY_SENSOR_EVT);
        }
      }
      break;

  case SENSOR_PERI1:
      Environmental_GetParameter( SENSOR_PERI1, &newValue );
      sensorHumPeriod = newValue*SENSOR_PERIOD_RESOLUTION;
      break;

      
  case SENSOR_CONF2: 
      Environmental_GetParameter( SENSOR_CONF2, &newValue );

      if ( newValue == ST_CFG_SENSOR_DISABLE )
      {
        if(uvEnabled)
        {
          uvEnabled = FALSE;
          osal_set_event( WeatherStation_TaskID, SBP_UV_SENSOR_EVT);
        }
      }
      else if ( newValue == ST_CFG_SENSOR_ENABLE )
      {
        if(!uvEnabled)
        {
          uvEnabled = TRUE;
          HalBarTurnOn();
          osal_set_event( WeatherStation_TaskID, SBP_UV_SENSOR_EVT);
        }
      }
      break;

  case SENSOR_PERI2:
      Environmental_GetParameter( SENSOR_PERI2, &newValue );
      sensorUvPeriod = newValue*SENSOR_PERIOD_RESOLUTION;
      break;

    default:
      // Should not get here
      break;
  }
}

/*********************************************************************
 * @fn      gasChangeCB
 *
 * @brief   Callback from Gas Service indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void gasChangeCB( uint8 paramID )
{
  uint8 newValue;

  switch (paramID)
  {
    case SENSOR_CONF:
      Gas_GetParameter( SENSOR_CONF, &newValue );

      if ( newValue == ST_CFG_SENSOR_DISABLE )
      {
        if(gasEnabled)
        {
          gasEnabled = FALSE;
          osal_set_event( WeatherStation_TaskID, SBP_MQ7_SENSOR_EVT);
        }
      }
      else if ( newValue == ST_CFG_SENSOR_ENABLE )
      {
        if(!gasEnabled)
        {
          gasEnabled = TRUE;
          osal_set_event( WeatherStation_TaskID, SBP_MQ7_SENSOR_EVT);
        }
      }
      break;

    case SENSOR_PERI:
      Gas_GetParameter( SENSOR_PERI, &newValue );
      sensorGasPeriod = newValue*SENSOR_PERIOD_RESOLUTION;
      break;

    default:
      // Should not get here
      break;
  }
}

/*********************************************************************
 * @fn      micChangeCB
 *
 * @brief   Callback from Microphone Service indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void micChangeCB( uint8 paramID )
{
  uint8 newValue;

  switch (paramID)
  {
    case SENSOR_CONF:
      Mic_GetParameter( SENSOR_CONF, &newValue );

      if ( newValue == ST_CFG_SENSOR_DISABLE )
      {
        if(micEnabled)
        {
          micEnabled = FALSE;
          osal_set_event( WeatherStation_TaskID, SBP_MIC_SENSOR_EVT);
        }
      }
      else if ( newValue == ST_CFG_SENSOR_ENABLE )
      {
        if(!micEnabled)
        {
          micEnabled = TRUE;
          osal_set_event( WeatherStation_TaskID, SBP_MIC_SENSOR_EVT);
        }
      }
      break;

    case SENSOR_PERI:
      Gas_GetParameter( SENSOR_PERI, &newValue );
      sensorMicPeriod = newValue*SENSOR_PERIOD_RESOLUTION;
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
    case ENVIRONMENTAL_SERV_UUID:
      Environmental_SetParameter( paramID, paramLen, pData);
      break;

    case GAS_SERV_UUID:
      Gas_SetParameter( paramID, paramLen, pData);
      break;

    case MIC_SERV_UUID:
      Mic_SetParameter( paramID, paramLen, pData);
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
  resetCharacteristicValue( ENVIRONMENTAL_SERV_UUID, SENSOR_DATA, 0, BAROMETER_DATA_LEN);
  resetCharacteristicValue( ENVIRONMENTAL_SERV_UUID, SENSOR_CONF, ST_CFG_SENSOR_DISABLE, sizeof( uint8 ));
  resetCharacteristicValue( ENVIRONMENTAL_SERV_UUID, SENSOR_PERI, BAR_DEFAULT_PERIOD / SENSOR_PERIOD_RESOLUTION, sizeof ( uint8 ));

  resetCharacteristicValue( ENVIRONMENTAL_SERV_UUID, SENSOR_DATA1, 0, TEMP_DATA_LEN);
  resetCharacteristicValue( ENVIRONMENTAL_SERV_UUID, SENSOR_DATA2, 0, TEMP_DATA_LEN);
  resetCharacteristicValue( ENVIRONMENTAL_SERV_UUID, SENSOR_CONF1, ST_CFG_SENSOR_DISABLE, sizeof( uint8 ));
  resetCharacteristicValue( ENVIRONMENTAL_SERV_UUID, SENSOR_PERI1, HUM_DEFAULT_PERIOD / SENSOR_PERIOD_RESOLUTION, sizeof ( uint8 ));

  resetCharacteristicValue( ENVIRONMENTAL_SERV_UUID, SENSOR_DATA3, 0, ALS_DATA_LEN);
  resetCharacteristicValue( ENVIRONMENTAL_SERV_UUID, SENSOR_CONF2, ST_CFG_SENSOR_DISABLE, sizeof( uint8 ));
  resetCharacteristicValue( ENVIRONMENTAL_SERV_UUID, SENSOR_PERI2, UV_DEFAULT_PERIOD / SENSOR_PERIOD_RESOLUTION, sizeof ( uint8 ));

  resetCharacteristicValue( GAS_SERV_UUID, SENSOR_DATA, 0, GAS_DATA_LEN);
  resetCharacteristicValue( GAS_SERV_UUID, SENSOR_CONF, ST_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
  resetCharacteristicValue( GAS_SERV_UUID, SENSOR_PERI, GAS_DEFAULT_PERIOD / SENSOR_PERIOD_RESOLUTION, sizeof ( uint8 ));

  resetCharacteristicValue( MIC_SERV_UUID, SENSOR_DATA, 0, MIC_DATA_LEN);
  resetCharacteristicValue( MIC_SERV_UUID, SENSOR_CONF, ST_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
  resetCharacteristicValue( MIC_SERV_UUID, SENSOR_PERI, MIC_DEFAULT_PERIOD / SENSOR_PERIOD_RESOLUTION, sizeof ( uint8 ));
}

/*********************************************************************
*********************************************************************/
