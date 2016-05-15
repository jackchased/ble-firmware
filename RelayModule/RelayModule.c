/**************************************************************************************************
  Filename:       RelayModule.c
  Revised:        $Date: 2010-08-06 08:56:11 -0700 (Fri, 06 Aug 2010) $
  Revision:       $Revision: 23333 $

  Description:    This file contains the Relay Module sample application
                  for use with the CC2541 Bluetooth Low Energy Protocol Stack.

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
#include "hal_key.h"

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
#include "powerservice.h"
#include "relayservice.h"

// Sensor drivers
#include "RelayModule.h"

#include "hal_din.h"
#include "hal_relay.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// How often to perform sensor reads (milliseconds)
#define AIN_DEFAULT_PERIOD                    2000
#define PWR_DEFAULT_PERIOD                    2000

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
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     80

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         8

// Company Identifier: Texas Instruments Inc. (13)
#define TI_COMPANY_ID                         0x000D

#define INVALID_CONNHANDLE                    0xFFFF

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

/* Ative delay: 125 cycles ~1 msec */
#define HAL_DELAY(n) st( { volatile uint32 i; for (i=0; i<(n); i++) { }; } )

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
static uint8 RelayModule_TaskID;   // Task ID for internal task/event processing

static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
  // complete name
  0x0C,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  0x52,   // 'R'
  0x65,   // 'e'
  0x6C,   // 'l'
  0x61,   // 'a'
  0x79,   // 'y'
  0x4D,   // 'M'
  0x6F,   // 'o'
  0x64,   // 'd'
  0x75,   // 'u'
  0x6C,   // 'l'
  0x65,   // 'e'


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
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
};

// GAP GATT Attributes
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "Relay Module";

// Sensor State Variables
static bool   ainEnabled = FALSE;
static bool   pwrEnabled = FALSE;
static bool   relayEnabled = FALSE;

static uint8  relayState = 0;

static uint16 sensorAinPeriod = AIN_DEFAULT_PERIOD;
static uint16 sensorPwrPeriod = PWR_DEFAULT_PERIOD;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void RelayModule_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );

static uint16 readAdcData( uint8 channel );
static uint16 sampleSensorValue( uint8 channel );
static void readPower(void);

static void ainChangeCB( uint8 paramID );
static void powerChangeCB( uint8 paramID );
static void relayChangeCB( uint8 paramID );

static void resetSensorSetup( void );
static void resetCharacteristicValue( uint16 servID, uint8 paramID, uint8 value, uint8 paramLen );
static void resetCharacteristicValues( void );

#if defined( CC2540_MINIDK )
static void RelayModule_HandleKeys( uint8 shift, uint8 keys );
#endif

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t RelayModule_PeripheralCBs =
{
  peripheralStateNotificationCB,  // Profile State Change Callbacks
  NULL                            // When a valid RSSI is read from controller (not used by application)
};

// GAP Bond Manager Callbacks
static gapBondCBs_t RelayModule_BondMgrCBs =
{
  NULL,                     // Passcode callback (not used by application)
  NULL                      // Pairing / Bonding state Callback (not used by application)
};

// Simple GATT Profile Callbacks
static sensorCBs_t RelayModule_AinCBs =
{
  ainChangeCB,              // Characteristic value change callback
};

static sensorCBs_t RelayModule_PowerCBs =
{
  powerChangeCB,    // Charactersitic value change callback
};

static sensorCBs_t RelayModule_RelayCBs =
{
  relayChangeCB,   // Charactersitic value change callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      RelayModule_Init
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
void RelayModule_Init( uint8 task_id )
{
  RelayModule_TaskID = task_id;

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

  // Add services
  GGS_AddService( GATT_ALL_SERVICES );            // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
  DevInfo_AddService();                           // Device Information Service
  Din_AddService( GATT_ALL_SERVICES );            // Digital Input Service
  Ain_AddService( GATT_ALL_SERVICES );            // Analog Input Service
  Power_AddService( GATT_ALL_SERVICES );
  Relay_AddService( GATT_ALL_SERVICES );
  
#if defined FEATURE_OAD
  VOID OADTarget_AddService();                    // OAD Profile
#endif

  // Setup the Seensor Profile Characteristic Values
  resetCharacteristicValues();

  // Initialise sensor drivers
  HalDinInit();
  HalRelayInit();

  // Register callback with SimpleGATTprofile
  VOID Ain_RegisterAppCBs( &RelayModule_AinCBs );
  VOID Power_RegisterAppCBs( &RelayModule_PowerCBs );
  VOID Relay_RegisterAppCBs( &RelayModule_RelayCBs );
  
  // Enable clock divide on halt
  // This reduces active current while radio is active and CC254x MCU
  // is halted
  //HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT );

  // Setup a delayed profile startup
  osal_set_event( RelayModule_TaskID, SBP_START_DEVICE_EVT );
  HalLedSet (HAL_LED_1, HAL_LED_MODE_ON);
}

/*********************************************************************
 * @fn      RelayModule_ProcessEvent
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
uint16 RelayModule_ProcessEvent( uint8 task_id, uint16 events )
{

  VOID task_id; // OSAL required parameter that isn't used in this function

  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( RelayModule_TaskID )) != NULL )
    {
      RelayModule_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & SBP_START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPRole_StartDevice( &RelayModule_PeripheralCBs );

    // Start Bond Manager
    VOID GAPBondMgr_Register( &RelayModule_BondMgrCBs );

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
      
      osal_start_timerEx( RelayModule_TaskID, SBP_READ_AIN_EVT, sensorAinPeriod );
    }
    else
    {
      resetCharacteristicValue( AIN_SERV_UUID, SENSOR_DATA, 0, AIN_DATA_LEN);
      resetCharacteristicValue( AIN_SERV_UUID, SENSOR_CONF, ST_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
    }

    return (events ^ SBP_READ_AIN_EVT);
  }

  //////////////////////////
  //        POWER         //
  //////////////////////////
  if ( events & SBP_READ_POWER_EVT )
  {
    if(pwrEnabled)
    {
      readPower();
      osal_start_timerEx( RelayModule_TaskID, SBP_READ_POWER_EVT, sensorPwrPeriod );
    }
    else
    {
      resetCharacteristicValue( POWER_SERV_UUID, SENSOR_DATA, 0, POWER_DATA_LEN);
      resetCharacteristicValue( POWER_SERV_UUID, SENSOR_DATA1, 0, CURRENT_DATA_LEN);
      resetCharacteristicValue( POWER_SERV_UUID, SENSOR_CONF, ST_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
    }

    return (events ^ SBP_READ_POWER_EVT);
  }
  
  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      RelayModule_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void RelayModule_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
  #if defined( CC2540_MINIDK )
    case KEY_CHANGE:
      RelayModule_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;
  #endif // #if defined( CC2540_MINIDK )

  default:
    // do nothing
    break;
  }
}

#if defined( CC2540_MINIDK )
/*********************************************************************
 * @fn      RelayModule_HandleKeys
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
static void RelayModule_HandleKeys( uint8 shift, uint8 keys )
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
  if (ainEnabled)
  {
    ainEnabled = FALSE;
  }
  
  if (pwrEnabled)
  {
    pwrEnabled = FALSE;
  }
  resetCharacteristicValue( POWER_SERV_UUID, SENSOR_DATA, 0, POWER_DATA_LEN);
  resetCharacteristicValue( POWER_SERV_UUID, SENSOR_DATA1, 0, CURRENT_DATA_LEN);
  resetCharacteristicValue( POWER_SERV_UUID, SENSOR_CONF, ST_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
  resetCharacteristicValue( POWER_SERV_UUID, SENSOR_PERI, PWR_DEFAULT_PERIOD / SENSOR_PERIOD_RESOLUTION, sizeof ( uint8 ));
/*
  if (relayEnabled)
  {
    relayEnabled = FALSE;
    HalRelayOff();
  }
*/
  // Reset all characteristics values
  //resetCharacteristicValues();
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
          osal_set_event( RelayModule_TaskID, SBP_READ_AIN_EVT);
        }
      }
      else if ( newValue == ST_CFG_SENSOR_ENABLE )
      {
        if(!ainEnabled)
        {
          ainEnabled = TRUE;
          osal_set_event( RelayModule_TaskID, SBP_READ_AIN_EVT);
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
 * @fn      powerChangeCB
 *
 * @brief   Callback from Current Service indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void powerChangeCB( uint8 paramID )
{
  uint8 newValue;

  switch (paramID)
  {
    case SENSOR_CONF:
      Power_GetParameter( SENSOR_CONF, &newValue );

      if ( newValue == ST_CFG_SENSOR_DISABLE )
      {
        if(pwrEnabled)
        {
          pwrEnabled = FALSE;
          osal_set_event( RelayModule_TaskID, SBP_READ_POWER_EVT);
        }
      }
      else if ( newValue == ST_CFG_SENSOR_ENABLE )
      {
        if(!pwrEnabled)
        {
          pwrEnabled = TRUE;
          osal_set_event( RelayModule_TaskID, SBP_READ_POWER_EVT);
        }
      }
      break;

    case SENSOR_PERI:
      Power_GetParameter( SENSOR_PERI, &newValue );
      sensorPwrPeriod = newValue*SENSOR_PERIOD_RESOLUTION;
      break;

    default:
      // Should not get here
      break;
  }
}

/*********************************************************************
 * @fn      relayChangeCB
 *
 * @brief   Callback from Buzzer Service indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void relayChangeCB( uint8 paramID )
{
  uint8 newValue;

  switch (paramID)
  {
    case SENSOR_CONF:
      Relay_GetParameter( SENSOR_CONF, &newValue );

      if ( newValue == ST_CFG_SENSOR_DISABLE )
      {
        if(relayEnabled)
        {
          relayEnabled = FALSE;
          HalRelayOff();
          relayState = 0;
          Relay_SetParameter(SENSOR_DATA, RELAY_DATA_LEN, &newValue);
        }
      }
      else if ( newValue == ST_CFG_SENSOR_ENABLE )
      {
        if(!relayEnabled)
        {
          relayEnabled = TRUE;
          HalRelayOn();
          relayState = 1;
          Relay_SetParameter(SENSOR_DATA, RELAY_DATA_LEN, &newValue);
        }
      }
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

    case POWER_SERV_UUID:
      Power_SetParameter( paramID, paramLen, pData);
      break;

    case RELAY_SERV_UUID:
      Relay_SetParameter( paramID, paramLen, pData);
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

  resetCharacteristicValue( POWER_SERV_UUID, SENSOR_DATA, 0, POWER_DATA_LEN);
  resetCharacteristicValue( POWER_SERV_UUID, SENSOR_DATA1, 0, CURRENT_DATA_LEN);
  resetCharacteristicValue( POWER_SERV_UUID, SENSOR_CONF, ST_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
  resetCharacteristicValue( POWER_SERV_UUID, SENSOR_PERI, PWR_DEFAULT_PERIOD / SENSOR_PERIOD_RESOLUTION, sizeof ( uint8 ));

  resetCharacteristicValue( RELAY_SERV_UUID, SENSOR_DATA, 0, RELAY_DATA_LEN);
  resetCharacteristicValue( RELAY_SERV_UUID, SENSOR_CONF, ST_CFG_SENSOR_DISABLE, sizeof ( uint8 ));
}

static void readPower( void )
{
  uint16 current, zeroCurrSample = 0, readAvg = 0;
  
  uint16 MeasuredCurrentValue;
  uint32 MeasuredPowerValue;

  zeroCurrSample = sampleSensorValue(HAL_ADC_CHANNEL_4);
  readAvg = sampleSensorValue(HAL_ADC_CHANNEL_0);

  if (relayState == 0){
    zeroCurrSample += 64;
  } else if (relayState == 1) {
    zeroCurrSample -= 25;
  }
  
  if(readAvg > zeroCurrSample){
    current = readAvg - zeroCurrSample;
  }else{
    current = zeroCurrSample - readAvg;
  }

  if(current < 50) {
    current = 0;
  }

  MeasuredCurrentValue = (uint16)(((float)current / 8191) * 3300 * (100 / 26.163));  // Unit: mA
  MeasuredPowerValue = MeasuredCurrentValue * 110;  // Unit: mW

  Power_SetParameter( SENSOR_DATA, POWER_DATA_LEN, &MeasuredPowerValue );
  Power_SetParameter( SENSOR_DATA1, CURRENT_DATA_LEN, &MeasuredCurrentValue );
}

static uint16 sampleSensorValue( uint8 channel )
{
  uint8 i;
  uint16 sensorValue = 0, sensorMax = 0;

  HalAdcSetReference(HAL_ADC_REF_AVDD);
  
  for (i = 0; i < 128; i++){
    sensorValue = HalAdcRead(channel, HAL_ADC_RESOLUTION_14);
    if(sensorValue > sensorMax){
      sensorMax = sensorValue;
    }
  }
  
  return sensorMax;
}

/*********************************************************************
*********************************************************************/
