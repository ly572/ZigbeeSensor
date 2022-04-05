#include "OSAL.h"
#include "ZGlobals.h"
#include "AF.h"
#include "aps_groups.h"
#include "ZDApp.h"

#include "SampleApp.h"
#include "SampleAppHw.h"

#include "OnBoard.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "MT_UART.h"
#include "MT_APP.h"
#include "MT.h"

#include "hal_adc.h"
#include<stdio.h>
/*********************************************************************
 * MACROS
 */
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr)[0])
#define DATA_PIN P0_5            // ����P0.5��Ϊ�������������
#define PEOPLE_DATA_PIN P0_4     // ����P0.4��Ϊ������⴫�����������
#define LED1 P1_0                // ����P1.0��ΪLED1


#define HAL_ADC_DEC_064     0x00    /* Decimate by 64 : 8-bit resolution */
#define HAL_ADC_DEC_128     0x10    /* Decimate by 128 : 10-bit resolution */
#define HAL_ADC_DEC_256     0x20    /* Decimate by 256 : 12-bit resolution */
#define HAL_ADC_DEC_512     0x30    /* Decimate by 512 : 14-bit resolution */
#define HAL_ADC_DEC_BITS    0x30    /* Bits [5:4] */


uint16 ad1;
uint16 ad2;
//�趨�豸��� ����д�ն˽ڵ�֮ǰҪ�� ��Ӧ������ն˽ڵ�
uint8 addr = '2';
/*********************************************************************
 * CONSTANTS
 */
//uint8 light_flag = 0;
//uint8 light_flag_bllow = 0;
uint16 counter = 10;
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// This list should be filled with Application specific Cluster IDs.
const cId_t SampleApp_ClusterList[SAMPLEAPP_MAX_CLUSTERS] =
{
  SAMPLEAPP_PERIODIC_CLUSTERID,
  SAMPLEAPP_FLASH_CLUSTERID
};

const SimpleDescriptionFormat_t SampleApp_SimpleDesc =
{
  SAMPLEAPP_ENDPOINT,              //  int Endpoint;
  SAMPLEAPP_PROFID,                //  uint16 AppProfId[2];
  SAMPLEAPP_DEVICEID,              //  uint16 AppDeviceId[2];
  SAMPLEAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
  SAMPLEAPP_FLAGS,                 //  int   AppFlags:4;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList,  //  uint8 *pAppInClusterList;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList   //  uint8 *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in SampleApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t SampleApp_epDesc;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
uint8 SampleApp_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // SampleApp_Init() is called.
devStates_t SampleApp_NwkState;

uint8 SampleApp_TransID;  // This is the unique message ID (counter)

afAddrType_t SampleApp_Periodic_DstAddr; //�㲥
afAddrType_t SampleApp_Flash_DstAddr;    //�鲥
afAddrType_t SampleApp_P2P_DstAddr;       //�㲥

aps_Group_t SampleApp_Group;

uint8 SampleAppPeriodicCounter = 0;
uint8 SampleAppFlashCounter = 0;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void SampleApp_HandleKeys( uint8 shift, uint8 keys );
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void SampleApp_SendPeriodicMessage( void );
void SampleApp_SendFlashMessage( uint16 flashTime );
void SampleApp_Send_P2P_Message(void);
uint16 ReadLightData( void );
void Delay(unsigned int t); //��������
void SampleApp_SerialCMD(mtOSALSerialData_t *cmdMsg);
/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SampleApp_Init
 *
 * @brief   Initialization function for the Generic App Task.
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
void SampleApp_Init( uint8 task_id )
{ 
  SampleApp_TaskID = task_id;
  SampleApp_NwkState = DEV_INIT;
  SampleApp_TransID = 0;
  
  MT_UartInit();                  //���ڳ�ʼ��
  MT_UartRegisterTaskID(task_id); //ע�ᴮ������
  // ����ǿ��IO�ڳ�ʼ��
  P0SEL &= ~0x20;                 //����P0.5Ϊ��ͨIO��
  P0DIR &= ~0x20;                 //P0.5����Ϊ�����
  // ��������ӦIO�ڳ�ʼ��
  P0SEL &= ~0x10;                 //����P0.4Ϊ��ͨIO��
  P0DIR &= ~0x10;                 //P0.4����Ϊ�����
  
  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().

 #if defined ( BUILD_ALL_DEVICES )
  // The "Demo" target is setup to have BUILD_ALL_DEVICES and HOLD_AUTO_START
  // We are looking at a jumper (defined in SampleAppHw.c) to be jumpered
  // together - if they are - we will start up a coordinator. Otherwise,
  // the device will start as a router.
  if ( readCoordinatorJumper() )
    zgDeviceLogicalType = ZG_DEVICETYPE_COORDINATOR;
  else
    zgDeviceLogicalType = ZG_DEVICETYPE_ROUTER;
#endif // BUILD_ALL_DEVICES

#if defined ( HOLD_AUTO_START )
  // HOLD_AUTO_START is a compile option that will surpress ZDApp
  //  from starting the device and wait for the application to
  //  start the device.
  ZDOInitDevice(0);
#endif

  // Setup for the periodic message's destination address
  // Broadcast to everyone
  SampleApp_Periodic_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
  SampleApp_Periodic_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Periodic_DstAddr.addr.shortAddr = 0xFFFF;

  // Setup for the flash command's destination address - Group 1
  SampleApp_Flash_DstAddr.addrMode = (afAddrMode_t)afAddrGroup;
  SampleApp_Flash_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Flash_DstAddr.addr.shortAddr = SAMPLEAPP_FLASH_GROUP;
  
  SampleApp_P2P_DstAddr.addrMode = (afAddrMode_t)Addr16Bit; //�㲥 
  SampleApp_P2P_DstAddr.endPoint = SAMPLEAPP_ENDPOINT; 
  SampleApp_P2P_DstAddr.addr.shortAddr = 0x0000;            //����Э����

  // Fill out the endpoint description.
  SampleApp_epDesc.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_epDesc.task_id = &SampleApp_TaskID;
  SampleApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&SampleApp_SimpleDesc;
  SampleApp_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &SampleApp_epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( SampleApp_TaskID );

  // By default, all devices start out in Group 1
  SampleApp_Group.ID = 0x0001;
  osal_memcpy( SampleApp_Group.name, "Group 1", 7 );
  aps_AddGroup( SAMPLEAPP_ENDPOINT, &SampleApp_Group );

#if defined ( LCD_SUPPORTED )
  HalLcdWriteString( "SampleApp", HAL_LCD_LINE_1 );
#endif
}

/*********************************************************************
 * @fn      SampleApp_ProcessEvent
 *
 * @brief   Generic Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
uint16 SampleApp_ProcessEvent( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        case CMD_SERIAL_MSG:  //�����յ����ݺ���MT_UART�㴫�ݹ��������� 
          SampleApp_SerialCMD((mtOSALSerialData_t *)MSGpkt); // �Զ��崦����
          break;
        // Received when a key is pressed
        case KEY_CHANGE:
          SampleApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        // Received when a messages is received (OTA) for this endpoint
        case AF_INCOMING_MSG_CMD:
          SampleApp_MessageMSGCB( MSGpkt );
          break;

        // Received whenever the device changes state in the network
        case ZDO_STATE_CHANGE:
          SampleApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( //(SampleApp_NwkState == DEV_ZB_COORD) ||
                 (SampleApp_NwkState == DEV_ROUTER)
              || (SampleApp_NwkState == DEV_END_DEVICE) )
          {
            // Start sending the periodic message in a regular interval.
            osal_start_timerEx( SampleApp_TaskID,
                              SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
                              SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT );
          }
          else
          {
            // Device is no longer in the network
          }
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next - if one is available
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  // Send a message out - This event is generated by a timer
  //  (setup in SampleApp_Init()).
  if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT )
  {
    // Send the periodic message
    //SampleApp_SendPeriodicMessage();
    SampleApp_Send_P2P_Message();

    // Setup to send message again in normal period (+ a little jitter)
    osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
        (SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT + (osal_rand() & 0x00FF)) );

    // return unprocessed events
    return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT);
  }

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * Event Generation Functions
 */
/*********************************************************************
 * @fn      SampleApp_HandleKeys
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
void SampleApp_HandleKeys( uint8 shift, uint8 keys )
{
  (void)shift;  // Intentionally unreferenced parameter
  
  if ( keys & HAL_KEY_SW_1 )
  {
    /* This key sends the Flash Command is sent to Group 1.
     * This device will not receive the Flash Command from this
     * device (even if it belongs to group 1).
     */
    SampleApp_SendFlashMessage( SAMPLEAPP_FLASH_DURATION );
    
  }

  if ( keys & HAL_KEY_SW_2 )
  {
    /* The Flashr Command is sent to Group 1.
     * This key toggles this device in and out of group 1.
     * If this device doesn't belong to group 1, this application
     * will not receive the Flash command sent to group 1.
     */
    
    aps_Group_t *grp;
    grp = aps_FindGroup( SAMPLEAPP_ENDPOINT, SAMPLEAPP_FLASH_GROUP );
    if ( grp )
    {
      // Remove from the group
      aps_RemoveGroup( SAMPLEAPP_ENDPOINT, SAMPLEAPP_FLASH_GROUP );
    }
    else
    {
      // Add to the flash group
      aps_AddGroup( SAMPLEAPP_ENDPOINT, &SampleApp_Group );
    }
  }
  
  // ����S1����led
  if ( keys & HAL_KEY_SW_6 )
  {
    //HalUARTWrite(0,"KEY S2\n",7); 
    // HalLedBlink( HAL_LED_1, 2,50, 500 );
    LED1 = ~LED1;
  }
  
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      SampleApp_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  uint16 flashTime;
 // uint8 a = 0x31;


  switch ( pkt->clusterId )
  {
    
    case SAMPLEAPP_P2P_CLUSTERID:
      if(SampleApp_NwkState == DEV_ZB_COORD)
      {
       //��ӡ�鿴�豸����Ƿ���ȷ
       //HalUARTWrite(0,(pkt->cmd.Data) , 1);
       //HalUARTWrite(0,(pkt->cmd.Data+1) , 3);
       //HalUARTWrite(0,(pkt->cmd.Data+4) , 1);

        if(pkt->cmd.Data[4] == '1') {
          ad1 = pkt->srcAddr.addr.shortAddr; // �����豸1�ĵ�ַ
          // ��������
          // �������ݴ���
          if(pkt->cmd.Data[0] == '1') {
            HalUARTWrite(0, "���Ҽ��㣺", 12);
            HalUARTWrite(0, pkt->cmd.Data+4, 1);
            HalUARTWrite(0, " �Ƿ����ˣ�", 11);   //��ʾ���յ�����
            HalUARTWrite(0, "��", 2);
       //     HalUARTWrite(0, "\n", 1);
          }
          else {
          // ���˾��� ���ڵ���������ʾNo
            HalUARTWrite(0, "���Ҽ��㣺", 12);
            HalUARTWrite(0, pkt->cmd.Data+4, 1);
            HalUARTWrite(0, " �Ƿ����ˣ�", 11);   //��ʾ���յ�����
            HalUARTWrite(0, "��", 2);
        //    HalUARTWrite(0, "\n", 1);         // �س�����
          }
          // �������ݴ���
          HalUARTWrite(0, "���㣺", 12);
          HalUARTWrite(0, pkt->cmd.Data+4, 1);
          HalUARTWrite(0, " ����ǿ�ȣ�", 11);
          HalUARTWrite(0, pkt->cmd.Data+1, 3);
          HalUARTWrite(0, "\n", 1);  
          }
      
        // �ж��豸�Ƿ���2��
        if(pkt->cmd.Data[4] == '2') {
          ad2 = pkt->srcAddr.addr.shortAddr; // �����豸2�ĵ�ַ
          // ��������
           //�������ݴ���
          if(pkt->cmd.Data[0] == '1') {
            HalUARTWrite(0, "���Ҽ��㣺", 12);
            HalUARTWrite(0, pkt->cmd.Data+4, 1);
            HalUARTWrite(0, " �Ƿ����ˣ�", 11);   //��ʾ���յ�����
            HalUARTWrite(0, "��", 2);
          }
          else {
          // ���˾��� ���ڵ���������ʾNo
            HalUARTWrite(0, "���Ҽ��㣺", 12);
            HalUARTWrite(0, pkt->cmd.Data+4, 1);
            HalUARTWrite(0, " �Ƿ����ˣ�", 11);   //��ʾ���յ�����
            HalUARTWrite(0, "��", 2);         // �س�����
          }
          // �������ݴ���
          HalUARTWrite(0, "�豸", 4);
          HalUARTWrite(0, pkt->cmd.Data+4, 1);
          HalUARTWrite(0, " ����ǿ�ȣ�", 11);
          HalUARTWrite(0, pkt->cmd.Data+1, 3);
          HalUARTWrite(0, "\n", 1);  
        }
      }
      else if(SampleApp_NwkState == DEV_END_DEVICE)
      {
        // Ƶ�������п���ָ���
        HalLedBlink( HAL_LED_2, 2,50, 500 );
        if(pkt->cmd.Data[1] == '1') {
          if(pkt->cmd.Data[2] == '1') {
            // ����
            LED1 = 0;
            counter = 0;
            //Delay(10000);
          }else if(pkt->cmd.Data[2] == '0') {
            // �ص�
          
            LED1 = 1;
            counter = 10;
          }
        }
        // �ж��Ƿ��Ƕ�2���豸���п���
        if(pkt->cmd.Data[1] == '2') {
          if(pkt->cmd.Data[2] == '1') {
            // ����
            LED1 = 0;
            counter = 0;
            //Delay(10000);
          }else if(pkt->cmd.Data[2] == '0') {
            // �ص�
            LED1 = 1;
            counter = 10;
          }
        }
      }
      
      
      
      break;    
    case SAMPLEAPP_PERIODIC_CLUSTERID:
      break;

    case SAMPLEAPP_FLASH_CLUSTERID:
      flashTime = BUILD_UINT16(pkt->cmd.Data[1], pkt->cmd.Data[2] );
      HalLedBlink( HAL_LED_4, 4, 50, (flashTime / 4) );
      break;
    
  }
}

void SampleApp_SendPeriodicMessage( void )
{
  if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_PERIODIC_CLUSTERID,
                       1,
                       (uint8*)&SampleAppPeriodicCounter,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
  }
  else
  {
    // Error occurred in request to send.
  }
}


void SampleApp_SendFlashMessage( uint16 flashTime )
{
  uint8 buffer[3];
  buffer[0] = (uint8)(SampleAppFlashCounter++);
  buffer[1] = LO_UINT16( flashTime );
  buffer[2] = HI_UINT16( flashTime ); 

  if ( AF_DataRequest( &SampleApp_Flash_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_FLASH_CLUSTERID,
                       3,
                       buffer,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
  }
  else
  {
    // Error occurred in request to send.
  }
}

//�㲥

//�ն˽ڵ�Ҫ��������
void SampleApp_Send_P2P_Message( void )
{
  //���������ݵķ���
  char mydata[5];
  uint8 hongwai_flag = 0;
  // �������ɼ�
  byte state;
  if(PEOPLE_DATA_PIN == 1)
  { 
    MicroWait (100);     // Wait 10ms
    if(PEOPLE_DATA_PIN == 1)
    {
      state = 0x31;       
      // ���˽��� LED1��
      //LED1 = 0; 
      hongwai_flag = 1;
    }
  }  
  else 
  {  
    state = 0x30;  
    // ���˽��� LED1��
    //LED1 = 1; 
    hongwai_flag = 0;
   
  }  
  
  // ����������������
  mydata[0] = state;
  //

  
    // ���ղɼ�
  uint16 value;
  
  //osal_memset(str, 0, 9);
  value = ReadLightData();
  // ����������
  sprintf(mydata+1, "%03d", value); 
 // ���Թ�������
  //mydata[1] = value;
  HalUARTWrite(0, mydata+1 , 3);
  HalUARTWrite(0, "\n", 1); 
  HalUARTWrite(0, mydata , 1);
  HalUARTWrite(0, "\n", 1); 
  // ����豸���
  mydata[4] = addr;

  
  //�����ݷ��ͳ�ȥ
  if ( AF_DataRequest( &SampleApp_P2P_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_P2P_CLUSTERID,
                       5,
                       (uint8 *)mydata,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
  }
  else
  {
    // Error occurred in request to send.
  }
  

  //  PWM����  ������ֵ��������������ֵ
  if(value <= 55 && hongwai_flag == 1) {
    counter = 0;
    LED1 = 0;
  }else{
    if(counter == 10){
      LED1 = 1;
    }else{
      counter++;
    }
    
  }
   
  
}


// ��ʱ����
void Delay(unsigned int m)
{
    unsigned int i;
    unsigned int j;
    for(i=0;i<m;i++)
      for(j=0;j<109;j++);
}




//������ȡ����
uint16 ReadLightData( void )
{
  uint16 reading = 0;
  
  P0DIR &= ~0x40;  // ����P0.6Ϊ���뷽ʽ
  asm("NOP");asm("NOP");
  
  /* Clear ADC interrupt flag */
  ADCIF = 0;
  
  ADCCON3 = (0x80 | HAL_ADC_DEC_064 | HAL_ADC_CHANNEL_6);//ͨ��6
  
  /* Wait for the conversion to finish */
  while ( !ADCIF );
  
  asm("NOP");asm("NOP");
  
  /* Read the result */
  reading = ADCL;
  reading |= (int16) (ADCH << 8);
  reading >>= 8;
  
  reading = 128 - reading;
  
  return reading;
}

void SampleApp_SerialCMD(mtOSALSerialData_t *cmdMsg)
{
  uint8 i,len,*str=NULL;     //len�������ݳ���
  str=cmdMsg->msg;          //ָ�����ݿ�ͷ
  len=*str;                 //msg��ĵ�1���ֽڴ����������ݳ���

  /********��ӡ�����ڽ��յ������ݣ�������ʾ*********/
  
  // ������Դ��ڷ����������ݽ����ж� ���紮�ڷ���11 1�����豸��� 1������  

  for(i=1;i<=len;i++)
  HalUARTWrite(0,str+i,1 ); 
  HalUARTWrite(0,"\n",1 );//����  
  if(str[1] == '1') {
    SampleApp_P2P_DstAddr.addr.shortAddr = ad1;
  }
  else if(str[1] == '2') {
    SampleApp_P2P_DstAddr.addr.shortAddr = ad2;
  }
  
  if ( AF_DataRequest( &SampleApp_P2P_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_P2P_CLUSTERID,
                       3,
                       str,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
  }
  else
  {

  }
}

