//*****************************************************************************
// Copyright (C) 2014 Texas Instruments Incorporated
//
// All rights reserved. Property of Texas Instruments Incorporated.
// Restricted rights to use, duplicate or disclose this code are
// granted through contract.
// The program may not be used without the written permission of
// Texas Instruments Incorporated or against the terms and conditions
// stipulated in the agreement under which this program has been supplied,
// and under no circumstances can it be used with non-TI connectivity device.
//
//*****************************************************************************

//*****************************************************************************
//
// Application Name     -   MQTT Client
// Application Overview -   This application acts as a MQTT client and connects
//                          to the IBM MQTT broker, simultaneously we can
//                          connect a web client from a web browser. Both
//                          clients can inter-communicate using appropriate
//                          topic names.
//
// Application Details  -
// http://processors.wiki.ti.com/index.php/CC32xx_MQTT_Client
// or
// docs\examples\CC32xx_MQTT_Client.pdf
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup mqtt_client
//! @{
//
//*****************************************************************************

// Standard includes��׼����
#include <stdlib.h>

// simplelink includes
#include "simplelink.h"

// driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "interrupt.h"
#include "rom_map.h"
#include "prcm.h"
#include "uart.h"
#include "timer.h"

// common interface includes�����Ľӿڰ���
#include "network_if.h"
#ifndef NOTERM
#include "uart_if.h"
#endif

#include "button_if.h"
#include "gpio_if.h"
#include "timer_if.h"
#include "common.h"

// Standard includes1
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

// Driverlib includes
#include "utils.h"
#include "hw_common_reg.h"
#include "hw_adc.h"
#include "hw_ints.h"
#include "hw_gprcm.h"
#include "rom.h"
#include "pin.h"
#include "gpio.h"
#include "adc.h"

#include "temp.h"

#include "utils.h"


#include "sl_mqtt_client.h"

// application specific includes �ض�Ӧ�ó������
#include "pinmux.h"

#define APPLICATION_VERSION 	"1.1.1"

/*Operate Lib in MQTT 3.1 mode. ��MQTT 3.1ģʽ�²���Lib.*/
#define MQTT_3_1_1              false /*MQTT 3.1.1 */
#define MQTT_3_1                true /*MQTT 3.1*/

#define WILL_TOPIC              "Client"
#define WILL_MSG                "Client Stopped"
#define WILL_QOS                QOS2
#define WILL_RETAIN             false

/*Defining Broker IP address and port Number. �������IP��ַ�Ͷ˿ں�*/
#define SERVER_ADDRESS          "47.93.253.168"//"123.206.23.115"//"47.101.60.213"//"m14.cloudmqtt.com"//
#define PORT_NUMBER             1883//18534//9000//

#define MAX_BROKER_CONN         1

#define SERVER_MODE             MQTT_3_1
/*Specifying Receive time out for the Receive task. ָ����������Ľ���ʱ��*/
#define RCV_TIMEOUT             30

/*Background receive task priority ���������������ȼ�*/
#define TASK_PRIORITY           3

/* Keep Alive Timer value ά�ּ�ʱ��ֵ*/
#define KEEP_ALIVE_TIMER        25

/*Clean session flag ����Ự��־*/
#define CLEAN_SESSION           true

/*Retain Flag. Used in publish message ������־.���ڷ�����Ϣ. */
#define RETAIN                  1

/*Defining Publish Topic ���巢������*/
#define PUB_TOPIC       "/SmartPot/send"

/*Defining Number of topics ������������*/
#define TOPIC_COUNT             1

/*Defining Subscription Topic Values ���嶩������ֵ*/
#define TOPIC1                  "/SmartPot/receive"

/*�����������*/
#define DateHeatingOn				"\"watering\":\"1\""
#define DateHeatingOff				"\"watering\":\"0\""
#define DateOxygenOn				"\"LED\":\"0\""
#define DateOxygenOff				"\"LED\":\"1\""

/*Defining QOS levels ����QOS����*/
#define QOS0                    0
#define QOS1                    1
#define QOS2                    2

/*Spawn task priority and OSI Stack Size �����������ȼ���OSI��ջ��С*/
#define OSI_STACK_SIZE          2048
#define UART_PRINT              Report

typedef struct connection_config{
    SlMqttClientCtxCfg_t broker_config;
    void *clt_ctx;
    unsigned char *client_id;
    unsigned char *usr_name;
    unsigned char *usr_pwd;
    bool is_clean;
    unsigned int keep_alive_time;
    SlMqttClientCbs_t CallBAcks;
    int num_topics;
    char *topic[TOPIC_COUNT];
    unsigned char qos[TOPIC_COUNT];
    SlMqttWill_t will_params;
    bool is_connected;
}connect_config;

typedef enum
{
    PUSH_BUTTON_SW2_PRESSED,
    PUSH_BUTTON_SW3_PRESSED,
    BROKER_DISCONNECTION
}events;

typedef struct
{
	void * hndl;
	events event;
}event_msg;

//*****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES ���غ���ԭ��
//*****************************************************************************
static void
Mqtt_Recv(void *app_hndl, const char  *topstr, long top_len, const void *payload,
          long pay_len, bool dup,unsigned char qos, bool retain);
static void sl_MqttEvt(void *app_hndl,long evt, const void *buf,
                       unsigned long len);
static void sl_MqttDisconnect(void *app_hndl);
void pushButtonInterruptHandler2();
void pushButtonInterruptHandler3();
void TimerPeriodicIntHandler(void);
void LedTimerConfigNStart();
void LedTimerDeinitStop();
void BoardInit(void);
static void DisplayBanner(char * AppName);
void MqttClient(void *pvParameters);
//*****************************************************************************
//                 GLOBAL VARIABLES -- Start ȫ�ֱ���������ʼ
//*****************************************************************************
#ifdef USE_FREERTOS
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#endif

unsigned short g_usTimerInts;
/* AP Security Parameters AP��ȫ����*/
SlSecParams_t SecurityParams = {0};

/*Message Queue ��Ϣ����*/
OsiMsgQ_t g_PBQueue;

/* connection configuration �������� */
connect_config usr_connect_config[] =
{
    {
        {
            {
                SL_MQTT_NETCONN_URL,
                SERVER_ADDRESS,
                PORT_NUMBER,
                0,
                0,
                0,
                NULL
            },
            SERVER_MODE,
            true,
        },
        NULL,
        "SmartPotUser1",
        //NULL,
        //NULL,
		"aoonsqmp",
		"8cDKdRxcTDXu",
        true,
        KEEP_ALIVE_TIMER,
        {Mqtt_Recv, sl_MqttEvt, sl_MqttDisconnect},
        TOPIC_COUNT,
        {TOPIC1},
        {QOS2},
        {WILL_TOPIC,WILL_MSG,WILL_QOS,WILL_RETAIN},
        false
    }
};

/* library configuration ������*/
SlMqttClientLibCfg_t Mqtt_Client={
    1882,
    TASK_PRIORITY,
    30,
    true,
    (long(*)(const char *, ...))UART_PRINT
};

/*Publishing topics and messages �����������Ϣ*/
const char *pub_topic_sw3 = PUB_TOPIC;
char *data_sw3={"{\"temp\":\"00.00\",\"moisture\":\"0\",\"watering\":\"0\",\"LED\":\"1\"}"};
void *app_hndl = (void*)usr_connect_config;
//*****************************************************************************
//                 GLOBAL VARIABLES -- End ȫ�ֱ�����������
//*****************************************************************************

//****************************************************************************
//! Defines Mqtt_Pub_Message_Receive event handler. ����Mqtt_Pub_Message_Receive�¼��������
//! Client App needs to register this event handler with sl_ExtLib_mqtt_Init �ͻ���Ӧ�ó�����Ҫ��slextlibmqttinitע������¼��������
//! API. Background receive task invokes this handler whenever MQTT Client API.��̨����������MQTT�ͻ��˵��ô˴������
//! receives a Publish Message from the broker. �Ӵ������շ�����Ϣ��
//!
//!\param[out]     topstr => pointer to topic of the message ָ����Ϣ�����ָ��
//!\param[out]     top_len => topic length ����ĳ���
//!\param[out]     payload => pointer to payload ָ��ָ����
//!\param[out]     pay_len => payload length ��Ч�غɳ���
//!\param[out]     retain => Tells whether its a Retained message or not �����������Ƿ�������Ϣ
//!\param[out]     dup => Tells whether its a duplicate message or not ������������һ���ظ�����Ϣ
//!\param[out]     qos => Tells the Qos level ������Qos����
//!
//!\return none
//****************************************************************************
static void
Mqtt_Recv(void *app_hndl, const char  *topstr, long top_len, const void *payload,
                       long pay_len, bool dup,unsigned char qos, bool retain)
{
    //*****

    char *output_str=(char*)malloc(top_len+1);
    memset(output_str,'\0',top_len+1);
    strncpy(output_str, (char*)topstr, top_len);
    output_str[top_len]='\0';

    if(strncmp(output_str,TOPIC1, top_len) == 0)
    {
    //    ToggleLedState(LED1);
    //}

    UART_PRINT("\n\rPublish Message Received");
    UART_PRINT("\n\rTopic: ");
    UART_PRINT("%s",output_str);
    free(output_str);
    UART_PRINT(" [Qos: %d] ",qos);
    if(retain)
      UART_PRINT(" [Retained]");
    if(dup)
      UART_PRINT(" [Duplicate]");
    
    output_str=(char*)malloc(pay_len+1);
    memset(output_str,'\0',pay_len+1);
    strncpy(output_str, (char*)payload, pay_len);
    output_str[pay_len]='\0';
    /////////////////////////////////////
    //�Ƿ������Ȱ�
    char *pay1 = strchr(output_str,',');
    char *pay2 =pay1-15;
    char c = *pay1;
    *pay1 = '\0';
    char *dev=(char*)malloc(pay_len+1);
    strcpy(dev,pay2+1);
    UART_PRINT("\n\rdate1: %s\n\r",dev);
    *pay1 = c;
    if(strcmp(dev,DateHeatingOn) == 0)
    {
    	MAP_GPIOPinWrite(GPIOA2_BASE,0x02,0x02);
    	*(data_sw3+43) = '1';
    	UART_PRINT("heating condition is: on\n\r");
    }
    else if(strcmp(dev,DateHeatingOff) == 0)
    {
    	MAP_GPIOPinWrite(GPIOA2_BASE,0x02,0);
    	*(data_sw3+43) = '0';
    	UART_PRINT("heating condition is: off\n\r");
    }
    free(dev);

    //�Ƿ���������
    pay2 = pay1;
    dev=(char*)malloc(pay_len+1);
    pay1 = strchr(pay1+1,'}');
    c = *pay1;
    *pay1 = '\0';
    strcpy(dev,pay2+1);
    UART_PRINT("\n\rdate2: %s\n\r",dev);
    *pay1 = c;
    if(strcmp(dev,DateOxygenOn) == 0)
        {
        	MAP_GPIOPinWrite(GPIOA1_BASE,0x04,0x04);
        	*(data_sw3+53) = '0';
        	UART_PRINT("oxygen condition is: on\n\r");
        }
    else if(strcmp(dev,DateOxygenOff) == 0)
        {
        	MAP_GPIOPinWrite(GPIOA1_BASE,0x04,0);
        	*(data_sw3+53) = '1';
        	UART_PRINT("oxygen condition is: off\n\r");
        }
    free(dev);

    /////////////////////////////////////

    UART_PRINT("\n\rData is: ");
    UART_PRINT("%s",(char*)output_str);
    UART_PRINT("\n\r");
    free(output_str);
    }
    return;
}

//****************************************************************************
//! Defines sl_MqttEvt event handler. ����sl_MqttEvt�¼��������
//! Client App needs to register this event handler with sl_ExtLib_mqtt_Init �ͻ���Ӧ�ó�����Ҫ��slextlibmqttinitע������¼��������
//! API. Background receive task invokes this handler whenever MQTT Client 
//! receives an ack(whenever user is in non-blocking mode) or encounters an error.
//!
//! param[out]      evt => Event that invokes the handler. Event can be of the
//!                        following types: ���ô��������¼����¼��������������ͣ�
//!                        MQTT_ACK - Ack Received Ack�յ�
//!                        MQTT_ERROR - unknown error δ֪�Ĵ���
//!                        
//!  
//! \param[out]     buf => points to buffer ָ�򻺳���
//! \param[out]     len => buffer length ����������
//!       
//! \return none
//****************************************************************************
static void
sl_MqttEvt(void *app_hndl, long evt, const void *buf,unsigned long len)
{
    int i;
    switch(evt)
    {
      case SL_MQTT_CL_EVT_PUBACK:
        UART_PRINT("PubAck:\n\r");
        UART_PRINT("%s\n\r",buf);
        break;
    
      case SL_MQTT_CL_EVT_SUBACK:
        UART_PRINT("\n\rGranted QoS Levels are:\n\r");
        
        for(i=0;i<len;i++)
        {
          UART_PRINT("QoS %d\n\r",((unsigned char*)buf)[i]);
        }
        break;
        
      case SL_MQTT_CL_EVT_UNSUBACK:
        UART_PRINT("UnSub Ack \n\r");
        UART_PRINT("%s\n\r",buf);
        break;
    
      default:
        break;
  
    }
}

//****************************************************************************
//
//! callback event in case of MQTT disconnection ��MQTT�Ͽ�������»ص��¼�
//!
//! \param app_hndl is the handle for the disconnected connection �����ӶϿ����ӵľ��
//!
//! return none
//
//****************************************************************************
static void
sl_MqttDisconnect(void *app_hndl)
{
    connect_config *local_con_conf;
    event_msg msg;
    local_con_conf = app_hndl;
    msg.hndl = app_hndl;
    msg.event = BROKER_DISCONNECTION;

    UART_PRINT("disconnect from broker %s\r\n",
           (local_con_conf->broker_config).server_info.server_addr);
    local_con_conf->is_connected = false;
    //
    // write message indicating publish message д��Ϣָʾ������Ϣ
    //
    osi_MsgQWrite(&g_PBQueue,&msg,OSI_NO_WAIT);

}

//****************************************************************************
//
//! Push Button Handler1(GPIOS2). Press push button2 (GPIOSW2) Whenever user
//! wants to publish a message. Write message into message queue signaling the
//!    event publish messages
//!��ťHandler1(GPIOS2)�����û���Ҫ������Ϣʱ�����°�ť2��gpiv2��������Ϣд����Ϣ���У������¼�������Ϣ
//!
//! \param none
//!
//! return none
//
//****************************************************************************
void pushButtonInterruptHandler2()
{
	event_msg msg;

    msg.event = PUSH_BUTTON_SW2_PRESSED;
    msg.hndl = NULL;
    //
    // write message indicating publish message д��Ϣָʾ������Ϣ
    //
    osi_MsgQWrite(&g_PBQueue,&msg,OSI_NO_WAIT);
}

//****************************************************************************
//
//! Push Button Handler3(GPIOS3). Press push button3 (GPIOSW3) Whenever user
//! wants to publish a message. Write message into message queue signaling the
//!    event publish messages
//!��ťHandler3(GPIOS3)�����û���Ҫ������Ϣʱ�����°�ť3��gpiv3��������Ϣд����Ϣ���У������¼�������Ϣ
//!
//! \param none
//!
//! return none
//
//****************************************************************************
void pushButtonInterruptHandler3()
{
	event_msg msg;
	msg.event = PUSH_BUTTON_SW3_PRESSED;
    msg.hndl = NULL;
    //
    // write message indicating exit from sending loop д��Ϣָʾ�˳�����ѭ��
    //
    osi_MsgQWrite(&g_PBQueue,&msg,OSI_NO_WAIT);

}

//*****************************************************************************
//
//! Periodic Timer Interrupt Handler ���ڶ�ʱ���жϴ������
//!
//! \param None
//!
//! \return None
//
//*****************************************************************************
void
TimerPeriodicIntHandler(void)
{
    unsigned long ulInts;

    //
    // Clear all pending interrupts from the timer we are
    // currently using. ������ǵ�ǰ����ʹ�õļ�ʱ���е�����δ���жϡ�
    //
    ulInts = MAP_TimerIntStatus(TIMERA0_BASE, true);
    MAP_TimerIntClear(TIMERA0_BASE, ulInts);

    //
    // Increment our interrupt counter. �������ǵ��жϡ�
    //
    g_usTimerInts++;
    if(!(g_usTimerInts & 0x1))
    {
        //
        // Off Led �ر�LED
        //
        GPIO_IF_LedOff(MCU_RED_LED_GPIO);
    }
    else
    {
        //
        // On Led ��LED
        //
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
    }
}

//****************************************************************************
//
//! Function to configure and start timer to blink the LED while device is
//! trying to connect to an AP ���豸��ͼ���ӵ�AP��ʱ�����ú�������ʱ������˸LED��
//!
//! \param none
//!
//! return none
//
//****************************************************************************
void LedTimerConfigNStart()
{
    //
    // Configure Timer for blinking the LED for IP acquisition ���ü�ʱ����������˸LED����IP��ȡ
    //
    Timer_IF_Init(PRCM_TIMERA0,TIMERA0_BASE,TIMER_CFG_PERIODIC,TIMER_A,0);
    Timer_IF_IntSetup(TIMERA0_BASE,TIMER_A,TimerPeriodicIntHandler);
    Timer_IF_Start(TIMERA0_BASE,TIMER_A,100);
}

//****************************************************************************
//
//! Disable the LED blinking Timer as Device is connected to AP ���豸���ӵ�APʱ������LED��˸��ʱ��
//!
//! \param none
//!
//! return none
//
//****************************************************************************
void LedTimerDeinitStop()
{
    //
    // Disable the LED blinking Timer as Device is connected to AP ���豸���ӵ�APʱ������LED��˸��ʱ��
    //
    Timer_IF_Stop(TIMERA0_BASE,TIMER_A);
    Timer_IF_DeInit(TIMERA0_BASE,TIMER_A);

}

//*****************************************************************************
//
//! Board Initialization & Configuration ���ʼ��������
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
void BoardInit(void)
{
    /* In case of TI-RTOS vector table is initialize by OS itself ���TI-RTOSʸ�������ɲ���ϵͳ�����ʼ����*/
    #ifndef USE_TIRTOS
    //
    // Set vector table base ��ʸ�������
    //
    #if defined(ccs)
        IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
    #endif
    #if defined(ewarm)
        IntVTableBaseSet((unsigned long)&__vector_table);
    #endif
    #endif
    //
    // Enable Processor ������ʹ��
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

//*****************************************************************************
//
//! Application startup display on UART Ӧ�ó���������ʾ��UART��
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
static void
DisplayBanner(char * AppName)
{

    UART_PRINT("\n\n\n\r");
    UART_PRINT("\t\t *************************************************\n\r");
    UART_PRINT("\t\t    CC3200 %s Application       \n\r", AppName);
    UART_PRINT("\t\t *************************************************\n\r");
    UART_PRINT("\n\n\n\r");
}
  
extern volatile unsigned long g_ulStatus;
//*****************************************************************************
//
//! Task implementing MQTT client communication to other web client through
//!    a broker ͨ��������ʵ��MQTT�ͻ���ͨ�ŵ�����web�ͻ��˵�����
//!
//! \param  none
//!
//! This function
//!    1. Initializes network driver and connects to the default AP ��ʼ�����������������ӵ�Ĭ��AP
//!    2. Initializes the mqtt library and set up MQTT connection configurations ��ʼ��MQTT�Ⲣ����MQTT��������
//!    3. set up the button events and their callbacks(for publishing) ���ð�ť�¼������ǵĻص������ڷ�����
//!    4. handles the callback signals ����ص��ź�
//!
//! \return None
//!
//*****************************************************************************
void MqttClient(void *pvParameters)
{
    
    long lRetVal = -1;
    int iCount = 0;
    int iNumBroker = 0;
    int iConnBroker = 0;
    event_msg RecvQue;
    unsigned char policyVal;
    
    connect_config *local_con_conf = (connect_config *)app_hndl;

    //
    // Configure LED ����LED
    //
    GPIO_IF_LedConfigure(LED1|LED2|LED3);

    GPIO_IF_LedOff(MCU_RED_LED_GPIO);
    GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);

    //
    // Reset The state of the machine ���û�����״̬
    //
    Network_IF_ResetMCUStateMachine();

    //
    // Start the driver ������������
    //
    lRetVal = Network_IF_InitDriver(ROLE_STA);
    if(lRetVal < 0)
    {
       UART_PRINT("Failed to start SimpleLink Device\n\r",lRetVal);
       LOOP_FOREVER();
    }

    // switch on Green LED to indicate Simplelink is properly up ����ɫLED��ʾSimplelink��������
    GPIO_IF_LedOn(MCU_ON_IND);

    // Start Timer to blink Red LED till AP connection ������ʱ������˸��ɫLEDֱ��AP����
    LedTimerConfigNStart();

    // Initialize AP security params AP��ȫ��ʼ������
    SecurityParams.Key = (signed char *)SECURITY_KEY;
    SecurityParams.KeyLen = strlen(SECURITY_KEY);
    SecurityParams.Type = SECURITY_TYPE;

    //
    // Connect to the Access Point ���ӵ����ʵ�
    //
    lRetVal = Network_IF_ConnectAP(SSID_NAME, SecurityParams);
    if(lRetVal < 0)
    {
       UART_PRINT("Connection to an AP failed\n\r");
       LOOP_FOREVER();
    }

    lRetVal = sl_WlanProfileAdd(SSID_NAME,strlen(SSID_NAME),0,&SecurityParams,0,1,0);

    //set AUTO policy
    lRetVal = sl_WlanPolicySet(SL_POLICY_CONNECTION,
                      SL_CONNECTION_POLICY(1,0,0,0,0),
                      &policyVal, 1 /*PolicyValLen*/);    
    
    //
    // Disable the LED blinking Timer as Device is connected to AP ���豸���ӵ�APʱ������LED��˸��ʱ��
    //
    LedTimerDeinitStop();

    //
    // Switch ON RED LED to indicate that Device acquired an IP �򿪺�ɫLED��ʾ�豸�����IP
    //
    GPIO_IF_LedOn(MCU_IP_ALLOC_IND);

    UtilsDelay(20000000);

    GPIO_IF_LedOff(MCU_RED_LED_GPIO);
    GPIO_IF_LedOff(MCU_ORANGE_LED_GPIO);
    GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);
   
    //
    // Register Push Button Handlers ע�ᰴť�Ĵ������
    //
    Button_IF_Init(pushButtonInterruptHandler2,pushButtonInterruptHandler3);
    
    //
    // Initialze MQTT client lib ��ʼ��MQTT�ͻ���lib
    //
    lRetVal = sl_ExtLib_MqttClientInit(&Mqtt_Client);
    if(lRetVal != 0)
    {
        // lib initialization failed ���ɳ�ʼ��ʧ��
        UART_PRINT("MQTT Client lib initialization failed\n\r");
        LOOP_FOREVER();
    }
    
    /******************* connection to the broker ���ӵ����� ***************************/
    iNumBroker = sizeof(usr_connect_config)/sizeof(connect_config);
    if(iNumBroker > MAX_BROKER_CONN)
    {
        UART_PRINT("Num of brokers are more then max num of brokers\n\r");
        LOOP_FOREVER();
    }

connect_to_broker:
    while(iCount < iNumBroker)
    {
        //create client context �����ͻ���������
        local_con_conf[iCount].clt_ctx =
        sl_ExtLib_MqttClientCtxCreate(&local_con_conf[iCount].broker_config,
                                      &local_con_conf[iCount].CallBAcks,
                                      &(local_con_conf[iCount]));

        //
        // Set Client ID ���ÿͻ���ID
        //
        sl_ExtLib_MqttClientSet((void*)local_con_conf[iCount].clt_ctx,
                            SL_MQTT_PARAM_CLIENT_ID,
                            local_con_conf[iCount].client_id,
                            strlen((char*)(local_con_conf[iCount].client_id)));

        //
        // Set will Params ���ò���
        //
        if(local_con_conf[iCount].will_params.will_topic != NULL)
        {
            sl_ExtLib_MqttClientSet((void*)local_con_conf[iCount].clt_ctx,
                                    SL_MQTT_PARAM_WILL_PARAM,
                                    &(local_con_conf[iCount].will_params),
                                    sizeof(SlMqttWill_t));
        }

        //
        // setting username and password �����û���������
        //
        if(local_con_conf[iCount].usr_name != NULL)
        {
            sl_ExtLib_MqttClientSet((void*)local_con_conf[iCount].clt_ctx,
                                SL_MQTT_PARAM_USER_NAME,
                                local_con_conf[iCount].usr_name,
                                strlen((char*)local_con_conf[iCount].usr_name));

            if(local_con_conf[iCount].usr_pwd != NULL)
            {
                sl_ExtLib_MqttClientSet((void*)local_con_conf[iCount].clt_ctx,
                                SL_MQTT_PARAM_PASS_WORD,
                                local_con_conf[iCount].usr_pwd,
                                strlen((char*)local_con_conf[iCount].usr_pwd));
            }
        }

        //
        // connection to the broker ���ӵ�����
        //
        if((sl_ExtLib_MqttClientConnect((void*)local_con_conf[iCount].clt_ctx,
                            local_con_conf[iCount].is_clean,
                            local_con_conf[iCount].keep_alive_time) & 0xFF) != 0)
        {
            UART_PRINT("\n\rBroker connect fail for conn no. %d \n\r",iCount+1);
            
            //delete the context for this connection ɾ�������ӵ�������
            sl_ExtLib_MqttClientCtxDelete(local_con_conf[iCount].clt_ctx);
            
            break;
        }
        else
        {
            UART_PRINT("\n\rSuccess: conn to Broker no. %d\n\r ", iCount+1);
            local_con_conf[iCount].is_connected = true;
            iConnBroker++;
        }

        //
        // Subscribe to topics ��������
        //

        if(sl_ExtLib_MqttClientSub((void*)local_con_conf[iCount].clt_ctx,
                                   local_con_conf[iCount].topic,
                                   local_con_conf[iCount].qos, TOPIC_COUNT) < 0)
        {
            UART_PRINT("\n\r Subscription Error for conn no. %d\n\r", iCount+1);
            UART_PRINT("Disconnecting from the broker\r\n");
            sl_ExtLib_MqttClientDisconnect(local_con_conf[iCount].clt_ctx);
            local_con_conf[iCount].is_connected = false;
            
            //delete the context for this connection ɾ�������ӵ�������
            sl_ExtLib_MqttClientCtxDelete(local_con_conf[iCount].clt_ctx);
            iConnBroker--;
            break;
        }
        else
        {
            int iSub;
            UART_PRINT("Client subscribed on following topics:\n\r");
            for(iSub = 0; iSub < local_con_conf[iCount].num_topics; iSub++)
            {
                UART_PRINT("%s\n\r", local_con_conf[iCount].topic[iSub]);
            }
        }
        iCount++;
    }

    if(iConnBroker < 1)
    {
        //
        // no succesful connection to broker ��brokerû�гɹ�����ϵ
        //
        goto end;
    }

    iCount = 0;

    for(;;)
    {
        osi_MsgQRead( &g_PBQueue, &RecvQue, OSI_WAIT_FOREVER);

        pushButtonInterruptHandler3();
        osi_Sleep(600);
        if(PUSH_BUTTON_SW3_PRESSED == RecvQue.event)
        {
            //Button_IF_EnableInterrupt(SW3);
            //
            // send publish message ���ͷ�����Ϣ
            //
        	//float temp = 23.25;
        	float temp = ReadTemp();
            int temp1 = (int)temp;
            int temp2 = (int)((int)(100*temp)%100);
            itoa(temp1, data_sw3+9);
            itoa(temp2, data_sw3+12);

            int LiquidLevel = 0;
            if(MAP_GPIOPinRead(GPIOA1_BASE,0x80))
            {
            	LiquidLevel = 1;
            }
            else
            {
            	LiquidLevel = 0;
            }
            itoa(LiquidLevel,data_sw3+28);
            sl_ExtLib_MqttClientSend((void*)local_con_conf[iCount].clt_ctx,
                    pub_topic_sw3,data_sw3,strlen((char*)data_sw3),QOS2,RETAIN);
            UART_PRINT("\n\r CC3200 Publishes the following message \n\r");
            UART_PRINT("Topic: %s\n\r",pub_topic_sw3);
            UART_PRINT("Data: %s\n\r",data_sw3);
            //UART_PRINT("\n\rpH Voltage is %f\n\r",ReadPh());//ReadPh();
        }
        else if(BROKER_DISCONNECTION == RecvQue.event)

        {
            iConnBroker--;
            /* Derive the value of the local_con_conf or clt_ctx from the message ����Ϣ�л��local_con_conf��clt_ctx��ֵ*/
			sl_ExtLib_MqttClientCtxDelete(((connect_config*)(RecvQue.hndl))->clt_ctx);
            
            if(!IS_CONNECTED(g_ulStatus))
            {
                UART_PRINT("device has disconnected from AP \n\r");
                
                UART_PRINT("retry connection to the AP\n\r");
                
                while(!(IS_CONNECTED(g_ulStatus)) || !(IS_IP_ACQUIRED(g_ulStatus)))
                {
                    osi_Sleep(10);
                }
                goto connect_to_broker;
                
            }
            if(iConnBroker < 1)
            {
                //
                // device not connected to any broker �������κδ�����豸
                //
                goto end;
            }
        }
    }
end:
    //
    // Deinitia lizating the client library ȡ���ͻ��˿⣿
    //
    sl_ExtLib_MqttClientExit();
    UART_PRINT("\n\r Exiting the Application\n\r");
    
    LOOP_FOREVER();
}

//*****************************************************************************
//
//! Main 
//!
//! \param  none
//!
//! This function
//!    1. Invokes the SLHost task ����SLHost����
//!    2. Invokes the MqttClient ����MqttClient
//!
//! \return None
//!
//*****************************************************************************
void main()
{ 
    long lRetVal = -1;
    //
    // Initialize the board configurations ���г�ʼ������
    //
    BoardInit();

    //
    // Pinmux for UART
    //
    PinMuxConfig();

    //
    // Configuring UART
    //
    InitTerm();

    //
    // Display Application Banner ��ʾӦ�ó����Banner
    //
    DisplayBanner("MQTT_Client");

    //
    // Start the SimpleLink Host ��ʼSimpleLink����
    //
    lRetVal = VStartSimpleLinkSpawnTask(SPAWN_TASK_PRIORITY);
    if(lRetVal < 0)
    {
        ERR_PRINT(lRetVal);
        LOOP_FOREVER();
    }

    //
    // Start the MQTT Client task ����MQTT�ͻ�������
    //
    osi_MsgQCreate(&g_PBQueue,"PBQueue",sizeof(event_msg),10);
    lRetVal = osi_TaskCreate(MqttClient,
                            (const signed char *)"Mqtt Client App",
                            OSI_STACK_SIZE, NULL, 2, NULL );

    if(lRetVal < 0)
    {
        ERR_PRINT(lRetVal);
        LOOP_FOREVER();
    }
    //
    // Start the task scheduler �������������
    //
    osi_start();
}

