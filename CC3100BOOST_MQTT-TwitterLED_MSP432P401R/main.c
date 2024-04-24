// Standard includes
#include <stdlib.h>
#include <string.h>

#include "driverlib.h"
#include "simplelink.h"
#include "sl_common.h"
#include "MQTTClient.h"
#include "../inc/Clock.h"
#include "..\inc\TimerA0.h"
#include "..\inc\PWM.h"
#include "..\inc\MotorSimple.h"
#include "../inc/CortexM.h"
#include "../inc/Motor.h"
#include "../inc/TimerA1.h"
#include "../inc/TimerA2.h"
#include "../inc/Tachometer.h"
#include "../inc/I2CB1.h"
#include "../inc/opt3101.h"
#include "../inc/LaunchPad.h"
#include "../inc/Reflectance.h"
#include "msp.h"
#include "../inc/LPF.h"
#include "../inc/Bump.h"
#include "../inc/UART0.h"
#include "../inc/SSD1306.h"
#include "../inc/FFT.h"

// Select one of the following three output possibilities
// define USENOKIA
#define USEOLED 1
#define USEUART 1

#ifdef USENOKIA
// this batch configures for LCD
#include "../inc/Nokia5110.h"
#define Init Nokia5110_Init
#define Clear Nokia5110_Clear
#define SetCursor Nokia5110_SetCursor
#define OutString Nokia5110_OutString
#define OutChar Nokia5110_OutChar
#define OutUDec Nokia5110_OutUDec
#define OutSDec Nokia5110_OutSDec
#endif

#ifdef USEOLED
// this batch configures for OLED

void OLEDinit(void){SSD1306_Init(SSD1306_SWITCHCAPVCC);}
#define Init OLEDinit
#define Clear SSD1306_Clear
#define SetCursor SSD1306_SetCursor
#define OutChar SSD1306_OutChar
#define OutString SSD1306_OutString
#define OutUDec SSD1306_OutUDec
#define OutSDec SSD1306_OutSDec
#endif

#ifdef USEUART
// this batch configures for UART link to PC
#include "../inc/UART0.h"
void UartSetCur(uint8_t newX, uint8_t newY){
  if(newX == 6){
    UART0_OutString("\n\rTxChannel= ");
    UART0_OutUDec(newY-1);
    UART0_OutString(" Distance= ");
  }else{
    UART0_OutString("\n\r");
  }
}
void UartClear(void){UART0_OutString("\n\r");};
#define Init UART0_Init
#define Clear UartClear
#define SetCursor UartSetCur
#define OutString UART0_OutString
#define OutChar UART0_OutChar
#define OutUDec UART0_OutUDec
#define OutSDec UART0_OutSDec
#endif


uint32_t Distances[3];
uint32_t FilteredDistances[3];
uint32_t Amplitudes[3];
uint32_t Noises[3];
uint32_t TxChannel;
uint32_t StartTime;
uint32_t TimeToConvert; // in msec
bool pollDistanceSensor(void){
  if(OPT3101_CheckDistanceSensor()){
    TxChannel = OPT3101_GetMeasurement(Distances,Amplitudes);
    return true;
  }
  return false;
}




// calibrated for 500mm track
// right is raw sensor data from right sensor
// return calibrated distance from center of Robot to right wall
int32_t Right(int32_t right){
  return  (right*(59*right + 7305) + 2348974)/32768;
}
// left is raw sensor data from left sensor
// return calibrated distance from center of Robot to left wall
int32_t Left(int32_t left){
  return (1247*left)/2048 + 22;
}

void mainUNO(void){ // main3interrupt implementation
  int i = 0;
  uint32_t channel = 1;
  DisableInterrupts();
  Clock_Init48MHz();
  SysTick->LOAD = 0x00FFFFFF;           // maximum reload value
  SysTick->CTRL = 0x00000005;           // enable SysTick with no interrupts
  I2CB1_Init(30); // baud rate = 12MHz/30=400kHz
  Init();
  Clear();
  OutString("OPT3101");
#ifndef USEUART
  SetCursor(0, 1);
  OutString("Left =");
  SetCursor(0, 2);
  OutString("Centr=");
  SetCursor(0, 3);
  OutString("Right=");
  SetCursor(0, 4);
  OutString("Interrupts");
  SetCursor(0, 5);
  OutString("NL=");
  SetCursor(0, 6);
  OutString("NC=");
  SetCursor(0, 7);
  OutString("NR=");
#endif
  OPT3101_Init();
  OPT3101_Setup();
  OPT3101_CalibrateInternalCrosstalk();
  OPT3101_ArmInterrupts(&TxChannel, Distances, Amplitudes);
  StartTime = SysTick->VAL;
  TxChannel = 3;
  OPT3101_StartMeasurementChannel(channel);
  LPF_Init(100,8);
  LPF_Init2(100,8);
  LPF_Init3(100,8);
#ifdef USEUART
  OutString("Digital filter size=8\n");
#endif
  EnableInterrupts();
  while(1){
    if(TxChannel <= 2){ // 0,1,2 means new data
      if(TxChannel==0){
        FilteredDistances[0] = Left(LPF_Calc(Distances[0]));
      }else if(TxChannel==1){
        FilteredDistances[1] = LPF_Calc2(Distances[1]);
      }else {
        FilteredDistances[2] = Right(LPF_Calc3(Distances[2]));
      }
#ifndef USEUART
      SetCursor(6, TxChannel+1);
      OutUDec(FilteredDistances[TxChannel]);
#endif
      TxChannel = 3; // 3 means no data
      channel = (channel+1)%3;
      OPT3101_StartMeasurementChannel(channel);
      i = i + 1;
    }
    if(i >= 90){ // every 3 seconds
      i = 0;
      Noises[0] = Noise();
      Noises[1] = Noise2();
      Noises[2] = Noise3();
#ifdef USEUART
      OutChar('\n');
      SetCursor(0, 1);
      OutString("Left ="); OutUDec(Distances[0]); OutChar(','); OutUDec(FilteredDistances[0]);
      SetCursor(0, 2);
      OutString("Centr="); OutUDec(Distances[1]); OutChar(','); OutUDec(FilteredDistances[1]);
      SetCursor(0, 3);
      OutString("Right="); OutUDec(Distances[2]); OutChar(',');  OutUDec(FilteredDistances[2]);
      SetCursor(3, 5);
      OutString("NL="); OutUDec((uint16_t)Noises[0]);  OutChar(','); OutUDec(Amplitudes[0]);
      SetCursor(3, 6);
      OutString("NC="); OutUDec((uint16_t)Noises[1]); OutChar(','); OutUDec(Amplitudes[1]);
      SetCursor(3, 7);
      OutString("NR="); OutUDec((uint16_t)Noises[2]); OutChar(','); OutUDec(Amplitudes[2]);
#else
      SetCursor(3, 5);
      OutUDec((uint16_t)Noises[0]);  OutChar(','); OutUDec(Amplitudes[0]);
      SetCursor(3, 6);
      OutUDec((uint16_t)Noises[1]); OutChar(','); OutUDec(Amplitudes[1]);
      SetCursor(3, 7);
      OutUDec((uint16_t)Noises[2]); OutChar(','); OutUDec(Amplitudes[2]);
#endif
    }
    WaitForInterrupt();
  }
}


#define N 1024
uint32_t Data[N];
#define M 1024
uint16_t Histogram[M];
uint32_t Sum;      // sum of data
uint32_t Sum2;     // sum of (data-average)^2
uint32_t Average;  // average of data = sum/N
uint32_t Variance; // =sum2/(N-1)
uint32_t Sigma;    // standard deviation = sqrt(Variance)


// assumes track is 500mm
int32_t Mode=0; // 0 stop, 1 run
int32_t Error;
//int32_t Ki=1;  // integral controller gain
//int32_t Kp=4;  // proportional controller gain //was 4
//int32_t UR, UL;  // PWM duty 0 to 14,998

#define TOOCLOSE 200 //was 200
#define DESIRED 250 //was 250
#define TOOFAR 400 // was 400

//change constants
int32_t Kp=18;  // proportional controller gain //was 18,   30
int32_t Ki=0;  // integral controller gain      //was 0 ,   0
int32_t Kd=3;  // derivtive controller gain     //was 3 ,   2
int32_t PWMNOMINAL = 6500; // was 2500             //was 5000  7000
int32_t SWING = 2000; //was 1000                //was 2000  2000
int32_t SWING_CONSTANT = 60;                    //was 60    70
#define MAXSWING 2000                           //was 2000
#define DESIRED 250                             //was 250   100
int32_t UR, UL;  // PWM duty 0 to 14,998
int32_t SetPoint = 250; // mm //was 250
int32_t LeftDistance,CenterDistance,RightDistance; // mm

//#define PWMNOMINAL 5000 // was 2500
//#define SWING 2000 //was 1000
//#define PWMMIN (PWMNOMINAL-SWING)
//#define PWMMAX (PWMNOMINAL+SWING)

int32_t errorL;
int32_t errorR;
int32_t perrorL = 0;
int32_t perrorR = 0;
int32_t integralL;
int32_t integralR;
int32_t derL;
int32_t derR;
int32_t der;
int32_t pError = 0;

void Controller(void){ // runs at 100 Hz
    if(Mode){
    //    if((LeftDistance>DESIRED) && (RightDistance>DESIRED)){ // looking for if both are lost go straight
    //      SetPoint = (LeftDistance+RightDistance)/2;
    //    } else{
    //      SetPoint = DESIRED;
    //    }


        if( (LeftDistance >= DESIRED)){ // when going straight is null and left is null
            SetPoint = (CenterDistance + LeftDistance)/2;             // then go left meaning empty space(CenterDistance >= DESIRED) &&
        } else if( (RightDistance >= DESIRED)){
            SetPoint = (CenterDistance + RightDistance)/2;
        } else {
            SetPoint = DESIRED;
        }


//        if( (RightDistance >= DESIRED +250) && LeftDistance < DESIRED){
//                Motor_Forward(8000, 3500);
//        }
//            else if((LeftDistance >= DESIRED+250) && (RightDistance < DESIRED )){
//                Motor_Forward(3500, 8000);
//              }
//        else{

            //Calculate Error signals
            errorL = SetPoint - LeftDistance;
            errorR = SetPoint - RightDistance;

            integralL += errorL;
            integralR += errorR;

            derL = errorL - perrorL;
            derR = errorR - perrorR;

            perrorL = errorL;
            perrorR = errorR;

            UL = PWMNOMINAL + (Kp *errorL) + (Ki * integralL) + (Kd * derL);
            UR = PWMNOMINAL + (Kp *errorR) + (Ki * integralR) + (Kd * derR);

            // PWMNOMINAL -> max duty cycle
            // SWING      -> offset

            // turning right
            if(UR < (PWMNOMINAL-SWING)){ // <
                UR = PWMNOMINAL-SWING; // 3,000 to 7,000


            }

            // turning left
            if(UR > (PWMNOMINAL+SWING)){ // >
                UR = PWMNOMINAL+SWING;
        //            SWING += SWING_CONSTANT;
            }

            // turning  left
            if(UL < (PWMNOMINAL-SWING)){ // <
                UL = PWMNOMINAL-SWING; // 3,000 to 7,000
        //            SWING += SWING_CONSTANT;
            }

            // turning right
            if(UL > (PWMNOMINAL+SWING)){ // >
                UL = PWMNOMINAL+SWING;
        //            SWING += SWING_CONSTANT;
            }
            SWING += SWING_CONSTANT;
            if(SWING <= MAXSWING){
                SWING = MAXSWING;
            }

            Motor_Forward(UL,UR);
//        }

      }

}

void Controller_Right(void){ // runs at 100 Hz
    PWMNOMINAL = 3500;

  if(Mode){
//    if((RightDistance>DESIRED)){
//      SetPoint = (RightDistance)/2;
//    }else{
//      SetPoint = DESIRED;
//    }
    /*if(LeftDistance < RightDistance ){
      Error = LeftDistance-SetPoint;
    }else {
      Error = SetPoint-RightDistance;
    }*/

    // my thoughts

      if( (LeftDistance >= DESIRED)){ // when going straight is null and left is null
          SetPoint = (CenterDistance + LeftDistance)/2;             // then go left meaning empty space(CenterDistance >= DESIRED) &&
      } else if( (RightDistance >= DESIRED)){
          SetPoint = (CenterDistance + RightDistance)/2;
      } else {
          SetPoint = DESIRED;
      }


    Error = SetPoint-RightDistance;
    //UR = UR + Ki*Error;      // adjust right motor
    UR = PWMNOMINAL+ 25*Error+ 3*der; // proportional control
    UR = UR + 1*Error;      // adjust right motor
    UL = PWMNOMINAL- 25*Error+ 3*der; // proportional control
    if(UR < (PWMNOMINAL-SWING)){ // <
                UR = PWMNOMINAL-SWING; // 3,000 to 7,000


            }

            // turning left
            if(UR > (PWMNOMINAL+SWING)){ // >
                UR = PWMNOMINAL+SWING;
        //            SWING += SWING_CONSTANT;
            }

            // turning  left
            if(UL < (PWMNOMINAL-SWING)){ // <
                UL = PWMNOMINAL-SWING; // 3,000 to 7,000
        //            SWING += SWING_CONSTANT;
            }

            // turning right
            if(UL > (PWMNOMINAL+SWING)){ // >
                UL = PWMNOMINAL+SWING;
        //            SWING += SWING_CONSTANT;
            }
            SWING += SWING_CONSTANT;
            if(SWING <= MAXSWING){
                SWING = MAXSWING;
            }

    //turns left if the center measurement and right measurement is small enough that we will hit the wall if we don't turn
    if((RightDistance<250) && (CenterDistance <250)){
        UL = 0;
        UR = PWMNOMINAL;
    }

    Motor_Forward(UL,UR);

  }
}



void Controller_Left(void){ // runs at 100 Hz
  if(Mode){
    if((LeftDistance>DESIRED)){
      SetPoint = (LeftDistance)/2;
    }else{
      SetPoint = DESIRED;
    }

    //int der;
    //int Error;
    Error = SetPoint-LeftDistance;

    der = Error - pError;

    pError = Error;

    //UR = UR + Ki*Error;      // adjust right motor
    UL = PWMNOMINAL+ 32*Error + 3*der; // proportional control
    UL = UL + 1*Error;      // adjust right motor
    UR = PWMNOMINAL- 32*Error + 3*der; // proportional control
    if(UR < (PWMNOMINAL-SWING)) UR = PWMNOMINAL-SWING; // 3,000 to 7,000
    if(UR > (PWMNOMINAL+SWING)) UR = PWMNOMINAL+SWING;
    if(UL < (PWMNOMINAL-SWING)) UL = PWMNOMINAL-SWING; // 3,000 to 7,000
    if(UL > (PWMNOMINAL+SWING)) UL = PWMNOMINAL+SWING;

    //turns left if the center measurement and right measurement is small enough that we will hit the wall if we don't turn
    if((LeftDistance<250) && (CenterDistance <250)){
        UR = 0;
        UL = PWMNOMINAL;
    }

    Motor_Forward(UL,UR);


  }
}


void Pause(void){int i;
  while(Bump_Read()){ // wait for release
    Clock_Delay1ms(200); LaunchPad_Output(0); // off
    Clock_Delay1ms(200); LaunchPad_Output(1); // red
  }
  while(Bump_Read()==0){// wait for touch
    Clock_Delay1ms(100); LaunchPad_Output(0); // off
    Clock_Delay1ms(100); LaunchPad_Output(3); // red/green
  }
  while(Bump_Read()){ // wait for release
    Clock_Delay1ms(100); LaunchPad_Output(0); // off
    Clock_Delay1ms(100); LaunchPad_Output(4); // blue
  }
  for(i=1000;i>100;i=i-200){
    Clock_Delay1ms(i); LaunchPad_Output(0); // off
    Clock_Delay1ms(i); LaunchPad_Output(2); // green
  }
  // restart Jacki
  UR = UL = PWMNOMINAL;    // reset parameters
  Mode = 1;

}


/*
 * Values for below macros shall be modified per the access-point's (AP) properties
 * SimpleLink device will connect to following AP when the application is executed
 */

#define SSID_NAME       "ECE DESIGN LAB 2.4"       /* Access point name to connect to. */
#define SEC_TYPE        SL_SEC_TYPE_WPA_WPA2     /* Security type of the Access piont */
#define PASSKEY         "ecedesignlab12345"   /* Password in case of secure AP */
#define PASSKEY_LEN     pal_Strlen(PASSKEY)  /* Password length in case of secure AP */

/*
 * MQTT server and topic properties that shall be modified per application
 */
#define MQTT_BROKER_SERVER  "broker.hivemq.com"
#define SUBSCRIBE_TOPIC "EcE"
#define MAX_SPEED "ECE1188/Loki/Max_Speed"
#define NUM_CRASHES "ECE1188/Loki/Number_Crashes"
#define TRACK_TIME "ECE1188/Loki/Track_Time"
#define LEFT_DISTANCE "ECE1188/Loki/Dist_Left/Log"
#define CENTER_DISTANCE "ECE1188/Loki/Dist_Center/Log"
#define RIGHT_DISTANCE "ECE1188/Loki/Dist_Right/Log"

// MQTT message buffer size
#define BUFF_SIZE 32


#define APPLICATION_VERSION "1.0.0"

#define MCLK_FREQUENCY 48000000
#define PWM_PERIOD 255

#define SL_STOP_TIMEOUT        0xFF

#define SMALL_BUF           32
#define MAX_SEND_BUF_SIZE   512
#define MAX_SEND_RCV_SIZE   1024





uint16_t motorSpeed;


volatile uint16_t leftTach;
enum TachDirection leftDir;
volatile int32_t leftSteps;
volatile uint16_t rightTach;
enum TachDirection rightDir;
volatile int32_t rightSteps;
volatile uint32_t Average_RPM_L;
volatile uint32_t Average_RPM_R;

char leftRPMString[3];
char rightRPMString[3];
char leftDistance[3];
char rightDistance[3];
char centerDistance[3];



/* Application specific status/error codes */
typedef enum{
    DEVICE_NOT_IN_STATION_MODE = -0x7D0,        /* Choosing this number to avoid overlap with host-driver's error codes */
    HTTP_SEND_ERROR = DEVICE_NOT_IN_STATION_MODE - 1,
    HTTP_RECV_ERROR = HTTP_SEND_ERROR - 1,
    HTTP_INVALID_RESPONSE = HTTP_RECV_ERROR -1,
    STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;

#define min(X,Y) ((X) < (Y) ? (X) : (Y))


/*
 * GLOBAL VARIABLES -- Start
 */
/* Button debounce state variables */
volatile unsigned int S1buttonDebounce = 0;
volatile unsigned int S2buttonDebounce = 0;
volatile int publishID = 0;

unsigned char macAddressVal[SL_MAC_ADDR_LEN];
unsigned char macAddressLen = SL_MAC_ADDR_LEN;

char macStr[18];        // Formatted MAC Address String
char uniqueID[9];       // Unique ID generated from TLV RAND NUM and MAC Address

Network n;
Client hMQTTClient;     // MQTT Client

_u32  g_Status = 0;
struct{
    _u8 Recvbuff[MAX_SEND_RCV_SIZE];
    _u8 SendBuff[MAX_SEND_BUF_SIZE];

    _u8 HostName[SMALL_BUF];
    _u8 CityName[SMALL_BUF];

    _u32 DestinationIP;
    _i16 SockID;
}g_AppData;

/* Port mapper configuration register */
const uint8_t port_mapping[] =
{
    //Port P2:
    PM_TA0CCR1A, PM_TA0CCR2A, PM_TA0CCR3A, PM_NONE, PM_TA1CCR1A, PM_NONE, PM_NONE, PM_NONE
};

/* TimerA UpMode Configuration Parameter */
const Timer_A_UpModeConfig upConfig =
{
        TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_8,          // SMCLK/8 = 6MHz
        90000,                                  // 15ms debounce period
        TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ,    // Enable CCR0 interrupt
        TIMER_A_DO_CLEAR                        // Clear value
};

/*
 * GLOBAL VARIABLES -- End
 */


/*
 * STATIC FUNCTION DEFINITIONS -- Start
 */
static _i32 establishConnectionWithAP();
static _i32 configureSimpleLinkToDefaultState();
static _i32 initializeAppVariables();
static void displayBanner();
static void messageArrived(MessageData*);
static void generateUniqueID();


/*
 * STATIC FUNCTION DEFINITIONS -- End
 */


/*
 * ASYNCHRONOUS EVENT HANDLERS -- Start
 */

/*!
    \brief This function handles WLAN events

    \param[in]      pWlanEvent is the event passed to the handler

    \return         None

    \note

    \warning
*/



void utoa(unsigned int n, char returnVal[3])
{

   int numDigits = 0;
   int helper;
   int i;
   for(i = 2; i >= 0; i--){
       helper = n%10;
       returnVal[i] = helper + 48;
       n /= 10;
   }

}


void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent)
{
    if(pWlanEvent == NULL)
        CLI_Write(" [WLAN EVENT] NULL Pointer Error \n\r");

    switch(pWlanEvent->Event)
    {
        case SL_WLAN_CONNECT_EVENT:
        {
            SET_STATUS_BIT(g_Status, STATUS_BIT_CONNECTION);

            /*
             * Information about the connected AP (like name, MAC etc) will be
             * available in 'slWlanConnectAsyncResponse_t' - Applications
             * can use it if required
             *
             * slWlanConnectAsyncResponse_t *pEventData = NULL;
             * pEventData = &pWlanEvent->EventData.STAandP2PModeWlanConnected;
             *
             */
        }
        break;

        case SL_WLAN_DISCONNECT_EVENT:
        {
            slWlanConnectAsyncResponse_t*  pEventData = NULL;

            CLR_STATUS_BIT(g_Status, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_Status, STATUS_BIT_IP_ACQUIRED);

            pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

            /* If the user has initiated 'Disconnect' request, 'reason_code' is SL_USER_INITIATED_DISCONNECTION */
            if(SL_USER_INITIATED_DISCONNECTION == pEventData->reason_code)
            {
                CLI_Write(" Device disconnected from the AP on application's request \n\r");
            }
            else
            {
                CLI_Write(" Device disconnected from the AP on an ERROR..!! \n\r");
            }
        }
        break;

        default:
        {
            CLI_Write(" [WLAN EVENT] Unexpected event \n\r");
        }
        break;
    }
}

/*!
    \brief This function handles events for IP address acquisition via DHCP
           indication

    \param[in]      pNetAppEvent is the event passed to the handler

    \return         None

    \note

    \warning
*/
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
    if(pNetAppEvent == NULL)
        CLI_Write(" [NETAPP EVENT] NULL Pointer Error \n\r");

    switch(pNetAppEvent->Event)
    {
        case SL_NETAPP_IPV4_IPACQUIRED_EVENT:
        {
            SET_STATUS_BIT(g_Status, STATUS_BIT_IP_ACQUIRED);

            /*
             * Information about the connected AP's IP, gateway, DNS etc
             * will be available in 'SlIpV4AcquiredAsync_t' - Applications
             * can use it if required
             *
             * SlIpV4AcquiredAsync_t *pEventData = NULL;
             * pEventData = &pNetAppEvent->EventData.ipAcquiredV4;
             * <gateway_ip> = pEventData->gateway;
             *
             */
        }
        break;

        default:
        {
            CLI_Write(" [NETAPP EVENT] Unexpected event \n\r");
        }
        break;
    }
}

/*!
    \brief This function handles callback for the HTTP server events

    \param[in]      pHttpEvent - Contains the relevant event information
    \param[in]      pHttpResponse - Should be filled by the user with the
                    relevant response information

    \return         None

    \note

    \warning
*/
void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pHttpEvent,
                                  SlHttpServerResponse_t *pHttpResponse)
{
    /*
     * This application doesn't work with HTTP server - Hence these
     * events are not handled here
     */
    CLI_Write(" [HTTP EVENT] Unexpected event \n\r");
}

/*!
    \brief This function handles general error events indication

    \param[in]      pDevEvent is the event passed to the handler

    \return         None
*/
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
    /*
     * Most of the general errors are not FATAL are are to be handled
     * appropriately by the application
     */
    CLI_Write(" [GENERAL EVENT] \n\r");
}

/*!
    \brief This function handles socket events indication

    \param[in]      pSock is the event passed to the handler

    \return         None
*/
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
    if(pSock == NULL)
        CLI_Write(" [SOCK EVENT] NULL Pointer Error \n\r");

    switch( pSock->Event )
    {
        case SL_SOCKET_TX_FAILED_EVENT:
        {
            /*
            * TX Failed
            *
            * Information about the socket descriptor and status will be
            * available in 'SlSockEventData_t' - Applications can use it if
            * required
            *
            * SlSockEventData_t *pEventData = NULL;
            * pEventData = & pSock->EventData;
            */
            switch( pSock->EventData.status )
            {
                case SL_ECLOSE:
                    CLI_Write(" [SOCK EVENT] Close socket operation failed to transmit all queued packets\n\r");
                break;


                default:
                    CLI_Write(" [SOCK EVENT] Unexpected event \n\r");
                break;
            }
        }
        break;

        default:
            CLI_Write(" [SOCK EVENT] Unexpected event \n\r");
        break;
    }
}
/*
 * ASYNCHRONOUS EVENT HANDLERS -- End
 */


/*
 * Application's entry point
 */

void function(){

}


int16_t numCrashed = 0;
int16_t trackTime = 0;
int32_t maxSpeed = 0;
int32_t leftDistanceMeasurements[1000];
int32_t centerDistanceMeasurements[1000];
int32_t rightDistanceMeasurements[1000];
int32_t numMeasurements = 0;
int8_t timingVar = 0;
int8_t input = 0;
char stringHelper[3];
uint8_t takeValues = 0;
int on = 0;
int control = 0;

int print_To_IoT(/*int argc, char** argv*/)
{
    takeValues = 0;

    uint32_t channel = 1;
    //TimerA0_Init(&function, 5000);   // initialize timer A0 to period of 5000
    SysTick->LOAD = 0x00FFFFFF;           // maximum reload value
    SysTick->CTRL = 0x00000005;           // enable SysTick with no interrupts
    I2CB1_Init(30); // baud rate = 12MHz/30=400kHz
    //Init();
    //Clear();
    OPT3101_Init();
    OPT3101_Setup();
    OPT3101_CalibrateInternalCrosstalk();
    OPT3101_StartMeasurementChannel(channel);
    StartTime = SysTick->VAL;

    int i = 0;
    uint32_t distanceAverage = 0;


    _i32 retVal = -1;

    retVal = initializeAppVariables();
    ASSERT_ON_ERROR(retVal);


    /* Stop WDT and initialize the system-clock of the MCU */
    stopWDT();
    initClk();


    /* GPIO Setup for Pins 2.0-2.2 */
    //MAP_PMAP_configurePorts((const uint8_t *) port_mapping, PMAP_P2MAP, 1,
    //PMAP_DISABLE_RECONFIGURATION);

    //MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,
    //GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);



    /* Confinguring P1.1 & P1.4 as an input and enabling interrupts */
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN4);
    GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN4);
    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN4);
    GPIO_interruptEdgeSelect(GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN4, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN4);

    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);

    /* Configure TimerA0 for RGB LED*/
    /*TA0CCR0 = PWM_PERIOD;                   // PWM Period
    TA0CCTL1 = OUTMOD_7;                    // CCR1 reset/set
    TA0CCR1 = PWM_PERIOD * (0/255);                 // CCR1 PWM duty cycle
    TA0CCTL2 = OUTMOD_7;                    // CCR2 reset/set
    TA0CCR2 = PWM_PERIOD * (0/255);                 // CCR2 PWM duty cycle
    TA0CCTL3 = OUTMOD_7;                    // CCR3 reset/set
    TA0CCR3 = PWM_PERIOD * (0/255);                 // CCR3 PWM duty cycle
    TA0CTL = TASSEL__SMCLK | MC__UP | TACLR;  // SMCLK, up mode, clear TAR
    */
    /* Configuring TimerA1 for Up Mode */
    Timer_A_configureUpMode(TIMER_A1_BASE, &upConfig);

    Interrupt_enableInterrupt(INT_TA1_0);
    Interrupt_enableInterrupt(INT_PORT1);
    Interrupt_enableMaster();

    /* Configure command line interface */
    CLI_Configure();

    displayBanner();

    /*
     * Following function configures the device to default state by cleaning
     * the persistent settings stored in NVMEM (viz. connection profiles &
     * policies, power policy etc)
     *
     * Applications may choose to skip this step if the developer is sure
     * that the device is in its default state at start of application
     *
     * Note that all profiles and persistent settings that were done on the
     * device will be lost
     */
    retVal = configureSimpleLinkToDefaultState();
    if(retVal < 0)
    {
        if (DEVICE_NOT_IN_STATION_MODE == retVal)
            CLI_Write(" Failed to configure the device in its default state \n\r");

        LOOP_FOREVER();
    }

    CLI_Write(" Device is configured in default state \n\r");

    /*
     * Assumption is that the device is configured in station mode already
     * and it is in its default state
     */
    retVal = sl_Start(0, 0, 0);
    if ((retVal < 0) ||
        (ROLE_STA != retVal) )
    {
        CLI_Write(" Failed to start the device \n\r");
        LOOP_FOREVER();
    }

    CLI_Write(" Device started as STATION \n\r");

    /* Connecting to WLAN AP */
    retVal = establishConnectionWithAP();
    if(retVal < 0)
    {
        CLI_Write(" Failed to establish connection w/ an AP \n\r");
        LOOP_FOREVER();
    }

    CLI_Write(" Connection established w/ AP and IP is acquired \n\r");

    // Obtain MAC Address
    sl_NetCfgGet(SL_MAC_ADDRESS_GET,NULL,&macAddressLen,(unsigned char *)macAddressVal);

    // Print MAC Addres to be formatted string
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
            macAddressVal[0], macAddressVal[1], macAddressVal[2], macAddressVal[3], macAddressVal[4], macAddressVal[5]);

    // Generate 32bit unique ID from TLV Random Number and MAC Address
    generateUniqueID();

    int rc = 0;
    unsigned char buf[100];
    unsigned char readbuf[100];

    NewNetwork(&n);
    rc = ConnectNetwork(&n, MQTT_BROKER_SERVER, 1883);

    if (rc != 0) {
        CLI_Write(" Failed to connect to MQTT broker \n\r");
        LOOP_FOREVER();
    }
    CLI_Write(" Connected to MQTT broker \n\r");

    MQTTClient(&hMQTTClient, &n, 1000, buf, 100, readbuf, 100);
    MQTTPacket_connectData cdata = MQTTPacket_connectData_initializer;
    cdata.MQTTVersion = 3;
    cdata.clientID.cstring = uniqueID;
    rc = MQTTConnect(&hMQTTClient, &cdata);

    if (rc != 0) {
        CLI_Write(" Failed to start MQTT client \n\r");
        LOOP_FOREVER();
    }
    CLI_Write(" Started MQTT client successfully \n\r");

    rc = MQTTSubscribe(&hMQTTClient, SUBSCRIBE_TOPIC, QOS0, messageArrived);

    if (rc != 0) {
        CLI_Write(" Failed to subscribe to /msp/cc3100/demo topic \n\r");
        LOOP_FOREVER();
    }
    CLI_Write(" Subscribed to /msp/cc3100/demo topic \n\r");

    rc = MQTTSubscribe(&hMQTTClient, uniqueID, QOS0, messageArrived);

    if (rc != 0) {
        CLI_Write(" Failed to subscribe to uniqueID topic \n\r");
        LOOP_FOREVER();
    }
    CLI_Write(" Subscribed to uniqueID topic \n\r");

    //while(1){
        rc = MQTTYield(&hMQTTClient, 10);
       // if (rc != 0) {
         //   CLI_Write(" MQTT failed to yield \n\r");
           // LOOP_FOREVER();
        //}



        //int rc = 0;

        //print max speed
        utoa(maxSpeed, stringHelper);
        rc = 0;
        MQTTMessage msgMaxSpeed;
        msgMaxSpeed.dup = 0;
        msgMaxSpeed.id = 0;
        msgMaxSpeed.payload = stringHelper;
        msgMaxSpeed.payloadlen = 3;
        msgMaxSpeed.qos = QOS0;
        msgMaxSpeed.retained = 0;
        rc = MQTTPublish(&hMQTTClient, MAX_SPEED, &msgMaxSpeed);

        //print numCrashes
        utoa(numCrashed, stringHelper);
        rc = 0;
        MQTTMessage msgNumCrashes;
        msgNumCrashes.dup = 0;
        msgNumCrashes.id = 0;
        msgNumCrashes.payload = stringHelper;
        msgNumCrashes.payloadlen = 3;
        msgNumCrashes.qos = QOS0;
        msgNumCrashes.retained = 0;
        rc = MQTTPublish(&hMQTTClient, NUM_CRASHES, &msgNumCrashes);

        //print trackTime
        utoa(trackTime, stringHelper);
        rc = 0;
        MQTTMessage msgTrackTime;
        msgTrackTime.dup = 0;
        msgTrackTime.id = 0;
        msgTrackTime.payload = stringHelper;
        msgTrackTime.payloadlen = 3;
        msgTrackTime.qos = QOS0;
        msgTrackTime.retained = 0;
        rc = MQTTPublish(&hMQTTClient, TRACK_TIME, &msgTrackTime);

        //print all distances
        //int i;
        for(i = 0; i < numMeasurements; i++){

            //int rc = 0;
            //print left distance
            utoa(leftDistanceMeasurements[i], stringHelper);
            rc = 0;
            MQTTMessage msgLeftDistance;
            msgLeftDistance.dup = 0;
            msgLeftDistance.id = 0;
            msgLeftDistance.payload = stringHelper;
            msgLeftDistance.payloadlen = 3;
            msgLeftDistance.qos = QOS0;
            msgLeftDistance.retained = 0;
            rc = MQTTPublish(&hMQTTClient, LEFT_DISTANCE, &msgLeftDistance);

            //print center distance
            utoa(centerDistanceMeasurements[i], stringHelper);
            rc = 0;
            MQTTMessage msgCenterDistance;
            msgCenterDistance.dup = 0;
            msgCenterDistance.id = 0;
            msgCenterDistance.payload = stringHelper;
            msgCenterDistance.payloadlen = 3;
            msgCenterDistance.qos = QOS0;
            msgCenterDistance.retained = 0;
            rc = MQTTPublish(&hMQTTClient, CENTER_DISTANCE, &msgCenterDistance);


            //print right distance
            utoa(rightDistanceMeasurements[i], stringHelper);
            rc = 0;
            MQTTMessage msgRightDistance;
            msgRightDistance.dup = 0;
            msgRightDistance.id = 0;
            msgRightDistance.payload = stringHelper;
            msgRightDistance.payloadlen = 3;
            msgRightDistance.qos = QOS0;
            msgRightDistance.retained = 0;
            rc = MQTTPublish(&hMQTTClient, RIGHT_DISTANCE, &msgRightDistance);

            Delay(50);

        }


        CLI_Write(" Published unique ID successfully \n\r");

        //break;


    //}
}

uint16_t timingIntHardware = 0;

void updateIoTValues(){

    /*char command[10];
    UART0_InString(command, 9);

    if (command[0] == 'S'){
        on = 0;
    }
    if(command[0] == 'F'){
        on = 1;
    }*/
    if(takeValues == 1){
        if (timingIntHardware == 163){
        leftDistanceMeasurements[numMeasurements] = LeftDistance;
                centerDistanceMeasurements[numMeasurements] = CenterDistance;
                rightDistanceMeasurements[numMeasurements] = RightDistance;

                if(numMeasurements == 999){
                    numMeasurements = 0;
                }
                else{
                    numMeasurements++;
                }
         trackTime++;
         timingIntHardware = 0;
        }
        else{
            timingIntHardware++;
        }
    }
}

uint8_t previousDetect = 1;

void main(void){ // wallFollow wall following implementation
  previousDetect = 1;
  takeValues = 0;
  int i = 0;
  uint32_t channel = 1;
  DisableInterrupts();
  Clock_Init48MHz();
  TimerA2_Init(&updateIoTValues, 3000);   // initialize timer A0 to period of 15000
  Bump_Init();
  //SysTick_Init(48000, 2);          // set up SysTick for 1000 Hz interrupts
  LaunchPad_Init(); // built-in switches and LEDs
  Reflectance_Init();
  Motor_Init();
  Motor_Stop(); // initialize and stop
  Tachometer_Init();
  Mode = 0;
  I2CB1_Init(30); // baud rate = 12MHz/30=400kHz
  Init();
  Clear();
  OutString("OPT3101");
  SetCursor(0, 1);
  OutString("L=");
  SetCursor(0, 2);
  OutString("C=");
  SetCursor(0, 3);
  OutString("R=");
  SetCursor(0, 4);
  OutString("Wall follow");
  SetCursor(0, 5);
  OutString("SP=");
  SetCursor(0, 6);
  OutString("Er=");
  SetCursor(0, 7);
  OutString("U =");
  OPT3101_Init();
  OPT3101_Setup();
  OPT3101_CalibrateInternalCrosstalk();
  OPT3101_ArmInterrupts(&TxChannel, Distances, Amplitudes);
  TxChannel = 3;
  OPT3101_StartMeasurementChannel(channel);
  LPF_Init(100,8);
  LPF_Init2(100,8);
  LPF_Init3(100,8);
  UR = UL = PWMNOMINAL; //initial power
  Pause();
  EnableInterrupts();
  takeValues = 1;
  while(1){
      while((EUSCI_A0->IFG&0x01) == 0){
          if(on == 0)
          {
              char cemt;
              cemt = UART0_InChar();
              break;
          }
    takeValues = 1;
    //if((EUSCI_A0->IFG&0x01) == 0){break;}
    if(Bump_Read()){ // collision
      numCrashed++;
      Motor_Forward(0,0);
      Clock_Delay1ms(500);
    }
    //if((EUSCI_A0->IFG&0x01) == 0){break;}
    if(on == 0)
    {
        Motor_Forward(0,0);
    }
    //if((EUSCI_A0->IFG&0x01) == 0){break;}
    if(Reflectance_Read(750) == 0){
        previousDetect = 0;
    }
    //if((EUSCI_A0->IFG&0x01) == 0){break;}
    else{
        if(previousDetect == 0){
            Motor_Stop();
            print_To_IoT();
            Pause();
        }
        previousDetect = 1;
    }
    //if((EUSCI_A0->IFG&0x01) == 0){break;}
    if(TxChannel <= 2){ // 0,1,2 means new data
      if(TxChannel==0){
        if(Amplitudes[0] > 1000){
          LeftDistance = FilteredDistances[0] = Left(LPF_Calc(Distances[0]));
        }else{
          LeftDistance = FilteredDistances[0] = 500;
        }
      }else if(TxChannel==1){
        if(Amplitudes[1] > 1000){
          CenterDistance = FilteredDistances[1] = LPF_Calc2(Distances[1]);
        }else{
          CenterDistance = FilteredDistances[1] = 500;
        }
      }else {
        if(Amplitudes[2] > 1000){
          RightDistance = FilteredDistances[2] = Right(LPF_Calc3(Distances[2]));
        }else{
          RightDistance = FilteredDistances[2] = 500;
        }
      }
      SetCursor(2, TxChannel+1);
      OutUDec(FilteredDistances[TxChannel]); OutChar(','); OutUDec(Amplitudes[TxChannel]);
      TxChannel = 3; // 3 means no data
      channel = (channel+1)%3;
      OPT3101_StartMeasurementChannel(channel);
      i = i + 1;
    }
    //if((EUSCI_A0->IFG&0x01) == 0){break;}
    if (on == 1)
    {
        if (control == 0){
            Controller();
        }
        else if (control == 1){
            Controller_Left();
        }
        else if (control == 2){
            Controller_Right();
        }
    }
    //if((EUSCI_A0->IFG&0x01) == 0){break;}
    if(i >= 100){
      i = 0;
      SetCursor(3, 5);
      OutUDec(SetPoint);
      SetCursor(3, 6);
      OutSDec(Error);
      SetCursor(3, 7);
      OutUDec(UL); OutChar(','); OutUDec(UR);
    }
    //if((EUSCI_A0->IFG&0x01) == 0){break;}

    Tachometer_Get(&leftTach, &leftDir, &leftSteps, &rightTach, &rightDir, &rightSteps);
    //if((EUSCI_A0->IFG&0x01) == 0){break;}
    Average_RPM_L = leftTach/800;
    //if((EUSCI_A0->IFG&0x01) == 0){break;}
    Average_RPM_R = rightTach/800;
    //if((EUSCI_A0->IFG&0x01) == 0){break;}
    if(Average_RPM_L > maxSpeed){
        maxSpeed = Average_RPM_L;
    }
    //if((EUSCI_A0->IFG&0x01) == 0){break;}
    if(Average_RPM_R > maxSpeed){
        maxSpeed = Average_RPM_R;
    }
    //if((EUSCI_A0->IFG&0x01) == 0){break;}


    WaitForInterrupt();
  }
  char character = (char)(EUSCI_A0->RXBUF);
  if(character == 'F') // 0x460A0D
  {
      on = 1;
  }
  else if(character == 'S'){ // 0x530A0D
      on = 0;
  }
  else if(character == 'C'){ // 0x430A0D
      control = 0;
      LaunchPad_Output(0);
      LaunchPad_Output(1); // Red
  }
  else if(character == 'L'){ // 0x4C0A0D
      control = 1;
      LaunchPad_Output(0);
      LaunchPad_Output(2); // Green
  }
  else if(character == 'R'){ // 0x520A0C
      control = 2;
      LaunchPad_Output(0);
      LaunchPad_Output(3); // Blue
  }
  }
}



static void generateUniqueID() {
    CRC32_setSeed(TLV->RANDOM_NUM_1, CRC32_MODE);
    CRC32_set32BitData(TLV->RANDOM_NUM_2);
    CRC32_set32BitData(TLV->RANDOM_NUM_3);
    CRC32_set32BitData(TLV->RANDOM_NUM_4);
    int i;
    for (i = 0; i < 6; i++)
    CRC32_set8BitData(macAddressVal[i], CRC32_MODE);

    uint32_t crcResult = CRC32_getResult(CRC32_MODE);
    sprintf(uniqueID, "%06X", crcResult);
}

//****************************************************************************
//
//!    \brief MQTT message received callback - Called when a subscribed topic
//!                                            receives a message.
//! \param[in]                  data is the data passed to the callback
//!
//! \return                        None
//
//****************************************************************************
static void messageArrived(MessageData* data) {
    char buf[BUFF_SIZE];

    char *tok;
    long color;

    // Check for buffer overflow
    if (data->topicName->lenstring.len >= BUFF_SIZE) {
//      UART_PRINT("Topic name too long!\n\r");
        return;
    }
    if (data->message->payloadlen >= BUFF_SIZE) {
//      UART_PRINT("Payload too long!\n\r");
        return;
    }

    strncpy(buf, data->topicName->lenstring.data,
        min(BUFF_SIZE, data->topicName->lenstring.len));
    buf[data->topicName->lenstring.len] = 0;



    strncpy(buf, data->message->payload,
        min(BUFF_SIZE, data->message->payloadlen));
    buf[data->message->payloadlen] = 0;

    //CLI_Write(buf);

    if(buf[0] == 'g'){
        PWM_Duty1(1000);
        PWM_Duty2(1000);
        motorSpeed = 500;
    }
    else if(buf[0] == 's'){
        PWM_Duty1(0);
        PWM_Duty2(0);
        motorSpeed = 0;
    }


    return;
}

/*
 * Port 1 interrupt handler. This handler is called whenever the switch attached
 * to P1.1 is pressed.
 */
void PORT1_IRQHandler(void)
{
    uint32_t status = GPIO_getEnabledInterruptStatus(GPIO_PORT_P1);
    GPIO_clearInterruptFlag(GPIO_PORT_P1, status);

    if (status & GPIO_PIN1)
    {
        if (S1buttonDebounce == 0)
        {
            S1buttonDebounce = 1;

            GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);

            // Publish the unique ID
            publishID = 1;

            MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
        }
    }
    if (status & GPIO_PIN4)
    {
        if (S2buttonDebounce == 0)
        {
            S2buttonDebounce = 1;

            CLI_Write(" MAC Address: \n\r ");
            CLI_Write(macStr);
            CLI_Write("\n\r");

            MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
        }
    }
}

/*
void TA1_0_IRQHandler(void)
{
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
    if (P1IN & GPIO_PIN1)
    {
        S1buttonDebounce = 0;
    }
    if (P1IN & GPIO_PIN4)
    {
        S2buttonDebounce = 0;
    }

    if ((P1IN & GPIO_PIN1) && (P1IN & GPIO_PIN4))
    {
        Timer_A_stopTimer(TIMER_A1_BASE);
    }
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE,
                TIMER_A_CAPTURECOMPARE_REGISTER_0);
}*/


/*!
    \brief This function configure the SimpleLink device in its default state. It:
           - Sets the mode to STATION
           - Configures connection policy to Auto and AutoSmartConfig
           - Deletes all the stored profiles
           - Enables DHCP
           - Disables Scan policy
           - Sets Tx power to maximum
           - Sets power policy to normal
           - Unregisters mDNS services
           - Remove all filters

    \param[in]      none

    \return         On success, zero is returned. On error, negative is returned
*/
static _i32 configureSimpleLinkToDefaultState()
{
    SlVersionFull   ver = {0};
    _WlanRxFilterOperationCommandBuff_t  RxFilterIdMask = {0};

    _u8           val = 1;
    _u8           configOpt = 0;
    _u8           configLen = 0;
    _u8           power = 0;

    _i32          retVal = -1;
    _i32          mode = -1;

    mode = sl_Start(0, 0, 0);
    ASSERT_ON_ERROR(mode);

    /* If the device is not in station-mode, try configuring it in station-mode */
    if (ROLE_STA != mode)
    {
        if (ROLE_AP == mode)
        {
            /* If the device is in AP mode, we need to wait for this event before doing anything */
            while(!IS_IP_ACQUIRED(g_Status)) { _SlNonOsMainLoopTask(); }
        }

        /* Switch to STA role and restart */
        retVal = sl_WlanSetMode(ROLE_STA);
        ASSERT_ON_ERROR(retVal);

        retVal = sl_Stop(SL_STOP_TIMEOUT);
        ASSERT_ON_ERROR(retVal);

        retVal = sl_Start(0, 0, 0);
        ASSERT_ON_ERROR(retVal);

        /* Check if the device is in station again */
        if (ROLE_STA != retVal)
        {
            /* We don't want to proceed if the device is not coming up in station-mode */
            ASSERT_ON_ERROR(DEVICE_NOT_IN_STATION_MODE);
        }
    }

    /* Get the device's version-information */
    configOpt = SL_DEVICE_GENERAL_VERSION;
    configLen = sizeof(ver);
    retVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &configOpt, &configLen, (_u8 *)(&ver));
    ASSERT_ON_ERROR(retVal);

    /* Set connection policy to Auto + SmartConfig (Device's default connection policy) */
    retVal = sl_WlanPolicySet(SL_POLICY_CONNECTION, SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
    ASSERT_ON_ERROR(retVal);

    /* Remove all profiles */
    retVal = sl_WlanProfileDel(0xFF);
    ASSERT_ON_ERROR(retVal);

    /*
     * Device in station-mode. Disconnect previous connection if any
     * The function returns 0 if 'Disconnected done', negative number if already disconnected
     * Wait for 'disconnection' event if 0 is returned, Ignore other return-codes
     */
    retVal = sl_WlanDisconnect();
    if(0 == retVal)
    {
        /* Wait */
        while(IS_CONNECTED(g_Status)) { _SlNonOsMainLoopTask(); }
    }

    /* Enable DHCP client*/
    retVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,&val);
    ASSERT_ON_ERROR(retVal);

    /* Disable scan */
    configOpt = SL_SCAN_POLICY(0);
    retVal = sl_WlanPolicySet(SL_POLICY_SCAN , configOpt, NULL, 0);
    ASSERT_ON_ERROR(retVal);

    /* Set Tx power level for station mode
       Number between 0-15, as dB offset from max power - 0 will set maximum power */
    power = 0;
    retVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID, WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (_u8 *)&power);
    ASSERT_ON_ERROR(retVal);

    /* Set PM policy to normal */
    retVal = sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY, NULL, 0);
    ASSERT_ON_ERROR(retVal);

    /* Unregister mDNS services */
    retVal = sl_NetAppMDNSUnRegisterService(0, 0);
    ASSERT_ON_ERROR(retVal);

    /* Remove  all 64 filters (8*8) */
    pal_Memset(RxFilterIdMask.FilterIdMask, 0xFF, 8);
    retVal = sl_WlanRxFilterSet(SL_REMOVE_RX_FILTER, (_u8 *)&RxFilterIdMask,
                       sizeof(_WlanRxFilterOperationCommandBuff_t));
    ASSERT_ON_ERROR(retVal);

    retVal = sl_Stop(SL_STOP_TIMEOUT);
    ASSERT_ON_ERROR(retVal);

    retVal = initializeAppVariables();
    ASSERT_ON_ERROR(retVal);

    return retVal; /* Success */
}

/*!
    \brief Connecting to a WLAN Access point

    This function connects to the required AP (SSID_NAME).
    The function will return once we are connected and have acquired IP address

    \param[in]  None

    \return     0 on success, negative error-code on error

    \note

    \warning    If the WLAN connection fails or we don't acquire an IP address,
                We will be stuck in this function forever.
*/
static _i32 establishConnectionWithAP()
{
    SlSecParams_t secParams = {0};
    _i32 retVal = 0;

    secParams.Key = PASSKEY;
    secParams.KeyLen = PASSKEY_LEN;
    secParams.Type = SEC_TYPE;

    retVal = sl_WlanConnect(SSID_NAME, pal_Strlen(SSID_NAME), 0, &secParams, 0);
    ASSERT_ON_ERROR(retVal);

    /* Wait */
    while((!IS_CONNECTED(g_Status)) || (!IS_IP_ACQUIRED(g_Status))) { _SlNonOsMainLoopTask(); }

    return SUCCESS;
}

/*!
    \brief This function initializes the application variables

    \param[in]  None

    \return     0 on success, negative error-code on error
*/
static _i32 initializeAppVariables()
{
    g_Status = 0;
    pal_Memset(&g_AppData, 0, sizeof(g_AppData));

    return SUCCESS;
}

/*!
    \brief This function displays the application's banner

    \param      None

    \return     None
*/
static void displayBanner()
{
    CLI_Write("\n\r\n\r");
    CLI_Write(" MQTT Twitter Controlled RGB LED - Version ");
    CLI_Write(APPLICATION_VERSION);
    CLI_Write("\n\r*******************************************************************************\n\r");
}



