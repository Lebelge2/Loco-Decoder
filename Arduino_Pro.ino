// Program sources:
// Geoff Bunza  https://forum.mrhmag.com/post/sma20-low-cost-17-channel-dcc-decoders-ver-6-01-with-soundtriggered-soundstepperdual-motorled-and-12201920
// Alex Shepherd https://github.com/mrrwa/NmraDcc/blob/master/examples/NmraDccMultiFunctionMotorDecoder/NmraDccMultiFunctionMotorDecoder.ino
//
// Contact: lebelge2@yahoo.fr
//
//---------------------- MOBILE LOCOMOTIVE DECODER FOR ARDUINO PRO MINI AND PRO MICRO(5V 16MHz) ------ -------------------------
//   V.1.  Date 02-08-23
//   Necessary libraries are here: https://d28lcup14p4e72.cloudfront.net/259338/7563408/new-multifunction-decoderv6_1%20MRH_V6_01_SMA.zip
//   Copy the libraries: DFPlayer_Mini_Mp3, SoftwareServo.
//   Copy the last NmraDcc Library V 2.0.0.  from: http://mrrwa.org/download/
//   Important: La librairie NmraDcc V 2.0.0. ne positionne pas le bit RailCom dans le CV 29, voir la modif. à faire dans le NmraDpp.Cpp
//   à la fin de ce code.

// - Short and long addresses
// - 14, 28 or 128 speed steps
// - 28 Functions
// - Extern MP3 sound module
// - Servos (max 5)
// - 2 Inputs Trigger
// - Shunting mode ( Vit/2 and FL-RL ON)
// - Acceleration/Deceleration
// - RailCom
// - ABC braking (left or right rail or both)
// - Speed table
// - CV: 1, 2, 3, 4, 5, 7, 8, 17, 18, 26, 27, 28, 29, 33 to 41, 67 to 94.
// - Modification of CVs with control unit or serial monitor.
//
//------- Pin Arduino ------------------------------------
// Pin Trigger    0 Input Trigger 1
//                8 Input Trigger 2
// Pin RailCom    1 Output serie datas RailCom 250kb (TX)
//                3 Input CutOut
//                4 Output RailCom Enable
// Pin DCC        2 Input Intx DCC
// Pin ABC        6 Input Intx ABC (A5 for Mini)
// Pin Player     5 Input Busy module MP3
//                7 TX Module MP3
// Pin Moteur     9 Output motor
//               10 Output motor
// Pin Functions Pro Micro      19 20 21 18 15 14 16      (See cv33 to 39)
// Pin Functions Pro Mini       15 16 17 14 13 12 11      (See cv33 to 39)

#include <NmraDcc.h>
#include <SoftwareServo.h>
#include <SoftwareSerial.h>
#include <DFPlayer_Mini_Mp3.h>
#include <avr/wdt.h>
//****************************************** DEFINES **********************************************************
#define Monitor          // If use serial monitor    Si moniteur série utilisé
#define Francais         // Messages du moniteur en français
//#define English         // Monitor messages in English
#define BrakeABC         // If use Brake ABC         Si freinage ABC utilisé
#define RailCom          // If use RailCom           Si RailCom utilisé
//#define F13_F28          // Add functions 13 to 28.  Ajout des fonctions 13 à 28

// !!!!!!!!!!!  When updating the program, the modified CVs will be overwritten by the "factory" CVs.  !!!!!!!!!!!!
#define DECODER_LOADED   // Remove the "//" to keep modified CVs.
//**************************************************************************************************************
#if defined (__AVR_ATmega32U4__)          // *** Arduino Pro Micro ***
#elif defined (__AVR_ATmega328P__)        // *** Arduino Pro Mini ***
#else
#error "Unsupported CPU, you need to add another configuration section for your CPU"
#endif

#define DCC_PIN   2                             // INTx 
#define CutOut_Pin 3                            // INTx
#define RcEnable  4                             // __--__
#define MOTOR_PWM_FWD_PIN 9                     // PWM 
#define MOTOR_PWM_REV_PIN 10                    //  ""
#define servo_start_delay 50
#define servo_init_delay  7
#define servo_slowdown    4                     //servo loop counter limit
SoftwareServo servo[5];
int servo_slow_counter =  0;                    //servo loop counter to slowdown servo transit
uint8_t fpins[5];
uint8_t busy_pin = 5;                           // DFPlayer Busy status pin
uint8_t ledState;
uint8_t NewLedStateF0;
uint8_t lastLedStateF0;
uint8_t Direction = 0;
uint8_t newDirection = 0;
uint8_t lastDirection = 0;
uint8_t newSpeed = 0;
uint8_t lastSpeed = 0;
uint8_t newPwm = 0;
uint8_t newPwm1;
uint8_t newPwm2;
uint8_t lastPwm;
float   ad_Pwm = 0;
uint8_t ad_Pwm2;
uint8_t numSpeedSteps;
uint8_t LED_PIN_FL;                   // FL   Light
uint8_t LED_PIN_RL;                   // RL   Light
uint8_t Function1;                    // F1   Light, servo, sound
uint8_t Function2;                    // F2   Light, servo, sound
uint8_t Function3;                    // F3   Light, servo, sound
uint8_t Function4;                    // F4   Light, servo, sound
uint8_t Function5;                    // F5   Light, servo, sound
uint8_t Function6;                    // F6 to F28, only sounds
uint8_t Function7, Function8, Function9, Function10, Function11, Function12;
uint8_t num_active_fpins = 16;        // Number max Functions
#if defined F13_F28
uint8_t Function13, Function14, Function15, Function16, Function17, Function18, Function19, Function20;
uint8_t Function21, Function22, Function23, Function24, Function25, Function26, Function27, Function28;
#endif
uint8_t RcData[10];                  // Data RailCom Ch1 et Ch2
uint8_t Trame;                       // Even/odd, RailCom frame
uint8_t cv1, cv17, cv18, cv26, cv27, cv28, cv29;
uint8_t vStart;                      // cv2
uint8_t acc_Rate;                    // cv3
uint8_t dec_Rate;                    // cv4
float Step_acc, Step_dec;
uint8_t vHigh;                       // cv5
uint8_t ABC;                         // cv27
uint8_t RailComOn;                   // cv29
uint8_t InvSens;                     // cv29
uint8_t TabVit;                      // cv29
uint8_t AdrEtend;                    // cv29
uint8_t cv_value;
uint8_t Arret;                       // ABC
uint8_t TableVitesse[28];
uint8_t Step;                        // ABC
NmraDcc  Dcc ;
DCC_MSG  Packet ;
SoftwareSerial mySerial(8, 7);       // RX, TX serial to DFPlayer
uint8_t i, ii, t;
uint8_t vScaleFactor;
uint8_t modSpeed;
uint8_t modSteps;
uint8_t TabValBit[20];              // ABC
uint8_t ValABCTrack1;               // ABC
uint8_t ValABCTrack2;               // ABC
uint8_t MoyenneTrack;               // ABC
uint8_t Commande;                   // Serial monitor
uint8_t Marqueur;                   // Logic analizer
unsigned long currentTime = 0;      // Function millis
unsigned long previousTime = 0;     // Function millis
uint8_t Shunting;                   // Manoeuvre
uint8_t StateShunting;
uint8_t StateTimer;

struct QUEUE
{
  int inuse;
  int current_position;
  int increment;
  int stop_value;
  int start_value;
};
QUEUE *ftn_queue = new QUEUE[17];

struct CVPair                        // Structure for CV Values Table
{
  uint16_t  CV;
  uint8_t   Value;
};
CVPair FactoryDefaultCVs [] =        // Default CV Values Table
{
  {1, 3},       // CV1 PRIMARY Addr
  {2, 120},     // CV2 VSTART
  {3, 4},       // CV3 Acc_Rate
  {4, 4},       // CV4 Dec_Rate
  {5, 255},     // CV5 VHIGH
  {7, 3},       // CV7 Version
  {8, 13},      // CV8 Constructeur
  {9, 0},
  {10, 0},
  {11, 0},      // CV11 Frame refresh time
  {17, 0},      // CV17 ExtAddr1
  {18, 0},      // CV18 ExtAddr2
  {26, 10},     // CV26 ABC Level
  {27, 3},      // CV27 ABC braking (Bit 0 to 1 = right rail, Bit 1 to 1 = left rail, both at 1 = two directions)
  {28, 1},      // CV28 RailCom config   Bit 0 = Ch1 ON, Bit 1 = Ch2 ON
  {29, 8},      // CV29 Config data      RailCom ON = 8   0b 0000 1000
  {30, 0},      // CV30 Error code
  {31, 0},      // CV31 RailCom
  {32, 255},    // CV32 RailCom
  //--------------------------------- Assignment of outputs FL, RL et F1 à F8 ------------------------------------------
#if defined (__AVR_ATmega32U4__)
  {33, 21},      // CV33  FL          Pin number Arduino
  {34, 20},      // CV34  RL          Pin number Arduino
  {35, 19},      // CV35  F1          Pin number Arduino
  {36, 18},      // CV36  F2          Pin number Arduino
  {37, 15},      // CV37  F3          Pin number Arduino
  {38, 14},      // CV38  F4          Pin number Arduino
  {39, 16},      // CV39  F5          Pin number Arduino
#endif
#if defined (__AVR_ATmega328P__)
  {33, 15},      // CV33  FL
  {34, 16},      // CV34  RL
  {35, 17},      // CV35  F1
  {36, 14},      // CV36  F2
  {37, 13},      // CV37  F3
  {38, 12},      // CV38  F4
  {39, 11},      // CV39  F5
#endif
  //---------------------------------------------- Speed table ----------------------------------------------
  {67, 1},
  {68, 2},
  {69, 3},
  {70, 5},
  {71, 8},
  {72, 12},
  {73, 16},
  {74, 21},
  {75, 26},
  {76, 33},
  {77, 39},
  {78, 47},
  {79, 55},
  {80, 64},
  {81, 73},
  {82, 83},
  {83, 94},
  {84, 105},
  {85, 117},
  {86, 129},
  {87, 143},
  {88, 156},
  {89, 171},
  {90, 186},
  {91, 202},
  {92, 218},
  {93, 235},
  {94, 255},
  //----------------------------------------------- Function F1 ----------------------------------------------
  {120, 0},     // 0 = LED On/Off, 1 = LED Blink, 2 = Servo, 3 = Audio MP3 (Long,loop), 4 = Audio MP3 (Short), 5 = Shunting/Manoeuvre
  {121, 2},     // Blink = Rate, Servo = Rate, Audio = Volume(0-30)              Servo Rate  <=  5
  {122, 0},     // Servo = Start Position, Audio = Audio Track                   Servo Start = 0  End = 180   For rot. 180°
  {123, 180},     // Servo = End Position                                        Blink Freq. = This val line  / Blink Rate
  //----------------------------------------------- Function F2 ----------------------------------------------
  {124, 1},     //  0 = LED On/Off, 1 = LED Blink, 2 = Servo, 3 = Audio MP3 (Long,loop), 4 = Audio MP3 (Short), 5 = Shunting/Manoeuvre
  {125, 4},    //  Blink = Rate, Servo = Rate, Audio = Volume(0-30)
  {126, 1},     //  Servo = Start Position, Audio = Audio Track
  {127, 100},    //  Servo = End Position
  //----------------------------------------------- Function F3 ----------------------------------------------
  {128, 5},     //  0 = LED On/Off, 1 = LED Blink, 2 = Servo, 3 = Audio MP3 (Long,loop), 4 = Audio MP3 (Short), 5 = Shunting/Manoeuvre
  {129, 2},    //  Blink = Rate, Servo = Rate, Audio = Volume(0-30)
  {130, 2},     //  Servo = Start Position, Audio = Audio Track
  {131, 200},   //  Servo = End Position
  //----------------------------------------------- Function F4 ----------------------------------------------
  {132, 2},     //  0 = LED On/Off, 1 = LED Blink, 2 = Servo, 3 = Audio MP3 (Long,loop), 4 = Audio MP3 (Short), 5 = Shunting/Manoeuvre
  {133, 3},    //  Blink = Rate, Servo = Rate, Audio = Volume(0-30)
  {134, 3},     //  Servo = Start Position, Audio = Audio Track
  {135, 100},   //  Servo = End Position
  //----------------------------------------------- Function F5 ----------------------------------------------
  {136, 1},     //  0 = LED On/Off, 1 = LED Blink, 2 = Servo, 3 = Audio MP3 (Long,loop), 4 = Audio MP3 (Short), 5 = Shunting/Manoeuvre
  {137, 4},    //  Blink = Rate, Servo = Rate, Audio = Volume(0-30)
  {138, 4},     //  Servo = Start Position, Audio = Audio Track
  {139, 200},    //  Servo = End Position
  //----------------------------------------------- Function F6 ----------------------------------------------
  {140, 4},     //  3 = Audio MP3 (Long,loop), 4 = Audio MP3 (Short)
  {141, 25},    //  Volume(0-30)
  {142, 2},     //  Track
  {143, 140},   //
  //----------------------------------------------- Function F7 ----------------------------------------------
  {144, 4},     //  3 = Audio MP3 (Long,loop), 4 = Audio MP3 (Short)
  {145, 25},    //  Volume(0-30)
  {146, 8},     //  Track
  {147, 140},   //
  //----------------------------------------------- Function F8 ----------------------------------------------
  {148, 3},     //  3 = Audio MP3 (Long,loop), 4 = Audio MP3 (Short)
  {149, 25},    //  Volume(0-30)
  {150, 4},     //  Track
  {151, 140},   //
  //----------------------------------------------- Function F9 ----------------------------------------------
  {152, 3},     //  3 = Audio MP3 (Long,loop), 4 = Audio MP3 (Short)
  {153, 25},    //  Volume(0-30)
  {154, 4},     //  Track
  {155, 0},     //
  //----------------------------------------------- Function F10 ----------------------------------------------
  {156, 3},     //  3 = Audio MP3 (Long,loop), 4 = Audio MP3 (Short)
  {157, 25},    //  Volume(0-30)
  {158, 10},     //  Track
  {159, 0},     //
  //----------------------------------------------- Function F11 ----------------------------------------------
  {160, 3},     //  3 = Audio MP3 (Long,loop), 4 = Audio MP3 (Short)
  {161, 25},    //  Volume(0-30)
  {162, 11},     //  Track
  {163, 0},     //
  //----------------------------------------------- Function F12 ----------------------------------------------
  {164, 3},     //  3 = Audio MP3 (Long,loop), 4 = Audio MP3 (Short)
  {165, 25},    //  Volume(0-30)
  {166, 12},     //  Track
  {167, 0},     //
#if defined F13_F28
  //----------------------------------------------- Function F13 ----------------------------------------------
  {168, 3},     //  3 = Audio MP3 (Long,loop), 4 = Audio MP3 (Short)
  {169, 25},    //  Volume(0-30)
  {170, 13},     //  Track
  {171, 0},     //
  //----------------------------------------------- Function F14 ----------------------------------------------
  {172, 3},     //  3 = Audio MP3 (Long,loop), 4 = Audio MP3 (Short)
  {173, 25},    //  Volume(0-30)
  {174, 14},     //  Track
  {175, 0},     //
  //----------------------------------------------- Function F15 ----------------------------------------------
  {176, 3},     //  3 = Audio MP3 (Long,loop), 4 = Audio MP3 (Short)
  {177, 25},    //  Volume(0-30)
  {178, 15},     //  Track
  {179, 0},     //
  //----------------------------------------------- Function F16 ----------------------------------------------
  {180, 3},     //  3 = Audio MP3 (Long,loop), 4 = Audio MP3 (Short)
  {181, 25},    //  Volume(0-30)
  {182, 16},     //  Track
  {183, 0},     //
  //----------------------------------------------- Function F17 ----------------------------------------------
  {184, 0},     //  3 = Audio MP3 (Long,loop), 4 = Audio MP3 (Short)
  {185, 25},    //  Volume(0-30)
  {186, 17},     //  Track
  {187, 0},     //
  //----------------------------------------------- Function F18 ----------------------------------------------
  {188, 3},     //  3 = Audio MP3 (Long,loop), 4 = Audio MP3 (Short)
  {189, 25},    //  Volume(0-30)
  {190, 18},     //  Track
  {191, 0},     //
  //----------------------------------------------- Function F19 ----------------------------------------------
  {192, 3},     //  3 = Audio MP3 (Long,loop), 4 = Audio MP3 (Short)
  {193, 25},    //  Volume(0-30)
  {194, 19},     //  Track
  {195, 0},     //
  //----------------------------------------------- Function F20 ----------------------------------------------
  {196, 3},     //  3 = Audio MP3 (Long,loop), 4 = Audio MP3 (Short)
  {197, 25},    //  Volume(0-30)
  {198, 20},     //  Track
  {199, 0},     //
  //----------------------------------------------- Function F21 ----------------------------------------------
  {200, 3},     //  3 = Audio MP3 (Long,loop), 4 = Audio MP3 (Short)
  {201, 25},    //  Volume(0-30)
  {202, 21},     //  Track
  {203, 0},     //
  //----------------------------------------------- Function F22 ----------------------------------------------
  {204, 0},     //  3 = Audio MP3 (Long,loop), 4 = Audio MP3 (Short)
  {205, 25},    //  Volume(0-30)
  {206, 22},     //  Track
  {207, 0},     //
  //----------------------------------------------- Function F23 ----------------------------------------------
  {208, 3},     //  3 = Audio MP3 (Long,loop), 4 = Audio MP3 (Short)
  {209, 25},    //  Volume(0-30)
  {210, 23},     //  Track
  {211, 0},     //
  //----------------------------------------------- Function F24 ----------------------------------------------
  {212, 3},     //  3 = Audio MP3 (Long,loop), 4 = Audio MP3 (Short)
  {213, 25},    //  Volume(0-30)
  {214, 24},     //  Track
  {215, 0},     //
  //----------------------------------------------- Function F25 ----------------------------------------------
  {216, 3},     //  3 = Audio MP3 (Long,loop), 4 = Audio MP3 (Short)
  {217, 25},    //  Volume(0-30)
  {218, 25},     //  Track
  {219, 0},     //
  //----------------------------------------------- Function F26 ----------------------------------------------
  {300, 3},     //  3 = Audio MP3 (Long,loop), 4 = Audio MP3 (Short)
  {301, 25},    //  Volume(0-30)
  {302, 26},     //  Track
  {303, 0},     //
  //----------------------------------------------- Function F27 ----------------------------------------------
  {304, 0},     //  3 = Audio MP3 (Long,loop), 4 = Audio MP3 (Short)
  {305, 25},    //  Volume(0-30)
  {306, 27},     //  Track
  {307, 0},     //
  //----------------------------------------------- Function F28 ----------------------------------------------
  {308, 3},     //  3 = Audio MP3 (Long,loop), 4 = Audio MP3 (Short)
  {309, 25},    //  Volume(0-30)
  {310, 28},     //  Track
  {311, 0},
#endif
};
//---------------------------------------------------- RailCom--------------------------------------------------------------------
const byte railcom_encode[] = {0xAC, 0xAA, 0xA9, 0xA5, 0xA3, 0xA6, 0x9C, 0x9A, 0x99, 0x95, 0x93, 0x96, 0x8E, 0x8D, 0x8B, 0xB1, 0xB2, 0xB4, 0xB8, 0x74, 0x72, 0x6C, 0x6A, 0x69, 0x65, 0x63, 0x66, 0x5C, 0x5A, 0x59, 0x55, 0x53,
                               0x56, 0x4E, 0x4D, 0x4B, 0x47, 0x71, 0xE8, 0xE4, 0xE2, 0xD1, 0xC9, 0xC5, 0xD8, 0xD4, 0xD2, 0xCA, 0xC6, 0xCC, 0x78, 0x17, 0x1B, 0x1D, 0x1E, 0x2E, 0x36, 0x3A, 0x27, 0x2B, 0x2D, 0x35, 0x39, 0x33,
                               0x0F, 0xF0, 0xE1, 0x1F
                              };
//-------------------------------------------------------------------------------------------------------------------------------------
uint8_t FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs) / sizeof(CVPair);
//---------------------------------------------- Reset decoder, write to CV 8 ----------------------------------------------------------
void notifyCVResetFactoryDefault() {
  FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs) / sizeof(CVPair);
  FactoryDefault();
};
//----------------------------------------- Speed and Direction ----------------------------------------------------------------------------------------
void notifyDccSpeed( uint16_t Addr, DCC_ADDR_TYPE AddrType, uint8_t Speed, DCC_DIRECTION Dir, DCC_SPEED_STEPS SpeedSteps ) {
  newDirection = Dir; newSpeed = Speed; numSpeedSteps = SpeedSteps;
  if (StateShunting == 1)                     // Shuting mode
    newSpeed = newSpeed / 2;                  // Slow speed
  if (Commande == '6')                        // Trace vitesse sur Traceur série
    Serial.println(newSpeed);
};
//------------------------------------------ Functions -------------------------------------------------------------------------------
void notifyDccFunc( uint16_t Addr, DCC_ADDR_TYPE AddrType, FN_GROUP FuncGrp, uint8_t FuncState)  {
  switch (FuncGrp) {
    case FN_0_4:    //Function Group 1 F0 F4 F3 F2 F1
      NewLedStateF0 = (FuncState & FN_BIT_00) >> 4 ? 1 : 0;
      exec_function( 1, Function1, (FuncState & FN_BIT_01));
      exec_function( 2, Function2, (FuncState & FN_BIT_02) >> 1);
      exec_function( 3, Function3, (FuncState & FN_BIT_03) >> 2 );
      exec_function( 4, Function4, (FuncState & FN_BIT_04) >> 3 );
      break;
    case FN_5_8:    //Function Group 1 S FFFF == 1 F8 F7 F6 F5  &  == 0  F12 F11 F10 F9 F8
      exec_function( 5, Function5, (FuncState & FN_BIT_05));
      exec_function( 6, Function6, (FuncState & FN_BIT_06) >> 1 );
      exec_function( 7, Function7, (FuncState & FN_BIT_07) >> 2 );
      exec_function( 8, Function8, (FuncState & FN_BIT_08) >> 3 );
      break;
    case FN_9_12:
      exec_function( 9, Function9,   (FuncState & FN_BIT_09));
      exec_function( 10, Function10, (FuncState & FN_BIT_10) >> 1 );
      exec_function( 11, Function11, (FuncState & FN_BIT_11) >> 2 );
      exec_function( 12, Function12, (FuncState & FN_BIT_12) >> 3 );
      break;
#if defined F13_F28
    case FN_13_20:   //Function Group 2 FuncState == F20-F13 Function Control
      exec_function( 13, Function13, (FuncState & FN_BIT_13));
      exec_function( 14, Function14, (FuncState & FN_BIT_14) >> 1 );
      exec_function( 15, Function15, (FuncState & FN_BIT_15) >> 2 );
      exec_function( 16, Function16, (FuncState & FN_BIT_16) >> 3 );
      exec_function( 17, Function17, (FuncState & FN_BIT_17) >> 4 );
      exec_function( 18, Function18, (FuncState & FN_BIT_18) >> 5 );
      exec_function( 19, Function19, (FuncState & FN_BIT_19) >> 6 );
      exec_function( 20, Function20, (FuncState & FN_BIT_20) >> 7 );
      break;
    case FN_21_28:
      exec_function( 21, Function21, (FuncState & FN_BIT_21));
      exec_function( 22, Function22, (FuncState & FN_BIT_22) >> 1 );
      exec_function( 23, Function23, (FuncState & FN_BIT_23) >> 2 );
      exec_function( 24, Function24, (FuncState & FN_BIT_24) >> 3 );
      exec_function( 25, Function25, (FuncState & FN_BIT_25) >> 4 );
      exec_function( 26, Function26, (FuncState & FN_BIT_26) >> 5 );
      exec_function( 27, Function27, (FuncState & FN_BIT_27) >> 6 );
      exec_function( 28, Function28, (FuncState & FN_BIT_28) >> 7 );
      break;
#endif
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
// Config: 0 = LED On/Off,  1 = LED Blink,  2 = Servo,  3 = Audio MP3 (Long,loop), 4 = Audio MP3 (Short),  5 = Shunting mode
//--------------------------------------------------------------------------------------------------------------------------------------
void exec_function (int function, int pin, int FuncState)  {
  function--;   // !!!!!!!!!!
  switch ( Dcc.getCV( 120 + (function * 4)) )  {
    case 0:                                                                // On - Off LED
      digitalWrite (pin, FuncState);
      ftn_queue[function].inuse = 0;
      break;

    case 1:                                                                 // Blinking LED
      if ((ftn_queue[function].inuse == 0) && (FuncState == 1))  {
        ftn_queue[function].inuse = 1;
        ftn_queue[function].start_value = 0;
        digitalWrite(pin, 0);
        ftn_queue[function].stop_value = int(Dcc.getCV( 123 + (function * 4)));
      } else {
        if ((ftn_queue[function].inuse == 1) && (FuncState == 0)) {
          ftn_queue[function].inuse = 0;
          digitalWrite(pin, 0);
        }
      }
      break;

    case 2:                                                            // Servo
      if (ftn_queue[function].inuse == 0)  {
        ftn_queue[function].inuse = 1;
        servo[function].attach(pin);
      }
      if (FuncState == 1) ftn_queue[function].increment = char ( Dcc.getCV( 121 + (function * 4)));
      else ftn_queue[function].increment = - char(Dcc.getCV( 121 + (function * 4)));
      if (FuncState == 1) ftn_queue[function].stop_value = Dcc.getCV( 123 + (function * 4));
      else ftn_queue[function].stop_value = Dcc.getCV( 122 + (function * 4));
      break;

    case 3:                                                            // Audio Play (Long, loop)
      if ((digitalRead(busy_pin) == HIGH) && (FuncState != 0)) {       // Audio Off = Not Playing     Occupé; état bas
        ftn_queue[function].inuse = 1;
        mp3_set_volume (ftn_queue[function].increment);
        mp3_play (ftn_queue[function].start_value);                    //  play clip function
      }
      if ((digitalRead(busy_pin) == LOW) && (FuncState == 0)) {        // Audio On = Playing          Occupé; état bas
        ftn_queue[function].inuse = 0;                                 // Function turned off so get ready to stop
      }
      break;

    case 4:                                                            // Audio Play (Short)
      if (digitalRead(busy_pin) == HIGH) {                             // Audio Off = Not Playing     Occupé; état bas
        if (ftn_queue[function].inuse != FuncState) {
          mp3_set_volume (ftn_queue[function].increment);
          mp3_play (ftn_queue[function].start_value);                 //  play clip function
          ftn_queue[function].inuse = FuncState;
        }
      }
      break;
    case 5:                                                  // Shunting mode  Mode manoeuvre
      if (FuncState == 1) {
        digitalWrite (LED_PIN_FL, HIGH);                     // FL RL    ON
        digitalWrite (LED_PIN_RL, HIGH);
        StateShunting = 1;
      }
      else {
        if (StateShunting == 1) {
          digitalWrite (LED_PIN_FL, LOW);                    // FL RL   OFF
          digitalWrite (LED_PIN_RL, LOW);
          StateShunting = 0;
          NewLedStateF0 = 0;
        }
      }
      ftn_queue[function].inuse = 0;
      break;
    default:
      ftn_queue[function].inuse = 0;
      break;
  }
}
//-----------------------------------------------------------------------------------------------------------------------
uint8_t MoyenneTab( uint8_t x) {    //  Depending on the value of x (0 or 1), adds and averages the even or odd cells of the table.
  word MoyenneABC = 0;
  for (int i = x; i < 16 + x; i += 2)
    MoyenneABC +=  TabValBit[i] ;
  return MoyenneABC / 8 ;
}
//---------------------------------------------------------------------------------------------------------------------------
void BitChange() {                                       // This Function waits for a rising or falling edge.
  ii = 0;                                                // ii = To exit loops if no DCC signal.
  if (Step % 2 == 0) {
    do {
      ii++;
    } while  ((digitalRead(DCC_PIN) == 0) && (ii != 0));    // Waits Falling edge
    do {
      ii++;
    } while  ((digitalRead(DCC_PIN) == 1) && (ii != 0));    // Wait Rising edge
  }
  else {
    do {
      ii++;
    } while  ((digitalRead(DCC_PIN) == 1) && (ii != 0));    // Wait Rising edge
    do {
      ii++;
    } while  ((digitalRead(DCC_PIN) == 0) && (ii != 0));    // Waits Falling edge
  }
}
//----------------------------------------------- Ack CV -------------------------------------------------------------------------
void notifyCVAck(void) {
  digitalWrite(MOTOR_PWM_FWD_PIN, HIGH);                // Motor ON
  digitalWrite(MOTOR_PWM_REV_PIN, LOW);
  digitalWrite(LED_PIN_FL, HIGH);                       // FL ON
  digitalWrite(LED_PIN_RL, HIGH);                       // RL ON
  delay( 8 );
  digitalWrite(MOTOR_PWM_FWD_PIN, LOW);                 // All OFF
  digitalWrite(MOTOR_PWM_REV_PIN, LOW);
  digitalWrite(LED_PIN_FL, LOW);
  digitalWrite(LED_PIN_RL, LOW);
}
//---------------------------------------------- Set Function ----------------------------------------------------------
void SetFunction() {
  LED_PIN_FL = Dcc.getCV(33);          // FL
  LED_PIN_RL = Dcc.getCV(34);          // RL
  Function1 = Dcc.getCV(35);           // F1
  Function2 = Dcc.getCV(36);           // F2
  Function3 = Dcc.getCV(37);           // F3
  Function4 = Dcc.getCV(38);           // F4
  Function5 = Dcc.getCV(39);           // F5
}
//-------------------------------------------- Set Pin Output ---------------------------------------------------------
void SetPinOutput() {
  pinMode(LED_PIN_FL, OUTPUT);
  pinMode(LED_PIN_RL, OUTPUT);
  pinMode(Function1, OUTPUT);
  pinMode(Function2, OUTPUT);
  pinMode(Function3, OUTPUT);
  pinMode(Function4, OUTPUT);
  pinMode(Function5, OUTPUT);
  pinMode(MOTOR_PWM_FWD_PIN, OUTPUT);
  pinMode(MOTOR_PWM_REV_PIN, OUTPUT);
  digitalWrite(MOTOR_PWM_FWD_PIN, LOW);
  digitalWrite(MOTOR_PWM_REV_PIN, LOW);
  pinMode(RcEnable, OUTPUT);                        // RailCom
  pinMode(0, INPUT_PULLUP);                         // RX Trigger 1
  pinMode(8, INPUT_PULLUP);                         // RX Trigger 2
}//-------------------------------------- SetEeprom ---------------------------------------------------------------
void SetEeprom() {
#if defined(DECODER_LOADED)
  FactoryDefault();
#endif
}
void FactoryDefault() {
  for (int j = 0; j < FactoryDefaultCVIndex; j++ )
    Dcc.setCV( FactoryDefaultCVs[j].CV, FactoryDefaultCVs[j].Value);
  digitalWrite(LED_PIN_FL, 1);                      // FL and RL ON
  digitalWrite(LED_PIN_RL, 1);
  delay (1000);                                     // 1 seconde
  digitalWrite(LED_PIN_FL, 0);                      // All OFF
  digitalWrite(LED_PIN_RL, 0);
}
//----------------------------------------- Set CV -------------------------------------------------------------------
void SetCV() {                                     // Read the current CV values
  cv1 = Dcc.getCV(1);
  vStart = Dcc.getCV(2);
  acc_Rate = Dcc.getCV(3);  if (acc_Rate > 0) Step_acc = 10 / acc_Rate;
  dec_Rate = Dcc.getCV(4);  if (dec_Rate > 0) Step_dec = 10 / dec_Rate;
  vHigh = Dcc.getCV(5);
  cv17 = Dcc.getCV(17);
  cv18 = Dcc.getCV(18);
  cv26 = Dcc.getCV(26);
  cv27 = Dcc.getCV(27);
  cv28 = Dcc.getCV(28);
  cv29 = Dcc.getCV(29);
  if (bitRead(cv29, 0) == 1)  InvSens = 1;   else InvSens = 0;     // Test bit 0 de cv 29 (Reverse direction)
  if (bitRead(cv29, 3) == 1)  RailComOn = 1; else RailComOn = 0;   // Test bit 3 de cv 29 (RailCom On or Off)
  if (bitRead(cv29, 4) == 1)  TabVit = 1;    else TabVit = 0;      // Test bit 4 de cv 29 (Speed Table  28 steps)
  if (bitRead(cv29, 5) == 1)  AdrEtend = 1;  else AdrEtend = 0;    // Test bit 5 de cv 29 (Extended address)
  for (int i = 0; i < 28; i++)
    TableVitesse[i] = Dcc.getCV(i + 67);

  for (i = 0; i < 5; i++)
    fpins[i] = Dcc.getCV(35 + i);

  if ((vHigh > 1) && (vHigh > vStart))                             // Calculate PWM value in the range 1..255
    vScaleFactor = vHigh - vStart;
  else
    vScaleFactor = 255 - vStart;
}
//----------------------------------------- Set DFPlayer --------------------------------------------------------------
void SetDFPlayer() {
  pinMode (busy_pin, INPUT);                            // MUST NOT Pull Up == 3.3V device output pin
  mySerial.begin (9600);                                // MP3
  mp3_set_serial (mySerial);                            // set softwareSerial for DFPlayer-mini mp3 module
  mp3_reset ();
  delay(100);
  mp3_set_volume (25);
  delay(50);
}
//-------------------------------------- Function RailCom -------------------------------------------------------
#if defined RailCom
static void append12(uint8_t nibble, uint8_t data, uint8_t i) {
  RcData[i++] = railcom_encode[((nibble << 2) | (data >> 6)) & 0x3F];
  RcData[i++] = railcom_encode[data & 0x3f];
}
//-------------------------------------- Set RailCom ---------------------------------------------------------------
void SetRailCom() {
  if (AdrEtend == 1) {                      // Extended address
    uint16_t Add = ((cv17 - 192) * 256 ) + cv18;
    append12(1, highByte(Add), 0);          // Encode nibble 1 adress loco
    append12(2, (128 + lowByte(Add)), 2);   // Encode nibble 2 adress loco
  }
  else {                                    // Primary address
    append12(1, 0, 0);                      // Encode nibble 1 adress loco
    append12(2, cv1, 2);                    // Encode nibble 2 adress loco
  }
}
//----------------------------------------- Send -----------------------------------------------------
void Send(byte data) {
#if defined (__AVR_ATmega32U4__)
  while (!( UCSR1A & (1 << UDRE1))) {};
  UDR1 = data;
#endif
#if defined(__AVR_ATmega328P__)
  while (!( UCSR0A & (1 << UDRE0))) {};
  UDR0 = data;
#endif
}
//------------------------------------- Interrup. RailCom CutOut-------------------------------------------------------
void CutOut() {                            // Interruption
  if (RailComOn == 0) return;              // Si pas en service, retour
  digitalWrite(RcEnable, 1);               // _____------
  delayMicroseconds(45);                   // 80µs (+ 15µs)
  if ((cv28 == 1) || (cv28 == 3)) {        //
    Ch1();                                 // Chanel 1
    delayMicroseconds(110);
    if (cv28 == 1) {
      digitalWrite(RcEnable, 0);           // ------______
      return;
    }
    Ch2();                                 // Chanel 2   193µs (+ 15µs)
    delayMicroseconds(80);
    digitalWrite(RcEnable, 0);             // ------______
  }
}
//------------------------------------------- RailCom Ch1---------------------------------------------------------------------
void Ch1() {
  Trame ++;                                // Envoi alternatif des deux datagrammes
  if (Trame % 2 == 0) {
    Send (RcData[0]);  Send (RcData[1]);
  }
  else {
    Send (RcData[2]);  Send (RcData[3]);
  }
}
//------------------------------------------- RailCom Ch2 ---------------------------------------------------------------------
void Ch2() {        // !!!!!!! Pour test, ne fonctionne pas !!! For testing, not working !!!!!!!!
  Send (RcData[4]);
  Send (RcData[5]);
  Send (RcData[6]);
  Send (RcData[7]);
  Send (RcData[8]);
  Send (RcData[9]);
}
#endif
//---------------------------------------------- Inter ABC -----------------------------------------------------------------
ISR(ADC_vect) {                         // Interruption
  if (ADCH < 25)  return;               // Rejects erroneous measurements
  TabValBit[Step] = ADCH;
}
//---------------------------------------------- Set ABC ---------------------------------------------------------------
void SetABC() {                         // Analog/digital converter configuration
# if defined BrakeABC
  ADCSRA = B10001100;                   // ADEN  ADSC  ADATE ADIF ADIE ADPS2 ADPS1 ADPS0  with Interruption
  ADCSRB = B00000000;                   //             MUX5 = 0
#if defined (__AVR_ATmega32U4__)
  ADCSRB = B00100000;                   //             MUX5 = 1
  ADMUX =  B01100010;                   // REFS1 REFS0 ADLAR MUX4 MUX3 MUX2  MUX1  MUX0   (ADC10)
#endif
#if defined(__AVR_ATmega328P__)
  ADMUX =  B01100101;                   // REFS1 REFS0 ADLAR MUX4 MUX3 MUX2  MUX1  MUX0   (ADC5)
#endif
#endif
}
//----------------------------------------------- Accel / Decel ----------------------------------------------------------------
void AccelDecel() {
  if (lastDirection != newDirection) {               // Si changement de sens, la loco doit décélérer jusqu'à l'arrêt.
    Arret = 1;
    //if (digitalRead(busy_pin) == HIGH) {
      // mp3_set_volume (30);
      // mp3_play (3);                               // Play braking sound !!!
    //}
  }
  if (Arret == 1) {                                  // Si Arrêt = 1
    if (ad_Pwm > 1)  ad_Pwm -= Step_dec;             // Loco décélère jusqu'à l'arrêt.
  }
  else  {                                            // Si Arrêt = 0
    if (ad_Pwm < newPwm) ad_Pwm += Step_acc;         // Loco accélère.
    if (ad_Pwm > newPwm) ad_Pwm -= Step_dec;         // Loco décélère.
  }
  if (ad_Pwm < 2) {
    Arret = 0;                                       // Loco arrêtée et peut redémarrer dans l'autre sens.
    Direction = newDirection ;
  }
  if (((acc_Rate == 0) && (ad_Pwm < newPwm)) || ((dec_Rate == 0) && (ad_Pwm > newPwm))) {    // Si CV3 ou CV4 = 0, pas d'acc./décél.
    ad_Pwm = newPwm; Direction = newDirection ;
  }
}
//------------------------------------- ABC braking ------ Freinage ABC ------------------------------------------------------
void DetectABC() {
  if (cv27 == 0) return;
# if defined BrakeABC
  Step++;
  BitChange();                                     // Attend changement bit
  delayMicroseconds(10);                           // Attendre milieu du bit
  if (digitalRead(RcEnable) == 0)                  // Pas de mesure dans le CutOut
    ADCSRA |= (1 << ADSC);                         // Start Conversion
  if (Step > 17) {                                 // Calcul ABC après 16 échantillons
    Step = 0;
    ValABCTrack1 = MoyenneTab(0);                  // moyenne 16 échantillons niveaux bas
    ValABCTrack2 = MoyenneTab(1);                  // moyenne 16 échantillons niveaux haus
    MoyenneTrack = abs(ValABCTrack1 - ValABCTrack2); // Différence
    if (Commande == '7')                           // Trace ABC sur Traceur série
    {
      Serial.print(ValABCTrack1);                  // Ligne bleu, tension rail 1
      Serial.print(",");
      Serial.print(ValABCTrack2);                  // Ligne rouge, tension rail 2
      Serial.print(",");
      Serial.println(MoyenneTrack);                // Ligne verte, différence de tension
    }
    if (MoyenneTrack > cv26) {
      switch (cv27) {
        case 1:                                    // Freinage ABC rail droit (Bit0 à 1)
          if (ValABCTrack1 < ValABCTrack2) Arret = 1;
          break;
        case 2:                                    // Freinage ABC rail gauche (Bit1 à 1)
          if (ValABCTrack1 > ValABCTrack2) Arret = 1;
          break;
        case 3:                                    // Freinage ABC deux sens (Bit0 et Bit1 à 1)
          Arret = 1;
          break;
      }
    }
    else {
      Arret = 0;                                   // Pas de détection, pas de freinage
    }
  }
#endif
}
//---------------------------------------------- Setup -------------------------------------------------------------
void setup() {
  Serial.begin(250000);                              // Débit Moniteur
#if defined (__AVR_ATmega32U4__)                     // Pro Micro
  Dcc.pin(1, 2, 0);                                  // Int1 sur Pin 2      DCC
#if defined RailCom
  attachInterrupt(0, CutOut, FALLING);               // Int0 sur Pin 3      CutOut
#endif
  Serial1.begin(250000);                             // Débit RailCom
#endif
#if defined(__AVR_ATmega328P__)                      // Pro Mini
  Dcc.pin(0, 2, 0);                                  // Int0 sur Pin 2      DCC
  attachInterrupt(1, CutOut, FALLING);               // Int1 sur Pin 3      CutOut
#endif
  Dcc.init( MAN_ID_DIY, 10, FLAGS_MY_ADDRESS_ONLY | FLAGS_AUTO_FACTORY_DEFAULT, 0 );
  SetFunction();
  SetPinOutput();
  SetEeprom();
  SetCV();
  SetABC();
  SetDFPlayer();

#if defined RailCom
  SetRailCom();
#endif
#if defined F13_F28
  num_active_fpins = 28;        // Number max Functions
#endif

  for ( int i = 0; i < num_active_fpins; i++) {
    cv_value = Dcc.getCV( 120 + (i * 4)) ;
    switch ( cv_value ) {
      case 0:                                                 // LED on/off
        ftn_queue[i].inuse = 0;
        break;
      case 1:                                                 // LED Blink
        {
          ftn_queue[i].inuse = 0;
          ftn_queue[i].current_position = 0;
          ftn_queue[i].start_value = 0;
          ftn_queue[i].increment = int (char (Dcc.getCV( 121 + (i * 4))));
          digitalWrite(fpins[i], 0);
          ftn_queue[i].stop_value = int(Dcc.getCV( 123 + (i * 4))) ;
        }
        break;
      case 2:                                                 //servo
        {
          //ftn_queue[i].current_position = int (Dcc.getCV( 34 + (i * 5)));
          ftn_queue[i].stop_value = int (Dcc.getCV( 123 + (i * 4)));
          ftn_queue[i].start_value = int (Dcc.getCV( 122 + (i * 4)));
          ftn_queue[i].increment = -int (char (Dcc.getCV( 121 + (i * 4))));
          // attaches servo on pin to the servo object
          servo[i].attach(fpins[i]);
          servo[i].write(ftn_queue[i].start_value);
          for (t = 0; t < servo_start_delay; t++)
          {
            SoftwareServo::refresh();
            delay(servo_init_delay);
          }
          ftn_queue[i].inuse = 0;
          servo[i].detach();
        }
        break;
      case 3:                                                 // Audio Track Play
      case 4:
        ftn_queue[i].inuse = 0;
        ftn_queue[i].increment = int (char (Dcc.getCV( 121 + (i * 4))));
        ftn_queue[i].start_value = int (Dcc.getCV( 122 + (i * 4)));
        break;
      default:
        break;
    }
  }
  wdt_enable(WDTO_500MS);                             // Init Watchdog.
}
//---------------------------------------------- Loop (2.2 ms.) -----------------------------------------------------------------------------
void loop() {
  wdt_reset();                                       // Watchdog
#if defined Monitor
  if (Serial.available() > 0)                        // Si commande reçue du moniteur série.
    Commandes();                                     // Traitement de la commande.
#endif
#if defined (__AVR_ATmega32U4__)                     // Pro Micro
  if (Serial1.available() > 0)                       // Input Trigger 1
    Trigger1();
#endif
  if (mySerial.available() > 0)                      // Input Trigger 2
    Trigger2();

  Dcc.process() ;
  SoftwareServo::refresh();
  delay(2);

  if (lastSpeed != newSpeed) {                       // Si la vitesse à changé
    lastSpeed = newSpeed;
    if (newSpeed < 3) {                              // Stop if speed = 0 or 1
      newPwm  = 0;
    }
    else { //                              -------- vHigh  vStart ---------
      modSpeed = newSpeed - 1;
      modSteps = numSpeedSteps - 1;
      newPwm = (uint8_t) vStart + modSpeed * vScaleFactor / modSteps;
      //Serial.print(newPwm);Serial.print("  "); Serial.print(vStart);Serial.print("  ");Serial.print(modSpeed);Serial.print("  ");Serial.print(vScaleFactor);Serial.print("  ");Serial.println(modSteps);
    }
  }

  DetectABC();

  //------------------------------------------- Mise à jour de la vitesse ---------------------------------
  currentTime = millis();
  if ((currentTime - previousTime) > 35) {         // 35 ms.
    previousTime = currentTime;
    // ------ Table de vitesse ----------
    if (TabVit == 1) {
      newPwm1 = newPwm / 9.14;
      newPwm = TableVitesse[newPwm1];
    }
    // ------ Accel_Decel ---------
    AccelDecel();
    // ------ Motor ---------
    if (ad_Pwm != lastPwm) {                               // Vitesse change ?
      if (Direction == 1) {                                // Marche avant.
        analogWrite(MOTOR_PWM_REV_PIN, ad_Pwm);
        digitalWrite(MOTOR_PWM_FWD_PIN, LOW);
      }
      else {                                               // Marche arrière.
        digitalWrite(MOTOR_PWM_REV_PIN, LOW);
        analogWrite(MOTOR_PWM_FWD_PIN, ad_Pwm);
      }
      lastPwm = ad_Pwm;
    }
    if (Commande == '8') {                               // Accél_Décél sur Traceur série
      Serial.print (newPwm);
      Serial.print (",");
      Serial.println (ad_Pwm);
    }
  }
  // -------- Invertion  feux avant / arrière ----------
  if ((lastDirection != Direction) || (lastLedStateF0 != NewLedStateF0))    // Handle Direction and Headlight changes
  {
    lastDirection = newDirection;   lastLedStateF0 = NewLedStateF0;
    if (NewLedStateF0)
    {
      digitalWrite(LED_PIN_FL, newDirection ? HIGH : LOW);
      digitalWrite(LED_PIN_RL, newDirection ? LOW : HIGH);
    }
    else
    {
      digitalWrite(LED_PIN_FL, LOW);
      digitalWrite(LED_PIN_RL, LOW);
    }
  }

  for (int i = 0; i < num_active_fpins; i++) {
    if (ftn_queue[i].inuse == 1)  {
      switch (Dcc.getCV( 120 + (i * 4))) {
        case 0:
          break;
        case 1:                                              // Blink
          ftn_queue[i].current_position = ftn_queue[i].current_position + ftn_queue[i].increment;
          if (ftn_queue[i].current_position > ftn_queue[i].stop_value) {
            ftn_queue[i].start_value = ~ftn_queue[i].start_value;
            digitalWrite(fpins[i], ftn_queue[i].start_value);
            ftn_queue[i].current_position = 0;
            ftn_queue[i].stop_value = int(Dcc.getCV( 123 + (i * 4)));
          }
          break;
        case 2:                                               // Servo
          {
            if (servo_slow_counter++ > servo_slowdown)
            {
              ftn_queue[i].current_position = ftn_queue[i].current_position + ftn_queue[i].increment;
              if (ftn_queue[i].increment > 0) {
                if (ftn_queue[i].current_position > ftn_queue[i].stop_value) {
                  ftn_queue[i].current_position = ftn_queue[i].stop_value;
                  ftn_queue[i].inuse = 0;
                  servo[i].detach();
                }
              }
              if (ftn_queue[i].increment < 0) {
                if (ftn_queue[i].current_position < ftn_queue[i].start_value) {
                  ftn_queue[i].current_position = ftn_queue[i].start_value;
                  ftn_queue[i].inuse = 0;
                  servo[i].detach();
                }
              }
              servo[i].write(ftn_queue[i].current_position);
              servo_slow_counter = 0;
            }
          }
          break;
        case 3:                                              // Audio Track Play
          if (digitalRead(busy_pin) == HIGH) {               // Occupé, état bas
            ftn_queue[i].inuse = 0;
          }
          break;
        default:
          break;
      }
    }
  }
}
//--------------------------------------------------- Input Trigger 1 and 2 ---------------------------------------------------------
void Trigger1() {
#if defined (__AVR_ATmega32U4__)                     // Trigger 1, only Pro Micro
  while (Serial1.available()) {
    char inChar = Serial1.read();
  }
  //*************** put your code here *************
  digitalWrite(LED_PIN_FL, 1);  // Example
  delay (200);
  digitalWrite(LED_PIN_FL, 0);
#endif
}
void Trigger2() {
  while (mySerial.available()) {
    char inChar = mySerial.read();
  }
  //*************** put your code here *************
  //  mp3_set_volume (30);        // Example
  //  mp3_play (10);              //  Horn / Claxon
}
//--------------------------------------------------- Serial Monitor --------------------------------------------------------
#if defined Monitor
void Commandes () {
  int i; int j;
#if defined Francais
  char Dea[] = "Désactivé";
  char Act[] = "Activé";
  char Select[] = "Choisissez";
  char ListCv[] = " 1 --- Liste des CV ---";
  char ModCv[] = " 2 --- Modification des CV ---";
  char RstCv[] = " 3 --- Reset des CV ---";
  char TabV[] = " 4 --- Courbe de vitesses --- ";
  char ManV[] = " 6 --- Trace manette vitesse, ";
  char BrakABC[] = "Freinage ABC";
  char Stop[] = " 0 --- STOP moniteur ---";
  char OpenTS[] = "ouvrir le Traceur série ---";
  char ChanelRC[] = "RailCom canal";
#endif
#if defined English
  char Dea[] = "Disabled";
  char Act[] = "Enabled";
  char Select[] = "Choose";
  char ListCv[] = " 1 --- CV List ---";
  char ModCv[] = " 2 --- CV Editing ---";
  char RstCv[] = " 3 --- CV Reset ---";
  char TabV[] = " 4 --- speed curve --- ";
  char ManV[] = " 6 --- Trace control vitesse, ";
  char BrakABC[] = "Brake ABC";
  char Stop[] = " 0 --- STOP monitor ---";
  char OpenTS[] = "open Serial Plotter ---";
  char ChanelRC[] = "RailCom chanel";
#endif
  char Cv[] = " CV: ";
  char Val[] = "   Val = ";
  char TestRC[] = " 9 --- Test RailCom ---";
  char Space[] = "--------------------------------";

  uint8_t c = Serial.read();                       // Lit le carractère
  switch (c) {
    case '0':
      Commande = 0;                                // 0  Arrête le défilement sur moniteur
      break;
    case '1':                                      // 1: Liste des CV
      Serial.println (ListCv);
      for (i = 1; i < 43; i++) {
        Serial.print (Cv);
        Serial.print (i);
        Serial.print (Val);
        Serial.println (Dcc.getCV(i));
      }
      Serial.print (Space); Serial.print (TabV); Serial.println (Space);
      for (i = 67; i < 95; i++) {
        Serial.print (Cv);
        Serial.print (i);
        Serial.print (Val);
        Serial.println (Dcc.getCV(i));
      }
      for (i = 120; i < 184; i++) {
        if (i % 4 == 0) {
          Serial.print(Space);
          Serial.print(" F");
          Serial.print (i / 4 - 29);
          Serial.println (Space);
        }
        Serial.print (Cv); Serial.print (i); Serial.print (Val); Serial.println (Dcc.getCV(i));
      }
      Serial.println ("");
      break;
    case '2':                                      // 2: Modifier un CV
      Serial.println (ModCv);
      Serial.println (F("Numéro du CV à modifier puis Enter"));
      c = entradaSerie (183);
      i = c;
      Serial.print ("CV: ");
      Serial.print (c);
      Serial.print (" Valeur = ");
      Serial.println (Dcc.getCV(c));
      Serial.println ("Entrer nouvelle valeur");
      c = entradaSerie (255);
      Dcc.setCV (i, c);
      Serial.print ("CV: ");
      Serial.print (i);
      Serial.println (" modifié");
      Serial.print ("Nouvelle valeur = ");
      Serial.println (Dcc.getCV(i));
      SetCV();                                           // Réinit. les CV
      Serial.println ("");
      break;
    case '3':                                            // 3: Reset des CV
      Serial.println ( RstCv);
      Serial.println (F("Reset factory CV, are you sure ?  0 = No    1 = Yes"));
      c = entradaSerie (1);
      if (c == 1) {
        FactoryDefault();
        Serial.println ("Done");
      }
      else
        Serial.println ("Canceled");
      Serial.println ("");
      break;

    case '4':                                              // 4: Table de vitesse, graphique
      uint8_t x ; uint8_t y;
      uint8_t TableVitesse2[28];
      Serial.println (TabV);                               // Table de vitesse
      for (i = 0; i < 28; i++)
        TableVitesse2[i] = TableVitesse[i] / 8;            // Copie les valeur divisées par 8 dans tableau 2

      for (y = 32; y >= 0; y--) {                          // Boucle Y (32 lignes)
        Serial.print ("|");                                // Debut ligne, écrit: |
        for (x = 0; x < 28; x++) {                         // Boucle X (28 pas)
          Serial.print ("   ");                            // Avance curseur
          if ( TableVitesse2[x] == y) {                    // Cherche dans tableau valeur = Y
            Serial.print ("*   ");                         // Trouvé, écrit *
            Serial.print (TableVitesse[x]); Serial.print (" (Cv"); Serial.print (x + 67); Serial.print (")");
            goto a;                                        // Sort de la boucle X
          }
        }
a:
        Serial.println ("");                               // Ligne suivante
      }
      Serial.print (Space); Serial.print (Space); Serial.println (Space);
      Serial.print ("Val Cv ");
      for (i = 0; i < 28; i++) {
        Serial.print(TableVitesse[i], DEC);
        Serial.print (" ");
      }
      Serial.println ("");
      break;

    case '5':                                              // 5:   CV 27, 28, 29
      Serial.println ("--- CV27 ---");  Serial.print (" Val = "); Serial.println (cv27);
      switch (cv27) {
        case 1:  Serial.print (BrakABC); Serial.println(" rail droit (Bit0 à 1)");
          break;
        case 2:  Serial.print (BrakABC) ; Serial.println(" rail gauche (Bit1 à 1)");
          break;
        case 3:  Serial.print (BrakABC) ; Serial.println(" deux sens (Bit0 et Bit1 à 1)");
          break;
        default: Serial.print ("Pas de "); Serial.println(BrakABC);
          break;
      }
      Serial.println ("--- CV28 ---");  Serial.print (" Valeur = "); Serial.println (cv28);
      switch (cv28) {
        case 1:  Serial.print (ChanelRC); Serial.println (" 1 (Bit0 à 1)");
          break;
        case 2:  Serial.print (ChanelRC); Serial.println (" 2  (Bit1 à 1)");
          break;
        case 3:  Serial.print (ChanelRC); Serial.println (" 1 et 2 (Bit0 et Bit1 à 1)");
          break;
        default: Serial.println ("No Tx RailCom");
          break;
      }
      Serial.println ("--- CV29 ---"); Serial.print (" Valeur = "); Serial.println (cv29);
      Serial.print ("Sens inverse:  ");
      if (cv29 & CV29_LOCO_DIR) Serial.println (Act); else Serial.println (Dea);  // Teste bit 0 de cv 29 (Sens inverse de marche)
      Serial.print ("RailCom:       ");
      if (cv29 & CV29_RAILCOM_ENABLE) Serial.println (Act); else Serial.println (Dea);   // Teste bit 3 de cv 29 (RailCom On ou Off)
      Serial.print ("Table vitesse: ");
      if (cv29 & CV29_SPEED_TABLE_ENABLE) Serial.println (Act); else Serial.println (Dea);  // Teste bit 4 de cv 29 (Table de vitesse programmable 28 pas)
      Serial.println ("");
      break;

    case '6':                                              // Trace vitesse
    case '7':                                              // Trace ABC
    case '8':                                              // Trace Acc / Dec
      Commande = c;
      break;
    case '9':                                              // Test RailCom
#if defined RailCom
      Serial.println (TestRC);
      Serial.println ("Canal 1 taper 1, Canal 2 taper 2  puis Enter");
      c = entradaSerie (2);
      if (c == 1) {                             // Canal 1
        Serial.println ("Canal 1; Entrer adresse loco. puis Enter");
        c = entradaSerie (255);
        if (c > 127) {                          // Primary or extended address?
          append12(1, highByte(c), 0);          // Encode nibble 1 adress loco
          append12(2, (128 + lowByte(c)), 2);   // Encode nibble 2 adress loco
        }
        else {
          append12(1, 0, 0);                    // Encode nibble 1 adress loco
          append12(2, c, 2);                    // Encode nibble 2 adress loco
        }
        Serial.print ("Data envoyés: "); Serial.print (RcData[0], HEX); Serial.print("  "); Serial.print (RcData[1], HEX);
        Serial.print("  "); Serial.print (RcData[2], HEX); Serial.print("  "); Serial.println (RcData[3], HEX);
      }
      if (c == 2) {                             // Canal 2
        Serial.println ("Canal 2; Entrer le data puis Enter");
        c = entradaSerie (255);
        append12(0, c, 4);                      // Encode nibble 0
        Serial.print ("Data envoyés: "); Serial.print (RcData[4], HEX); Serial.print("  "); Serial.println (RcData[5], HEX);
      }
      Serial.println ("");
#endif
      break;
    case '\n':
    case '\r':
    default:
      Serial.println (Space);
      Serial.println ("Choisissez");
      Serial.println (ListCv);
      Serial.println (ModCv);
      Serial.println (RstCv);
      Serial.println (TabV);
      Serial.println (" 5 --- CV 27, 28, 29 ---");
      Serial.print (ManV); Serial.println (OpenTS);
      Serial.print (" 7 --- Trace ABC, "); Serial.println (OpenTS);
      Serial.print (" 8 --- Trace Accél/Décel, "); Serial.println (OpenTS);
      Serial.println (TestRC);
      Serial.println (Stop);
      Serial.println (Space);
      Serial.println ("");
      break;
  }
}

//------------------------------------------------------------------------------------------------------------------------------
int entradaSerie (int maximo) {
  unsigned int  c = 0, valor = 0, n = 0;
  do {
    if (Serial.available() > 0) {
      c = Serial.read();
      if (isDigit(c)) {
        valor = (valor * 10) + (c - '0');
        n++;
      }
    }
  } while ((c != '\n') || (n == 0));
  valor = constrain (valor, 0, maximo);
  return (valor);
}
#endif
//------------------------------------- Marqueur pour analyseur logique --------------------------------------
void marqueur() {
  digitalWrite(RcEnable, 1);
  digitalWrite(RcEnable, 0);
}
//--------------------------------------- Modification dans NmraDpp.Cpp --------------------------------------
/*
  uint8_t writeCV (unsigned int CV, uint8_t Value)
  {
    switch (CV)
    {
    case CV_29_CONFIG:
        // copy addressmode Bit to Flags
      //  Value = Value &  ~CV29_RAILCOM_ENABLE;   // Bidi (RailCom) Bit must not be enabled,
        // because you cannot build a Bidi decoder with this lib.
      //  DccProcState.cv29Value = Value;
        DccProcState.Flags = (DccProcState.Flags & ~FLAGS_CV29_BITS) | (Value & FLAGS_CV29_BITS);
    // no break, because myDccAdress must also be reset
    case CV_ACCESSORY_DECODER_ADDRESS_LSB: // Also same CV for CV_MULTIFUNCTION_PRIMARY_ADDRESS
    case CV_ACCESSORY_DECODER_ADDRESS_MSB:
    case CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB:
    case CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB:
        DccProcState.myDccAddress = -1; // Assume any CV Write Operation might change the Address
    }
*/
