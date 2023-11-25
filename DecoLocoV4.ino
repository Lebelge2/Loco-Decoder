// Program sources:
// Geoff Bunza  https://forum.mrhmag.com/post/sma20-low-cost-17-channel-dcc-decoders-ver-6-01-with-soundtriggered-soundstepperdual-motorled-and-12201920
//
// Contact: lebelge2@yahoo.fr
// Dernière modif: 25/11/23
// Version V.4.0.0
//---------------------- MOBILE LOCOMOTIVE DECODER FOR ARDUINO PRO MINI (5V 16MHz) ------ -------------------------
//
//   Necessary libraries are here: https://d28lcup14p4e72.cloudfront.net/259338/7563408/new-multifunction-decoderv6_1%20MRH_V6_01_SMA.zip
//   Copy the librarie:  SoftwareServo.
//   Copy the   AP_DCC_library.h   from: https://github.com/aikopras/AP_DCC_library
//
// - Short and long addresses
// - 14, 28 or 128 speed steps
// - 8 Functions
// - Extern MP3 sound module  28 sounds
// - Servos
// - Inputs counter / Trigger
// - Shunting mode (mode manoeuvre)
// - Acceleration/Deceleration
// - RailCom
// - ABC braking (left or right rail or both)
// - Speed table
// - Modification of CVs with control unit or serial monitor.
//
//------- Pin Arduino Pro Mini ------------------------------------
// Pin RailCom    1 Output serie datas RailCom 250kb (TX)
//                3 Input CutOut
//                4 Output RailCom Enable
// Pin DCC        2 Input Track 1
//                6 Input Track 2
// Pin BEMF       A0 Input BEMF motor
// Pin ABC        A7 Input ABC
// Pin Player     5 Input Busy module MP3
//                7 TX Module MP3
// Pin Moteur     9 Output motor
//               10 Output motor
// Pin Functions A3 A2 A1 13 12 11 A5 A4     (See cv33 to 40)

#if defined (__AVR_ATmega328P__)                    // *** Arduino Pro Mini ***
#else
#error "Unsupported CPU, you need to add another configuration section for your CPU"
#endif

//#define DisplayOLED                          // Si OLED utilisé

#include <Arduino.h>
#include <AP_DCC_library.h>                    // Aiko Pras
#include <SoftwareServo.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <avr/wdt.h>
#ifdef DisplayOLED                           
#include <Wire.h>
#include "SSD1306Ascii.h"                       // https://www.arduino.cc/reference/en/libraries/ssd1306ascii/
#include "SSD1306AsciiAvrI2c.h"
SSD1306AsciiAvrI2c display;
#endif

#define Track1   2                              // INTx Track 1
#define CutOut_Pin 3                            // INTx
#define RcEnable  4                             // __--__
#define Track2   6                              // Track 2
#define MOTOR_PWM_FWD_PIN 9                     // PWM Motor
#define MOTOR_PWM_REV_PIN 10                    //  "" Motor
#define servo_start_delay 50
#define servo_init_delay  7
#define servo_slowdown    4                     //servo loop counter limit
#define RECIP_256 0.00390625f                   // 1/256
#define RECIP_16  0.0625f                       // 1/16

SoftwareServo servo[5];
SoftwareSerial mySerial(8, 7);                  // RX, TX serial to DFPlayer
int servo_slow_counter =  0;
uint8_t fpins[10];
uint8_t busy_pin = 5;                           // DFPlayer Busy status pin
uint8_t NewLedStateF0;
uint8_t lastLedStateF0;
uint8_t Direction = 1;
uint8_t newDirection = 0;
uint8_t lastDirection = 0;
uint8_t Speed = 0;
uint8_t LED_PIN_FL;                  // FL   Light
uint8_t LED_PIN_RL;                  // RL   Light
uint8_t num_active_fpins = 6;        // Number max Functions
uint8_t RcData[10];                  // Data RailCom Ch1 et Ch2
uint8_t Trame;                       // Even/odd, RailCom frame
uint8_t cv1, cv17, cv18, cv23, cv24, cv26, cv27, cv28, cv29, cv54, cv57, cv58, cv59;
uint8_t vStart;                      // cv2
uint8_t acc_Rate;                    // cv3
uint8_t dec_Rate;                    // cv4
float Step_acc, Step_dec;
uint8_t vHigh;                       // cv5
uint8_t vMid;                        // cv6
uint8_t CV11timeout;                 // cv11
uint8_t acc_Adjust;                  // cv23
uint8_t dec_Adjust;                  // cv24
uint8_t ABC;                         // cv27
uint8_t cv_value;
uint8_t Arret;                       // ABC
uint8_t TableVitesse[30];
uint8_t Step1;                        // ABC
uint8_t Step2;                        // BEMF
uint8_t i, ii, t;
uint8_t TabValBit[20];              // ABC
uint16_t ValABCTrack1;              // ABC
uint16_t ValABCTrack2;              // ABC
uint16_t DifferenceTrack;           // ABC
uint16_t LevelABC;                  // ABC
uint8_t TabValBEMF[10];             // BEMF
uint8_t ValBEMF;                    // BEMF
uint8_t Commande;                   // Serial monitor
unsigned long currentTime = 0;      // Function millis
unsigned long previousTime = 0;     // Function millis
unsigned long currentTimeDcc = 0;   // Function millis
unsigned long previousTimeDcc = 0;  // Function millis
unsigned long currentTimeDfp = 0;   // Function millis
unsigned long previousTimeDfp = 0;  // Function millis
uint8_t Shunting;                   // Manoeuvre
uint8_t StateShunting;
uint8_t adjust;
uint8_t DCCTimeout;
uint16_t nADC;
uint8_t Frame;
extern Dcc dcc;                  // This object is instantiated in DCC_Library.cpp
extern Loco locoCmd;             // To retrieve the data from loco commands  (7 & 14 bit)
extern CvAccess cvCmd;           // To retrieve the data from pom and sm commands
const uint8_t SM = 1;
const uint8_t PoM = 2;
uint8_t MemoFunctF1F4;
uint8_t MemoFunctF5F8;
uint16_t MemoFunctF9F12;
uint32_t MemoFunctF13F20;
uint32_t MemoFunctF21F28;
uint32_t AncFunct;
uint32_t FunctPressed;
uint32_t Funct32bit;
uint8_t CurrentPWM, AncPWM;       // PWM
uint16_t ErreurXOR;
uint8_t SongDesired;
uint8_t RepeatPlay;
bool Ch2Available;
//SSD1306AsciiAvrI2c display;
//--------------------------------------- Structures --------------------------------------------------------
union tDCCFlags {
  struct {
    unsigned AnalogMode      : 1;
    unsigned DigitalMode     : 1;
    unsigned ForceDCCStop    : 1;
    unsigned Direction       : 1;
    unsigned ABCBrake        : 1;
    unsigned ADCSample       : 1;
    unsigned InvSens         : 1;
    unsigned AlgPwrCnvCtrl   : 1;
    unsigned AdvAck          : 1;
    unsigned SpeedTable      : 1;
    unsigned ExtAddr         : 1;
  };
  struct {
    byte Bytes[2];
  };
};
union  tDCCFlags    Flags;
union tBipolar {
  signed char   sVal;
  unsigned char uVal;
};
// feeding path of drive demands:
//      DCC value -> Desired
//      Desired -> Acceleration limiting -> Current
//      Current -> Speed Table -> Derived
//      Derived -> Back EMF -> Actual
//      Actual -> PWM output (HF or LF)
struct tPWM {
  struct {
    byte Desired;                // desired drive: input demand from DCC packet
    byte Current;                // current drive: slews towards Desired (acceleration)
    byte Derived;                // derived drive: effectively SpeedTable[Current]
    volatile  byte Actual;       // actual drive:  to be applied to PWM peripheral
  } Drive;
  struct {
    int  Count;                  // count 
    byte Rate;                   // further limit acceleration rate, larger = slower
  } Accel;
  byte CV9_Period;               // LF PWM period basis
  volatile  byte StepCount;      // fraction of LF PWM Period  (Count / 256)
};
struct tPWM    PWM;

struct tBEMF {
  float Integrator;
  byte CutOff;
  float Kp;
  float Ki;
  float Kfr;
};
struct tBEMF   BEMF;
union tPWMmode {
  struct {
    unsigned sw0  : 1;
    unsigned sw1  : 1;
    unsigned sw2  : 1;
    unsigned sw3  : 1;
    unsigned BEMF : 1;      // bit4, enable Back EMF
    unsigned HF   : 1;      // bit5, Enable HF PWM
  };
  byte Val;
};
union  tPWMmode   CV54_PWMmode;

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
  {1, 3},       // PRIMARY Addr
  {2, 80},      // VSTART           (12v/255)*80 =  3,8v au cran 1 à PWM 490 Hz
  {3, 5},       // Acc_Rate
  {4, 5},       // Dec_Rate
  {5, 255},     // VHIGH             255 => 12v  (U max)
  {6, 185},     // VMID             (12v/255)*185 = 8,7v
  {7, 4},       // Version
  {8, 13},      // Constructeur
  {9, 0},
  {10, 127},    // EMF CutOff.  Reduce if lower BEMF cutoff wanted
  {11, 0},      // CV11timeout Packet_TO 0 is off, else value in seconds
  {12, 0},      //
  {13, 0},      //
  {14, 0},      //
  {15, 0},      //
  {16, 0},      //
  {17, 0},      // ExtAddr1
  {18, 0},      // ExtAddr2
  {19, 0},      // Consist_addr
  {20, 0},      //
  {21, 0},      // CAct_F1_F8
  {22, 0},      // CAct_LGT
  {23, 0},      // Acc_Adjust
  {24, 0},      // Dec_Adjust
  {25, 0},      //
  {26, 90},     // ABC Level
  {27, 0},      // ABC braking (Bit 0 to 1 = right rail, Bit 1 to 1 = left rail, both at 1 = two directions)
  {28, 1},      // RailCom config   Bit 0 = Ch1 ON, Bit 1 = Ch2 ON
  {29, 0},      // Config data   Inverse sens = 1    RailCom = 8   Speed table =16    Adresse étendue = 32
  {30, 0},      // Error code
  {31, 0},      //
  {32, 255},    //
  //--------------------------------- Assignment of outputs FL, RL and F0 to F7 ------------------------------------------
  {33, 17},      // FL  A3
  {34, 16},      // RL  A2
  {35, 15},      // F1  A1
  {36, 13},      // F2  13
  {37, 12},      // F3  12
  {38, 11},      // F4  11
  {39, 18},      // F5  A4  SDA   i2c
  {40, 19},      // F6  A5  SCL   i2c
  {41, 0},       //
  {42, 0},       //
  {43, 0},       //
  {44, 0},       //
  {45, 0},       //
  {46, 0},       //
  {47, 0},       //
  {48, 0},       //
  {49, 0},       //
  {50, 0},       //
  {51, 0},       // bit0 à 1 si on veut un arrêt sur une distance constante.
  {52, 0},       // distance d'arrêt constante (0-255, valeur d'usine = 50)
  {53, 0},       // vitesse du ralenti (0-255, valeur d'usine = 48)
  //---------------------------------------- BEMF --------------------------------------------
  {54, 0x30},    // PWM Mode - HF PWM, BEMF on, no low speed LF.    bit4, enable Back EMF
  {55, 0x10},    // Ki - A conservative value, may be increased
  {56, 0x50},    // Kp - Reduce for coreless motors
  {57, 1},       // Kfr - Play around with this
  {58, 3},       // Fréquence PWM moteur: 1 = 31372.55 Hz   2 = 3921.16 Hz   3 = 490.20 Hz   4 = 122.55 Hz    5 = 30.64 Hz
  {59, 0},       //
  {60, 13},      //
  {61, 0},       //
  {62, 0},       //
  {63, 0},       //
  {64, 0},       //
  {65, 0},       //
  {66, 0},       //
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
  //--------------------------------------------- Speed table end ---------------------------------------------------------------
  {95, 0},
  {96, 0},       //
  {97, 0},       //
  {98, 0},       //
  {99, 0},
  {100, 0},      //
  {101, 0},      //
  {102, 0},      //
  {103, 0},
  {104, 0},      //
  {105, 0},
  {106, 0},
  {107, 0},
  {108, 0},      //
  {109, 0},      //
  {110, 0},      //
  {111, 0},
  {112, 200},   // Clignotement LED lent
  {113, 100},   // Clignotement LED moyen
  {114, 50},    // Clignotement LED rapide
  {115, 10},    // Clignotement LED stromboscope
  {116, 3},     // Servo Rate  <=  5
  {117, 0},     // Start Position servo
  {118, 180},   // End Position servo
  {119, 0},
  {120, 0},
  //----------------------------------------------- Functions  ----------------------------------------------
  // 0 = LED On/Off, 1 = LED cligno lent, 2 = LED cligno moyen, 3 = LED cligno rapide, 4 = LED stromboscope
  // 5 = LED Alternantes   7 = Servo,  8 = Shunting/Manoeuvre
  {121, 0},   // F1
  {122, 5},   // F2
  {123, 3},   // F3
  {124, 4},   // F4
  {125, 0},   // F5
  {126, 0},   // F6
  {127, 0},   //
  {128, 0},   //
};
//---------------------------------------------------- RailCom--------------------------------------------------------------------
const byte railcom_encode[] = {0xAC, 0xAA, 0xA9, 0xA5, 0xA3, 0xA6, 0x9C, 0x9A, 0x99, 0x95, 0x93, 0x96, 0x8E, 0x8D, 0x8B, 0xB1, 0xB2, 0xB4, 0xB8, 0x74, 0x72, 0x6C, 0x6A, 0x69, 0x65, 0x63, 0x66, 0x5C, 0x5A, 0x59, 0x55, 0x53,
                               0x56, 0x4E, 0x4D, 0x4B, 0x47, 0x71, 0xE8, 0xE4, 0xE2, 0xD1, 0xC9, 0xC5, 0xD8, 0xD4, 0xD2, 0xCA, 0xC6, 0xCC, 0x78, 0x17, 0x1B, 0x1D, 0x1E, 0x2E, 0x36, 0x3A, 0x27, 0x2B, 0x2D, 0x35, 0x39, 0x33,
                               0x0F, 0xF0, 0xE1, 0x1F
                              };
//-------------------------------------------------------------------------------------------------------------------------------------
uint8_t FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs) / sizeof(CVPair);
uint8_t getCV (uint16_t CV) {
  return EEPROM.read(CV);
}
//------------------------------------------ Functions -------------------------------------------------------------------------------
void Functions(uint8_t FuncGrp, uint8_t FuncState)  {
  Funct32bit = 0;
  switch (FuncGrp) {
    case 1:
      NewLedStateF0 = (FuncState) >> 4 ? 1 : 0;
      exec_function( 1, fpins[0], (bitRead(FuncState, 0)));
      exec_function( 2, fpins[1], (bitRead((FuncState >> 1), 0)));
      exec_function( 3, fpins[2], (bitRead((FuncState >> 2), 0)));
      exec_function( 4, fpins[3], (bitRead((FuncState >> 3), 0)));
      MemoFunctF1F4 = FuncState & 0x0F;                          // MemoFunct F1 to F4
      break;
    case 2:
      exec_function( 5, fpins[4], (bitRead(FuncState, 0)));
      exec_function( 6, fpins[5], (bitRead((FuncState >> 1), 0)));
      //exec_function( 7, fpins[6], (bitRead((FuncState >> 2), 0)));
      MemoFunctF5F8 = FuncState;                                 // MemoFunct F5 to F8
      break;
    case 3: MemoFunctF9F12 = FuncState; break;                   // MemoFunct F9 to F12
    case 4: MemoFunctF13F20 = FuncState; break;                  // MemoFunct F13 to F20
    case 5: MemoFunctF21F28 = FuncState; break;                  // MemoFunct F21 to F28
    default:
      break;
  }

  Funct32bit |= (MemoFunctF5F8 << 4) | MemoFunctF1F4;            // 32 29  28    21  20    13  12 9  8 5   4  1
  Funct32bit |= (MemoFunctF9F12 << 8);                           //  0000  00000000  00000000  0000  0000  0000
  Funct32bit |= (MemoFunctF13F20 << 12) ;                        // Mémorise l'états des fonctions
  Funct32bit |= (MemoFunctF21F28 << 20) ;                        // dans une variable type: long (32 bits)
  FunctPressed = Funct32bit ^ AncFunct;                          // Détermine la dernière touche fonction pressée
  AncFunct = Funct32bit;                                         // Mémo états des fonctions
  SongDesired = 0;
  for ( int i = 0; i < 32; i++) {                                // Tester les 32 bits
    if (bitRead(FunctPressed, i))                                // Recherche le numéro de la dernière fonction pressée
      SongDesired = SongDesired + (i + 1);                       // Numéro du son = numéro fonction
  }
  // Serial.println (SongDesired);
  if (SongDesired % 2 == 0) {                                    // Numéro pair ?
    if (bitRead(Funct32bit, SongDesired - 1) == 0) {             // Pair: Fonctions bistables, MP3 en boucle
      Player_CMD(0x16, 0, 0);                                   // MP3 Stop
      RepeatPlay = 0;
    }
    else {
      Player_CMD(0x12, 0, SongDesired);                         // MP3 Play    Ex.  F2 ==> play "mp3/0002.mp3"
      RepeatPlay = 1;
    }
  }
  else {                                                          // Impair: Fonctions astables, Mp3 courts
    RepeatPlay = 0;
    Player_CMD(0x12, 0, SongDesired);                            // MP3 Play   Ex.  F5 ==> play "mp3/0005.mp3"
  }
}
//------------------------------------------------------------------------------------------------------------------------------
void exec_function (int function, int pin, int FuncState)  {
  function--;
  cv_value = getCV( 121 + function) ;
  switch (cv_value)  {
    case 0:                                                                // On - Off LED
      digitalWrite (pin, FuncState);
      ftn_queue[function].inuse = 0;
      break;
    case 1:                                                                 // Blinking LED
    case 2:
    case 3:
    case 4:
      if ((ftn_queue[function].inuse == 0) && (FuncState == 1))  {
        ftn_queue[function].inuse = 1;
        ftn_queue[function].start_value = 0;
        digitalWrite(pin, 0);                                                  // Clignotant_
        if (cv_value == 1) ftn_queue[function].stop_value = int(getCV(112)) ;  // lent
        if (cv_value == 2) ftn_queue[function].stop_value = int(getCV(113)) ;  // moyen
        if (cv_value == 3) ftn_queue[function].stop_value = int(getCV(114)) ;  // rapide
        if (cv_value == 4) ftn_queue[function].stop_value = int(getCV(115)) ;  // effet stomboscopique
      } else {
        if ((ftn_queue[function].inuse == 1) && (FuncState == 0)) {
          ftn_queue[function].inuse = 0;
          digitalWrite(pin, 0);
        }
      }
      break;
    case 5:                                                              // Blinking LED PAIR
      if ((ftn_queue[function].inuse == 0) && (FuncState == 1))  {
        ftn_queue[function].inuse = 1;
        ftn_queue[function].start_value = 0;
        digitalWrite(fpins[function], 0);
        digitalWrite(fpins[function + 1], 1);
        ftn_queue[function].stop_value = int(getCV(112) * 2);
      } else {
        if (FuncState == 0) {
          ftn_queue[function].inuse = 0;
          digitalWrite(fpins[function], 0);
          digitalWrite(fpins[function + 1], 0);
        }
      }
      break;
    case 7:                                                            // Servo
      if (ftn_queue[function].inuse == 0)  {
        ftn_queue[function].inuse = 1;
        servo[function].attach(pin);
      }
      if (FuncState == 1)
        ftn_queue[function].increment = char ( getCV(116));
      else
        ftn_queue[function].increment = - char(getCV(116));
      if (FuncState == 1)
        ftn_queue[function].stop_value = getCV(118);
      else
        ftn_queue[function].stop_value = getCV(118);
      break;
    case 8:                                                  // Shunting mode  Mode manoeuvre
      if (FuncState == 1)
        StateShunting = 1;
      else
        StateShunting = 0;
      break;
    default:
      ftn_queue[function].inuse = 0;
      break;
  }
}
//---------------------------------------------------------------------------------------------------------------------------
void BitChange() {                                       // This Function waits for a rising or falling edge.
  ii = 0;                                                // ii = To exit loops if no DCC signal.
  if (Step1 % 2 == 0) {
    do {
      ii++;
    } while  ((digitalRead(Track1) == 0) && (ii != 0));    // Waits Falling edge
    do {
      ii++;
    } while  ((digitalRead(Track1) == 1) && (ii != 0));    // Wait Rising edge
  }
  else {
    do {
      ii++;
    } while  ((digitalRead(Track1) == 1) && (ii != 0));    // Wait Rising edge
    do {
      ii++;
    } while  ((digitalRead(Track1) == 0) && (ii != 0));    // Waits Falling edge
  }
}
//-------------------------------------------- Set Pin Output ---------------------------------------------------------
void SetPin() {
  pinMode(LED_PIN_FL, OUTPUT);                     // 15   A3
  pinMode(LED_PIN_RL, OUTPUT);                     // 16   A2
  for (i = 0; i < num_active_fpins; i++)
    pinMode(fpins[i], OUTPUT);                     // A1, 13, 12, 11, A4, A5

  pinMode(MOTOR_PWM_FWD_PIN, OUTPUT);
  pinMode(MOTOR_PWM_REV_PIN, OUTPUT);
  digitalWrite(MOTOR_PWM_FWD_PIN, LOW);
  digitalWrite(MOTOR_PWM_REV_PIN, LOW);
  pinMode(CutOut_Pin, INPUT);
  pinMode(busy_pin, INPUT);
  pinMode(RcEnable, OUTPUT);                        // RailCom
  pinMode(Track1, INPUT);
  pinMode(Track2, INPUT);
}
//-------------------------------------- SetEeprom ---------------------------------------------------------------
void SetEeprom() {
  if (EEPROM.read(8) != 13) {                                                      // Si EEPROM vide (Première programmation)
    for (int j = 0; j < FactoryDefaultCVIndex; j++ ) {
      EEPROM.update( FactoryDefaultCVs[j].CV, FactoryDefaultCVs[j].Value);         // Copie CV dans zone modifiable
    }
  }
  for (int j = 0; j < FactoryDefaultCVIndex; j++ )
    EEPROM.update(( FactoryDefaultCVs[j].CV) + 128, FactoryDefaultCVs[j].Value);   // Copie CV dans zone non modifiable (CV usine)
}
void FactoryDefault() {                                                            // Reset usine
  wdt_reset();                                                                     // Watchdog
  for (int j = 0; j < 128; j++ )
    EEPROM.update(j, EEPROM.read(j + 128));                                        // Copie CV usine dans zone modifiable
  Update();
}
//----------------------------------------- Set CV -------------------------------------------------------------------
void SetCV() {                                     // Read the current CV values
  cv1 = getCV(1);
  vStart = getCV(2);
  acc_Rate = getCV(3);
  dec_Rate = getCV(4);
  vHigh = getCV(5);
  vMid =  getCV(6);
  CV11timeout = getCV(11);
  cv17 = getCV(17);
  cv18 = getCV(18);
  BEMF.CutOff = getCV(20);
  LevelABC = getCV(26) * 10;
  cv27 = getCV(27);
  cv28 = getCV(28);
  cv29 = getCV(29);
  if (bitRead(cv29, 0) == 1)  Flags.InvSens = 1;   else Flags.InvSens = 0;        // Test bit 0 de cv 29 (Reverse direction)
  if (bitRead(cv29, 3) == 1)  Flags.AdvAck = 1; else Flags.AdvAck  = 0;           // Test bit 3 de cv 29 (RailCom On-Off)
  if (bitRead(cv29, 4) == 1)  Flags.SpeedTable  = 1; else Flags.SpeedTable = 0;   // Test bit 4 de cv 29 (Speed Table 28 steps)
  if (bitRead(cv29, 5) == 1)  Flags.ExtAddr = 1;  else Flags.ExtAddr = 0;         // Test bit 5 de cv 29 (Extended address)
  LED_PIN_FL = getCV(33);          // FL
  LED_PIN_RL = getCV(34);          // RL
  for (i = 0; i < 7; i++)
    fpins[i] = getCV(35 + i);      // Fonctions
  for (int i = 0; i < 28; i++)
    TableVitesse[i] = getCV(i + 67);
  i++;
  TableVitesse[i] = getCV(i + 67);
  CV54_PWMmode.Val = getCV(54);
  BEMF.Ki = (float)getCV(55) * RECIP_256;  // 0 -> 100% Ki: Integration contribution
  BEMF.Kp = (float)getCV(56) * RECIP_16;
  BEMF.Kfr = (float)getCV(57) * RECIP_256;  // 0 -> 100% Kfr: feed forward contribution
  cv58 = getCV(58);
}
//--------------------------------------------- Update --------------------------------------------------------------
void Update() {
  wdt_reset();
  SetCV();                                  // Update CV
  wdt_reset();
  SetRailCom();                             // Update RailCom
}
//--------------------------------------------- Set DFPlayer --------------------------------------------------------
void SetDFPlayer() {
  Player_CMD(0x0c, 0, 0);                           // Reset
  delay(200);
  Player_CMD(0x06, 0, 28);                          // Volume à 28
  delay(50);
}
//---------------------------------------- DFPlayer CMD ------------------------------------------------------------------
void Player_CMD(byte CMD, byte Par1, byte Par2) {
  word checksum = -(0xFF +  06 + CMD + 00 + Par1 + Par2);
  byte Command_line[10] = {0x7E, 0xFF,  06, CMD, 00, Par1, Par2, highByte(checksum), lowByte(checksum), 0xEF };
  wdt_reset();
  for (byte k = 0; k < 10; k++)
    mySerial.write(Command_line[k]);
}
//-------------------------------------- Function RailCom -------------------------------------------------------
static void append12(uint8_t nibble, uint8_t data, uint8_t i) {
  RcData[i++] = railcom_encode[((nibble << 2) | (data >> 6)) & 0x3F];
  RcData[i++] = railcom_encode[data & 0x3f];
}
//-------------------------------------- Set RailCom ------------------------------------------------------------
void SetRailCom() {
  if (Flags.ExtAddr) {                      // Extended address
    uint16_t Add = ((cv17 - 192) * 256 ) + cv18;
    append12(1, highByte(Add), 0);          // Encode nibble 1 adress loco
    append12(2, lowByte(Add), 2);           // Encode nibble 2 adress loco
  }
  else {                                    // Primary address
    append12(1, 0, 0);                      // Encode nibble 1 adress loco
    append12(2, cv1, 2);                    // Encode nibble 2 adress loco
  }
}
//------------------------------------- Interrup. RailCom CutOut-------------------------------------------------------
void CutOut() {                            // Interruption
  if (Flags.AnalogMode) return;            // No railCom if Analog mode
  if (!Flags.AdvAck) return;               // Si pas en service, retour
  digitalWrite(RcEnable, 1);               // _____------
  delayMicroseconds(35);                   // 80µs (+ 15µs)
  if ((cv28 == 1) || (cv28 == 3)) {        //
    SendCh1();                                 // Chanel 1
    delayMicroseconds(110);
    if (cv28 == 1) {
      digitalWrite(RcEnable, 0);           // ------______
      return;
    }
    SendCh2();                                 // 193µs (+ 15µs)
    delayMicroseconds(80);
    digitalWrite(RcEnable, 0);             // ------______
  }
}
//------------------------------------------- RailCom Ch1---------------------------------------------------------------------
void SendCh1() {
  Trame ++;                                // Envoi alternatif des deux datagrammes
  if (Trame % 2 == 0) {
    Serial.write(RcData[0]);
    Serial.write(RcData[1]);
  }
  else {
    Serial.write(RcData[2]);
    Serial.write(RcData[3]);
  }
}
//------------------------------------------- RailCom Ch2 ---------------------------------------------------------------------
void SendCh2() {
  // append12(0, 3, 4);   // CV 3
  //if (Ch2Available == true) {
  Serial.write(RcData[4]);
  Serial.write(RcData[5]);
  //Serial.write(RcData[6]);
  //Serial.write(RcData[7]);
  //Serial.write(RcData[8]);
  //Serial.write(RcData[9]);
  //Ch2Available = false;
  // }
}
//---------------------------------------------- Inter ADC -----------------------------------------------------------------
ISR(ADC_vect) {                         // Interruption
  if (bitRead(ADMUX, 0)) {              // ADC A7
    if (Step1 % 2 == 0)
      ValABCTrack1 += ADCH;
    else
      ValABCTrack2 += ADCH;
  }
  // marqueur();
}
//---------------------------------------------- Set ADC ---------------------------------------------------------------
void SetADC() {                         // Analog/digital converter configuration
  ADCSRA = B10001100;                   // ADEN  ADSC  ADATE  ADIF  ADIE  ADPS2  ADPS1  ADPS0  with Interruption
  ADCSRB = B00000000;                   //             MUX5 = 0
}
//----------------------------------------------- PWM --------------------------------------------------------------------
void SetPWM() {
  if ((cv58 > 0) && (cv58 < 6))                  // 1 à 5
    TCCR1B = TCCR1B & B11111000 | cv58;
  else
    TCCR1B = TCCR1B & B11111000 | B00000011;    // 490.20 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000001;    // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000011;    // set timer 1 divisor to    64 for PWM frequency of   490.20 Hz (The DEFAULT)
  //TCCR1B = TCCR1B & B11111000 | B00000100;    // set timer 1 divisor to   256 for PWM frequency of   122.55 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000101;    // set timer 1 divisor to  1024 for PWM frequency of    30.64 Hz
}
//-------------------------------------------- Display OLED ---------------------------------------------------------------
void SetOLED() {
  #ifdef DisplayOLED
  display.begin(&Adafruit128x32, 0x3C);
  display.setFont(System5x7);
  display.set1X();
  //display.set2X();
  display.setCursor(40, 0);
  display.print("THALYS");
  //display.setCursor(20, 0);
  //display.print("Bruxelles");
  //display.setCursor(20, 1);
  //display.print("255 Kmh");
  //display.setCursor(20, 2);
  //display.print("Ch. De Gaule");
  #endif
}
//------------------------------------- ABC braking ------ Freinage ABC ------------------------------------------------------
void DetectABC() {
  if (cv27 == 0) return;
  ADMUX =  B01100111;                                 // DAC A7    Left Ajust
  Step1++;
  BitChange();
  delayMicroseconds(10);
  if (Step1 < 254) {
    // marqueur();
    bitSet(ADCSRA, ADSC);                             // Start Conversion
  }
  if (Step1 == 0) {                                   // Calcul ABC après 255 échantillons
    if ( ValABCTrack1 >  ValABCTrack2)
      DifferenceTrack = (ValABCTrack1 - ValABCTrack2);
    else
      DifferenceTrack = (ValABCTrack2 - ValABCTrack1);
    if (Commande == '5')                              // Trace ABC sur Traceur série
    {
      Serial.print(ValABCTrack1);                     // Ligne bleu, tensions cumulées rail 1
      Serial.print(",");
      Serial.print(ValABCTrack2);                     // Ligne rouge, tensions cumulées rail 2
      Serial.print(",");
      Serial.println(DifferenceTrack);                // Ligne verte, différence des tensions
    }
    if (DifferenceTrack > LevelABC) {                 // Cv26 Sensibilité  (Cv26 * 10)
      switch (cv27) {
        case 1:                                       // Freinage ABC rail droit (Bit0 à 1)
          if (ValABCTrack1 > ValABCTrack2) {
            // Serial.println("ABC rail droit");
            Flags.ABCBrake = 1;
          }
          break;
        case 2:                                       // Freinage ABC rail gauche (Bit1 à 1)
          if (ValABCTrack1 < ValABCTrack2) {
            //Serial.println("ABC rail gauche");
            Flags.ABCBrake = 1;
          }
          break;
        case 3:                                        // Freinage ABC deux sens (Bit0 et Bit1 à 1)
          // Serial.println("ABC rail gauche et droit");
          Flags.ABCBrake = 1;
          break;
      }
    }
    else {                                               // Pas de détection, pas de freinage
      //Serial.println("Pas ABC");
      Flags.ABCBrake = 0;
      if (StateShunting)
        PWM.Drive.Desired = ((Speed / 2) & 0x7f );              // Shuting mode   Slow speed
      else
        PWM.Drive.Desired = (Speed & 0x7f );
    }
    ValABCTrack1 = 0;
    ValABCTrack2 = 0;
  }
}
//------------------------------------------------ AnalogMode-------------------------------------------------------------
void ManageAnalogControl() {
  if (Flags.AnalogMode) {                              // Analog mode
    if ((currentTimeDcc - previousTimeDcc) > 500) {        // No DCC signal since 500 millis secondes
      previousTimeDcc = currentTimeDcc;
      Serial.println ("Analog mode");
      if ((digitalRead (Track1 == 1)) && (digitalRead (Track2 == 0)))      // Track1 posit. and Track2 negat. ?
        Flags.Direction = 1;
      else if ((digitalRead (Track1 == 0)) && (digitalRead (Track2 == 1))) // Track2 posit. and Track1 negat. ?
        Flags.Direction = 0;
      else {
        Serial.println ("DC PWM");
        return;                                                         // Undetermined ! (because DC PWM)
      }

      if (Flags.Direction = 1) {                           // Marche avant.
        digitalWrite(MOTOR_PWM_FWD_PIN, HIGH);
        digitalWrite(MOTOR_PWM_REV_PIN, LOW);
        digitalWrite(LED_PIN_FL, HIGH);                    // Feux avants
        digitalWrite(LED_PIN_RL, LOW);
      }
      else   {                                             // Marche arrière.
        digitalWrite(MOTOR_PWM_FWD_PIN, LOW);
        digitalWrite(MOTOR_PWM_REV_PIN, HIGH);
        digitalWrite(LED_PIN_FL, LOW);
        digitalWrite(LED_PIN_RL, HIGH);                    // Feux arrières
      }
    }
  }
}
//----------------------------------------------- Accel / Decel ----------------------------------------------------------------
void ManageAcceleration() {               // Desired -> Acceleration limiting -> Current
  byte adjust;
  // Manage acceleration between speed points Only allow acceleration adjustments ~ 3.5ms
  // The acceleration rate is then further controlled by CVs 3, 4, 23 & 24
  currentTime = millis();
  if ((currentTime - previousTime) > 3.5) {                 // 3.5 ms.
    previousTime = currentTime;
    if (PWM.Accel.Rate)
      PWM.Accel.Rate--;
    if (PWM.Accel.Rate == 0) {
      if (lastDirection != newDirection) Arret = 1;        // Si changement de sens, la loco doit décélérer jusqu'à l'arrêt.
      if (PWM.Drive.Current != PWM.Drive.Desired) {
        if (Arret == 1) {                                  // Si Arrêt = 1
          if (PWM.Drive.Current > 1) {
            PWM.Drive.Current--;              // decelerate
            PWM.Accel.Rate = dec_Rate;        // set deceleration rate
            adjust = dec_Adjust;              // adjustment Cv24
          }
        }
        else {
          if (PWM.Drive.Current > PWM.Drive.Desired) {
            PWM.Drive.Current--;              // decelerate
            PWM.Accel.Rate = dec_Rate;        // set deceleration rate
            adjust = dec_Adjust;              // adjustment Cv24
          }
          else {
            PWM.Drive.Current++;              // accelerate
            PWM.Accel.Rate = acc_Rate;        // set acceleration rate
            adjust = acc_Adjust;              // adjustment Cv23
          }
        }
        // if (adjust & 0x80)                  // add or subtract adjustment?
        //  PWM.Accel.Rate -= (adjust & 0x7f);
        // else
        //   PWM.Accel.Rate += (adjust & 0x7f);

        if (PWM.Accel.Rate == 0)
          PWM.Drive.Current = PWM.Drive.Desired;
        if (PWM.Drive.Current < 2) {
          Arret = 0;                              // Loco arrêtée et peut redémarrer dans l'autre sens.
          Direction = newDirection ;
        }

        SplineIt();                                // changed current drive, get speed table result

        //}
        if (Commande == '6') {                      // Graphique sur traceur série
          Serial.print (PWM.Drive.Desired);
          Serial.print (",");
          Serial.print (PWM.Drive.Current);
          Serial.print (",");
          Serial.print (PWM.Drive.Derived);
          Serial.print (",");
          Serial.println (PWM.Drive.Actual);
        }
      }
    }
  }
}
//---------------------------------------------- Spline -------------------------------
void SplineIt() {                              // Current -> Speed Table -> Derived
  uint8_t  LowerKnee, UpperKnee, Difference, V1, Modulo;
  word   M1;
  if (PWM.Drive.Current == 0) {                // no speed demand, STOP!
    PWM.Drive.Derived = 0;
    return;
  }
  if (Flags.SpeedTable) {                       // using speed table Cv29 bit4
    V1 = PWM.Drive.Current  * 2;
    M1 = constrain (V1 / 9, 0, 27);
    Modulo = V1 % 9;                            //
    LowerKnee = TableVitesse[M1];               // get lower knee
    if (Modulo == 0)                            //
      UpperKnee = LowerKnee;                    //
    else                                        //
      UpperKnee = TableVitesse[M1 + 1];         // get upper knee
    Difference = UpperKnee - LowerKnee;         // difference between knee values
    PWM.Drive.Derived = (LowerKnee + (Difference * Modulo) / 10) ; // add interpolation to lower speed table value and save as PWM value
    return;
  }
  //Serial.println(vStart);
  if (PWM.Drive.Current < 64)
    PWM.Drive.Derived = map(PWM.Drive.Current, 0, 63, vStart, vMid);   // Calcul entre V Star. et V Mid.
  else
    PWM.Drive.Derived = map(PWM.Drive.Current, 64, 126, vMid, vHigh);  // Calcul entre V Mid. et V High.
}
//------------------------------- BEMF en développement, ne fonctionne pas ! ------------------------------------------------------------------

void ManageBackEMF() {                          //  Derived -> Back EMF -> Actual
  float Err, Sum,  fADCval, fDemand;
  byte drive;
  // only do back EMF if all the following apply
  //  1. we have a speed demand
  //  2. back emf operation is enabled
  //  3. demand is below cutoff value
  // SetPWMDrive(PWM.Drive.Derived);
  PWM.Drive.Actual = PWM.Drive.Derived;
  return;
  /*
    if (PWM.Drive.Derived == 0) {
      //SetPWMDrive(PWM.Drive.Derived);                      // back emf not allowed, set PWM Thresh directly to demand
      PWM.Drive.Actual = PWM.Drive.Derived;
      return;
    }
    if (!CV54_PWMmode.BEMF || (PWM.Drive.Derived > BEMF.CutOff)) { // bit4, enable Back EMF    PWM.Drive.Derived > Val Cv10 (127)
      //SetPWMDrive(PWM.Drive.Derived);             // back emf not allowed, set PWM Thresh directly to demand
      PWM.Drive.Actual = PWM.Drive.Derived;
      return;
    }

    Frame++;
    if (Frame < 10)
      return;
    Frame = 0;
    ADMUX =  B01100000;                                // DAC A0 BEMF  Left Ajust
    Step2++;
    digitalWrite(MOTOR_PWM_FWD_PIN, LOW);               // Motor OFF
    digitalWrite(MOTOR_PWM_REV_PIN, LOW);
    delayMicroseconds(500);
    //marqueur();
    bitSet(ADCSRA, ADSC);                           // Start Conversion
    //ADCSRA |= (1 << ADSC);                         // Start Conversion
    delayMicroseconds(100);
    // nADC = ADCH;
    // if (Step2 > 17) {                                 // Calcul BEFM après 16 échantillons
    //   Step2 = 0;
    //   ValABCTrack1 = MoyenneTab(0);                  // moyenne 16 échantillons niveaux bas
    //nADC = ValBEMF;

    if (Commande == '4') {                // Graphique sur traceur série
      Serial.print (0);
      Serial.print (",");
      Serial.print (PWM.Drive.Derived);
      Serial.print (",");
      Serial.print (nADC);
      Serial.print (",");
      Serial.println (260);
    }
      fDemand = (float)(PWM.Drive.Derived);
      fADCval = (float)(nADC) * 0.25f;     // then convert into FP, scaling back to nominal 8 bits, keeping remainder
      Err = fDemand - fADCval;             // input error to integrator
      BEMF.Integrator += Err;              // integrate error
      BEMF.Integrator = constrain (BEMF.Integrator, 0, 255);  // limit bounds of integrator
      Err = fDemand * BEMF.Kfr;            // feed forward component
      //                                      Combine integrator, feedforward and feedback components
      Sum = BEMF.Integrator * BEMF.Ki;     // scale integrator
      Sum += Err - fADCval;                // + Feed Forward - BEMF
      Sum *= BEMF.Kp;                      // scale output
      Sum = constrain (Sum, 0, 255);       // limit extents
      drive = (byte)Sum;
      //SetPWMDrive(drive);
      Serial.print (fDemand);  Serial.print ("  ");
      Serial.print (fADCval);  Serial.print ("  ");
      Serial.print (Err);  Serial.print ("  ");
      Serial.println (Sum);
    // }
  */
}
//---------------------------------------------- Setup -------------------------------------------------------------
void setup() {
  Serial.begin(250000);                              // Débit Moniteur / RailCom
  mySerial.begin(9600);                              // Débit MP3
  dcc.attach(2);                                     // Int0 sur Pin 2    Dcc
  Flags.AnalogMode = 1;                              // Analog mode
  Flags.ForceDCCStop = 0;
  Flags.ABCBrake = 0;
  SetCV();
  SetPin();
  SetEeprom();
  SetADC();
  SetPWM();
  SetDFPlayer();                                      // Extern player
  SetRailCom();
  SetOLED();                                           // Display OLED  128 x 32
  if (Flags.AdvAck)                                    // RailCom ON
    attachInterrupt(1, CutOut, FALLING);               // Int1 sur Pin 3    CutOut
  if (Flags.ExtAddr)
    locoCmd.setMyAddress(((cv17 - 192) << 8) | cv18);  // Extended adress
  else
    locoCmd.setMyAddress(cv1);                         // Primary adress
  for ( int i = 0; i < num_active_fpins; i++) {
    cv_value = getCV( 121 + i) ;
    switch ( cv_value ) {
      case 0:                                           // LED on/off
        ftn_queue[i].inuse = 0;
        break;
      case 1:                                           // LED Blink
      case 2:
      case 3:
      case 4:
        {
          ftn_queue[i].inuse = 0;
          ftn_queue[i].current_position = 0;
          ftn_queue[i].start_value = 0;
          ftn_queue[i].increment = 1;
          digitalWrite(fpins[i], 0);                                        // Clignotant_
          if (cv_value == 1) ftn_queue[i].stop_value = int(getCV(112)) ;    // lent
          if (cv_value == 2) ftn_queue[i].stop_value = int(getCV(113)) ;    // moyen
          if (cv_value == 3) ftn_queue[i].stop_value = int(getCV(114)) ;    // rapide
          if (cv_value == 4) ftn_queue[i].stop_value = int(getCV(115)) ;    // effet stomboscopique
        }
        break;
      case 5:                                            // DOUBLE ALTERNATING LED Blink
        {
          ftn_queue[i].inuse = 0;
          ftn_queue[i].current_position = 0;
          ftn_queue[i].start_value = 0;
          ftn_queue[i].increment = 1;
          digitalWrite(fpins[i], 0);
          digitalWrite(fpins[i + 1], 0);
          ftn_queue[i].stop_value = int(getCV(112) * 2);
        }
        break;

      case 7:                                                 //servo
        {
          ftn_queue[i].stop_value = int (getCV(118));
          ftn_queue[i].start_value = int (getCV(117));
          ftn_queue[i].increment = -int (char (getCV(116)));
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

      default:
        break;
    }
  }
  wdt_enable(WDTO_120MS);                                              //  Watchdog enable
}
//---------------------------------------------- Loop  -----------------------------------------------------------------------------
void loop() {
  wdt_reset();                                                        // Watchdog
  if (Serial.available() > 0) {                                       // Réception données du moniteur série
    Flags.AdvAck  = 0;                                                // Déactive RailCom car même canal
    Commandes();                                                      //
  }
  currentTimeDcc = millis();

  if (dcc.input()) {
    Flags.AnalogMode = 0;
    previousTimeDcc = currentTimeDcc;
    switch (dcc.cmdType) {
      case Dcc::MyLocoSpeedCmd :
        Flags.ForceDCCStop = 0;
        Speed--;
        Speed = (locoCmd.speed);
        // PWM.Drive.Desired = (Speed & 0x7f );
        if (Flags.InvSens)                                                // Bit0 Cv29 à 1, inverse sens marche
          newDirection = !(locoCmd.forward);
        else
          newDirection = (locoCmd.forward);
        if (StateShunting)
          PWM.Drive.Desired = ((Speed / 2) & 0x7f );              // Shuting mode   Slow speed
        else
          PWM.Drive.Desired = (Speed & 0x7f );

        //  Serial.print(StateShunting);
        //  Serial.print("  ");
        // Serial.println (PWM.Drive.Desired);
        break;
      case Dcc::ResetCmd : Flags.ForceDCCStop = 1; break;                // Stop
      case Dcc::MyEmergencyStopCmd: Flags.ForceDCCStop = 1; break;       // Stop
      case Dcc::MyLocoF0F4Cmd: Functions(1, locoCmd.F0F4); break;
      case Dcc::MyLocoF5F8Cmd: Functions(2, locoCmd.F5F8); break;
      case Dcc::MyLocoF9F12Cmd: Functions(3, locoCmd.F9F12); break;
      case Dcc::MyLocoF13F20Cmd: Functions(4, locoCmd.F13F20); break;
      case Dcc::MyLocoF21F28Cmd: Functions(5, locoCmd.F21F28); break;
      case Dcc::MyPomCmd : cv_operation(PoM); break;
      case Dcc::SmCmd : cv_operation(SM); break;
      default:
        break;
    }
  }
  delay(2);
  if (Flags.ABCBrake) PWM.Drive.Desired = 0;                          // Brake ABC
  ManageAnalogControl();
  if (Flags.AnalogMode) return;
  SoftwareServo::refresh();
  DetectABC();
  EffectUpdate();
  ManageAcceleration();
  ManageBackEMF();

  if (Flags.ForceDCCStop) EmergencyStop();                        // Emergency stop
  if (PWM.Drive.Current > 0) {                                    // ------ Motor ---------
    if (Direction == 1)    {                                      // Marche avant.
      analogWrite(MOTOR_PWM_REV_PIN, PWM.Drive.Actual);
      digitalWrite(MOTOR_PWM_FWD_PIN, LOW);
    }
    else   {                                                      // Marche arrière.
      digitalWrite(MOTOR_PWM_REV_PIN, LOW);
      analogWrite(MOTOR_PWM_FWD_PIN, PWM.Drive.Actual);
    }
  }
  else {
    digitalWrite(MOTOR_PWM_FWD_PIN, LOW);                          // Motor OFF
    digitalWrite(MOTOR_PWM_REV_PIN, LOW);
  }

  if (CV11timeout == 0)                                            // is packet timeout enabled?
    return;                                                        // no, keep on doing what we do

  DCCTimeout = (currentTimeDcc - previousTimeDcc) / 1000;

  if (DCCTimeout < CV11timeout)
    return;                                                        // no timeout yet, keep looping

  EmergencyStop();                                                 // timed out, emergency stop
}
//------------------------------------------ Emergency Stop --------------------------------------------------------------
void EmergencyStop() {
  PWM.Drive.Desired = 0;
  PWM.Drive.Actual = 0;
}
//------------------------------------------ Effet Update ------------------------------------------------------------------
void EffectUpdate() {
  if ((lastDirection != Direction) || (lastLedStateF0 != NewLedStateF0)) {
    lastDirection = newDirection;   lastLedStateF0 = NewLedStateF0;
    if (NewLedStateF0) {                                          // Invertion  feux avant / arrière
      digitalWrite(LED_PIN_FL, newDirection ? HIGH : LOW);
      digitalWrite(LED_PIN_RL, newDirection ? LOW : HIGH);
    }
    else {
      digitalWrite(LED_PIN_FL, LOW);
      digitalWrite(LED_PIN_RL, LOW);
    }
  }

  if ((RepeatPlay == 1) && (digitalRead(busy_pin) == 1)) {       // Audio Off état haut
    currentTimeDfp = millis();
    if ((currentTimeDfp - previousTimeDfp) > 300) {
      Player_CMD(0x12, 0, SongDesired);                        // MP3 RePlay
      previousTimeDfp = currentTimeDfp;
    }
  }

  for (byte i = 0; i < num_active_fpins; i++) {
    if (ftn_queue[i].inuse == 1)  {
      cv_value = getCV( 121 + i) ;
      switch (cv_value) {
        case 0:                                              // LED ON OFF
          break;
        case 1:                                              // LED Blink
        case 2:
        case 3:
        case 4:
          ftn_queue[i].current_position = ftn_queue[i].current_position + ftn_queue[i].increment;
          if (ftn_queue[i].current_position > ftn_queue[i].stop_value) {
            ftn_queue[i].start_value = ~ftn_queue[i].start_value;
            digitalWrite(fpins[i], ftn_queue[i].start_value);
            ftn_queue[i].current_position = 0;                              // Clignotant_
            if (cv_value == 1) ftn_queue[i].stop_value = int(getCV(112)) ;  // lent
            if (cv_value == 2) ftn_queue[i].stop_value = int(getCV(113)) ;  // moyen
            if (cv_value == 3) ftn_queue[i].stop_value = int(getCV(114)) ;  // rapide
            if (cv_value == 4) ftn_queue[i].stop_value = int(getCV(115)) ;  // effet stomboscopique
          }
          break;
        case 5:                                              // DOUBLE ALTERNATING LED Blink
          ftn_queue[i].current_position = ftn_queue[i].current_position + ftn_queue[i].increment;
          if (ftn_queue[i].current_position > ftn_queue[i].stop_value) {
            ftn_queue[i].start_value = ~ftn_queue[i].start_value;
            digitalWrite(fpins[i], ftn_queue[i].start_value);
            digitalWrite(fpins[i + 1], ~ftn_queue[i].start_value);
            ftn_queue[i].current_position = 0;
            ftn_queue[i].stop_value = int(getCV(112) * 2);
          }
          i++;
          break;
        case 7:                                               // Servo
          {
            if (servo_slow_counter++ > servo_slowdown) {
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

        default:
          break;
      }
    }
  }
}

//------------------------------------------------- Read Write Modify CV -----------------------------------------------------------------
void cv_operation(const uint8_t op_mode) {
  uint8_t ValCv;
  uint8_t NumCv = cvCmd.number ;
  ValCv = EEPROM.read(NumCv);
  switch (cvCmd.operation) {
    case CvAccess::verifyByte :
      if (ValCv == cvCmd.value) {
        if (op_mode == SM)  SendAck();
      }
      break;
    case CvAccess::writeByte :
      ValCv = cvCmd.value;
      EEPROM.write(cvCmd.number, ValCv);
      if (op_mode == SM) SendAck();
      if ((cvCmd.number == 8) && (ValCv == 8))  // Write 8 to CV8 ==> Reset factory
        FactoryDefault();
      Update();
      break;
    case CvAccess::bitManipulation :
      if (cvCmd.writecmd) {
        ValCv = cvCmd.writeBit(ValCv);
        if (op_mode == SM) SendAck();
      }
      else {
        if (cvCmd.verifyBit(ValCv)) {
          if (op_mode == SM) SendAck();     // Impultions
        }
      }
      break;
    //case CvAccess::reserved :             // Read 4 Byte ?
    default:
      break;
  }
}
//----------------------------------------------- Ack CV -------------------------------------------------------------------------
void SendAck(void) {
  wdt_reset();                                                        // Watchdog
  digitalWrite(MOTOR_PWM_FWD_PIN, HIGH);                // Motor ON
  digitalWrite(MOTOR_PWM_REV_PIN, LOW);
  digitalWrite(LED_PIN_FL, HIGH);                       // FL ON
  digitalWrite(LED_PIN_RL, HIGH);                       // RL ON
  delay( 6 );
  digitalWrite(MOTOR_PWM_FWD_PIN, LOW);                 // All OFF
  digitalWrite(MOTOR_PWM_REV_PIN, LOW);
  digitalWrite(LED_PIN_FL, LOW);
  digitalWrite(LED_PIN_RL, LOW);
}
//--------------------------------------------------- Serial Monitor --------------------------------------------------------
void Commandes () {
  int i; int j;
  uint8_t c = Serial.read();                       // Lit le carractère
  switch (c) {
    case '0':
      Commande = 0;                                // 0  Arrête le défilement sur moniteur
      break;
    case '1':                                      // 1: Liste des CV
      for (i = 1; i < 129; i++) {
        wdt_reset();                                 // Watchdog
        Serial.print (F("Cv "));  Serial.print (i);
        Serial.print (F("   Val ")); Serial.print (EEPROM.read (i));
        Serial.print (F("   (Usine ")); Serial.print (EEPROM.read (i + 128)); Serial.println (F(")"));
      }
      break;
    case '2':                                      // 2: Modifier un CV
      Serial.println (F("Num Cv ?"));
      i = entradaSerie (255);                      // Entrer num. Cv à modifier
      Serial.println (F("Val Cv ?"));
      j = entradaSerie (255);                      // Entrer nouvelle valeur
      EEPROM.write(i, j);
      Serial.println (F("Ok modif Cv"));           // Confirmation
      break;
    case '3':                                      // 3: Reset des CV
      Serial.println (F("Reset factory CV, are you sure ?  0 = No    1 = Yes"));
      c = entradaSerie (1);
      if (c == 1) {
        FactoryDefault();
        Serial.println (F("Done"));
      }
      else
        Serial.println (F("Canceled"));
      Serial.println ("");
      break;
    case '4':                                              // Trace BEMF
      Serial.println (F("Trace BEMF"));
      //Commande = c;
      break;
    case '5':                                              // Trace ABC
      Serial.println (F("Rail_droit Rail_gauche Différence"));
      Commande = c;
      break;
    case '6':                                              // Trace Acc / Dec
      Serial.println (F("Desired Current Derived Actual"));
      Commande = c;
      break;
    case '\n':
    case '\r':
      break;
    default:
      Serial.println (F("1: Liste des CV"));
      Serial.println (F("2: Modifier un CV"));
      Serial.println (F("3: Reset factory des CV"));
      Serial.println (F("4: Trace BEFM"));
      Serial.println (F("5: Trace ABC"));
      Serial.println (F("6: Trace vitesse"));
      Serial.println (F("0: Arrêt de la trace"));
      break;

  }
  Serial.println ("");
}
//------------------------------------------------------------------------------------------------------------------------------
int entradaSerie (int maximo) {
  unsigned int  c = 0, valor = 0, n = 0;
  do {
    wdt_reset();
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
//------------------------------------- Marqueur pour analyseur logique --------------------------------------
void marqueur() {
  digitalWrite(RcEnable, 1);
  digitalWrite(RcEnable, 0);
  //digitalWrite(8, 1);
  //digitalWrite(8, 0);
}
