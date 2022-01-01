/*
  Copyright (c) 2021 Historic Electric Ltd
  Permission is hereby granted, free of charge, to any person obtaining
  a copy of this software and associated documentation files (the
  "Software"), to deal in the Software without restriction, including
  without limitation the rights to use, copy, modify, merge, publish,
  distribute, sublicense, and/or sell copies of the Software, and to
  permit persons to whom the Software is furnished to do so, subject to
  the following conditions:
  The above copyright notice and this permission notice shall be included
  in all copies or substantial portions of the Software.


  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.cur
  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
  CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
  TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <Metro.h>
#include <FlexCAN_T4.h>
#include <SwitecX12.h>
#include <SwitecX25.h>
#include <U8g2lib.h>
#include <U8x8lib.h>
#include <Arduino.h>
#include <EEPROMex.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TimeLib.h>
#include <ezButton.h>

long looptime = 0;
long looptimeStart = 0;

//CAN Setup

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2;

#define NUM_TX_MAILBOXES 4
#define NUM_RX_MAILBOXES 4

CAN_message_t msg;

void canRX_555(const CAN_message_t &msg); //Data from VCU
void canRX_558(const CAN_message_t &msg); //Data from VCU
void canRX_560(const CAN_message_t &msg); //Data from VCU

void displayodometer();
void updateodometer();
void updateTrip();
void displayinfo();
void clockDisplay();
void showStats();
void readButtons();
void resetTrip();
void resetEfficney();
void updateEEPROM();

//Metro Timers

Metro timer50_1 = Metro(50);
Metro timer100_1 = Metro(100);
Metro timer500_1 = Metro(500);
Metro timer1000_1 = Metro(1000);
Metro timer5000_1 = Metro(5000);
Metro timer30s = Metro(30000);

time_t RTCTime;

//Oled Setups
#define OLED_RESET -1
U8G2_SH1106_128X64_NONAME_F_HW_I2C display_info(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
Adafruit_SSD1306 display_Speedo(128, 32, &Wire1, OLED_RESET);

//Mini Logo

#define logo_width 126
#define logo_height 64
static const unsigned char miniElogo[] U8X8_PROGMEM = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x2a, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x80, 0xb6, 0xff, 0xfd, 0xff, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x3d, 0xfd, 0xaa, 0xa2, 0x5a, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xa0, 0xea, 0xab, 0xf6, 0x3f, 0x40, 0xbd, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0xf0, 0xbf, 0x2e, 0x01, 0x80, 0x12, 0x60, 0x01, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0xa0, 0x06, 0x00, 0x1c, 0x00, 0x00, 0x82, 0x02, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0xe4, 0xf1, 0xf5, 0xf7, 0x0d, 0x00, 0x00, 0x0d, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x60, 0xff, 0x3f, 0xfc, 0xbf, 0x30, 0x0e, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32, 0xf8, 0xff, 0x3f, 0xfe, 0xff, 0x07, 0x10,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x31, 0xfc, 0xbf, 0x9f, 0xfe, 0xff, 0x0f,
    0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x10, 0xfe, 0xef, 0xbb, 0xfe, 0xff,
    0x0f, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x18, 0xff, 0xff, 0x3f, 0xff,
    0xf7, 0x1f, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x0c, 0xff, 0xfe, 0xbf,
    0xbf, 0x7d, 0xbf, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x84, 0xbf, 0xff,
    0x3f, 0xff, 0xff, 0x3f, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0xc6, 0xf7,
    0x7f, 0x37, 0xf7, 0xff, 0x77, 0x81, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0xc7,
    0xff, 0xef, 0xbf, 0xff, 0xff, 0xfd, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08,
    0xe3, 0xff, 0xff, 0x3f, 0xff, 0xdf, 0xff, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
    0x8d, 0xe3, 0xfd, 0xfb, 0x8f, 0xbe, 0xfb, 0xff, 0x01, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xf4, 0xbf, 0x41, 0x0a, 0x16, 0x90, 0xe4, 0x2d, 0x2d, 0x80, 0x05, 0x00, 0x00, 0x00, 0x00,
    0x40, 0x0b, 0xc4, 0x04, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x80, 0x1a, 0x00, 0x00, 0x00,
    0x00, 0x1c, 0x10, 0x00, 0xe0, 0xb2, 0xaa, 0x82, 0x00, 0x00, 0x00, 0xc0, 0x31, 0x00, 0x00,
    0x00, 0xc0, 0x83, 0x0e, 0x00, 0x89, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x27, 0x00,
    0x00, 0x00, 0x30, 0xe0, 0x01, 0x88, 0x96, 0x01, 0x80, 0x3a, 0x00, 0x00, 0x00, 0x40, 0x65,
    0x00, 0x00, 0x00, 0x4c, 0x3a, 0x00, 0x00, 0xc3, 0x00, 0xe0, 0x6f, 0x00, 0x00, 0x00, 0x40,
    0x41, 0x00, 0x00, 0x00, 0x9e, 0x06, 0x00, 0x00, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x40, 0x00, 0x00, 0x00, 0xa2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
    0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
    0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x01, 0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x06, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x01, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x0a, 0x05, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x55, 0x81, 0x00, 0x00, 0x00, 0x0a, 0xc0, 0x00, 0x28, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x40, 0x00, 0x84, 0x01, 0x00, 0x00, 0x0c, 0x10, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x0c, 0x7e,
    0x20, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0xd0, 0xf4, 0x13, 0x01, 0x00, 0x00, 0x08, 0x42,
    0x80, 0x81, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x10, 0x80, 0x02, 0x00, 0x00, 0x00, 0x18,
    0x22, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x0c, 0x00, 0x00, 0x00,
    0x08, 0x09, 0x00, 0x04, 0x02, 0x00, 0x00, 0x00, 0x02, 0x00, 0x02, 0x00, 0x58, 0x04, 0x00,
    0x00, 0x98, 0x02, 0x04, 0x08, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x81, 0x06, 0xb8, 0x03,
    0x00, 0x00, 0x18, 0x02, 0x59, 0x00, 0x06, 0x00, 0x00, 0x00, 0x02, 0x00, 0x41, 0x11, 0xb0,
    0x03, 0x00, 0x00, 0x28, 0x41, 0x2f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x80, 0xa0, 0x2f,
    0xa0, 0x03, 0x00, 0x00, 0x02, 0xc9, 0xde, 0x20, 0x00, 0x00, 0x00, 0x40, 0x3d, 0x72, 0xe1,
    0x0f, 0x84, 0x02, 0x00, 0x00, 0x20, 0x80, 0x7f, 0x60, 0x58, 0x00, 0x00, 0xb8, 0xc3, 0x6a,
    0x91, 0x5b, 0x7c, 0x00, 0x00, 0x00, 0x9a, 0xa2, 0xb7, 0xe0, 0xa8, 0xff, 0xbf, 0x02, 0x00,
    0x60, 0xf0, 0x7f, 0x08, 0x00, 0x00, 0x00, 0xc0, 0xc2, 0x1f, 0x21, 0xf4, 0xbe, 0x15, 0x00,
    0x3c, 0x71, 0x80, 0x17, 0x08, 0x00, 0x00, 0x00, 0x00, 0x40, 0xfe, 0x20, 0x08, 0x60, 0xf5,
    0xff, 0x83, 0xc0, 0x82, 0x26, 0x08, 0x00, 0x00, 0x00, 0x00, 0x82, 0x49, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0xa0, 0xc2, 0x1c, 0x08, 0x00, 0x00, 0x00, 0x00, 0x04, 0x6d, 0x10, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x29, 0x08,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x08, 0x04,
    0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x10,
    0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00,
    0x60, 0x51, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0xc3, 0x01, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00};

//Button related declerations
const int SHORT_PRESS_TIME = 1000;
ezButton button1(10); //info Display btn
ezButton button2(9);  //Trip Select / Reset btn
const int LONG_PRESS_TIME = 2000;
unsigned long pressedTime = 0;
unsigned long releasedTime = 0;
bool isPressing = false;
bool isPressingBtn2 = false;
bool isLongDetected = false;

//define Vars

const double StepsPerDegree = 3;
const unsigned int MaxMotorRotation = 315;                            // 315 max degrees of movement
const unsigned int MaxMotorSteps = MaxMotorRotation * StepsPerDegree; //MaxMotorRotation * StepsPerDegree;
const double SpeedoDegreesPerMPH = 2.5;
const double SecondsPerHour = 3600.0;
const int FeetPerMile = 5280;
volatile double motorStep;
double MinMotorStep;
float speed = 0.0;
int RPM = 0;
float speedRatio = 0.00826; //8.26mph per 1000rpm
unsigned long distsubtotal;
unsigned long FeetTravelled;

int VCU_Status;
int BMS_Status;
int BMS_SOC;
float BMS_currentact;
float BMS_avgtemp;
float BMS_packvoltage;
float kwInst;
int powerBar;

int chargerHVbatteryVolts; // scale *2
uint8_t avgChargerTemp;
uint8_t avgMotorTemp;
uint8_t active_map;

int numCanpkts = 0;

//setup the Speeedo motor
SwitecX25 Motor(MaxMotorSteps, 6, 5, 4, 3);

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

//Trip and ODO Vars

int disttenthsm = EEPROM.readInt(0);
int distm = EEPROM.readInt(5);
int distenm = EEPROM.readInt(10);
int disthundredm = EEPROM.readInt(15);
int disthousandm = EEPROM.readInt(20);
int disttenthousandm = EEPROM.readInt(25);
int disthundredthousandm = EEPROM.readInt(30);
float totalDistance = EEPROM.readFloat(35);
float trip = EEPROM.readFloat(40);
float lastMiles = EEPROM.readFloat(50);
int display_Speedo_option = EEPROM.readInt(55); //change to make persistent
int display_info_option = EEPROM.readInt(60);
float kwhCount = EEPROM.readFloat(65);              //used to work out efficeny
float milesPerKWH = EEPROM.readFloat(70);           //efficeny
float estimatedRange = EEPROM.readFloat(75);        //guess - o - meter
float efficenyDiatance = EEPROM.readFloat(80);      //stores distance since last efficency reset
float efficenyResetDistance = EEPROM.readFloat(85); //Distance at last reset.
int torqueReuest = 0;

void setup()
{
  display_info.clearBuffer();
  Serial.begin(115200);

  button1.setDebounceTime(50);
  button2.setDebounceTime(50);
  button1.setCountMode(COUNT_RISING);
  button2.setCountMode(COUNT_RISING);

  setSyncProvider(getTeensy3Time);

  //Set pins

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Can Setup

  Can1.begin();
  Can2.begin();

  Can1.setBaudRate(50000);
  Can2.setBaudRate(50000);

  Can1.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
  for (int i = 0; i < NUM_RX_MAILBOXES; i++)
  {
    Can1.setMB((FLEXCAN_MAILBOX)i, RX, STD);
  }
  Can2.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
  for (int i = 0; i < NUM_RX_MAILBOXES; i++)
  {
    Can2.setMB((FLEXCAN_MAILBOX)i, RX, STD);
  }

  for (int i = NUM_RX_MAILBOXES; i < (NUM_TX_MAILBOXES + NUM_RX_MAILBOXES); i++)
  {
    Can1.setMB((FLEXCAN_MAILBOX)i, TX, STD);
    Can2.setMB((FLEXCAN_MAILBOX)i, TX, STD);
  }

  Can1.setMBFilter(REJECT_ALL);
  Can1.enableMBInterrupts();
  Can2.setMBFilter(REJECT_ALL);
  Can2.enableMBInterrupts();

  Can1.setMBFilter(MB0, 0x555);   //Filter messages
  Can1.onReceive(MB0, canRX_555); //Call this function on RX 555
  Can1.setMBFilter(MB1, 0x558);   //Filter messages
  Can1.onReceive(MB1, canRX_558); //Call this function on RX 558
  Can1.setMBFilter(MB1, 0x560);   //Filter messages
  Can1.onReceive(MB1, canRX_560); //Call this function on RX 560

  Can1.mailboxStatus();
  Can2.mailboxStatus();

  //------------OLED Stuff--------------------------

  //display 1 welcome text
  display_Speedo.begin(SSD1306_SWITCHCAPVCC, 0x3c);
  display_Speedo.clearDisplay();
  display_Speedo.setTextSize(2);
  display_Speedo.setTextColor(WHITE);
  display_Speedo.setCursor(20, 10);
  display_Speedo.println("-MINI-E-");
  display_Speedo.display();

  // Display 2 Logo
  display_info.begin();
  display_info.clearBuffer();
  display_info.drawXBM(0, 0, logo_width, logo_height, miniElogo);
  display_info.sendBuffer();

  //----- Needel Swing  -----------------------

  // Motor.zero(); //Initialize stepper at 0 location
  Motor.zero();
  // Motor.setPosition(0);
  Motor.updateBlocking();
  delay(500);
  Motor.setPosition(0); //0MPH
  Motor.updateBlocking();
  delay(500);

  // Display animate

  for (int i = 0; i > -120; i -= 5)
  {
    display_info.clearBuffer();
    display_info.drawXBM(i, 0, logo_width, logo_height, miniElogo);
    display_info.sendBuffer();
  }

  display_info.clearBuffer();
  display_info.sendBuffer();
  display_info.setFont(u8g2_font_fub11_tf);
  display_info.drawStr(0, 20, "Connecting...");
  display_info.sendBuffer();
}
//-------------************** LOOP START ****************-------------//
void loop()
{
  looptimeStart = millis();
  display_info.clearBuffer();
  display_Speedo.clearDisplay();
  Motor.updateBlocking();

  button1.loop();
  button2.loop();

  Can1.events();
  Can2.events();
  readButtons();
  switch (VCU_Status)

  {
  case 1: //VCU Boot nothing to do here...
    break;

  case 2: //Ready

    display_info.setFont(u8g2_font_8x13B_tf);
    display_info.drawStr(0, 15, "Press Brake and ");
    display_info.drawStr(0, 30, "Key to Start");

    break;

  case 3: //drive N

    display_info.setFont(u8g2_font_fub30_tf);
    display_info.drawStr(93, 44, "N");
    display_info.drawFrame(90, 8, 35, 40);

    break;

  case 4: //drive F
    display_info.setFont(u8g2_font_fub30_tf);
    display_info.drawStr(96, 44, "F");
    display_info.drawFrame(90, 8, 35, 40);
    // Draw Active Map
    switch (active_map)
    {
    case 1:
      display_info.setFont(u8g2_font_fub11_tf);
      display_info.drawStr(74, 22, "N");
      display_info.drawFrame(72, 8, 16, 16);

      break;

    case 2:
      display_info.setFont(u8g2_font_fub11_tf);
      display_info.drawStr(74, 22, "E");
      display_info.drawFrame(72, 8, 16, 16);

      break;

    case 3:
      display_info.setFont(u8g2_font_fub11_tf);
      display_info.drawStr(74, 22, "S");
      display_info.drawFrame(72, 8, 16, 16);

      break;
    }

    break;

  case 5: // Drive Rev

    display_info.setFont(u8g2_font_fub30_tf);
    display_info.drawStr(94, 44, "R");
    display_info.drawFrame(90, 8, 35, 40);

    break;

  case 6: // Charging

    display_info.setFont(u8g2_font_fub30_tf);
    display_info.drawStr(94, 44, "C");
    display_info.drawFrame(90, 8, 35, 40);

    break;

  case 7: //Error!
    display_info.setFont(u8g2_font_fub30_tf);
    display_info.drawStr(91, 44, "E!");
    display_info.drawFrame(90, 8, 38, 40);

    break;
  }

  if (timer100_1.check() == 1) //events to run every 100ms
  {
    //Catch outlier values

    if (RPM > 15000)
    { //Catch outliers
      RPM = 0;
    }
    if (RPM < 0)
    { //RPM is negative in Rev - we can't go Negative
      RPM = 0;
    }

    speed = RPM * speedRatio;
    motorStep = (((speed)*SpeedoDegreesPerMPH) * StepsPerDegree);

    Motor.setPosition(motorStep);

    clockDisplay();
    if (VCU_Status != 2 && VCU_Status != 1)
    {
      displayinfo();
    }
    display_info.sendBuffer();
  }

  if (timer1000_1.check() == 1) //events to run every 1s

  { // Calculate distance based on speed and add it to the running total
    FeetTravelled = ((speed * FeetPerMile) / SecondsPerHour);
    distsubtotal += FeetTravelled;
    // Calcuate effiency
    if (VCU_Status != 6)
    {
      kwhCount += (kwInst / 3600.00);
      efficenyDiatance = totalDistance - efficenyResetDistance;
      milesPerKWH = (efficenyDiatance / kwhCount);
    }
    updateodometer();
    updateTrip();
    displayodometer();

    display_Speedo.display();
  }

  if (timer5000_1.check() == 1)
  {
    showStats();
  }

  if (timer30s.check() == 1)
  {
    updateEEPROM();
    estimatedRange = (milesPerKWH * 12.0) * (BMS_SOC / 100.0);
  }

  looptime = millis() - looptimeStart;
}
//-------------************** LOOP END ****************-------------//

void canRX_555(const CAN_message_t &msg) //Data from VCU
{

  RPM = (msg.buf[0] * 255 + msg.buf[1]);
  VCU_Status = msg.buf[2];
  BMS_Status = msg.buf[3];
  BMS_SOC = msg.buf[4];
  chargerHVbatteryVolts = msg.buf[5] * 2;
  avgChargerTemp = msg.buf[6];
  avgMotorTemp = msg.buf[7];
  numCanpkts++;
}

void canRX_558(const CAN_message_t &msg) //Data from VCU
{

  int amps = ((msg.buf[1] * 256) + (msg.buf[0]));
  BMS_currentact = ((amps - 3000) / 10);
  BMS_avgtemp = (((msg.buf[3] / 10.0) * 256) + (msg.buf[2] / 10.0));
  BMS_packvoltage = (((msg.buf[5] * 256) / 100.0) + (msg.buf[4] / 100.0));
  torqueReuest = ((((msg.buf[6] * 256) + msg.buf[7]) - 10000) / 10);

  kwInst = ((BMS_currentact * -1.0) * BMS_packvoltage) / 1000.0;
}

void canRX_560(const CAN_message_t &msg) //Data from VCU
{
  active_map = (msg.buf[1]);
}
void displayodometer()
{

  switch (display_Speedo_option)
  {
  case 0:
  {

    display_Speedo.setTextSize(3);
    display_Speedo.setTextColor(WHITE);

    display_Speedo.setCursor(5, 10);
    display_Speedo.print(disttenthousandm);

    display_Speedo.setCursor(25, 10);
    display_Speedo.print(disthousandm);

    display_Speedo.setCursor(45, 10);
    display_Speedo.print(disthundredm);

    display_Speedo.setCursor(65, 10);
    display_Speedo.print(distenm);

    display_Speedo.setCursor(85, 10);
    display_Speedo.print(distm);

    display_Speedo.drawLine(104, 0, 104, 32, WHITE);
    display_Speedo.setCursor(110, 10);
    display_Speedo.print(disttenthsm);

    break;
  }
  case 1:
  {

    display_Speedo.setTextSize(3);
    display_Speedo.setTextColor(WHITE);
    display_Speedo.setCursor(0, 10);
    display_Speedo.print("T:");
    display_Speedo.setCursor(35, 10);
    display_Speedo.print(trip, 1);

    break;
  }

  case 2:
  {
    display_Speedo.setTextSize(3);
    display_Speedo.setCursor(0, 10);
    display_Speedo.print(speed, 1);
    display_Speedo.setCursor(74, 10);
    display_Speedo.print("mph");

    break;
  }

  case 3:
  {

    display_Speedo.setTextSize(2);
    display_Speedo.setTextColor(WHITE);
    display_Speedo.setCursor(0, 10);
    display_Speedo.print("RPM:");
    display_Speedo.setCursor(50, 10);
    display_Speedo.print(RPM);
    break;
  }
  }
}

void updateodometer()
{

  while (distsubtotal >= 528)
  {
    ++disttenthsm;
    distsubtotal = 0;
  }

  if (disttenthsm > 9)
  {
    ++distm;
    disttenthsm = 0;
  }

  if (distm > 9)
  {
    ++distenm;
    distm = 0;
  }

  if (distenm > 9)
  {
    ++disthundredm;
    distenm = 0;
  }

  if (disthundredm > 9)
  {
    ++disthousandm;
    disthundredm = 0;
  }

  if (disthousandm > 9)
  {
    ++disttenthousandm;
    disthousandm = 0;
  }

  if (disttenthousandm > 9)
  {
    ++disthundredthousandm;
    disttenthousandm = 0;
  }
  // Add together the parts for a single total value, usefull for energy calculations etc
  totalDistance = (disthundredthousandm * 100000) + (disttenthousandm * 10000) + (disthousandm * 1000) + (disthundredm * 100) + (distenm * 10) + distm +
                  ((float)disttenthsm / 10);
}

//Update Trip
void updateTrip()
{
  trip = totalDistance - lastMiles;
}

void clockDisplay()
{
  char sHour[8];
  if (hour() < 10)
  {
    __itoa(hour(), sHour, 10);
    String sHr = String(sHour);
    sHr = "0" + sHr;
    sHr.toCharArray(sHour, 4);
  }
  else
  {
    __itoa(hour(), sHour, 10);
  }
  char sMinute[8];
  if (minute() < 10)
  {
    __itoa(minute(), sMinute, 10);
    String sMin = String(sMinute);
    sMin = "0" + sMin;
    sMin.toCharArray(sMinute, 4);
  }
  else
  {
    __itoa(minute(), sMinute, 10);
  }
  char sSecond[10];
  if (second() < 10)
  {
    __itoa(second(), sSecond, 10);
    String sSec = String(sSecond);
    sSec = "0" + sSec;
    sSec.toCharArray(sSecond, 4);
  }
  else
  {
    __itoa(second(), sSecond, 10);
  }

  display_info.setFont(u8g2_font_fub11_tf);
  display_info.drawStr(0, 64, sHour);
  display_info.drawStr(20, 64, ":");
  display_info.drawStr(24, 64, sMinute);
  display_info.drawStr(44, 64, ":");
  display_info.drawStr(48, 64, sSecond);

  // Display SOC

  display_info.drawRFrame(72, 55, 6, 9, 1);
  display_info.drawBox(74, 54, 2, 1);
  display_info.drawRBox(73, 59, 4, 5, 1);
  char sSOC[8];
  __itoa(BMS_SOC, sSOC, 10);
  display_info.drawStr(80, 64, sSOC);
  display_info.drawStr(110, 64, "%");
}

//Additional info Display

void displayinfo()
{

  switch (display_info_option)
  {
  case 7:
  {
    display_info.setFont(u8g2_font_fub11_tf);
    char sTrq[8];
    __itoa(torqueReuest, sTrq, 10);
    display_info.drawStr(0, 15, sTrq);
    display_info.drawStr(0, 30, "Trq");
    break;
  }
  case 6:
  {
    display_info.setFont(u8g2_font_fub11_tf);
    char sCRG_TMP[8];
    __itoa(avgChargerTemp, sCRG_TMP, 10);
    display_info.drawStr(0, 15, sCRG_TMP);

    display_info.drawStr(0, 30, "Crg Tmp");
    break;
  }
  case 5:
  {
    display_info.setFont(u8g2_font_fub11_tf);
    char sMTR_TMP[8];
    __itoa(avgMotorTemp, sMTR_TMP, 10);
    display_info.drawStr(0, 15, sMTR_TMP);
    display_info.drawStr(0, 30, "Mtr Tmp");
    break;
  }
  case 4:
  {
    display_info.setFont(u8g2_font_fub11_tf);
    char sBMS_HV[8];
    dtostrf(BMS_packvoltage, 6, 2, sBMS_HV);
    display_info.drawStr(0, 15, sBMS_HV);
    display_info.drawStr(0, 30, "BMS HV");
    break;
  }

  case 3:
  {
    display_info.setFont(u8g2_font_fub11_tf);
    char sBMS_AMP[8];
    dtostrf(BMS_currentact, 6, 2, sBMS_AMP);
    display_info.drawStr(0, 15, sBMS_AMP);
    display_info.drawStr(0, 30, "BMS A");
    break;
  }
  case 2:
  {
    display_info.setFont(u8g2_font_fub11_tf);
    char sKWInst[8];
    dtostrf(kwInst, 6, 2, sKWInst);
    display_info.drawStr(0, 15, sKWInst);
    display_info.drawStr(0, 30, "KW inst");
    break;
  }
  case 1:
  {
    display_info.setFont(u8g2_font_fub11_tf);
    char sMilesPerKw[8];
    dtostrf(milesPerKWH, 6, 2, sMilesPerKw);
    display_info.drawStr(0, 15, sMilesPerKw);
    display_info.drawStr(0, 30, "m/KWH");
    break;
  }
  case 0:
  {
    display_info.setFont(u8g2_font_fub11_tf);
    char sErange[8];
    dtostrf(estimatedRange, 6, 2, sErange);
    display_info.drawStr(0, 15, sErange);
    display_info.drawStr(0, 30, "Range");
    break;
  }
  }
  //Power Bar..
  display_info.drawRFrame(1, 36, 84, 14, 1);     //Outline
  display_info.drawHVLine(24, 34, 16, 1);        //zero line
  powerBar = map(kwInst, -20, 50, 0, 84);        //Map -20 to 50Kw
  display_info.drawRBox(1, 36, powerBar, 14, 1); // Bar
}

void showStats()
{
  Serial.println("");
  Serial.print(hour());
  Serial.print(":");
  Serial.print(minute());
  Serial.print(":");
  Serial.print(second());
  Serial.print("  | Looptime: ");
  Serial.println(looptime);
  Serial.print("VCU Status: ");
  Serial.print(VCU_Status);
  Serial.print(" BMS Status: ");
  Serial.print(BMS_Status);
  Serial.print(" Active Map:");
  Serial.print(active_map);
  Serial.print(" BMS SOC: ");
  Serial.print(BMS_SOC);
  Serial.print(" BMS Curr ACT: ");
  Serial.print(BMS_currentact);
  Serial.print(" BMS Pack Voltage: ");
  Serial.print(BMS_packvoltage);
  Serial.print("Charger HV Volts: ");
  Serial.print(chargerHVbatteryVolts);
  Serial.print(" Charger Temp: ");
  Serial.print(avgChargerTemp);
  Serial.print(" Motor Temp: ");
  Serial.println(avgMotorTemp);
  Serial.print("Speed MPH: ");
  Serial.print(speed);
  Serial.print(" RPM: ");
  Serial.print(RPM);
  Serial.print(" Motor Step: ");
  Serial.print(motorStep);
  Serial.print(" Number Of can Pkts: ");
  Serial.println(numCanpkts);
}

void readButtons()
{

  //Check button 1

  if (button1.isPressed())
  {
    pressedTime = millis();
    isPressing = true;
    isLongDetected = false;
  }

  if (button1.isReleased())
  {
    isPressing = false;
    releasedTime = millis();

    long pressDuration = releasedTime - pressedTime;

    if (pressDuration < SHORT_PRESS_TIME)
    {
      display_Speedo_option++;
      if (display_Speedo_option > 3) // loopround if on the last screen
      {

        display_Speedo_option = 0;
      }
    }
  }
  if (isPressing == true && isLongDetected == false)
  {
    long pressDuration = millis() - pressedTime;

    if (pressDuration > LONG_PRESS_TIME)
    {
      isLongDetected = true;
      resetTrip();
    }
  }

  //Check button 2

  if (button2.isPressed())
  {
    pressedTime = millis();
    isPressingBtn2 = true;
    isLongDetected = false;
  }

  if (button2.isReleased())
  {
    isPressingBtn2 = false;
    releasedTime = millis();

    long pressDuration = releasedTime - pressedTime;

    if (pressDuration < SHORT_PRESS_TIME)
    {
      display_info_option++;
      if (display_info_option > 7) // loopround if on the last screen
      {

        display_info_option = 0;
      }
    }
  }
  if (isPressingBtn2 == true && isLongDetected == false)
  {
    long pressDuration = millis() - pressedTime;

    if (pressDuration > LONG_PRESS_TIME)
    {
      isLongDetected = true;
      if (display_info_option == 1)
      {
        resetEfficney();
      }
    }
  }
}

void resetTrip()
{
  trip = 0;
  lastMiles = totalDistance;
  display_Speedo.clearDisplay();
  display_Speedo.setCursor(0, 0);
  display_Speedo.print(trip);
  lastMiles = totalDistance;
  display_Speedo_option = 1;
  updateEEPROM();
}

void resetEfficney()
{
  milesPerKWH = 0;
  estimatedRange = 0;
  efficenyDiatance = 0;
  efficenyResetDistance = totalDistance;
  kwhCount = 0;
}

void updateEEPROM()
{
  Serial.println("writing to EEPROM");
  EEPROM.updateInt(0, disttenthsm);
  EEPROM.updateInt(5, distm);
  EEPROM.updateInt(10, distenm);
  EEPROM.updateInt(15, disthundredm);
  EEPROM.updateInt(20, disthousandm);
  EEPROM.updateInt(25, disttenthousandm);
  EEPROM.updateInt(30, disthundredthousandm);
  EEPROM.updateFloat(35, totalDistance);
  EEPROM.updateFloat(40, trip);
  EEPROM.updateFloat(50, lastMiles);
  EEPROM.updateInt(55, display_Speedo_option);
  EEPROM.updateInt(60, display_info_option);
  EEPROM.updateFloat(65, kwhCount);
  EEPROM.updateFloat(70, milesPerKWH);
  EEPROM.updateFloat(75, estimatedRange);
  EEPROM.updateFloat(80, efficenyDiatance);
  EEPROM.updateFloat(85, efficenyResetDistance);

  Serial.println("Finsihed writing to EEPROM");
}
