//
//#include <Adafruit_MAX31865.h>
//#include <virtuabotixRTC.h>
//
//// Use software SPI: CS, DI, DO, CLK
//Adafruit_MAX31865 thermo = Adafruit_MAX31865(2, 3, 4, 5);
//
//// use hardware SPI, just pass in the CS pin
////Adafruit_MAX31865 thermo = Adafruit_MAX31865(10);
//
//// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
//#define RREF      4300.0
//// The 'nominal' 0-degrees-C resistance of the sensor
//// 100.0 for PT100, 1000.0 for PT1000
//#define RNOMINAL  1000.0
#include <FastLED.h>
#include <usb_serial.h>
#include <vector>
#include <cmath>
#include <virtuabotixRTC.h>
#include <Adafruit_MAX31865.h>
#include <Adafruit_INA260.h>

Adafruit_MAX31865 thermo = Adafruit_MAX31865(3, 4, 5, 6);
Adafruit_MAX31865 thermo6 = Adafruit_MAX31865(7,4,5,6);
Adafruit_MAX31865 thermo5 = Adafruit_MAX31865(8,4,5,6);
Adafruit_MAX31865 thermo3 = Adafruit_MAX31865(9,4,5,6);

Adafruit_INA260 ina260 = Adafruit_INA260();
virtuabotixRTC myRTC(21, 22, 23);

#define RREF      4300.0
#define RNOMINAL  1000.0 
#define Right_Motor_dir 9
#define Right_Motor_speed 8
#define Left_Motor_dir 38
#define Left_Motor_speed 37

#define EncoderLeftA 5
#define EncoderLeftB 6
#define EncoderRightA 23
#define EncoderRightB 22

#define PassiveModeIndicator 0
#define FullModeIndicator 1

#define led 13

#define rotationDelay 100 //100 //453 //18573
#define ID_DISTANCE 19
#define ID_ANGLE 20
#define ID_OI_MODE 35
#define ID_NUM_STREAM_PACKETS 38
#define ID_VEL 39
#define ID_RADIUS 40
#define ID_RIGHT_VEL 41
#define ID_LEFT_VEL 42
#define ID_LEFT_ENC 43
#define ID_RIGHT_ENC 44
#define ID_STASIS 58TTL level
#define BRIGHTNESS  100
#define LED_PIN     5
#define NUM_LEDS    10

CRGB leds[NUM_LEDS];
unsigned long lastTime = 0;
int i = 0;
unsigned long timerDelay = 5000;
String data;
String data2;
String data3;
String data4;
String data5;
String data6;

String apiKey  = "4xLL5J5X4z31BHrnTe";
String apiKey2 = "t8mhrIUGhA4oEEF3oB";
String apiKey3 = "Dn0SQBghlZ9dAqfmvP";
String apiKey4 = "XaHtyFFGWoyhvSxMIV";
String apiKey5 = "tr37pJArkCXxBRlSLv";
String apiKey6 = "CpsZ507rpyjFaiEfhp";
int d;
int mo;
int y;
int h;
int mi;
int s;
String currentdate;
String currenttime;
String se;
String minu;
String hou;
String da;
String mon;
const int Mode_OFF = 0;
const int Mode_PASSIVE = 1;
const int Mode_SAFE = 2;
const int Mode_FULL = 3;

volatile int systemMode = Mode_OFF; //Initialize in off mode.

bool Passive_Mode_On = false; // OC_START = 128
bool Power_Off = false; // OC_POWER = 133
bool Safe_Mode_On = false; //OC_SAFE =131
bool Full_Mode_On = false; //OC_FULL =132
bool Restart_Cmd = false; //OC_RESET =7
bool Stop_Cmd = false; //OC_STOP =173
bool Baud_Cmd= false; // OC_BAUD = 129
bool Control_Cmd= false; // OC_CONTROL = 130

bool Drive_Cmd= false; // OC_DRIVE = 137
bool Drive_Direct= false; // OC_DRIVE_DIRECT = 145
bool Drive_PWM= false; // OC_DRIVE_PWM = 146

bool Sensor_Cmd = false; // OC_SENSORS= 142
bool Query_list_Cmd = false; // OC_QUERY_LIST=149
bool Stream_Cmd = false; // OC_STREAM = 148
bool Toggle_stream_Cmd = false; // OC_TOGGLE_STREAM = 150

bool BadRequest = false;

volatile int16_t count_LeftA = 0;
volatile int16_t count_RightA = 0;

void setup() {
  
  FastLED.setBrightness(  BRIGHTNESS );
  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS); 
  Serial.begin(9600);
  Serial4.begin(9600);
  thermo.begin(MAX31865_4WIRE);  // set to 2WIRE or 4WIRE as necessary
  thermo6.begin(MAX31865_2WIRE);
  thermo5.begin(MAX31865_2WIRE); 
  thermo3.begin(MAX31865_2WIRE); 
  pinMode(17, INPUT_PULLUP);
  pinMode(37, INPUT_PULLUP);
  pinMode(2 , OUTPUT);
    //Set motor output pins
  pinMode(Left_Motor_dir, OUTPUT);
  pinMode(Right_Motor_dir, OUTPUT);

  //Set LED pin(s)
  pinMode(led, OUTPUT);

  //Set encoder inputs with A channels set with internal pullups
  pinMode(EncoderLeftA, INPUT_PULLUP);
  pinMode(EncoderLeftB, INPUT);
  pinMode(EncoderRightA, INPUT_PULLUP);
  pinMode(EncoderRightB, INPUT);

    //set encoder A channel with interrupts
  attachInterrupt(digitalPinToInterrupt(EncoderLeftA), encoder_leftA_fired, RISING);
  attachInterrupt(digitalPinToInterrupt(EncoderRightA), encoder_rightA_fired, RISING);

  //With pins setup, light onboard LED at low current.
  POWERLED(255);
  delay(1000);
  POWERLED(10);
  delay(500);
  POWERLED(0);
  delay(500);
  POWERLED(10);
  Serial.flush();
  Serial.clear();
  
  analogWrite(PassiveModeIndicator, 0);
  analogWrite(FullModeIndicator, 0);
  
  if(systemMode != Mode_OFF) systemMode = Mode_OFF; 
  
  
  //myRTC.setDS1302Time(20, 40, 15, 4, 14, 10, 2021); //Here you write your actual time/date as shown above 
                                                      //Upload your code and then comment this line and re-upload 
                                                      //Set UTC time time in 24 hr format 

}

void DeclarePassiveMode(){
  analogWrite(PassiveModeIndicator, 100);
  analogWrite(FullModeIndicator, 0);
}

void DeclareFullMode(){
  analogWrite(FullModeIndicator, 100);
  analogWrite(PassiveModeIndicator, 0);
}

void DeclareSafeMode(){
  analogWrite(FullModeIndicator, 0);
  analogWrite(PassiveModeIndicator, 0);
}



void loop() {

if (digitalRead(17) == HIGH){
    Doorsensor();
}
 if (digitalRead(37) == HIGH){
    Eswitch();
}


  myRTC.updateTime();
  d    = myRTC.dayofmonth;
  mo   = myRTC.month;
  y    = myRTC.year;
  h    = myRTC.hours;
  mi   = myRTC.minutes;
  s    = myRTC.seconds; 
  da   = String(d);
  mon  = String(mo);
  hou  = String(h);
  minu = String(mi);
  se   = String(s);
  
  if(se.length() == 1){
       se = "0"+se;
     }
  if(minu.length() == 1){
       minu = "0"+minu;
     }
  if(hou.length() == 1){
       hou = "0"+hou;
     }
  if(da.length() == 1){
       da = "0"+da;
     }
  if(mon.length() == 1){
       mon = "0"+mon;
     }  
                    
  currentdate = String(y)+String(mon)+String(da);
  currenttime = String(hou)+String(minu)+String(se);
  //Serial.println(String(se));
  
  uint16_t rtd = thermo.readRTD();
  float ratio = rtd;
  ratio /= 32768;
  
  uint16_t rtd6 = thermo6.readRTD();
  float ratio6 = rtd6;
  ratio6 /= 32768;
  
  uint16_t rtd5 = thermo5.readRTD();
  float ratio5 = rtd5;
  ratio5 /= 32768;
  
  uint16_t rtd3 = thermo3.readRTD();
  float ratio3 = rtd3;
  ratio3 /= 32768;
    
    
    if ((millis() - lastTime) > timerDelay) {

        int temp= (thermo.temperature(RNOMINAL,RREF));
        int temp6= (thermo6.temperature(RNOMINAL,RREF));
        int temp5= (thermo5.temperature(RNOMINAL,RREF));
        int temp3= (thermo3.temperature(RNOMINAL,RREF));
        
                 
        if (i == 1){  
        data= String(currentdate)+String(currenttime)+"000000V"+String(temp)+","+String(temp)+"I"+String(apiKey);   
        Serial4.println(data);
        Serial4.flush();
        Serial.println(data);
        if (temp > 25){
          digitalWrite(2 , HIGH);
          
        }
        else{
          digitalWrite(2 , LOW);
        }
        }
        
        if (i == 2){
        data2= String(currentdate)+String(currenttime)+"000000V"+String(temp6)+","+String(temp6)+"I"+String(apiKey2);
        Serial4.println(data2);
        Serial4.flush();
        Serial.println(data2);
        }
        if (i == 3){
        data3= String(currentdate)+String(currenttime)+"000000V"+String(temp5)+","+String(temp5)+"I"+String(apiKey3);
        Serial4.println(data3);
        Serial4.flush();
        Serial.println(data3);
        }
        if (i == 4){
        data4= String(currentdate)+String(currenttime)+"000000V"+String(temp3)+","+String(temp3)+"I"+String(apiKey4);
        Serial4.println(data4);
        Serial4.flush();
        Serial.println(data4);
        }
        if (i == 5){
        data5= String(currentdate)+String(currenttime)+"000000V"+String(temp)+","+String(temp)+"I"+String(apiKey5);
        Serial4.println(data5);
        Serial4.flush();
        Serial.println(data5);
        }
        if (i == 6){
        data6= String(currentdate)+String(currenttime)+"000000V"+String(temp)+","+String(temp)+"I"+String(apiKey6);
        Serial4.println(data6);
        Serial4.flush();
        Serial.println(data6);
         i=0;
        }
       
      lastTime = millis();
      i++;
    }
int   current= ina260.readCurrent();
int voltage  = ina260.readBusVoltage();
   Serial.println(current);
   Serial.println(voltage);

       switch(systemMode){
    case Mode_OFF: //OFF MODE
      if(Serial.available()){
        int stateByte = Serial.read();
        if(stateByte == 128) {
           Passive_Mode_On = true;
           Safe_Mode_On = false;
           Full_Mode_On = false;
           POWERLED(255); 
          }
        if(stateByte == 131){
           Passive_Mode_On = false;
           Safe_Mode_On = true;
           Full_Mode_On = false;
           POWERLED(0); 
          }
        if(stateByte == 132){
           Passive_Mode_On = false;
           Safe_Mode_On = false;
           Full_Mode_On = true;
           POWERLED(100);
          }
        }
      else{
        POWERLED(10);
      }
      break;
    case Mode_PASSIVE: //PASSIVE MODE
      if(Serial.available()){
        POWERLED(255);
        int firstByte = Serial.read();
        checkFirstByte(firstByte);
        if(!BadRequest && Passive_Mode_On){
          passiveMode();
          }
        if(!BadRequest && !Passive_Mode_On){
          break;
        }
        else {
          BadRequest = false;
          }
        }
      else{
        POWERLED(10);
      }
      break; 
    case Mode_SAFE: //SAFE MODE
      POWERLED(0);
      break;
    case Mode_FULL: //FULL MODE
      if(Serial.available()){
        POWERLED(100);
        int firstByte = Serial.read();
        checkFirstByte(firstByte);
        if(!BadRequest && Full_Mode_On){
          FullMode();
          }
        if(!BadRequest && !Full_Mode_On){
          break;
          }
        else {
          BadRequest = false;
          }
        }
      else{
        POWERLED(10);
      }
      break;
  }
  if(Passive_Mode_On) {
    systemMode = Mode_PASSIVE;
//    DeclarePassiveMode();
  }
  if(Safe_Mode_On) {
    systemMode = Mode_SAFE;
//    DeclareSafeMode();
  }
  if(Full_Mode_On) {
    systemMode = Mode_FULL;
//    DeclareFullMode();
  }
}

void passiveMode(){
  if(Stream_Cmd){
    sendEncoderStream();
//    Stream_Cmd = false;
  }
  if(Drive_Direct){
    sendDriveCmd();
    //sendEncoderStream();
    Drive_Direct =false;
  }
}

void FullMode(){
  if(Drive_Direct){
    sendDriveCmd();
//    sendEncoderStream();
    Drive_Direct =false;
  }
   if(Stream_Cmd){
    sendEncoderStream();
//    Stream_Cmd = false;
  }
}

void POWERLED(int intensity){
  analogWrite(led,intensity);
}

void checkFirstByte(int recievedByte){
    switch(recievedByte){
      case 128: //Passive mode
        Passive_Mode_On = true;
        Full_Mode_On = false;
        Safe_Mode_On = false;
        break;
      case 131: //Safe mode
        Passive_Mode_On = false;
        Full_Mode_On = false;
        Safe_Mode_On = true;
        break;
      case 132: //Full mode
        Passive_Mode_On = false;
        Full_Mode_On = true;
        Safe_Mode_On = false;
        break;
      case 133: //power off
        Power_Off = true;
        RESTART();
        break;
      case 7: //reset command
        Restart_Cmd = true;
        RESTART();
        break;
      case 173: //stop command
        Stop_Cmd = true;
        RESTART();
        break; 
      case 137: //NOT SETUP
        Drive_Cmd = true;
        break;
      case 145:
        Drive_Direct = true;
        break;
      case 146: //NOT SETUP
        Drive_PWM = true;
        break;
      case 142: //NOT SETUP
        Sensor_Cmd = true;
        break;
      case 149: //NOT SETUP
        Query_list_Cmd = true;
        break;
      case 148:
        Stream_Cmd = true;
        break;
      case 150: //NOT SETUP
        Toggle_stream_Cmd = true;
        break;
      default: //Unrecognized command
        BadRequest = true;
        break;
    }
}

void RESTART(){
  POWERLED(100);
  delay(500);
  POWERLED(255);
  delay(500);
  POWERLED(100);
  
  Passive_Mode_On = false; // OC_START = 128
  Power_Off = false; // OC_POWER = 133
  Safe_Mode_On = false; //OC_SAFE =131
  Full_Mode_On = false; //OC_FULL =132
  Restart_Cmd = false; //OC_RESET =7
  Stop_Cmd = false; //OC_STOP =173
  Baud_Cmd= false; // OC_BAUD = 129
  Control_Cmd= false; // OC_CONTROL = 130

  Drive_Cmd= false; // OC_DRIVE = 137
  Drive_Direct= false; // OC_DRIVE_DIRECT = 145
  Drive_PWM= false; // OC_DRIVE_PWM = 146

  Sensor_Cmd = false; // OC_SENSORS= 142
  Query_list_Cmd = false; // OC_QUERY_LIST=149
  Stream_Cmd = false; // OC_STREAM = 148
  Toggle_stream_Cmd = false; // OC_TOGGLE_STREAM = 150

  count_LeftA = 0;
  count_RightA = 0;
  
  systemMode = Mode_OFF; //initalize to OFF mode
  
  Serial.flush();
  Serial.clear();
  delay(500);
  POWERLED(10);
//  DeclareSafeMode();
}

void sendEncoderStream(){
  cli();
  uint8_t headerByte = 19;
  uint8_t numBytesExpected = 6;
  uint8_t LPacketID = 43;
  uint8_t RPacketID = 44; 
  uint8_t LEncoderHigh = static_cast<uint8_t>(count_LeftA >> 8);
  uint8_t LEncoderLow= static_cast<uint8_t>(count_LeftA & 0xff);
  uint8_t REncoderHigh= static_cast<uint8_t>(count_RightA >> 8);
  uint8_t REncoderLow= static_cast<uint8_t>(count_RightA & 0xff);
  uint8_t checksum = 256-(headerByte+numBytesExpected+LPacketID+RPacketID+LEncoderHigh+LEncoderLow+REncoderHigh+REncoderLow);
  Serial.write(headerByte);
  Serial.write(numBytesExpected);
  Serial.write(LPacketID);
  Serial.write(LEncoderHigh);
  Serial.write(LEncoderLow);
  Serial.write(RPacketID);
  Serial.write(REncoderHigh);
  Serial.write(REncoderLow);
  Serial.write(checksum);
  sei();
}

void sendDriveCmd(){
  int count=0;
  int numBytes = 4;
  int buf[numBytes];
  while (count<numBytes) {
    if (Serial.available()) {  // receive up to 4 bytes into "buf"
      buf[count] = Serial.read();
      count++;
    }
  }
  uint8_t RghtHghByt = buf[0];
  uint8_t RghtLwByt = buf[1];
  uint8_t LftHghByt = buf[2];
  uint8_t LftLwByt = buf[3];
  int16_t RDrv = (RghtHghByt << 8) | RghtLwByt;
  int16_t LDrv = (LftHghByt << 8) | LftLwByt;
  int slowDown = 1.;

  driveMotors(vel_PWM_Convert(LDrv/slowDown),vel_PWM_Convert(RDrv/slowDown));
}

void encoder_leftA_fired() {
  //count_LeftA += digitalRead(EncoderLeftA);
  count_LeftA += -1 + 2*digitalRead(EncoderLeftB);
}

void encoder_rightA_fired() {
  //count_RightA += digitalRead(EncoderRightA);
  count_RightA += -1 + 2*digitalRead(EncoderRightB);
}

void Drv_fwrd(int LSpeed,int RSpeed){
  LEDFunc_frwd(100);
  int RightPWMCorrected = RightPWMfromCounts(LeftCountsfromPWM(LSpeed));
  digitalWrite(Right_Motor_dir, LOW);
  digitalWrite(Left_Motor_dir, LOW);
  analogWrite(Left_Motor_speed, LSpeed);
  analogWrite(Right_Motor_speed, RSpeed); //RightPWMCorrected); //removed!
  delay(rotationDelay);
  analogWrite(Left_Motor_speed, 0);
  analogWrite(Right_Motor_speed, 0);
  LEDFunc_Stop(100);
}

void Drv_bkwrd(int LSpeed,int RSpeed){
  LEDFunc_bkwd(100);
  int RightPWMCorrected = RightPWMfromCounts(LeftCountsfromPWM(LSpeed));
  digitalWrite(Right_Motor_dir, HIGH);
  digitalWrite(Left_Motor_dir, HIGH);
  analogWrite(Left_Motor_speed, -LSpeed);
  analogWrite(Right_Motor_speed, -RSpeed);//-RightPWMCorrected);//removed!
  delay(rotationDelay);
  analogWrite(Left_Motor_speed, 0);
  analogWrite(Right_Motor_speed, 0);
  LEDFunc_Stop(100);
}

void Drv_rght(int LSpeed,int RSpeed){
  digitalWrite(Right_Motor_dir, LOW);
  digitalWrite(Left_Motor_dir, HIGH);
  analogWrite(Left_Motor_speed, -LSpeed);
  analogWrite(Right_Motor_speed, +RSpeed);
  delay(rotationDelay);
  analogWrite(Left_Motor_speed, 0);
  analogWrite(Right_Motor_speed, 0);
}

void Drv_lft(int LSpeed,int RSpeed){
  digitalWrite(Right_Motor_dir, HIGH);
  digitalWrite(Left_Motor_dir, LOW);
  analogWrite(Left_Motor_speed, +LSpeed);
  analogWrite(Right_Motor_speed, -RSpeed);
  delay(rotationDelay);
  analogWrite(Left_Motor_speed, 0);
  analogWrite(Right_Motor_speed, 0);
}

void Drv_STOP(){
  LEDFunc_Stop(100);
  digitalWrite(Right_Motor_dir, HIGH);
  digitalWrite(Left_Motor_dir, LOW);
  analogWrite(Left_Motor_speed, 0);
  analogWrite(Right_Motor_speed, 0);
  delay(rotationDelay);
}


bool driveMotors(int LSpeed,int RSpeed){
  if(LSpeed >0 && RSpeed >0) Drv_fwrd(LSpeed,RSpeed);
  if(LSpeed <0 && RSpeed <0) Drv_bkwrd(LSpeed,RSpeed);
  if(LSpeed >0 && RSpeed <0) Drv_lft(LSpeed,RSpeed);
  if(LSpeed <0 && RSpeed >0) Drv_rght(LSpeed,RSpeed);
  if(LSpeed ==0 && RSpeed ==0) Drv_STOP();
  return true;
}

int vel_PWM_Convert(int speed){
//  return int(speed*0.51);
//  return int(speed*0.1683);
   int velocity_conv = int(speed*0.172);
   //Added below
   if(velocity_conv <= 15 && velocity_conv >0) { 
    velocity_conv = 15;
   }
   if(velocity_conv >= -15 && velocity_conv <0) {
    velocity_conv = -15;
   }
   //added above.
   //return int(speed*0.172); //removed
   return velocity_conv;
}

float LeftCountsfromPWM(int x){
  float m =5.346;
  float b =-22.148;
  return m*x+b;
}

int RightCountsfromPWM(float x){
  float m = 5.483;
  float b =-17.956;
  return m*x+b;
}

float LeftPWMfromCounts(int y){
  float m =5.346;
  float b =-22.148;
  return int((y-b)/m);
}

int RightPWMfromCounts(float y){
   float m = 5.483;
  float b =-17.956;
  return int((y-b)/m);
}
void Doorsensor(){
  int sensorVal = digitalRead(17);
  
  if (sensorVal == HIGH) {
    //digitalWrite(13, LOW);
    Serial.println("door is open");
  } else {
    //digitalWrite(13, HIGH);
    Serial.println("door is closed");
  }
}

void Eswitch(){
  int switchVal = digitalRead(37);
  
  if (switchVal == HIGH) {
    //digitalWrite(13, LOW);
    Serial.println("switch is  open");
  } else {
    //digitalWrite(13, HIGH);
    Serial.println("switch is closed");
  }
}

void LEDFunc_frwd(int LEDSelect)
{
   for(int i =0; i<LEDSelect;i++)
    {
      leds[i] = CRGB(0, 255, 0);
      FastLED.show();
      delay(5);
      //FastLED.clear();
    }
}

void LEDFunc_bkwd(int LEDSelect)
{
   for(int i =0; i<LEDSelect;i++)
    {
      leds[i] = CRGB(255, 0, 0);
      FastLED.show();
      delay(5);
      //FastLED.clear();
    }
}
void LEDFunc_Stop(int LEDSelect)
{
   for(int i =0; i<LEDSelect;i++)
    {
      leds[i] = CRGB(0, 0, 0);
      FastLED.show();
      delay(5);
      //FastLED.clear();
    }
}
