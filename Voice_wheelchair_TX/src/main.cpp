#include <Arduino.h>
#include <AltSoftSerial.h>
#include <radio.h>

AltSoftSerial voiceSerial;
#define batteryPin A7
#define LED 4

#define DEBUG 1

#if DEBUG
#define DEBUG_BEGIN Serial.begin(9600)
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#endif

uint32_t lastTime = 0;

void blinkLED(int times){
  for(int i=0; i<times; i++){
    digitalWrite(LED, HIGH);
    delay(100);
    digitalWrite(LED, LOW);
    delay(100);
  }
}

// put function declarations here:
void initialize()
{
  
  DEBUG_BEGIN;
  // delay(3000);
  DEBUG_PRINTLN("Initializing...");
  delay(100);

  //Initialize Radio Module
  bool s = beginRadio();
  s? DEBUG_PRINTLN("Radio Module Initialized"): DEBUG_PRINTLN("Radio Module Failed to Initialize");
  delay(100);

  // initialize Voice Module
  voiceSerial.begin(9600);

  pinMode(LED, OUTPUT);
  pinMode(batteryPin, INPUT);
  blinkLED(3);
  delay(100);
  DEBUG_PRINTLN("Initialization Complete");

}

char* mapVoiceValue(int16_t value);

void setup()
{
  // put your setup code here, to run once:
  initialize();
  delay(100);
  lastTime = millis();
}

void loop()
{
    
  //recognizing voice commands
    if(voiceSerial.available() > 0){

      byte highByte = voiceSerial.read();
      byte lowByte = voiceSerial.read();
      int16_t voiceValue = (highByte << 8) | lowByte;
      DEBUG_PRINT("Voice Value: ");
      DEBUG_PRINTLN(voiceValue);
      radioSend(mapVoiceValue(voiceValue));
    }

    delay(100);

    int batteryValue = analogRead(batteryPin);
    float battLevel = batteryValue * 0.006588;
    // DEBUG_PRINT("Battery Level: ");
    // DEBUG_PRINTLN(battLevel);


    if(millis() - lastTime > 5000 && batteryValue > 3.3){
      lastTime = millis();
      digitalWrite(LED, HIGH);
      delay(100);
      digitalWrite(LED, LOW);
    }
} 

/*
 {{0x5A, 0x2D}, 2}, //Stop
  {{0x5A, 0x2E}, 2}, //Forward
  {{0x5A, 0x2F}, 2}, //Backward
  {{0x5A, 0x30}, 2}, //KeepForward
  {{0x5A, 0x31}, 2}, //KeepBackward
  {{0x5A, 0x32}, 2}, //KeepLeft
  {{0x5A, 0x33}, 2}, //KeepRight
  {{0x5A, 0x34}, 2}, //Left
  {{0x5A, 0x35}, 2}, //Right
  {{0x5A, 0x36}, 2}, //UTurn
  {{0x5A, 0x37}, 2}, //RotateChair
  {{0x5A, 0x38}, 2}, //SpeedUp
  {{0x5A, 0x39}, 2}, //SlowDown
  {{0x5A, 0x3A}, 2}, //StartCalibratio
  {{0x5A, 0x3B}, 2}, //EndCalibration
  {{0x5A, 0x3C}, 2}, //SelectLeft
  {{0x5A, 0x3D}, 2}, //SelectRight
*/


char* mapVoiceValue(int16_t value){
  String commandValue = String();
  switch (value) {
    case 0x5A2D:
      commandValue = "Stop";
      break;
    case 0x5A2E:
      commandValue = "Forward";
      break;
    case 0x5A2F:
      commandValue = "Backward";
      break;
    case 0x5A30:
      commandValue = "KeepForward";
      break;
    case 0x5A31:
      commandValue = "KeepBackward";
      break;
    case 0x5A32:
      commandValue = "KeepLeft";
      break;
    case 0x5A33:
      commandValue = "KeepRight";
      break;
    case 0x5A34:
      commandValue = "Left";
      break;
    case 0x5A35:
      commandValue = "Right";
      break;
    case 0x5A36:
      commandValue = "UTurn";
      break;
    case 0x5A37:
      commandValue = "RotateChair";
      break;
    case 0x5A38:
      commandValue = "SpeedUp";
      break;
    case 0x5A39:
      commandValue = "SlowDown";
      break;
    case 0x5A3A:  
      commandValue = "StartCalib";
      break;
    case 0x5A3B:
      commandValue = "EndCalib";
      break;
    case 0x5A3C:
      commandValue = "SelLeft";
      break;
    case 0x5A3D:
      commandValue = "SelRight";
      break;
    default:
      commandValue = "Unknown";
      break;
  }
  char* command = new char[commandValue.length() + 1];
  strcpy(command, commandValue.c_str());
  return command;
}

  


