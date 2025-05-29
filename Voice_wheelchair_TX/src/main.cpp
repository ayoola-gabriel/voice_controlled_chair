#include <Arduino.h>
#include <AltSoftSerial.h>
#include <radio.h>

AltSoftSerial voiceSerial;

// pin definitions
#define batteryPin A7
#define LED 4

#define stopButton 2
#define forwardButton 5
#define backwardButton 3
#define leftButton 7
#define rightButton 6

#define DEBUG 1

#if DEBUG
#define DEBUG_BEGIN Serial.begin(9600)
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#endif

uint32_t lastTime = 0;
bool buttonCommand = false;

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

  // Initialize pins
  pinMode(LED, OUTPUT);
  pinMode(batteryPin, INPUT);
  pinMode(stopButton, INPUT_PULLUP);
  pinMode(forwardButton, INPUT_PULLUP);
  pinMode(backwardButton, INPUT_PULLUP);  
  pinMode(leftButton, INPUT_PULLUP);
  pinMode(rightButton, INPUT_PULLUP);
  DEBUG_PRINTLN("Pins Initialized");
  delay(100);
  
  // blink LED to indicate initialization
  blinkLED(3);
  delay(100);

  PCICR |= (1 << PCIE2); // Enable pin change interrupt for digital pins 2-7
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT19) | (1 << PCINT23) | (1 << PCINT21) | (1 << PCINT22); // Enable pin change interrupt for stop, forward, backward, left, right buttons
  DEBUG_PRINTLN("Initialization Complete");

}

char* mapVoiceValue(int16_t value);

void setup()
{
  // put your setup code here, to run once:
  initialize();
  delay(500);
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


// Check battery level and blink LED if needed
    if(millis() - lastTime > 10000 && battLevel > 3.3){
      lastTime = millis();
      digitalWrite(LED, HIGH);
      delay(100);
      digitalWrite(LED, LOW);
    }
} 

// Interrupt Service Routine for pin change interrupts
ISR(PCINT2_vect) {
  // Check if the stop button is pressed
  if (digitalRead(stopButton) == LOW) {
    while (digitalRead(stopButton) == LOW); // wait for button release
    radioSend("Stop");
    buttonCommand = true;
  }

  // Check if the forward button is pressed
  else if (digitalRead(forwardButton) == LOW) {
    while (digitalRead(forwardButton) == LOW); // wait for button release
    radioSend("KeepForward");
    buttonCommand = true;
  }

  // Check if the backward button is pressed
  else if (digitalRead(backwardButton) == LOW) {
    while (digitalRead(backwardButton) == LOW); // wait for button release
    radioSend("KeepBackward");
    buttonCommand = true;
  }

  // Check if the left button is pressed
  else if (digitalRead(leftButton) == LOW) {
    while (digitalRead(leftButton) == LOW); // wait for button release
    buttonCommand = true;
    radioSend("Left");
  }

  // Check if the right button is pressed
  else if (digitalRead(rightButton) == LOW) {
    while (digitalRead(rightButton) == LOW); // wait for button release
    buttonCommand = true;
    radioSend("Right");
} else {
  buttonCommand = false; // No button pressed
  return;
}
}

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

  


