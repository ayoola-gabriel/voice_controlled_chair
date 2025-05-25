#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE A1
#define CSN A2
#define IRQ A3

const char* stopCommand = "Stop";
const char* forwardCommand = "Forward";
const char* backwardCommand = "Backward";
const char* keepForwardCommand = "KeepForward";
const char* keepBackwardCommand = "KeepBackward";
const char* keepLeftCommand = "KeepLeft";
const char* keepRightCommand = "KeepRight";
const char* leftCommand = "Left";
const char* rightCommand = "Right";
const char* uTurnCommand = "UTurn";
const char* rotateChairCommand = "RotateChair";
const char* speedUpCommand = "SpeedUp";
const char* slowDownCommand = "SlowDown";
const char* startCalibrationCommand = "StartCalib";
const char* endCalibrationCommand = "EndCalib";
const char* selectLeftCommand = "SelLeft";
const char* selectRightCommand = "SelRight";


// Define the nRF24L01 radio object
RF24 radio(CE, CSN); // CE, CSN pins
const byte address[6] = "CXL5G";

// Define the motor control pins
//PWM pins will connect to SD pin


#define leftMotorPWM 3
#define leftMotorForwad 4
#define leftMotorReverse 5

#define rightMotorForward 7
#define rightMotorReverse 8
#define rightMotorPWM 9

#define DEFAULT_SPEED 245
#define MAX_SPEED 250
#define MIN_SPEED 125

bool calibrationMode = false;
bool calibrateLeftMotor = false;
bool calibrateRightMotor = false;
uint16_t leftMotorCalibrationValue = DEFAULT_SPEED;
uint16_t rightMotorCalibrationValue = DEFAULT_SPEED;
uint8_t calibratedflag = 0;

uint8_t directionBuffer = 0;

// #define LED 2
#define LED 6
#define stopButton 2

#include <EEPROM.h>

// EEPROM addresses for storing values
const int speedAddress = 0;
const int leftMotorSpeedAddress = 1;
const int rightMotorSpeedAddress = 2;
const int calibrationStateAddress = 3;
const int calibrationFactorAddress = 6;
const int lockCounterAddress = 4;
const int doNotLockAddress = 5;

// Default values
uint16_t speedValue = 0;
uint16_t leftSpeedValue = 0;
uint16_t rightSpeedValue = 0;
uint16_t turnSpeed = 127;
uint8_t lockCounter = 0;
uint32_t commandTimeout = 5000;
uint32_t motorTimeout;
uint32_t lockTimer;
uint8_t doNotLockValue = 0;
float calibrationFactor = 0.00;

// Function prototypes
void processCommand(char* command);
bool loadValuesFromEEPROM();
void saveValueToEEPROM(int address, int value);
void stopLogic();
void forwardLogic();
void backwardLogic();
void rightLogic();
void leftLogic();
void clearEEPROM();
void blinkLED(int times);
void unlockAndPasswordMode();

void stopISR(){
  stopLogic();
}

// Setup function
void setup() {
  for(int i=3;i<11;i++){
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
  }

  Serial.begin(9600);
  Serial.println("Starting...");

  pinMode(stopButton, INPUT_PULLUP);
  pinMode(LED, OUTPUT);

  //clearEEPROM();

  bool s = loadValuesFromEEPROM();

  if(!s){
    leftSpeedValue = DEFAULT_SPEED;
    rightSpeedValue = DEFAULT_SPEED;
    calibrationFactor = 1.00;
    Serial.print("Default Left Speed = ");
    Serial.println(leftSpeedValue);
    Serial.print("Default Right Speed = ");
    Serial.println(rightSpeedValue);
    Serial.print("Default Calibration Factor = ");
    Serial.println(calibrationFactor);
  }

  // Routine to unlock to trial mode
    // Check if the button is pressed and held during power-up
   unlockAndPasswordMode();
  

  if(doNotLockValue == 0 && lockCounter >= 10){
      digitalWrite(LED, HIGH);
      while(true){
        delay(1000);
      }
  }

  digitalWrite(LED, HIGH);
  delay(3000);
  digitalWrite(LED, LOW);

//begin radio communication
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_HIGH);
  radio.startListening();

  commandTimeout = millis();
  motorTimeout = millis();
  lockTimer = millis();
  Serial.println("Setup complete");

  attachInterrupt(digitalPinToInterrupt(stopButton),stopISR,FALLING);

}

// Main loop
void loop() {
  if (radio.available()) {
    char text[32] = "";
    radio.read(&text, sizeof(text));
    // String command = String(text);
    Serial.print("Received command: ");
    Serial.println(text);
    processCommand(text);
    commandTimeout = millis();
    blinkLED(1);
  }
 
  switch(directionBuffer) {
    case 1:
    case 2:
      motorTimeout = 5000;
      break;
    case 3:
    case 4:
      motorTimeout = 2500;
      break;
    case 5:
      motorTimeout = 5000;
      // Implement U-Turn logic
      break;
  }

  if(directionBuffer>0 && millis() - commandTimeout > motorTimeout){
    stopLogic();
    directionBuffer = 0;
  }

  if(doNotLockValue == 0 && ((millis() - lockTimer )> 60000)){
    lockTimer = millis();
    lockCounter++;
    saveValueToEEPROM(lockCounterAddress, lockCounter);
  }
}

 
// Process the received command
void processCommand(char* command) {
  if(!strcmp(stopCommand, command)){
    Serial.println("Stopping chair");
    directionBuffer = 0;
    stopLogic();
  } else if(!strcmp(forwardCommand, command)){
    stopLogic();
    delay(100);
    forwardLogic();
    directionBuffer = 1;
    analogWrite(leftMotorPWM, leftSpeedValue);
    analogWrite(rightMotorPWM, rightSpeedValue);
  } else if(!strcmp(backwardCommand, command)){
    stopLogic();
    delay(100);
    backwardLogic();
    directionBuffer = 2;
    analogWrite(leftMotorPWM, leftSpeedValue);
    analogWrite(rightMotorPWM, rightSpeedValue);
  } else if(!strcmp(leftCommand, command)){
    leftLogic();
    directionBuffer = 3;
    analogWrite(leftMotorPWM, turnSpeed);
    analogWrite(rightMotorPWM, turnSpeed);
    delay(2000);
    stopLogic();
  } else if(!strcmp(rightCommand, command)){
    rightLogic();
    directionBuffer = 4;
    analogWrite(leftMotorPWM, turnSpeed);
    analogWrite(rightMotorPWM, turnSpeed);
    delay(2000);
    stopLogic(); 
  } else if(!strcmp(uTurnCommand, command)){
    // Implement U-Turn logic
    directionBuffer = 5;
  } else if(!strcmp(speedUpCommand, command)){
    if(calibrationMode == true){
      directionBuffer = 0;
      if(calibrateLeftMotor){
        leftMotorCalibrationValue += 5;
        if(leftMotorCalibrationValue > MAX_SPEED){
          leftMotorCalibrationValue = MAX_SPEED;
        }
        analogWrite(leftMotorPWM, leftMotorCalibrationValue);
        Serial.print("Left Motor Calibration Value = ");
        Serial.println(leftMotorCalibrationValue);
      } else if(calibrateRightMotor){
        rightMotorCalibrationValue += 5;
        if(rightMotorCalibrationValue > MAX_SPEED){
          rightMotorCalibrationValue = MAX_SPEED;
        }
        analogWrite(rightMotorPWM, rightMotorCalibrationValue);
        Serial.print("Right Motor Calibration Value = ");
        Serial.println(rightMotorCalibrationValue);
      }
    } else {
    leftSpeedValue += 12;
    if(leftSpeedValue > MAX_SPEED){
      leftSpeedValue = MAX_SPEED;
    }
    rightSpeedValue = calibrationFactor * leftSpeedValue;
    if(rightSpeedValue > MAX_SPEED){
      rightSpeedValue = MAX_SPEED;
    }
    if(directionBuffer == 1){
      forwardLogic();
    } else if(directionBuffer == 2){
      backwardLogic();
    } 
    analogWrite(leftMotorPWM, leftSpeedValue);
    analogWrite(rightMotorPWM, rightSpeedValue);
    Serial.print("Left Speed = ");
    Serial.print(leftSpeedValue);
    Serial.print("Right speed = ");
    Serial.println(rightSpeedValue);
  }
  } else if(!strcmp(slowDownCommand, command)){
    if(calibrationMode == true){
      if(calibrateLeftMotor){
        leftMotorCalibrationValue -= 5;
        if(leftMotorCalibrationValue < MIN_SPEED){
          leftMotorCalibrationValue = MIN_SPEED;
        }
        analogWrite(leftMotorPWM, leftMotorCalibrationValue);
        Serial.print("Left Motor Calibration Value = ");
        Serial.println(leftMotorCalibrationValue);
      } else if(calibrateRightMotor){
        rightMotorCalibrationValue -= 5;
        if(rightMotorCalibrationValue < MIN_SPEED){
          rightMotorCalibrationValue = MIN_SPEED;
        }
        analogWrite(rightMotorPWM, rightMotorCalibrationValue);
        Serial.print("Right Motor Calibration Value = ");
        Serial.println(rightMotorCalibrationValue);
      }
    } else {
    leftSpeedValue -= 12;
    if(leftSpeedValue < MIN_SPEED){
      leftSpeedValue = MIN_SPEED;
    }
    rightSpeedValue = calibrationFactor * leftSpeedValue;
    if(rightSpeedValue < MIN_SPEED){
      rightSpeedValue = MIN_SPEED;
    }
    if(directionBuffer == 1){
    forwardLogic();
    } else if(directionBuffer == 2){
      backwardLogic();
    } 
    analogWrite(leftMotorPWM, leftSpeedValue);
    analogWrite(rightMotorPWM, rightSpeedValue);
    Serial.print("Left Speed = ");
    Serial.print(leftSpeedValue);
    Serial.print("Right speed = ");
    Serial.println(rightSpeedValue);
    }
  } else if(!strcmp(startCalibrationCommand, command)){
    calibrationMode = true;
    Serial.println("Calibration mode started");
    // Start calibration
  } else if(!strcmp(endCalibrationCommand, command) && calibrationMode){
    calibrationMode = false;
    calibratedflag = 1;
    leftSpeedValue = leftMotorCalibrationValue;
    rightSpeedValue = rightMotorCalibrationValue;
    saveValueToEEPROM(leftMotorSpeedAddress, leftMotorCalibrationValue);
    saveValueToEEPROM(rightMotorSpeedAddress, rightMotorCalibrationValue);
    saveValueToEEPROM(calibrationStateAddress, calibratedflag);
    // calibrationFactor = leftMotorCalibrationValue / rightMotorCalibrationValue;
    // EEPROM.put(calibrationFactorAddress, calibrationFactor);
    calibrateLeftMotor = false;
    calibrateRightMotor = false;
    Serial.println("Calibration mode ended");
    // End calibration
  } else if(!strcmp(selectLeftCommand, command)){
    calibrateLeftMotor = true;
    calibrateRightMotor = false;
    // Select left
  } else if(!strcmp(selectRightCommand, command)){
    // Select right
    calibrateRightMotor = true;
    calibrateLeftMotor = false;
  } else if(!strcmp(keepForwardCommand, command)){
    // chair keeps going forward
    directionBuffer = 0;
    stopLogic();
    delay(100);
    forwardLogic();
    analogWrite(leftMotorPWM, leftSpeedValue);
    analogWrite(rightMotorPWM, rightSpeedValue);
  } else if(!strcmp(keepBackwardCommand, command)){
    // chair keeps going backward
    directionBuffer = 0;
    stopLogic();
    delay(100);
    backwardLogic();
    analogWrite(leftMotorPWM, leftSpeedValue);
    analogWrite(rightMotorPWM, rightSpeedValue);
  } else if(!strcmp(keepLeftCommand, command)){
    // chair in forward/backward left direction
    directionBuffer = 0;
    forwardLogic();
    analogWrite(leftMotorPWM, turnSpeed);
    analogWrite(rightMotorPWM, rightSpeedValue);
  } else if(!strcmp(keepRightCommand, command)){
    //chair in forward/backward right direction
    directionBuffer = 0;
    forwardLogic();
    analogWrite(leftMotorPWM, leftSpeedValue);
    analogWrite(rightMotorPWM, turnSpeed);
  } else if(!strcmp(rotateChairCommand, command)){
    // Rotate chair
    directionBuffer = 0;
    random(0, 2) == 0 ? leftLogic() : rightLogic();
    analogWrite(leftMotorPWM, turnSpeed);
    analogWrite(rightMotorPWM, turnSpeed);
  }
}

void stopLogic(){
      digitalWrite(leftMotorForwad, LOW);  // Left Motor
      digitalWrite(leftMotorReverse, LOW);
      digitalWrite(rightMotorForward, LOW);  // Right Motor
      digitalWrite(rightMotorReverse, LOW);

    analogWrite(leftMotorPWM,0);
    analogWrite(rightMotorPWM, 0);
}

void forwardLogic(){
      digitalWrite(leftMotorForwad, HIGH);  // Left Motor
      digitalWrite(leftMotorReverse, LOW);
      digitalWrite(rightMotorForward, HIGH);  // Right Motor
      digitalWrite(rightMotorReverse, LOW);
}

void backwardLogic(){
      digitalWrite(leftMotorForwad, LOW);  // Left Motor
      digitalWrite(leftMotorReverse, HIGH);
      digitalWrite(rightMotorForward, LOW);  // Right Motor
      digitalWrite(rightMotorReverse, HIGH);
}

void rightLogic(){
      digitalWrite(leftMotorForwad, HIGH);  // Left Motor
      digitalWrite(leftMotorReverse, LOW);
      digitalWrite(rightMotorForward, LOW);  // Right Motor
      digitalWrite(rightMotorReverse, HIGH);
}

void leftLogic(){
      digitalWrite(leftMotorForwad, LOW);  // Left Motor
      digitalWrite(leftMotorReverse, HIGH);
      digitalWrite(rightMotorForward, HIGH);  // Right Motor
      digitalWrite(rightMotorReverse, LOW);
}


// Function to load values from EEPROM
bool loadValuesFromEEPROM() {
  calibratedflag = EEPROM.read(calibrationStateAddress);
  lockCounter = EEPROM.read(lockCounterAddress);
  doNotLockValue = EEPROM.read(doNotLockAddress);
  Serial.print("Calibration flag = ");
  Serial.println(calibratedflag);
  Serial.print("Lock Counter = ");
  Serial.println(lockCounter);
  Serial.print("Do Not Lock Value = ");
  Serial.println(doNotLockValue);

  if(calibratedflag == 0){
    // return false;
    Serial.println("Chair not calibrated");
    return false;
  }
  leftSpeedValue = EEPROM.read(leftMotorSpeedAddress);
  rightSpeedValue = EEPROM.read(rightMotorSpeedAddress);
  // EEPROM.get(calibrationFactorAddress, calibrationFactor);
  Serial.print("Left Speed = ");
  Serial.println(leftSpeedValue);
  Serial.print("Right Speed = ");
  Serial.println(rightSpeedValue);
  calibrationFactor = (float)rightSpeedValue / leftSpeedValue;
  Serial.print("Calibration Factor = ");
  Serial.println(calibrationFactor);
  return true;
}

// Function to save values to EEPROM
void saveValueToEEPROM(int address, int value) {
  EEPROM.update(address, value);
}

void clearEEPROM(){
  for(int i = 0; i<512; i++){
    EEPROM.write(i, 0);
  }
  Serial.println("EEPROM cleared");
}

void blinkLED(int times){
  for(int i = 0; i<times; i++){
    digitalWrite(LED, HIGH);
    delay(200);
    digitalWrite(LED, LOW);
    delay(200);
  }
}

// Routine to unlock to trial mode and enter password mode
void unlockAndPasswordMode() {
  // Check if the button is pressed and held during power-up
  if (digitalRead(stopButton) == LOW) {
    unsigned long startTime = millis();
    while (millis() - startTime < 10000) {
      if (digitalRead(stopButton) == HIGH) {
        break;
      }
      delay(10);
    }
    if (millis() - startTime >= 10000) {
      blinkLED(5); // Blink LED 5 times
      Serial.println("Entering password mode");
      delay(1000); // Wait for 1 second

      // Enter password mode
      int password[3] = {0, 0, 0};
      for (int i = 0; i < 3; i++) {
        int count = 0;
        while (digitalRead(stopButton) == LOW) {
          delay(10);
        }
        while (digitalRead(stopButton) == HIGH) {
          delay(10);
        }
        while (digitalRead(stopButton) == LOW) {
          delay(10);
          count++;
          blinkLED(1); // Blink LED to indicate the entered number
        }
        password[i] = count;
        // blinkLED(count); // Blink LED to indicate the entered number
        delay(1000); // Wait for 1 second before next digit
      }

      // Check if the entered password is correct
      if (password[0] == 7 && password[1] == 2 && password[2] == 5) { // Example password: 1, 2, 3
        doNotLockValue = 1; // Reset lockCounter
        lockCounter = 0;
        saveValueToEEPROM(doNotLockAddress, doNotLockValue);
        saveValueToEEPROM(lockCounterAddress, lockCounter);
        Serial.println("Password Correct. Unlocking..."); // Indicate success
        blinkLED(3); // Indicate success
      } else {
        Serial.println("Incorrect password");
        digitalWrite(LED, HIGH); // Indicate failure
      }
    } else if (millis() - startTime >= 3000) {
      lockCounter = 0; // Reset lockCounter
      saveValueToEEPROM(lockCounterAddress, lockCounter);
      Serial.println("lockCounter reset to 0");
      blinkLED(3);
    }
  }
}


     
