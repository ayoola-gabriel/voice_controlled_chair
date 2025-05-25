#include <SPI.h>
#include <nRF24L01.h>
#include <printf.h>
#include <RF24_config.h>
#include <RF24.h>

#define CSN A0
#define CE A1
#define IRQ 10

uint16_t noRadioTimeOut = 0;

//create an RF24 object
RF24 radio(CE, CSN);  // CE, CSN


//address through which two modules communicate.
const byte address[6] = "CXL5G";

bool beginRadio()
{
  if (!radio.begin()) {
    // Serial.println(F("radio hardware is not responding!!"));
    return false;
  }

  // Serial.println(F("radio hardware is powered on"));
  
  radio.setPALevel(RF24_PA_HIGH);
  radio.openWritingPipe(address);
  radio.stopListening();
  return true;
}

bool radioSend(const char* msg){ 
  // radio.openWritingPipe(address);
  uint8_t len = strlen(msg);
  if (len > 32) {
    // Serial.println("Message too long");
    return false;
  }
  char toSend[32] = "";

  // strcpy(toSend, msg);
  sprintf(toSend, "%s", msg);
  Serial.print("SENDING: ");
  Serial.println(toSend);
  delay(1000);
  bool s = radio.write(&toSend, sizeof(toSend)); 
  return s;    
}           
    