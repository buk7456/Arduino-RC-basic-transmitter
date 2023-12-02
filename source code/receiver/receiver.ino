/***************************************************************************************************
  Firmware for the rc receiver.
  Target mcu: Atmega328p
  Compiles on Arduino 1.8.x or later
  Authors: buk7456 <buk7456@gmail.com>
  License: MIT
***************************************************************************************************/

#include "Arduino.h"
#include <Servo.h>
#include "LoRa.h"
#include "crc.h"

//-------- Pin mapping ----------------

#define PIN_CH1    7
#define PIN_CH2    2
#define PIN_CH3    3
#define PIN_CH4    4
#define PIN_CH5    5
#define PIN_CH6    6
#define PIN_LED    9
#define PIN_LORA_RESET  8
#define PIN_LORA_SS     10

//-------- Servos ---------------------

#define NUM_RC_CHANNELS  6

int16_t channelOut[NUM_RC_CHANNELS];

//Declare an array of servo objects
Servo myServo[NUM_RC_CHANNELS];

//Declare an output pins array
int16_t outputPin[NUM_RC_CHANNELS] = {PIN_CH1, PIN_CH2, PIN_CH3, PIN_CH4, PIN_CH5, PIN_CH6 };

//If your throttle axis is self centering, uncomment the following line
// #define THROTTLE_AXIS_SELF_CENTERING

//Failsafe on throttle channel
//When receiver loses signal, the throttle channel moves to the default position after some short delay.
//Other channels maintain their last received positions.
//In case you dont want this behaviour on the throttle channel, comment out the following line
#define THROTTLE_CHANNEL_FAILSAFE 

//-------- RF related -----------------

bool radioInitialised = false;

#define TX_POWER_DBM  14

#define FREQUENCY 433150000
// #define FREQUENCY 916100000

#define FIXED_PAYLOAD_SIZE ((((NUM_RC_CHANNELS * 10) + 7) / 8) + 1)
#define FIXED_PACKET_SIZE  (FIXED_PAYLOAD_SIZE + 4)

uint8_t msgBuff[FIXED_PACKET_SIZE]; //for received messages

enum {
  //packet types
  PAC_RC_DATA,
  PAC_INVALID,
};

uint32_t lastRCPacketMillis;

//Should be the same in transmitter and receiver code.
#define TRANSMITTER_ID  0x12 
#define RECEIVER_ID     0xCD 

//--------------- Function Declarations ----------
void writeOutputs();
void readReceivedMessage();
uint8_t checkReceivedMessage(uint8_t srcID, uint8_t destID);

//==================================================================================================

void setup()
{ 
  //--- setup pins
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  
  //--- initialise values, attach servos
  for(uint8_t chIdx = 0; chIdx < NUM_RC_CHANNELS; ++chIdx)
  {
    channelOut[chIdx] = 0;
    
    #if !defined (THROTTLE_AXIS_SELF_CENTERING)
    if(chIdx == 2)
      channelOut[chIdx] = -500;
    #endif
    
    //attach servos
    myServo[chIdx].attach(outputPin[chIdx]); 
  }

  //--- delay to allow module to be ready
  delay(100);

  //--- initialise radio module
  LoRa.setPins(PIN_LORA_SS, PIN_LORA_RESET); 
  if(LoRa.begin(FREQUENCY))
  {
    LoRa.setSpreadingFactor(7); 
    LoRa.setCodingRate4(5);
    LoRa.setSignalBandwidth(500E3);
    delay(20);
    //set rf power level
    LoRa.sleep();
    LoRa.setTxPower(TX_POWER_DBM);
    LoRa.idle();
    radioInitialised = true;
  }
  else
    radioInitialised = false;
}

//==================================================================================================

void loop()
{
  //--- COMMUNICATION
  doRfCommunication();

  //--- RC OUTPUTS
  writeOutputs();
}

//==================================================================================================

void writeOutputs()
{ 
  for(uint8_t chIdx = 0; chIdx < NUM_RC_CHANNELS; chIdx++)
  {
    int16_t val = map(channelOut[chIdx], -500, 500, 1000, 2000);
    val = constrain(val, 1000, 2000);
    myServo[chIdx].writeMicroseconds(val);
  }
}

//==================================================================================================

void doRfCommunication()
{
  if(!radioInitialised)
    return;
  
  //--- READ INCOMING PACKETS
 
  uint8_t packetType = PAC_INVALID;
  
  if(LoRa.parsePacket()) //received a packet
  {
    readReceivedMessage();
    packetType = checkReceivedMessage(TRANSMITTER_ID, RECEIVER_ID);
  }

  switch(packetType)
  {
    case PAC_RC_DATA:
      {
        lastRCPacketMillis = millis();
        digitalWrite(PIN_LED, HIGH);
        //Decode the rc data
        for(uint8_t chIdx = 0; chIdx < NUM_RC_CHANNELS; chIdx++)
        {
          uint8_t aIdx = 3 + chIdx + (chIdx / 4); //3 is the index in msgBuff where the data begins
          uint8_t bIdx = aIdx + 1;
          uint8_t aShift = ((chIdx % 4) + 1) * 2;
          uint8_t bShift = 8 - aShift;
          uint16_t aMask = ((uint16_t)0x0400) - ((uint16_t)1 << aShift);
          uint8_t  bMask = ((uint16_t)1 << (8-bShift)) - 1;
          channelOut[chIdx] = (((uint16_t)msgBuff[aIdx] << aShift) & aMask) | (((uint16_t)msgBuff[bIdx] >> bShift) & bMask);
          channelOut[chIdx] -= 500;
        }
      }
      break;
  }

  //--- TURN OFF LED TO INDICATE NO INCOMING RC DATA
  if(millis() - lastRCPacketMillis > 100)
    digitalWrite(PIN_LED, LOW);
  
  //--- Cut throttle channel if signal was lost
#if defined (THROTTLE_CHANNEL_FAILSAFE)
  if(millis() - lastRCPacketMillis > 1500)
  #if defined (THROTTLE_AXIS_SELF_CENTERING)
    channelOut[2] = 0;
  #else
    channelOut[2] = -500;
  #endif
#endif

}

//==================================================================================================

void readReceivedMessage()
{
  memset(msgBuff, 0, sizeof(msgBuff));
  uint8_t cntr = 0;
  while (LoRa.available() > 0) 
  {
    if(cntr < sizeof(msgBuff))
    {
      msgBuff[cntr] = LoRa.read();
      cntr++;
    }
    else // discard any extra data
      LoRa.read(); 
  }
}

//==================================================================================================

uint8_t checkReceivedMessage(uint8_t srcID, uint8_t destID)
{
  if(msgBuff[0] != srcID || msgBuff[1] != destID)
    return PAC_INVALID;
  
  //check crc
  uint8_t crcQQ = msgBuff[sizeof(msgBuff) - 1];
  uint8_t computedCRC = crc8(msgBuff, sizeof(msgBuff) - 1);
  if(crcQQ != computedCRC)
    return PAC_INVALID;
  
  return msgBuff[2];
}
