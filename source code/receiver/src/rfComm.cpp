
#include "Arduino.h"
#include <SPI.h>

#include "LoRa.h"
#include "../config.h"
#include "common.h"
#include "crc.h"
#include "rfComm.h"

#define TX_POWER_DBM  14

#if defined (ISM_433MHZ)
  #define FREQUENCY 433150000
#elif defined (ISM_915MHZ)
  #define FREQUENCY 916100000
#endif

#define FIXED_PAYLOAD_SIZE ((((NUM_RC_CHANNELS * 10) + 7) / 8) + 1)
#define FIXED_PACKET_SIZE  (FIXED_PAYLOAD_SIZE + 4)

uint8_t msgBuff[FIXED_PACKET_SIZE]; //for received messages

uint32_t lastRCPacketMillis;

enum {
  PAC_RC_DATA,
  PAC_INVALID,
};

bool radioInitialised = false;

//function declarations
void readReceivedMessage();
uint8_t checkReceivedMessage(uint8_t srcID, uint8_t destID);

//==================================================================================================

void initialiseRfModule()
{
  //setup lora module
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
    channelOut[2] = -500;
#endif

}

//================================= HELPERS ========================================================

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

//--------------------------------------------------------------------------------------------------

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

