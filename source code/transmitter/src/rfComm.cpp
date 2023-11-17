
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

uint8_t packet[FIXED_PACKET_SIZE];   //for messages to transmit

enum {
  PAC_RC_DATA,
};

bool radioInitialised = false;

//function declarations
void transmitRCdata();
void buildPacket(uint8_t srcID, uint8_t destID, uint8_t dataIdentifier, uint8_t *dataBuff, uint8_t dataLen);

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
    //set transmitter rf power level
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

  transmitRCdata();
}

//==================================================================================================

void transmitRCdata()
{
  //--- Encode data into a bit stream ---
  uint8_t dataToSend[FIXED_PAYLOAD_SIZE];
  memset(dataToSend, 0, sizeof(dataToSend));
  for(uint8_t chIdx = 0; chIdx < NUM_RC_CHANNELS; chIdx++)
  {
    uint16_t val = (channelOut[chIdx] + 500) & 0xFFFF; 
    //encode into bit stream, with resolution of 10 bits
    uint8_t aIdx = chIdx + (chIdx / 4);
    uint8_t bIdx = aIdx + 1;
    uint8_t aShift = ((chIdx % 4) + 1) * 2;
    uint8_t bShift = 8 - aShift;
    dataToSend[aIdx] |= ((val >> aShift) & 0xFF);
    dataToSend[bIdx] |= ((val << bShift) & 0xFF);
  }
  // dataToSend[sizeof(dataToSend) - 1]  = //flags here if any

  //--- Build packet and transmit ---
  buildPacket(TRANSMITTER_ID, RECEIVER_ID, PAC_RC_DATA, dataToSend, sizeof(dataToSend));
  if(LoRa.beginPacket())
  {
    LoRa.write(packet, sizeof(packet));
    LoRa.endPacket(true); //async
    delayMicroseconds(500);
  }
}

//==================================================================================================

void buildPacket(uint8_t srcID, uint8_t destID, uint8_t dataIdentifier, uint8_t *dataBuff, uint8_t dataLen)
{
  memset(packet, 0, sizeof(packet));
  packet[0] = srcID;
  packet[1] = destID;
  packet[2] = dataIdentifier;
  for(uint8_t i = 0; i < dataLen; i++)
  {
    uint8_t idx = 3 + i;
    if(idx < sizeof(packet) - 1)
    {
      packet[idx] = *dataBuff;
      ++dataBuff;
    }
  }
  //write crc byte at the end
  packet[sizeof(packet) - 1] = crc8(packet, sizeof(packet) - 1);
}
