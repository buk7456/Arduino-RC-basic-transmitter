/***************************************************************************************************
  Firmware for the rc transmitter.
  Target mcu: Atmega328p
  Compiles on Arduino 1.8.x or later
  Authors: buk7456 <buk7456@gmail.com>
  License: MIT
***************************************************************************************************/

#include "Arduino.h"
#include "crc.h"
#include "LoRa.h"

//-------- Pin mapping ----------------

#define PIN_X1_AXIS      A0    //Rud
#define PIN_Y1_AXIS      A1    //Thr
#define PIN_X2_AXIS      A2    //Ail
#define PIN_Y2_AXIS      A3    //Ele
#define PIN_KNOB_A       A4
#define PIN_SWA          2
#define PIN_LORA_RESET   8
#define PIN_LORA_SS      10

//-------- Controls -------------------

int16_t x1AxisIn;
int16_t y1AxisIn; 
int16_t x2AxisIn;
int16_t y2AxisIn;
int16_t knobAIn;
int16_t swAIn;
//If your throttle axis is self centering, uncomment the following line
// #define THROTTLE_AXIS_SELF_CENTERING

//-------- RC channel outputs ---------

#define NUM_RC_CHANNELS 6
int16_t  channelOut[NUM_RC_CHANNELS];  //Range is -500 to 500

//-------- RF related -----------------

bool radioInitialised = false;

#define TX_POWER_DBM  14

#define FREQUENCY 433150000
// #define FREQUENCY 916100000

#define FIXED_PAYLOAD_SIZE ((((NUM_RC_CHANNELS * 10) + 7) / 8) + 1)
#define FIXED_PACKET_SIZE  (FIXED_PAYLOAD_SIZE + 4)
uint8_t packet[FIXED_PACKET_SIZE];   //for messages to transmit

enum {
  //packet types
  PAC_RC_DATA,
};

//Should be the same in transmitter and receiver code.
#define TRANSMITTER_ID  0x12 
#define RECEIVER_ID     0xCD 

//-------- Function declarations ------

void readSticksAndSwitches();
void computeChannelOutputs();
int16_t deadzoneAndMap(int16_t input, int16_t minVal, int16_t centreVal, int16_t maxVal, 
                       int16_t deadzonePercent, int16_t mapMin, int16_t mapMax);
void transmitRCdata();
void buildPacket(uint8_t srcID, uint8_t destID, uint8_t dataIdentifier, uint8_t *dataBuff, uint8_t dataLen);

//==================================================================================================

void setup()
{
  pinMode(PIN_SWA, INPUT_PULLUP);

  //delay a bit to allow time for RF module to be ready
  delay(100);

  //Initialise LoRa module
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

void loop()
{
  static uint32_t previousMillis;
  const uint8_t fixedLoopTime = 20;  //in milliseconds
  if(millis() - previousMillis >= fixedLoopTime)
  {
    previousMillis = millis();
  
    //read the controls
    readSticksAndSwitches();
    
    //evaluate the rc channel outputs
    computeChannelOutputs();
    
    //rf communications
    transmitRCdata();
  }
}

////==================================================================================================

void readSticksAndSwitches()
{
  // Reads sticks, switch, knob. Also adds deadzones to sticks centers and extremes
  // to prevent jitter in servos when sticks are centered or at extremes.
  
  x1AxisIn = analogRead(PIN_X1_AXIS);
  x1AxisIn  = deadzoneAndMap(x1AxisIn, 16, 512, 1007, 5, -500, 500);

#if defined (THROTTLE_AXIS_SELF_CENTERING)
  y1AxisIn = analogRead(PIN_Y1_AXIS);
  y1AxisIn = deadzoneAndMap(y1AxisIn, 16, 512, 1007, 5, -500, 500);
#else
  y1AxisIn = analogRead(PIN_Y1_AXIS);
  y1AxisIn = map(y1AxisIn, 16, 1007, -500, 500);
  y1AxisIn = constrain(y1AxisIn, -500, 500);
#endif

  x2AxisIn = analogRead(PIN_X2_AXIS);
  x2AxisIn = deadzoneAndMap(x2AxisIn, 16, 512, 1007, 5, -500, 500);
  
  y2AxisIn = analogRead(PIN_Y2_AXIS);
  y2AxisIn = deadzoneAndMap(y2AxisIn, 16, 512, 1007, 5, -500, 500);
  
  knobAIn = analogRead(PIN_KNOB_A);
  knobAIn = map(knobAIn, 16, 1007, -500, 500); 
  knobAIn = constrain(knobAIn, -500, 500);

  swAIn = digitalRead(PIN_SWA) ? -500 : 500;
}

//==================================================================================================

int16_t deadzoneAndMap(int16_t input, int16_t minVal, int16_t centreVal, int16_t maxVal, 
                       int16_t deadzonePercent, int16_t mapMin, int16_t mapMax)
{
  int32_t deadznTmp = ((int32_t)(maxVal - minVal) * deadzonePercent) / 100; 
  int16_t deadznVal = deadznTmp / 2; //divide by 2 as we apply deadzone about center
  int16_t mapCenter = (mapMin / 2) + (mapMax / 2);
  //add dead zone and map the input
  int16_t output;
  if(input > centreVal + deadznVal)
    output = map(input, centreVal + deadznVal, maxVal, mapCenter + 1, mapMax);
  else if(input < centreVal - deadznVal)
    output = map(input, minVal, centreVal - deadznVal, mapMin, mapCenter - 1);
  else
    output = mapCenter;
  
  output = constrain(output, mapMin, mapMax);
  return output;
}

//==================================================================================================

void computeChannelOutputs()
{
  int16_t outVals[] = {x1AxisIn, y1AxisIn, x2AxisIn, y2AxisIn, knobAIn, swAIn};
  for(uint8_t i = 0; i < NUM_RC_CHANNELS && i < sizeof(outVals)/sizeof(outVals[0]); i++)
  {
    channelOut[i] = outVals[i];
  }
}

//==================================================================================================

void transmitRCdata()
{
  if(!radioInitialised)
    return;
  
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
