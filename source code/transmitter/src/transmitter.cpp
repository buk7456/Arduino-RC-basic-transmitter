
#include "Arduino.h"

#include "../config.h"
#include "common.h"
#include "crc.h"
#include "rfComm.h"

//---------- Function declarations ---------------

void readSticksAndSwitches();
void computeChannelOutputs();
int16_t deadzoneAndMap(int16_t input, int16_t minVal, int16_t centreVal, int16_t maxVal, 
                       int16_t deadzonePercent, int16_t mapMin, int16_t mapMax);

//==================================================================================================

void setup()
{
  pinMode(PIN_SWA, INPUT_PULLUP);

  //delay a bit to allow time for RF module to be ready
  delay(100);

  //Initialise module
  initialiseRfModule();
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
    doRfCommunication();
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

void computeChannelOutputs()
{
  int16_t outVals[] = {x1AxisIn, y1AxisIn, x2AxisIn, y2AxisIn, knobAIn, swAIn};
  for(uint8_t i = 0; i < NUM_RC_CHANNELS && i < sizeof(outVals)/sizeof(outVals[0]); i++)
  {
    channelOut[i] = outVals[i];
  }
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

