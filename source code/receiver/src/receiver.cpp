
#include "Arduino.h"
#include <Servo.h>

#include "../config.h"
#include "crc.h"
#include "common.h"
#include "rfComm.h"

//Declare an array of servo objects
Servo myServo[NUM_RC_CHANNELS];

//Declare an output pins array
int16_t outputPin[NUM_RC_CHANNELS] = {
  PIN_CH1,
  PIN_CH2,
  PIN_CH3, 
  PIN_CH4, 
  PIN_CH5, 
  PIN_CH6
};

//--------------- Function Declarations ----------
void writeOutputs();

//==================================================================================================

void setup()
{ 
  //--- setup pins
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  
  //--- initialise values, attach servos
  for(uint8_t chIdx = 0; chIdx < NUM_RC_CHANNELS; ++chIdx)
  {
    //for throttle channel, set initial to -500, otherwise set to 0
    channelOut[chIdx] = (chIdx == 2) ? -500 : 0;
    //attach servos
    myServo[chIdx].attach(outputPin[chIdx]); 
  }

  //--- delay to allow module to be ready
  delay(100);

  //--- initialise radio module
  initialiseRfModule();
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
