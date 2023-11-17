
#include "Arduino.h"
#include "common.h"

#if NUM_RC_CHANNELS < 4
  #error At least 4 channels
#elif NUM_RC_CHANNELS > 20
  #error At most 20 channels
#endif

int16_t channelOut[NUM_RC_CHANNELS];
