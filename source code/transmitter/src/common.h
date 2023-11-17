#ifndef _COMMON_H_
#define _COMMON_H_


#define NUM_RC_CHANNELS 6

//--- controls
extern int16_t x1AxisIn;
extern int16_t y1AxisIn; 
extern int16_t x2AxisIn;
extern int16_t y2AxisIn;
extern int16_t knobAIn;
extern int16_t swAIn;

//--- rc channel outputs
extern int16_t  channelOut[NUM_RC_CHANNELS];  //Range is -500 to 500

#endif //_COMMON_H_