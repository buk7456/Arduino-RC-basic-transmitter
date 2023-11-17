#ifndef _CONFIG_H_
#define _CONFIG_H_

//------------------- PIN MAPPING -----------------------------
//Name                   Arduino Pin

#define PIN_X1_AXIS      A0         //Rud
#define PIN_Y1_AXIS      A1         //Thr
#define PIN_X2_AXIS      A2         //Ail
#define PIN_Y2_AXIS      A3         //Ele
#define PIN_KNOB_A       A4

#define PIN_SWA          2

//Lora module
#define PIN_LORA_RESET   8
#define PIN_LORA_SS      10

//------------------- SELF CENTERING ON THROTTLE AXIS ---------
//If your throttle axis is self centering, uncomment the following line
// #define THROTTLE_AXIS_SELF_CENTERING

//------------------- RADIO FREQUENCY, SELECT ONLY ONE --------
#define ISM_433MHZ
// #define ISM_915MHZ

//------------------- TRANSMITTER AND RECEIVER ID -------------
//Should be the same in transmitter and receiver code.
#define TRANSMITTER_ID  0x12 
#define RECEIVER_ID     0xCD 


#endif  //_CONFIG_H_