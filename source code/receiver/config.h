#ifndef _CONFIG_H_
#define _CONFIG_H_

//------------------- PIN MAPPING -----------------------------
//Name                   Arduino Pin

#define PIN_CH1    7
#define PIN_CH2    2
#define PIN_CH3    3
#define PIN_CH4    4
#define PIN_CH5    5
#define PIN_CH6    6

#define PIN_LED    9

#define PIN_LORA_RESET  8
#define PIN_LORA_SS     10

//------------------- RADIO FREQUENCY, SELECT ONLY ONE --------
#define ISM_433MHZ
// #define ISM_915MHZ

//------------------- TRANSMITTER AND RECEIVER ID -------------
//Should be the same in transmitter and receiver code.
#define TRANSMITTER_ID  0x12 
#define RECEIVER_ID     0xCD 

//-------------------- FAILSAFE ON THROTTLE CHANNEL -----------
//When receiver loses signal, the throttle channel moves to the lowest position after some short delay.
//Other channels maintain their last received positions.
//In case you dont want this behaviour on the throttle channel, comment out the following line
#define THROTTLE_CHANNEL_FAILSAFE 


#endif  //_CONFIG_H_