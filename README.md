### Basic features
- 2 joystics, 1 knob, 1 switch.
- 6 RC channels, each channel encoded with 10 bits.
- Optional failsafe on throttle channel.

### Hardware and Schematics
The schematics are in the 'schematics' folder. 
The major components are as below.
- 2x Atmega328p microcontrollers.
- 2x SX1276/77/78/79 or RFM95 based Lora modules. 
  However any other module can possibly be used with some code modifications.

### Compiling the firmware
The code compiles on arduino ide 1.8.x or later. The code is self-contained so doesn't require installing any libraries. 
