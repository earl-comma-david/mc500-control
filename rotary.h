/*
 * Rotary encoder library for Arduino.
 */

#ifndef rotary_h
#define rotary_h

//#include "Arduino.h"

// Enable this to emit codes twice per step.
#define HALF_STEP

// Enable weak pullups
#define ENABLE_PULLUPS

// Values returned by 'process'
// No complete step yet.
#define DIR_NONE 0x0
// Clockwise step.
#define DIR_CW 0x10
// Anti-clockwise step.
#define DIR_CCW 0x20

class Rotary
{
  public:
    Rotary();
    Rotary(volatile uint8_t* pin1Register, uint8_t pin1, volatile uint8_t* pin2Register, uint8_t pin2);
    // Process pin(s)
    unsigned char Process();
  private:
    volatile uint8_t* _pin1Register;
    uint8_t _pin1;
    volatile uint8_t* _pin2Register;
    uint8_t _pin2;
    uint8_t _state;
    //unsigned char _state;
    //unsigned char pin1;
    //unsigned char pin2;
};

#endif
 