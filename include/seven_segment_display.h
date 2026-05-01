#pragma once

#include <avr/io.h>
#include "output_shift_register.h"

// Models a two-digit 7+1 segment display with nine/ten pins: seven display
// pins, DP pin, and two cathode drive pins. An `OutputShiftRegister` is used
// to drive the display pins.
class SevenSegmentDisplay
{
  private:
    // Segment lookup
    inline static const uint8_t _segmentTable[11] = {
        0b01111110, // 0
        0b00110000, // 1
        0b01101101, // 2
        0b01111001, // 3
        0b00110011, // 4
        0b01011011, // 5
        0b01011111, // 6
        0b01110000, // 7
        0b01111111, // 8
        0b01110011, // 9
        0b10000000, // .
    };

    // Cathode Drive Pins
    volatile uint8_t* _outputRegister;
    uint8_t _pinCathode0;
    uint8_t _pinCathode1;

    // Display Pin Drive
    OutputShiftRegister _outputSR;

    // Display State
    //uint8_t _value;
    uint8_t _pendingCathode;
    uint8_t _digits[2];
    uint8_t _currentDigit;

  public:
    SevenSegmentDisplay() {}

    SevenSegmentDisplay(
        volatile uint8_t* outputRegister,
        uint8_t pinCathode0,
        uint8_t pinCathode1,
        const OutputShiftRegister& outputSR)
        : _outputSR(outputSR),
          _outputRegister(outputRegister),
          _pinCathode0(pinCathode0),
          _pinCathode1(pinCathode1),
          //_value(0),
          _currentDigit(0)
    {
    }

    // Call from main loop whenever the displayed number changes
    void setValue(uint8_t value)
    {
        if (value > 99) {
            _digits[1] = _segmentTable[10];
            _digits[0] = _segmentTable[10];
        } else {
            // TODO: figure out how to link this to the pins
            _digits[1] = _segmentTable[value / 10]; // tens
            _digits[0] = _segmentTable[value % 10]; // ones
            //_value = value;
        }
    }

    void Update()
    {
        _currentDigit ^= 1;

        // 
        *_outputRegister |= ((1 << _pinCathode0) | (1 << _pinCathode1));

        uint8_t pattern = _digits[_currentDigit];

        _pendingCathode = (_currentDigit == 0) ? _pinCathode0 : _pinCathode1;

        _outputSR.shiftOut(pattern);

        enableCathode(); 
    }

    void enableCathode()
    {
        // Pull active cathode LOW to enable it
        *_outputRegister &= ~(1 << _pendingCathode);
    }
};
