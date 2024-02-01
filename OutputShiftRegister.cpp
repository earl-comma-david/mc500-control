#include <avr/io.h>

class OutputShiftRegister 
{
  private:
    volatile uint8_t* _outputRegister;
    uint8_t _pinSer;
    uint8_t _pinLatch;
    uint8_t _pinClk;

    void putBit(uint8_t bit)
    {
        if(bit == 0)
            *_outputRegister &= ~(1<<_pinSer);
        else
            *_outputRegister |= (1<<_pinSer);

        cycleClock();
    }

    void latch()
    {
        *_outputRegister |= (1<<_pinLatch);
        *_outputRegister &= ~(1<<_pinLatch);
    }

    void cycleClock()
    {
        *_outputRegister |= (1<<_pinClk);
        *_outputRegister &= ~(1<<_pinClk);
    }

  public:

    OutputShiftRegister()
    {
    }

    OutputShiftRegister(volatile uint8_t* outputRegister, uint8_t pinSer, uint8_t pinLatch, uint8_t pinClk)
    {
        _outputRegister = outputRegister;
        _pinSer = pinSer;
        _pinLatch = pinLatch;
        _pinClk = pinClk;
    }

    void shiftOut(uint16_t toPut)
    {
        uint8_t i;
        for(i = 0; i < 16; i++) {
            putBit(toPut & 1);
            toPut >>= 1;
        }

        latch();
    }
};
