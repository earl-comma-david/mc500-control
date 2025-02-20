#include <avr/io.h>

class InputShiftRegister 
{
  private:
    volatile uint8_t* _inputRegister;
    volatile uint8_t* _controlRegister;
    uint8_t _pinSer;
    uint8_t _pinLatch;
    uint8_t _pinClk;

  public:

    InputShiftRegister()
    {
    }

    InputShiftRegister(
        volatile uint8_t* inputRegister,
        uint8_t pinSer,
        volatile uint8_t* controlRegister,
        uint8_t pinLatch,
        uint8_t pinClk)
    {
        _inputRegister = inputRegister;
        _controlRegister = controlRegister;
        _pinSer = pinSer;
        _pinLatch = pinLatch;
        _pinClk = pinClk;

        goHi(_pinLatch);
        goLo(_pinClk);
    }

    volatile uint16_t read()
    {
        uint16_t word = 0x0000;

        goHi(_pinClk);
        latch();

        for (int i = 15; i >= 0; i--)
        {
            cycleClock();

            word |= ((uint8_t) bit_is_set(*_inputRegister, _pinSer) > 0) << i;
        }

        return word;
    }

    void latch()
    {
        goLo(_pinLatch);
        goHi(_pinLatch);
    }

    void cycleClock() {
        goHi(_pinClk);
        goLo(_pinClk);
    }

    inline void goHi(uint8_t pin)
    {
        *_controlRegister |= (1<<pin);
    }

    inline void goLo(uint8_t pin)
    {
        *_controlRegister &= ~(1<<pin);
    }
};
