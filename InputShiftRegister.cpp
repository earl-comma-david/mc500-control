#include <avr/cpufunc.h> 
#include <avr/io.h>
#include <math.h>
#include <util/delay.h>

#define PIN_LATCH PD2
#define PIN_CLK PD3
#define PIN_SER PD4

#define GOHI(pn) PORTD |= (1<<pn)
#define GOLO(pn) PORTD &= ~(1<<pn)

volatile uint16_t read();

class InputShiftRegister 
{
  public:

    InputShiftRegister()
    {
        GOHI(PIN_LATCH);
        GOLO(PIN_CLK);
    }

    volatile uint16_t read()
    {
        uint16_t word = 0x0000;

        GOHI(PIN_CLK);
        latch();

        for (int i = 15; i >= 0; i--)
        {
            cycleClock();
            // TODO: refactor to parameterize `PIND`
            word |= ((uint8_t) bit_is_set(PIND, PIN_SER) > 0) << i;
            shiftOffsetDelay(15-i);
        }

        return word;
    }

    void shiftOffsetDelay(int n)
    {
        for (int i = 0; i < n - 6; i++)
        {
            _NOP();
        }
    }

    void latch()
    {
        GOLO(PIN_LATCH);
        _delay_us(2);
        GOHI(PIN_LATCH);
    }

    void cycleClock() {
        GOHI(PIN_CLK);
        _delay_us(5);
        GOLO(PIN_CLK);
    }
};
