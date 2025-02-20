#include <avr/io.h>

#define MAX_CHECKS 24

class Debouncer
{
  private:
    uint16_t _state[MAX_CHECKS];
    uint8_t _index;

  public:
    Debouncer()
    {
    }

    uint16_t UpdateState(uint16_t value)
    //void UpdateState(uint16_t value)
    {
        uint16_t sample = ~value;

        static uint16_t state, cnt0, cnt1;
        uint16_t delta;

        delta = sample ^ state;
        cnt1 = (cnt1 ^ cnt0) & delta;
        cnt0 = ~cnt0 & delta;
        state ^= (cnt0 & cnt1);

        return ~state;
        //_state[_index++] = value;

        //if (_index >= MAX_CHECKS)
        //{
        //    _index=0;
        //}
    }

    uint16_t Debounce(void)
    {
        uint16_t debouncedState = 0xffff;
        for (uint8_t i=0; i<MAX_CHECKS; i++)
        {
            debouncedState = debouncedState & _state[i];
        }

        return debouncedState;
    }
};
