/*
 *   debounce.c Snigelens version of Peter Dannegger's debounce routines.
 *     Debounce up to eight buttons on one port.  $Rev: 577 $
 *      
 */

//#include "debounce.h"

//// Bite is set to one if a debounced press is detected
//volatile uint8_t buttons_down_b;
//volatile uint8_t buttons_down_c;
//volatile uint8_t buttons_down_d;
//
//// Return non-zero if a button matching mask is pressed
//uint8_t button_down_b(uint8_t button_mask)
//{
//    // ATOMIC_BLOCK is needed if debounce is called from within an ISR
//    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
//    {
//        // And with debounced state for a one if they match
//        button_mask &= buttons_down_b;
//        // Clear if there was a match
//        buttons_down_b ^= button_mask;
//    }
//    // Return non-zero if there was a match
//    return button_mask;
//}
//
//uint8_t button_down_c(uint8_t button_mask)
//{
//    // ATOMIC_BLOCK is needed if debounce is called from within an ISR
//    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
//    {
//        // And with debounced state for a one if they match
//        button_mask &= buttons_down_c;
//        // Clear if there was a match
//        buttons_down_c ^= button_mask;
//    }
//    // Return non-zero if there was a match
//    return button_mask;
//}
//
//uint8_t button_down_d(uint8_t button_mask)
//{
//    // ATOMIC_BLOCK is needed if debounce is called from within an ISR
//    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
//    {
//        // And with debounced state for a one if they match
//        button_mask &= buttons_down_d;
//        // Clear if there was a match
//        buttons_down_d ^= button_mask;
//    }
//    // Return non-zero if there was a match
//    return button_mask;
//}

void debounce_init(void)
{
    //// Button pins as input
    //BUTTON_DDR &= ~(BUTTON_MASK);
    //// Enable pullup on buttons
    //BUTTON_PORT |= BUTTON_MASK;
}

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
