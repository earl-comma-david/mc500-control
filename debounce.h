/*
 *   debounce.h. Snigelens version of Peter Dannegger's debounce routines.
 *     Debounce up to eight buttons on one port.  $Rev: 577 $
 */

#ifndef DEBOUNCE_H
#define DEBOUNCE_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

// Variable to tell that the button is pressed (and debounced).
// Can be read with button_down() which will clear it.
extern volatile uint8_t buttons_down_b;
extern volatile uint8_t buttons_down_c;
extern volatile uint8_t buttons_down_d;

// Return non-zero if a button matching mask is pressed.
uint8_t button_down_b(uint8_t button_mask);
uint8_t button_down_c(uint8_t button_mask);
uint8_t button_down_d(uint8_t button_mask);

// Make button pins inputs activate internal pullups.
void debounce_init(void);

// Decrease 2 bit vertical counter where mask = 1
// Set counters to binary 11 where mask = 0.
#define VC_DEC_OR_SET(high, low, mask)      \
    low = ~(low & mask);            \
    high = low ^ (high & mask)

static inline void debounce (uint8_t word)
//static inline void debounce (void)
{
    // Eight vertical two bit counters for number of equal states
    static uint8_t vcount_low_b = 0xFF, vcount_high_b = 0xFF;
    static uint8_t vcount_low_c = 0xFF, vcount_high_c = 0xFF;
    static uint8_t vcount_low_d = 0xFF, vcount_high_d = 0xFF;
    // Keeps track of current (debounced) state
    static uint8_t button_state_b = 0;
    static uint8_t button_state_c = 0;
    static uint8_t button_state_d = 0;

    // Read buttons (active low so invert with ~.  Xor with
    // button_state to see which ones are about to change state
    uint8_t state_changed_b = ~PINB ^ button_state_b;
    uint8_t state_changed_c = ~PINC ^ button_state_c;
    uint8_t state_changed_d = ~word ^ button_state_d;
    //uint8_t state_changed_d = ~PIND ^ button_state_d;

    // Decrease counters where state_changed = 1, set the others to 0b11.
    VC_DEC_OR_SET(vcount_high_b, vcount_low_b, state_changed_b);
    VC_DEC_OR_SET(vcount_high_c, vcount_low_c, state_changed_c);
    VC_DEC_OR_SET(vcount_high_d, vcount_low_d, state_changed_d);

    // Update state_changed to have a 1 only if the counter overflowed
    state_changed_b &= vcount_low_b & vcount_high_b;
    state_changed_c &= vcount_low_c & vcount_high_c;
    state_changed_d &= vcount_low_d & vcount_high_d;
    // Change button_state for the buttons who's counters rolled over
    button_state_b ^= state_changed_b;
    button_state_c ^= state_changed_c;
    button_state_d ^= state_changed_d;

    // Update button_down with buttons who's counters rolled over
    // and who's state us 1 (pressed)
    buttons_down_b |= button_state_b & state_changed_b;
    buttons_down_c |= button_state_c & state_changed_c;
    buttons_down_d |= button_state_d & state_changed_d;
}

#endif /* DEBOUNCE_H */
