/*
 *   debounce.c Snigelens version of Peter Dannegger's debounce routines.
 *     Debounce up to eight buttons on one port.  $Rev: 577 $
 *      
 */

#include "debounce.h"

// Bite is set to one if a debounced press is detected
volatile uint8_t buttons_down_b;
volatile uint8_t buttons_down_c;
volatile uint8_t buttons_down_d;

// Return non-zero if a button matching mask is pressed
uint8_t button_down_b(uint8_t button_mask)
{
    // ATOMIC_BLOCK is needed if debounce is called from within an ISR
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        // And with debounced state for a one if they match
        button_mask &= buttons_down_b;
        // Clear if there was a match
        buttons_down_b ^= button_mask;
    }
    // Return non-zero if there was a match
    return button_mask;
}

uint8_t button_down_c(uint8_t button_mask)
{
    // ATOMIC_BLOCK is needed if debounce is called from within an ISR
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        // And with debounced state for a one if they match
        button_mask &= buttons_down_c;
        // Clear if there was a match
        buttons_down_c ^= button_mask;
    }
    // Return non-zero if there was a match
    return button_mask;
}

uint8_t button_down_d(uint8_t button_mask)
{
    // ATOMIC_BLOCK is needed if debounce is called from within an ISR
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        // And with debounced state for a one if they match
        button_mask &= buttons_down_d;
        // Clear if there was a match
        buttons_down_d ^= button_mask;
    }
    // Return non-zero if there was a match
    return button_mask;
}

void debounce_init(void)
{
    //// Button pins as input
    //BUTTON_DDR &= ~(BUTTON_MASK);
    //// Enable pullup on buttons
    //BUTTON_PORT |= BUTTON_MASK;
}