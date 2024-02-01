#ifndef __AVR_ATmega328P__
  #define __AVR_ATmega328P__ 
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdbool.h>
#include <string.h> 
#include "config.h"
#include "twi-master.h"
#include "twi-master.c"
#include "Debouncer.cpp"
#include "RotaryEncoder.cpp"
#include "InputShiftRegister.cpp"
#include "OutputShiftRegister.cpp"
#include "ToggleSwitch.cpp"

#define AUDIO_SLAVE_ADDRESS 0x10
#define DIM_OFFSET 16

volatile uint16_t _commandWord = 0;
volatile uint16_t _switchState = 0b0100000001000000;
volatile uint8_t _attenuationHP = 127;
volatile uint8_t _attenuationMain = 127;
volatile bool _doScan;

InputShiftRegister _inputShiftRegister;
Debouncer _debouncer;
RotaryEncoder _rotaryEncoderMain;
RotaryEncoder _rotaryEncoderCue;

volatile int testCounter = 0;

// read input shift register every 1ms
ISR (TIMER0_OVF_vect)
{
    _commandWord = _debouncer.UpdateState(_inputShiftRegister.read());

    _doScan = true;
}

// read main rotary encoder upon pin change interrupt
ISR (PCINT0_vect)
{
    _rotaryEncoderMain.Scan();
}

// read cue rotary encoder upon pin change interrupt
ISR (PCINT1_vect)
{
    _rotaryEncoderCue.Scan();
}

void timer_init(void)
{
    // F_int = F_clk/(2*prescalar*256)
    // F_int = 16000000/64*256 // TODO: should there be a factor of two in the denominator?
    // F_int = 977Hz -> P = 0.00102
	TCCR0A = 0x00;                  // normal mode
	TCCR0B = (1<<CS01) | (1<<CS00); // prescalar: 64
	TIMSK0 = (1 << TOIE0) ;         // Timer 0 overflow interrupt enable
}
 
void pin_change_interrupt_init(void)
{
    PCICR |= (1<<PCIE1) | (1<<PCIE0);
    PCMSK1 |= (1<<PCINT9) | (1<<PCINT8);
    PCMSK0 |= (1<<PCINT1) | (1<<PCINT0);
}

void init_io_pins(void)
{
    // pullups
    PORTC |= (1<<PC5) | (1<<PC4);

    // outputs
    DDRB |= (1<<PB4);
    DDRB |= (1<<PB3);
    DDRB |= (1<<PB2);
    DDRD |= (1<<PD2);
    DDRD |= (1<<PD3);

    // inputs
    DDRB &= ~(1 << PINB5); // TODO: this seems to be necessary...y tho?
    DDRB &= ~(1 << PINB1);
    DDRB &= ~(1 << PINB0);
    DDRC &= ~(1 << PINC1);
    DDRC &= ~(1 << PINC0);
    DDRD &= ~(1 << PIND7);
    DDRD &= ~(1 << PIND6);
    DDRD &= ~(1 << PIND5);
    DDRD &= ~(1 << PIND4);
}

void init(void)
{
    timer_init();
    pin_change_interrupt_init();
    tw_init(TW_FREQ_250K, true);
    init_io_pins();

    UCSR0B = 0;

    _rotaryEncoderMain = RotaryEncoder(&PINB, PINB1, PINB0, &_attenuationMain);
    _rotaryEncoderCue = RotaryEncoder(&PINC, PINC1, PINC0, &_attenuationMain);
    //_rotaryEncoderCue = RotaryEncoder(&PINC, PINC1, PINC0, &_attenuationCue);
    _inputShiftRegister = InputShiftRegister(&PIND, PD4, &PORTD, PD2, PD3);
    _debouncer = Debouncer();

	sei();
}

int main (void)
{
    ExclusiveToggleSwitchGroup inputSwitchGroup = ExclusiveToggleSwitchGroup(
        ToggleSwitch(&_commandWord, 14, &_switchState, 14),
        ToggleSwitch(&_commandWord, 13, &_switchState, 13),
        ToggleSwitch(&_commandWord, 12, &_switchState, 12),
        ToggleSwitch(&_commandWord, 11, &_switchState, 11));
    ToggleSwitch monoSwitch = ToggleSwitch(&_commandWord, 10, &_switchState, 10);
    ToggleSwitch dimSwitch = ToggleSwitch(&_commandWord, 1, &_switchState, 1);

    ExclusiveToggleSwitchGroup outputSwitchGroup = ExclusiveToggleSwitchGroup(
        ToggleSwitch(&_commandWord, 6, &_switchState, 6),
        ToggleSwitch(&_commandWord, 5, &_switchState, 5),
        ToggleSwitch(&_commandWord, 4, &_switchState, 4));
    ToggleSwitch subSwitch = ToggleSwitch(&_commandWord, 3, &_switchState, 3);
    ToggleSwitch someSwitch = ToggleSwitch(&_commandWord, 2, &_switchState, 2);

    // `outputShiftRegister` shifts out _switchState to drive indication LEDs
    OutputShiftRegister outputShiftRegister = OutputShiftRegister(&PORTB, PB4, PB3, PB2);
	
    uint16_t lastSwitchState = 0;
    uint8_t lastAttenuationMain = 0;
    int txCounter = 0;
    bool doShiftOut = false;
    bool doI2cTx = false;

    int executionStartupCounter = 0;
    int executionStartupWindow = 4;

    init();

    while (true)
    {
        if (_doScan)
        {
            // hack to fix toggle switches initializing to on, presumably due
            // to spurious input on reset; throw away the first four scans.
            if (executionStartupCounter < executionStartupWindow)
            {
                executionStartupCounter += 1;
            }
            else
            {
                // switches scan `_commandWord` and put the results in `_switchState`.
                inputSwitchGroup.Scan();
                monoSwitch.Scan();
                dimSwitch.Scan();
                outputSwitchGroup.Scan();
                subSwitch.Scan();
                someSwitch.Scan();
            }
        }

        if (_switchState != lastSwitchState)
        {
            lastSwitchState = _switchState;
            doShiftOut = true;
            doI2cTx = true;
        }
        
        uint8_t attenuationMain = lastAttenuationMain;

        if (_attenuationMain != lastAttenuationMain)
        {
            attenuationMain = lastAttenuationMain = _attenuationMain;
            doI2cTx = true;
        }

        if (dimSwitch.Get())
        {
            if (attenuationMain + DIM_OFFSET > 127)
            {
                attenuationMain = 127;
            }
            else
            {
                attenuationMain = attenuationMain+DIM_OFFSET;
            }
        }

        if (doI2cTx || (_doScan && executionStartupCounter < executionStartupWindow))
        {
            uint8_t data[3] = { attenuationMain, static_cast<uint8_t>((lastSwitchState & 0xff00) >> 8), static_cast<uint8_t>(lastSwitchState & 0x00ff) };
            tw_master_transmit(AUDIO_SLAVE_ADDRESS, data, sizeof(data), false);
            doI2cTx = false;
        }

        if (doShiftOut)
        {
            outputShiftRegister.shiftOut(lastSwitchState);
            doShiftOut = false;
        }

        _doScan = false;
    }

    return 0;
}

