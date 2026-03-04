#ifndef __AVR_ATmega328P__
  #define __AVR_ATmega328P__ 
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdbool.h>
#include <string.h> 
#include "../config.h"
#include "../include/debouncer.h"
#include "../include/rotary_encoder.h"
#include "../include/input_shift_register.h"
#include "../include/output_shift_register.h"
#include "../include/toggle_switch.h"

#define AUDIO_SLAVE_ADDRESS 0x10
#define DIM_OFFSET 16

volatile uint16_t _commandWord = 0;
volatile uint16_t _switchState = 0b0100000001000000;
volatile uint8_t _displayState = 0;//b11111111;
volatile uint8_t _attenuationPhones = 127;
volatile uint8_t _attenuationMain = 127;
volatile bool _doScan;

InputShiftRegister _inputShiftRegister;
Debouncer _debouncer;
RotaryEncoder _rotaryEncoderMain;
RotaryEncoder _rotaryEncoderPhones;

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
    _rotaryEncoderPhones.Scan();
}

void timer_init(void)
{
    // F_int = F_clk/(2*prescalar*256)
    // F_int = 16000000/(64*256)
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
    // I2C pullups
    //PORTC |= (1<<PC5) | (1<<PC4);

    // outputs
    DDRB |= (1<<PB4);
    DDRB |= (1<<PB3);
    DDRB |= (1<<PB2);
    DDRC |= (1<<PC2);
    DDRC |= (1<<PC3);
    DDRD |= (1<<PD2);
    DDRD |= (1<<PD3);
    DDRD |= (1<<PD5);
    DDRD |= (1<<PD6);
    DDRD |= (1<<PD7);

    // inputs
    DDRB &= ~(1 << PINB5); // TODO: this seems to be necessary...y tho?
    DDRB &= ~(1 << PINB1);
    DDRB &= ~(1 << PINB0);
    DDRC &= ~(1 << PINC1);
    DDRC &= ~(1 << PINC0);
    DDRD &= ~(1 << PIND4);
}

void init(void)
{
    timer_init();
    pin_change_interrupt_init();
    init_io_pins();

    UCSR0B = 0;

    _rotaryEncoderMain = RotaryEncoder(&PINB, PINB1, PINB0, &_attenuationMain);
    _rotaryEncoderPhones = RotaryEncoder(&PINC, PINC1, PINC0, &_attenuationMain);//&_attenuationPhones);
    _inputShiftRegister = InputShiftRegister(&PIND, PIND4, &PORTD, PD2, PD3);
    _debouncer = Debouncer();

	sei();
}

int main (void)
{
    // 
    ExclusiveToggleSwitchGroup<uint16_t, uint16_t> inputSwitchGroup = ExclusiveToggleSwitchGroup<uint16_t, uint16_t>(
        ToggleSwitch<uint16_t, uint16_t>(&_commandWord, 15, &_switchState, 14),
        ToggleSwitch<uint16_t, uint16_t>(&_commandWord, 14, &_switchState, 13),
        ToggleSwitch<uint16_t, uint16_t>(&_commandWord, 13, &_switchState, 12),
        ToggleSwitch<uint16_t, uint16_t>(&_commandWord, 12, &_switchState, 11));
    ToggleSwitch<uint16_t, uint16_t> monoSwitch = ToggleSwitch<uint16_t, uint16_t>(&_commandWord, 11, &_switchState, 10);
    ToggleSwitch<uint16_t, uint16_t> dimSwitch = ToggleSwitch<uint16_t, uint16_t>(&_commandWord, 10, &_switchState, 9);

    ExclusiveToggleSwitchGroup<uint16_t, uint16_t> outputSwitchGroup = ExclusiveToggleSwitchGroup<uint16_t, uint16_t>(
        ToggleSwitch<uint16_t, uint16_t>(&_commandWord, 7, &_switchState, 6),
        ToggleSwitch<uint16_t, uint16_t>(&_commandWord, 6, &_switchState, 5),
        ToggleSwitch<uint16_t, uint16_t>(&_commandWord, 5, &_switchState, 4));
    ToggleSwitch<uint16_t, uint16_t> sysSwitch = ToggleSwitch<uint16_t, uint16_t>(&_commandWord, 4, &_switchState, 7);

    // `outputShiftRegister` shifts out _switchState to drive IO and attenuation Relays 
    OutputShiftRegister ioSR = OutputShiftRegister(&PORTB, PB4, PB3, PB2);
    // `outputSR` shifts out _switchState to drive IO Relays 
    OutputShiftRegister displaySR = OutputShiftRegister(&PORTD, PD7, PD6, PD5);
	
    uint16_t lastSwitchState = 0;
    uint8_t lastAttenuationMain = 0;
    uint8_t lastAttenuationPhones = 0;
    bool doShiftOutIo = false;
    bool doShiftOutDisplay = false;

    int executionStartupCounter = 0;
    int executionStartupWindow = 4;

    uint16_t fooCounter = 0;

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
                sysSwitch.Scan();

                if (executionStartupCounter++ % 8 == 0) {
                    _displayState += 1;

                    if (_displayState & 8) {

                        PORTC |= (1<<PC3);
                        PORTC &= ~(1<<PC2);
                    } else {
                        PORTC &= ~(1<<PC3);
                        PORTC |= (1<<PC2);
                    }
                }
                doShiftOutDisplay = true;
            }
        }


        if (_switchState != lastSwitchState)
        {
            lastSwitchState = _switchState;
            doShiftOutIo = true;
        }
        
        uint8_t attenuationMain = lastAttenuationMain;
        uint8_t attenuationPhones = lastAttenuationPhones;

        if (_attenuationMain != lastAttenuationMain)
        {
            attenuationMain = lastAttenuationMain = _attenuationMain;
            doShiftOutIo = true;
        }

        if (_attenuationPhones != lastAttenuationPhones)
        {
            attenuationPhones = lastAttenuationPhones = _attenuationPhones;
            doShiftOutIo = true;
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
            if (attenuationPhones + DIM_OFFSET > 127)
            {
                attenuationPhones = 127;
            }
            else
            {
                attenuationPhones = attenuationPhones+DIM_OFFSET;
            }
        }

        if (doShiftOutIo)
        {
            ioSR.shiftOut((uint8_t) (127 - attenuationMain), lastSwitchState);
            // TODO:
            //outputShiftRegister.shiftOut((uint8_t) (127 - attenuationMain), (uint8_t) (127 - attenuationPhones), lastSwitchState);
            // or
            //outputShiftRegister.shiftOut(inputSwitchState, (uint8_t) (127 - attenuationMain), (uint8_t) (127 - attenuationPhones), outputSwitchState);
            doShiftOutIo = false;
        }

        if (doShiftOutDisplay)
        {
            displaySR.shiftOut(_displayState);
            doShiftOutDisplay = false;
        }

        _doScan = false;
    }

    return 0;
}

