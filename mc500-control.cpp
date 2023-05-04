#ifndef __AVR_ATmega328P__
  #define __AVR_ATmega328P__ 
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdbool.h>
#include <string.h> 
#include <util/delay.h>
#include "debounce.h"
#include "debounce.cpp"
#include "rotary.h"
#include "rotary.cpp"
#include "ShiftRegister.cpp"
#include "InputShiftRegister.cpp"
#include "ToggleSwitch.cpp"
#include "twi-master.h"
#include "twi-master.c"
#include "config.h"

#define COUNTER 65500;
#define AUDIO_SLAVE_ADDRESS 0x10
#define DIM_OFFSET 16

volatile uint16_t _commandWord = 0;
volatile uint16_t _switchState = 0b0100000001000000;
volatile uint8_t _attenuationHP = 127;
volatile uint8_t _attenuationMain = 127;
volatile bool IsDim = false;
volatile bool DoScan;

//uint16_t Debounced_State; // Debounced state of the switches

InputShiftRegister _inputShiftRegister;
Debouncer _debouncer;

ISR (TIMER1_OVF_vect)
{
    DoScan = true;

	TCNT1 = COUNTER;

    _debouncer.UpdateState(_inputShiftRegister.read());
}

void rotary_handler_0()
{
    static uint8_t state = 3;
    static int8_t encval = 0;  
    static const int8_t enc_states [] PROGMEM = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};

    state = (state<<2) & 0x0f;
    uint8_t pinbRead = PINB;
    state |= bit_is_clear(pinbRead, PINB0)<<1 | bit_is_clear(pinbRead, PINB1);

    encval += pgm_read_byte(&(enc_states[state]));

    if( encval > 3 ) //four steps forward
    {
        encval = 0;

        if (_attenuationMain == 0)
        {
            return;
        }

        _attenuationMain -=1;
    }
    else if( encval < -3 ) //four steps backwards
    {
        encval = 0;

        if (_attenuationMain == 127)
        {
            return;
        }

        _attenuationMain +=1;
    }
}

void rotary_handler_1()
{
    static uint8_t state = 3;
    static int8_t encval = 0;  
    static const int8_t enc_states [] PROGMEM = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};

    state = (state<<2) & 0x0f;
    uint8_t pincRead = PINC;
    state |= bit_is_clear(pincRead, PINC1)<<PINC1 | bit_is_clear(pincRead, PINC0)<<PINC0;

    encval += pgm_read_byte(&(enc_states[state]));

    if( encval > 3 ) //four steps forward
    {
        encval = 0;

        if (_attenuationMain == 0)
        //if (_attenuationHP == 0)
        {
            return;
        }

        _attenuationMain -=1;
        //_attenuationHP -=1;
    }
    else if( encval < -3 ) //four steps backwards
    {
        encval = 0;

        if (_attenuationMain == 127)
        //if (_attenuationHP == 127)
        {
            return;
        }

        _attenuationMain +=1;
        //_attenuationHP +=1;
    }
}

ISR (PCINT0_vect)
{
    rotary_handler_0();
}

ISR (PCINT1_vect)
{
    rotary_handler_1();
}

void timer_init(void)
{
    TCNT1 = COUNTER; // for 1 sec at 16 MHz	
	TCCR1A = 0x00;
	TCCR1B = (1<<CS10) | (1<<CS12);  // Timer mode with 1024 prescler
	TIMSK1 = (1 << TOIE1) ;
}
 
void pin_change_interrupt_init(void)
{
    PCICR |= (1<<PCIE1) | (1<<PCIE0);
    PCMSK1 |= (1<<PCINT8) | (1<<PCINT9);
    PCMSK0 |= (1<<PCINT1) | (1<<PCINT0);
}

void interrupt_init(void)
{
    debounce_init();
    timer_init();
    pin_change_interrupt_init();
}

void init_io_pins(void)
{
    // pullups
    PORTB |= (1<<PB4) | (1<<PB3) | (1<<PB2) | (1<<PB1) | (1<<PB0);
    PORTC |= (1<<PC5) | (1<<PC4);
    //PORTD |= (1<<PD7) | (1<<PD6) | (1<<PD5);// /*| (1<<PD4) | (1<<PD3)*/ | (1<<PD2);

    // outputs
    DDRC |= (1<<PC3);
    DDRC |= (1<<PC2);
    DDRD |= (1<<PD1);

    DDRD |= (1<<PD2);
    DDRD |= (1<<PD3);

    // inputs
    DDRB &= ~(1 << PINB5); // TODO: this seems to be necessary...y tho?
    DDRB &= ~(1 << PINB4);
    DDRB &= ~(1 << PINB3);
    DDRB &= ~(1 << PINB2);
    DDRB &= ~(1 << PINB1);
    DDRB &= ~(1 << PINB0);
    DDRC &= ~(1 << PINC1);
    DDRC &= ~(1 << PINC0);
    DDRD &= ~(1 << PIND7);
    DDRD &= ~(1 << PIND6);
    DDRD &= ~(1 << PIND5);
    DDRD &= ~(1 << PIND4);
    //DDRD &= ~(1 << PIND3);
    //DDRD &= ~(1 << PIND2);
}

void init(void)
{
    interrupt_init();
    tw_init(TW_FREQ_250K, true);
    init_io_pins();

    UCSR0B = 0;

    _inputShiftRegister = InputShiftRegister();
    _debouncer = Debouncer();

	sei();
}

int main (void)
{
    init();
    
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

    ShiftRegister shiftRegister = ShiftRegister();
	
    uint16_t lastSwitchState = 0;
    uint8_t lastAttenuationMain = 0;
    int txCounter = 0;
    bool doShift = false;
    bool doI2cTx = false;

    int i = 0;

    while (true)
    {
        if (DoScan)
        {
            DoScan = false;

            _commandWord = _debouncer.Debounce();

            inputSwitchGroup.Scan();
            monoSwitch.Scan();
            dimSwitch.Scan();
            outputSwitchGroup.Scan();
            subSwitch.Scan();

            if (txCounter++ % 16 == 0)
            {
                if (_switchState != lastSwitchState)
                {
                    lastSwitchState = _switchState;
                    doShift = true;
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

                if (doI2cTx)
                {
                    uint8_t data[3] = { attenuationMain, static_cast<uint8_t>((lastSwitchState & 0xff00) >> 8), static_cast<uint8_t>(lastSwitchState & 0x00ff) };
                    tw_master_transmit(AUDIO_SLAVE_ADDRESS, data, sizeof(data), false);
                    doI2cTx = false;
                }

                if (doShift)
                {
                    shiftRegister.shiftOut(lastSwitchState);
                    doShift = false;
                }
            }
        }
    }

    return 0;
}

