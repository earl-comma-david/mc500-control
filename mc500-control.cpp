#ifndef __AVR_ATmega328P__
  #define __AVR_ATmega328P__ 
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "debounce.h"
#include "ToggleSwitch.cpp"
#include "debounce.cpp"
#include "twi-master.h"
#include "twi-master.c"
#include "rotary.h"
#include "rotary.cpp"

void setHigh (volatile uint8_t * reg, uint8_t pin) {
    *reg |= (1 << pin);
}

void setLow (volatile uint8_t * reg, uint8_t pin) {
    *reg &= ~(1 << pin);
}

volatile uint8_t _dataWord = 0b00101000;
volatile uint8_t _gainMain = 0;
volatile uint8_t _gainHP = 0;
volatile bool DoScan;

Rotary _rotaryEncoder;

#define COUNTER 65500;

#define INPUT_1_ID 1;
#define INPUT_2_ID 2;
#define INPUT_3_ID 3;
#define OUTPUT_1_ID 4;
#define OUTPUT_2_ID 5;
#define OUTPUT_3_ID 6;

#define INPUT_1 PB1;
#define INPUT_2 PB2;
#define INPUT_3 PB3;
#define OUTPUT_1 PB4;
#define OUTPUT_2 PC0;
#define OUTPUT_3 PC1;

ISR (TIMER1_OVF_vect)
{
    DoScan = true;
	TCNT1 = COUNTER;
}

ISR (PCINT1_vect)
{
    static uint8_t old_AB = 3;  //lookup table index
    static int8_t encval = 0;   //encoder value  
    static const int8_t enc_states [] PROGMEM = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};  //encoder lookup table

    old_AB = (old_AB<<2) & 0x0f;  //remember previous state
    //old_AB <<=2;  //remember previous state
    uint8_t pincRead = PINC;
    old_AB |= bit_is_clear(pincRead, PINC1)<<PINC1 | bit_is_clear(pincRead, PINC0)<<PINC0;

    encval += pgm_read_byte(&(enc_states[old_AB]));

    //PCIFR |= (1<<PCIF1);

    if( encval > 3 ) {  //four steps forward
        encval = 0;

        _gainMain +=1;
        PORTD |= 1<<PD5;
        _delay_ms(50);
        PORTD &= ~(1<<PD5);
        _delay_ms(50);
        PORTD |= 1<<PD5;
        _delay_ms(50);
        PORTD &= ~(1<<PD5);
    }
    else if( encval < -3 ) {  //four steps backwards
        encval = 0;

        if (_dataWord == 1)
        {
            _gainMain = 0;
        }
        else
        {
            _gainMain -=1;
        }
        PORTD |= 1<<PD5;
        _delay_ms(50);
        PORTD &= ~(1<<PD5);
    }

    //uint8_t rotaryResult = _rotaryEncoder.Process();
    //if (rotaryResult == DIR_CW)
    //{
    //    PORTD |= 1<<PD5;
    //    _delay_ms(5);
    //    PORTD &= ~(1<<PD5);
    //}
    //else if (rotaryResult == DIR_CCW)
    //{
    //    PORTD |= 1<<PD5;
    //    _delay_ms(5);
    //    PORTD &= ~(1<<PD5);
    //}
}

void timer_init(void)
{
    TCNT1 = COUNTER; //63974;   // for 1 sec at 16 MHz	
	TCCR1A = 0x00;
	TCCR1B = (1<<CS10) | (1<<CS12);  // Timer mode with 1024 prescler
	TIMSK1 = (1 << TOIE1) ;   // Enable timer1 overflow interrupt(TOIE1)
}
 
void pin_change_interrupt_init(void)
{
    //EICRA |= 0b00001111;
    //EICRA |= 0b00000101;
    //EIMSK |= 0b00000010;
    PCICR |= 1<<PCIE1;
    PCMSK1 |= (1<<PCINT8) | (1<<PCINT9);
}

void interrupt_init(void)
{
    debounce_init();
    timer_init();
    pin_change_interrupt_init();

	sei();
}

int main (void)
{
    // pullups
    PORTB |= (1<<PB0);
    //PORTC |= (1<<PC0) | (1<<PC1);
    PORTD |= (1<<PD6) | (1<<PD4) | (1<<PD3);

    // outputs
    DDRD |= (1<<PD5);

    // inputs
    DDRB &= ~(1 << PINB0);
    DDRC &= ~(1 << PINC0);
    DDRC &= ~(1 << PINC1);
    DDRD &= ~(1 << PIND6);

    _dataWord = 0b01010000;

    interrupt_init();
    tw_init(TW_FREQ_250K, false);

    ExclusiveToggleSwitchGroup inputSwitchGroup = ExclusiveToggleSwitchGroup(
        ToggleSwitch(&PINB, PINB0, &_dataWord, 6, 1),
        ToggleSwitch(&PIND, PIND6, &_dataWord, 5, 4));

    ExclusiveToggleSwitchGroup outputSwitchGroup = ExclusiveToggleSwitchGroup(
        ToggleSwitch(&PIND, PIND4, &_dataWord, 4, 4),
        ToggleSwitch(&PIND, PIND3, &_dataWord, 3, 4));
    
    _rotaryEncoder = Rotary(&PINC, PINC0, &PINC, PINC1);

    //ToggleSwitch monoSwitch = ToggleSwitch(&PINB, PINB0, &PORTB, PB5);
    //ToggleSwitch dimSwitch = ToggleSwitch(&PINC, PINC6, &PORTD, PD0);
	
    uint8_t lastDataWord = 0;
    int txCounter = 0;
    uint8_t waveform[4] = { /*0x80,*/ 0x40, 0x20, 0x10, 0x08 };
    while(1)
    {
        if (DoScan)
        {
            DoScan = false;
            debounce();

            inputSwitchGroup.Scan();
            outputSwitchGroup.Scan();
            ////monoSwitch.Scan();
            ////dimSwitch.Scan();

            //if (_dataWord == lastDataWord)
            //{
            //    continue;
            //}

            //for (int i = 0; i < _dataWord; i++)
            //{
                //PORTD |= 1<<PD5;
                //_delay_ms(175);
                //PORTD &= ~(1<<PD5);
            //}

            if (txCounter++ % 8 == 0)
            {
                if (_dataWord == lastDataWord)
                {
                    continue;
                }

                lastDataWord = _dataWord;

                ret_code_t error_code;
                uint8_t data[1] = { _dataWord };
                error_code = tw_master_transmit(0x10, data, sizeof(data), false);
            }
        }
    }

    return 0;
}

