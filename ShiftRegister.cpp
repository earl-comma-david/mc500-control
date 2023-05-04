#include <avr/io.h>
#include <util/delay.h>

//#define PIN_SER      PC3
//#define PIN_LATCH     PC2
//#define PIN_SRCLK    PC1
#define PIN_SER      PC3
#define PIN_LATCH     PD1
#define PIN_SRCLK    PC2

// NOTE: pin 10 (SRCLR') on register needs to go to VCC
// NOTE: pin 13 (OE') on register needs to go to GND

// write digital "high" to pin <pn> on portb
#define GOHI(pn) PORTC |= (1<<pn)
#define GOHI_LATCH(pn) PORTD |= (1<<pn)

// write digital "low" to pin <pn> on portb
#define GOLO(pn) PORTC &= ~(1<<pn)
#define GOLO_LATCH(pn) PORTD &= ~(1<<pn)

void put(uint8_t);
void putBit(uint8_t);
void latch();
void cycleClock();
void longDelay(uint16_t);
void setup();
uint8_t rotateLeft(uint8_t);
uint8_t rotateRight(uint8_t);

class ShiftRegister 
{
  public:

    ShiftRegister() {}

    //void setup() {
    //    // set output pins on portb (the only port on attiny85)
    //    DDRB |= (1 << PIN_SER) | (1 << PIN_SRCLK) | (1 << PIN_LATCH);
    //}

    //void longDelay(uint16_t ms) {
    //    for(ms /= 10; ms>0; ms--) _delay_ms(10);
    //}

    /*
    * Shifts bits left, but wraps the left-most bit back around
    */
    uint8_t rotateLeft(uint8_t toRotate) {
        uint8_t carry = (toRotate & 0b10000000) == 0b10000000;
        return (toRotate << 1 | carry);
    }

    /*
    * Shifts bits right, but wraps the right-most bit back around
    */
    uint8_t rotateRight(uint8_t toRotate) {
        uint8_t carry = ( toRotate & 1 ) ? 0b10000000 : 0;
        return (toRotate >> 1 | carry);
    }

    /*
    * Writes a byte out serially to PIN_SER
    */
    void shiftOut(uint16_t toPut) {
        uint8_t i;
        for(i = 0; i < 16; i++) {
            putBit(toPut & 1);
            toPut >>= 1;
        }

        latch();
    }

    /*
    * Writes a bit out serially to PIN_SER
    */
    void putBit(uint8_t bit) {
        if(bit == 0) GOLO(PIN_SER);
        else         GOHI(PIN_SER);

        cycleClock();
    }

    /*
    * Tells the shift register to output all the bits it's read
    */
    void latch() {
        //GOHI(PIN_LATCH);
        //GOLO(PIN_LATCH);
        GOHI_LATCH(PIN_LATCH);
        GOLO_LATCH(PIN_LATCH);
    }

    /*
    * Tells the shift register to read a bit
    */
    void cycleClock() {
        GOHI(PIN_SRCLK);
        GOLO(PIN_SRCLK);
    }
};
