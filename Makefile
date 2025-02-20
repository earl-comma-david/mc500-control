TARGET = mc500-control
SRCS=src/twi-master.c src/uart.c src/main.cc

MCU   = atmega328p
F_CPU = 16000000UL  
BAUD  = 9600UL
CPPFLAGS = -DF_CPU=$(F_CPU) -DBAUD=$(BAUD) -I. 

CC = avr-g++
OBJCOPY = avr-objcopy
CFLAGS=-std=c99 -Wall -g -Os -mmcu=${MCU} -DF_CPU=${F_CPU} -I.


all:
	${CC} ${CFLAGS} -o ${TARGET}.o ${SRCS}
	${OBJCOPY} -j .text -j .data -O ihex ${TARGET}.o ${TARGET}.hex

flash:
	avrdude -c arduino -p ${MCU} -P /dev/cu.usbserial-3 -b57600 -U flash:w:${TARGET}.hex:i

clean:
	rm -f *.o *.hex
