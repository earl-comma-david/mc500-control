TARGET = mc500-control
SRCS=mc500-control.cpp

#MCU   = m328p
MCU   = atmega328p
F_CPU = 16000000UL  
BAUD  = 9600UL
CPPFLAGS = -DF_CPU=$(F_CPU) -DBAUD=$(BAUD) -I. 

CC = avr-g++
OBJCOPY = avr-objcopy
CFLAGS=-std=c99 -Wall -g -Os -mmcu=${MCU} -DF_CPU=${F_CPU} -I.
# CFLAGS += -funsigned-char -funsigned-bitfields 
# CFLAGS += -fpack-struct -fshort-enums 
# CFLAGS += -ffunction-sections -fdata-sections 


all:
	${CC} ${CFLAGS} -o ${TARGET}.o ${SRCS}
	${OBJCOPY} -j .text -j .data -O ihex ${TARGET}.o ${TARGET}.hex

# "/Users/earl/Library/Arduino15/packages/arduino/tools/avrdude/6.3.0-arduino17/bin/avrdude" "-C/Users/earl/Library/Arduino15/packages/arduino/tools/avrdude/6.3.0-arduino17/etc/avrdude.conf" -v -V -patmega328p -carduino "-P/dev/cu.usbserial-1" -b57600 -D "-Uflash:w:/private/var/folders/m2/xysfsq9540x63rwhkn89w13w0000gn/T/arduino-sketch-C58A843F5ED5AE9B4017FBCB01018F26/Blink.ino.hex:i"
flash:
	avrdude -c arduino -p ${MCU} -P /dev/cu.usbserial-2 -b57600 -U flash:w:${TARGET}.hex:i
	#/Users/earl/Library/Arduino15/packages/arduino/tools/avrdude/6.3.0-arduino17/bin/avrdude -C/Users/earl/Library/Arduino15/packages/arduino/tools/avrdude/6.3.0-arduino17/etc/avrdude.conf -v -v -v -v -c arduino -p ${MCU} -P /dev/cu.usbserial-1 -b57600 -U flash:w:${TARGET}.hex:i
	#avrdude -v -v -v -v -c arduino -p ${MCU} -P /dev/cu.usbserial-1 -b115200 -U flash:w:${TARGET}.hex:i

	#avrdude -p ${MCU} -c usbtiny -U flash:w:${TARGET}.hex:i -F
	#avrdude -p ${MCU} -c usbtiny -P /dev/cu.usbserial-AB0O4PX8 -U flash:w:${TARGET}.hex:i -F

clean:
	rm -f *.o *.hex
