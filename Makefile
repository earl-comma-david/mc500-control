TARGET = mc500-control
SRCS=src/main.cc
OBJS   = $(SRCS:.cc=.o)

MCU   = atmega328p
F_CPU = 16000000UL  
BAUD  = 9600UL

FLASH_PORT = /dev/cu.usbserial-3
FLASH_BAUD = 57600

CXXFLAGS = -std=gnu++17 -Wall -g -Os -mmcu=$(MCU) -DF_CPU=$(F_CPU) -DBAUD=$(BAUD) -I.
LDFLAGS  = -mmcu=$(MCU)

CXX = avr-g++
OBJCOPY = avr-objcopy
AVRDUDE  = avrdude

all: $(TARGET).hex

%.o: %.cc
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(TARGET).elf: $(OBJS)
	$(CXX) $(LDFLAGS) -o $@ $^

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

flash: $(TARGET).hex
	$(AVRDUDE) -c arduino -p $(MCU) -P $(FLASH_PORT) -b $(FLASH_BAUD) -U flash:w:$<:i

flash-read:
	$(AVRDUDE) -c arduino -p $(MCU) -P $(FLASH_PORT) -b $(FLASH_BAUD) -U flash:r:$(TARGET).device.hex:i

clean:
	rm -f $(OBJS) $(TARGET).elf $(TARGET).hex
