# AVR-GCC Makefile
PROJECT=qi
SOURCES=qi.c
CC=avr-gcc
OBJCOPY=avr-objcopy
MMCU=attiny13
COM=COM2	
CFLAGS=-mmcu=$(MMCU) -Wall -std=gnu11

DEVICE=t13
PROGRAMMER_TYPE=iseavrprog
PROGRAMMER_PORT=usb

AVRDUDE = avrdude -P $(PROGRAMMER_PORT) -c $(PROGRAMMER_TYPE) -p $(DEVICE) -B32
AVRDUDE_SLOW = avrdude -P $(PROGRAMMER_PORT) -c $(PROGRAMMER_TYPE) -p $(DEVICE) -B32


$(PROJECT).hex: $(PROJECT).out
	$(OBJCOPY) -j .text -j .data -O ihex $(PROJECT).out $(PROJECT).hex

$(PROJECT).out: $(SOURCES)
	$(CC) $(CFLAGS) -Os -I./ -o $(PROJECT).out $(SOURCES)

flash: $(PROJECT).hex
	$(AVRDUDE) -U flash:w:$(PROJECT).hex:i

fuse: $(PROJECT).hex
	$(AVRDUDE_SLOW) -U lfuse:w:0x76:m -U hfuse:w:0xFF:m
	
clean:
	del -f $(PROJECT).out
	del -f $(PROJECT).hex
	del -f $(PROJECT).s
