ASFLAGS	= -I. -x assembler-with-cpp -mmcu=$(MCU_TARGET)
CFLAGS	= -I. -Wall -g -mmcu=$(MCU_TARGET) -std=c99 -DF_CPU=12000000UL
LDFLAS	=
LIBS	=

CC	= avr-gcc
OBJCOPY	= avr-objcopy

MCU_TARGET = atmega8

NAME	= usbavr-temp
SOURCES	= usb.c onewire/onewire.c usbdrv/usbdrv.c usbdrv/usbdrvasm.S

OBJECTS	:= $(patsubst %.c,%.o,$(patsubst %.S,%.o,$(SOURCES))) 

all: build

build: $(NAME).hex

clean:
	rm -f $(OBJECTS)
	rm -f $(NAME).elf
	rm -f $(NAME).hex

flash:
	avrdude -c usbasp -p m8 -U flash:w:$(NAME).hex

$(NAME).elf: $(OBJECTS)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

%.o: %.c
	$(CC) -Os $(CFLAGS) -c -o $@ $<

%.o: %.S
	$(CC) $(ASFLAGS) -c -o $@ $<
