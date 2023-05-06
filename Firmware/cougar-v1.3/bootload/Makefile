# AVR-GCC Makefile
# Adapted to WinAVR 20030913
#**************************************************************
.SUFFIXES:
.SUFFIXES:	.s .c .o .asm .cof .elf .eep .hex

#--- define some variables based on the AVR base path in $(AVR)
CC	= avr-gcc
AS	= avr-gcc
RM	= rm -f
RN	= mv
CP      = cp
BIN	= avr-objcopy
OBJCOPY = avr-objcopy
INCDIR	= .

#If all other steps compile ok then echo "Errors: none".
#Necessary for AVR Studio 3.x IDE to understand that everything went ok.
DONE    = @echo Errors: none


#--- default mcu type
MCU = atmega8


#--- default compiler flags -ahlmsn
CPFLAGS = -Os -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -Wall -Wstrict-prototypes -mmcu=$(MCU) -Wa,-adhlns=$(<:.c=.lst)

#--- default assembler flags 
ASFLAGS = -mmcu=$(MCU) -Wa,-mmcu=$(MCU),-gstabs

#--- default linker flags
#start=.text=address is specified in bytes - documentation memory map is in words!
#LDFLAGS = -Wl,--section-start=.text=0x1800,-Map=$(<:.o=.map),--cref -mmcu=$(MCU)
LDFLAGS = -Wl,--section-start=.text=0x1c00,-Map=$(<:.o=.map),--cref -mmcu=$(MCU)

#--- output format can be srec (Motorola), ihex (Intel HEX)
ROMFORMAT 	= ihex
EEPROMFORMAT    = ihex              # AVR Studio needs Intel HEX format


#--- compile: instructions to create assembler and/or object files from C source
%o : %c
	$(CC) -c $(CPFLAGS) -I$(INCDIR) $< -o $@

%s : %c
	$(CC) -S $(CPFLAGS) -I$(INCDIR) $< -o $@

%: %.c
	@echo "Error: target $@ not defined in Makefile"
	@exit 1

#--- assemble: instructions to create object file from assembler source
%o : %s
	$(AS) -x assembler-with-cpp $(ASFLAGS) -I$(INCDIR) -c $< -o $@

%o : %asm
	$(AS) -x assembler-with-cpp $(ASFLAGS) -I$(INCDIR) -c $< -o $@

#--- link: instructions to create elf output file from object files
%elf:	%o
	$(CC) $^ $(LIB) $(LDFLAGS) -o $@

#--- create AVR Studio cof file from elf output for use in debugging / simulating in AVR Studio 4.08 only
COFFCONVERT=$(OBJCOPY) --debugging \
	--change-section-address .data-0x800000 \
	--change-section-address .bss-0x800000 \
	--change-section-address .noinit-0x800000 \
	--change-section-address .eeprom-0x810000 
%cof: %elf
	$(COFFCONVERT) -O coff-ext-avr $< $@



#--- create flash and eeprom bin file (ihex, srec) from elf output file
%hex: %elf
	$(BIN) -O $(ROMFORMAT) -R .eeprom $< $@
	./genautocrc bootload.hex

%eep: %elf
	$(BIN) -j .eeprom --set-section-flags=.eeprom="alloc,load" --change-section-lma .eeprom=0 -O $(EEPROMFORMAT) $< $(@:.elf=.eep)


#If all other steps compile ok then echo "Errors: none".
#Necessary for AVR Studio 3.x to understand that everything went ok.
%ok:
	@echo "Errors: none" 



.PHONY	: begin
begin:	all


.phony:	clean
clean:
	$(RM) *.hex
	$(RM) *.eep
	$(RM) *.cof
	$(RM) *.elf
	$(RM) *.o
	$(RM) *.s
	$(RM) *.lst
	$(RM) *.map
	$(RM) autocrc.gen


#*************************** add your projects below **************************************


all:	bootload
	$(DONE)


#bootload:	bootload.hex bootload.cof
bootload:	bootload.hex

	
bootload.elf:	bootload.o misc.o

bootload.o:	bootload.s

bootload.s:	bootload.c autocrc.h

misc.o: 	misc.asm



#******************* write fuse and program commands below *********************

fuses:
	avrprog -writefuse ca2f

program:
	avrprog -blockmode32 -erase -file bootload.hex -program -verify -crc
