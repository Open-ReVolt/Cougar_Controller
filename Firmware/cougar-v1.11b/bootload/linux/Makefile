CC = gcc
CFLAGS = -Wall -O2

all: avrboot

avrboot.o: avrboot.c
	$(CC) $(CFLAGS) -c avrboot.c 

avrboot: avrboot.o
	$(CC) $(CFLAGS) avrboot.o -o avrboot 

clean: 
	rm -f *.o
	rm -f avrboot
	rm -f core
	rm -f *.core
	
install:
	cp -f avrboot /usr/bin/avrboot
