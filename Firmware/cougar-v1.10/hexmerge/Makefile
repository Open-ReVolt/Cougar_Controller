CC = gcc
CFLAGS = -Wall -O2

all: hexmerge

hexmerge.o: hexmerge.c
	$(CC) $(CFLAGS) -c hexmerge.c 

hexmerge: hexmerge.o
	$(CC) $(CFLAGS) hexmerge.o -o hexmerge 

clean: 
	rm -f *.o
	rm -f hexmerge
	rm -f core
	rm -f *.core
