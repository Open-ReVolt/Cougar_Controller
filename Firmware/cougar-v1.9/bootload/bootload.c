/*
Boot Loader for ATMEGA8
UART implemented in software so that any pins on the micro can be used

Fran Sabolich
*/

#include <avr/io.h>
#include <avr/boot.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <util/crc16.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// now using automatically generated CRC file
#include "autocrc.h"

// choose if we want EEprom support - comes at expense of less space for application
// be sure to select appropriate LDFLAGS with correct --section-start in Makefile
// program start address
//#define EE_SUPPORT

#ifdef EE_SUPPORT
	#define PROGSTART 0x1800
#else
	#define PROGSTART 0x1c00
#endif

//#define T_WT 3000										// for 1MHz
//#define T_WT 24000									// for 8MHz
//#define T_WT 44237									// for 14.746MHz
#define T_WT 48000										// for 16MHz

#define CMD_GET_SPM_PAGESIZE 0x0001
#define CMD_READ_FLASH 0x0002
#define CMD_READ_EE 0x0003
#define CMD_WRITE_FLASH 0x0004
#define CMD_WRITE_EE 0x0005
#define CMD_RUN 0x0006
#define CMD_OK 0x0100
#define CMD_FAIL 0x0200

#define CMD_HEADSIZE 6
#define CMD_MINSIZE 8

// serial command buffer
typedef struct {
	unsigned command;
	unsigned address;
	unsigned nbytes;
	unsigned char buffer[258];
} command_buffer;

void su_init(void);
int su_getchar(void);
void su_putchar(unsigned char);
unsigned su_waitpulse(unsigned start_count);
void do_reboot(void);

command_buffer cmd;

// calc CRC for program (firmware)
unsigned int calc_prog_crc(unsigned nbytes)
{
	unsigned n, crc;
	
	crc = 0xffff;
	for (n = PROGSTART; n < nbytes; n++) {
		crc = _crc_ccitt_update (crc, pgm_read_byte(n));
	}
	return(crc);
}

// calc ccitt CRC on buffer
unsigned int calc_crc(unsigned char *buf, unsigned nbytes)
{
	unsigned n, crc;
	
	crc = 0xffff;
	for (n = 0; n < nbytes; n++) {
		crc = _crc_ccitt_update (crc, *buf++);
	}
	return(crc);
}

// put buffer to software uart
void su_putbuf(char *buf, unsigned nbytes)
{
	unsigned x;
	
	for (x = 0; x < nbytes; x++) su_putchar(*buf++);
}

// get bytes from software uart (with timeout)
unsigned su_getbuf(char *buf, unsigned maxbytes)
{
	int x;
	unsigned nb;
	
	nb = 0;
	while (nb < maxbytes) {
		x = su_getchar();
		if (x < 0) break;
		*buf++ = x;
		nb++;
	}
	return(nb);
}

// program flash page
void program_flash_page(unsigned page, unsigned char *buf)
{
	unsigned i, w;

	// erase page
    boot_page_erase(page);
	boot_spm_busy_wait();
	eeprom_busy_wait();
	// fill buffer
	for (i=0; i<SPM_PAGESIZE; i+=2) {
		// Set up little-endian word.
		w = *buf++;
		w |= (*buf++) << 8;
		boot_page_fill((unsigned long)(page + i), w);
	}
	// write buffer to page
	boot_page_write(page);
	boot_spm_busy_wait();
	// Reenable RWW-section again. We need this if we want to jump back
	// to the application after bootloading.
	boot_rww_enable();
}

// listen to master
void be_slave(void)
{
	unsigned x, y, z;

	while (1) {
		x = su_getbuf((char *)&cmd, sizeof(cmd));
		if ((x >= CMD_MINSIZE)) {
			// possible command packet
			y = calc_crc((unsigned char *)&cmd, x - 2);
			z = *(unsigned *)(((char *)&cmd) + (x - 2));
			if (z == y) {
				// command packet with good CRC
				y = 0;
				switch ((unsigned char)cmd.command) {
					case CMD_GET_SPM_PAGESIZE:
						y = 2;
						*(unsigned *)cmd.buffer = SPM_PAGESIZE;
						break;
					case CMD_READ_FLASH:
						y = cmd.nbytes;
						memcpy_P(cmd.buffer, (void *)cmd.address, y);
						break;
					case CMD_WRITE_FLASH:
						if (cmd.nbytes == SPM_PAGESIZE) {
							if (cmd.address <= ((unsigned)PROGSTART - (unsigned)SPM_PAGESIZE)) {
								// ok, we won't write over ourself - do it
								program_flash_page(cmd.address, cmd.buffer);
							}
						}
						break;
					case CMD_RUN:
						return;
					#ifdef EE_SUPPORT	
					case CMD_READ_EE:
						y = cmd.nbytes;
						eeprom_read_block(cmd.buffer, (void *)cmd.address, y);
						break;
					case CMD_WRITE_EE:
						eeprom_write_block(cmd.buffer, (void *)cmd.address, cmd.nbytes);
						break;
					#endif
				}
				cmd.command |= CMD_OK;
				x = calc_crc((unsigned char *)&cmd, y + CMD_HEADSIZE);
				*(unsigned *)(&cmd.buffer[y]) = x;
				su_putbuf((char *)&cmd, y + CMD_MINSIZE);
			}
		}
	}
}

// execution starts here
int main(void)
{
	unsigned char lp, rv;
	char str[32];
		
#ifdef crc_address
	unsigned crc1, crc2;
	
	crc1 = pgm_read_word(crc_address);		// read program CRC
	crc2 = calc_prog_crc(crc_address);		// read program CRC
	if (crc1 != crc2) {
		// program CRC error
		do_reboot();						// attempt to start application code
	}
#endif
	su_init();
	lp = 0; rv = 1;
	while (lp++ < 20) {
		if (su_getchar() == ' ') {
			// got space character - probably bootloader
			rv = 0; break;
		}
	}
	if (!rv) {
		su_putbuf("OSCCAL=0\r\n", 10);
		// wait 20 milliseconds
		su_getchar(); su_getchar();
		// wait for "MON" command
		for (lp = 0; lp < 20; lp++) {
			rv = su_getbuf(str, sizeof(str) - 1);
			if (rv > 0) {
				str[rv] = 0;
				if (!strcmp("MON\r\n", str)) break;
				else rv = 0;
			}
		}
		if (rv) {
			// connected
			su_putbuf("OK\r\n", 4);
			be_slave();
		}
	}
	// no serial connection - jump to application code
	do_reboot();
	return(0);
}
