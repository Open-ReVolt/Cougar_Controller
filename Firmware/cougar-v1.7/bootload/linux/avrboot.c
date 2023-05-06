/*
  AVR boot loader communication code (Linux)

  Fran Sabolich
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <termios.h>
#include <time.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#define AVR_CMD_DELAY 10000

#define PACKED __attribute__((packed))

#define CMD_MAXTIME 5000000

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
	unsigned short command;
	unsigned short address;
	unsigned short nbytes;
	unsigned char buffer[258];
} PACKED command_buffer;

#define max_progsize 0x4000

unsigned short prog[max_progsize];
unsigned char prog_val[max_progsize];
int ihex_ok = 0;

int program = 0;
int verify = 0;
int fname_argno = 0;
int eeread_addr = 0;
int eeread_nb = 0;
int flashread_addr = 0;
int flashread_nw = 0;
int run = 0;
int restart = 0;
int crc = 0;

unsigned short crc_ccitt_update (unsigned short crc, unsigned char data)
{
	data ^= crc & 0xff;
	data ^= data << 4;
	return ((((unsigned short)data << 8) | (crc >> 8)) ^ (unsigned char)(data >> 4) 
		^ ((unsigned short)data << 3));
}

// calc ccitt CRC on buffer
unsigned short calc_crc(unsigned char *buf, unsigned nbytes)
{
	unsigned n;
	unsigned short crc;
	
	crc = 0xffff;
	for (n = 0; n < nbytes; n++) {
		crc = crc_ccitt_update (crc, *buf++);
	}
	return(crc);
}

int read_ihex(char *fname)
{
	int x, y, nr, chk, val, adr, adrmax, adrmin;
	FILE *fh;
	char buf[1024+2];
	char data[8];
	char len[8];
	char addr[8];
	char rectype[8];
	char chksum[8];
	int vlen, vaddr, vrectype, vchksum;
	unsigned short sv, crcval;
		
	nr = 0; adrmax = 0; adrmin = 0x010000;
	fh = fopen(fname, "r");
	if (!fh) return(-1);
	while (!feof(fh)) {
		buf[0] = 0;
		fgets(buf, 1024, fh);
		if (buf[0] == ':') {
			if (strlen(buf) >= 11) {
				memset(len, 0, 8);
				memset(addr, 0, 8);
				memset(rectype, 0, 8);
				memset(chksum, 0, 8);
				memcpy(len, &buf[1], 2);
				memcpy(addr, &buf[3], 4);
				memcpy(rectype, &buf[7], 2);
				x = sscanf(len, "%x", &vlen);
				if (x == 1) x = sscanf(addr, "%x", &vaddr);
				if (x == 1) x = sscanf(rectype, "%x", &vrectype);
				if (x == 1) {
					if (!(vlen & 1) && !(vaddr & 1) && (vrectype == 0)) {
						// length even (or zero), address even (or zero), and code record
						if (strlen(buf) >= ((2 * vlen) + 11)) {
							memcpy(chksum, &buf[(2 * vlen) + 9], 2);
							vchksum = -1;
							sscanf(chksum, "%x", &vchksum);
							chk = 0;
							for (x = 0; x < (vlen + 4); x++) {
								memset(data, 0, 8);
								memcpy(data, &buf[(2 * x) + 1], 2);
								y = -1;
								sscanf(data, "%x", &y);
								if (y >= 0) chk = (chk + y) & 0xff;
							}
							chk = (0 - chk) & 0xff;
							if (chk == vchksum) {
								val = 0;
								adr = vaddr / 2;
								//if (adr > adrmax) adrmax = adr;
								for (x = 0; x < vlen; x++) {
									memset(data, 0, 8);
									memcpy(data, &buf[(2 * x) + 9], 2);
									y = -1;
									sscanf(data, "%x", &y);
									if (y >= 0) {
										if (!(x & 1)) {
											// even byte
											val = y;
											if (adr < adrmin) adrmin = adr;
										}
										else {
											// odd byte
											val = val | (y << 8);
											if (adr < max_progsize) {
												prog[adr] = val;
												prog_val[adr] = 1;
												nr = nr + 1;
											}
											adr = adr + 1;
											if (adr > adrmax) adrmax = adr;
										}
									}
								}
							}
						}
					}
					else if ((vlen == 0) && (vrectype == 1)) {
						// length zero and end of file record
						memcpy(chksum, &buf[9], 2);
						vchksum = -1;
						sscanf(chksum, "%x", &vchksum);
						if (vchksum == 0xff) {
							break;
						}
					}
				}
			}
		}
	}
	// crc here
	if (crc && (adrmax > 0) && (adrmin < 0x010000)) {
		crcval = 0xffff;
		for (x = adrmin; x < adrmax; x++) {
			if (prog_val[x]) sv = prog[x];
			else sv = 0xffff;
			crcval = crc_ccitt_update (crcval, (unsigned char)(sv & 0xff));
			crcval = crc_ccitt_update (crcval, (unsigned char)(sv >> 8));
		}
		prog[adrmax] = crcval;
		prog_val[adrmax] = 1;
		fprintf(stderr, "start for CRC calc at byte 0x%04X\n", (adrmin * 2));
		fprintf(stderr, "CRC = 0x%04X at byte 0x%04X\n", (int)prog[adrmax], (adrmax * 2));
		
	}
	fclose(fh);
	return(nr);
}

// open serial device and set to correct baudrate and parity
int open_device(char *dev)
{
	int x, fd;
	struct termios tmios;
	struct termios *tm;
	
	tm = &tmios;
	fd = open(dev, O_RDWR | O_EXCL);
	if (fd == -1) {
		fprintf(stderr, "od-open()");
		return(-1);
	}
	if (isatty(fd)) {
		memset(tm, 0, sizeof(struct termios));
		tm->c_cflag = CREAD | CLOCAL | HUPCL | CSIZE | CS8;
		cfsetospeed(tm, B19200);
		cfsetispeed(tm, B19200);
		x = tcsetattr(fd, TCSANOW, tm);
		if (x == -1) {
			fprintf(stderr, "od-tcsetattr() %s\n", strerror(errno));
			close(fd);
			return(-1);
		}
		x = tcflush(fd, TCIOFLUSH);
		if (x == -1) {
			fprintf(stderr, "od-tcflush()");
			close(fd);
			return(-1);
		}
		
	}
	return(fd);
}

int timed_read(int fd, char *buf, int maxbytes, int usec)
{
	int x, bufpos;
	fd_set rfds;
	struct timeval tv;

	bufpos = 0;
	while (bufpos < maxbytes) {
		FD_ZERO(&rfds);
		FD_SET(fd, &rfds);
		tv.tv_sec = 0;
		tv.tv_usec = usec;
		x = select(fd + 1, &rfds, NULL, NULL, &tv);
		if (x > 0) {
			// at least one selector set
			if (FD_ISSET(fd, &rfds)) {
				x = read(fd, &buf[bufpos], maxbytes - bufpos);
				if (x < 0) {
					// error
					if (errno != EINTR) return(-1);
				}
				else if (x == 0) return(-1);	// connection broken
				else {
					// got some data
					bufpos = bufpos + x;
				}
			}
		}
		else if (x < 0) {
			// error
			if (errno != EINTR) return(-1);
		}
		else break;								// timeout
	}
	return(bufpos);
}

// establish connection with AVR (autobaud sequence)
int establish_connection(int fd)
{
	int x, y, z, state;
	char buf[65];
	
	state = 0;
	while (1) {
		memset(buf, 0, sizeof(buf));
		x = timed_read(fd, buf, sizeof(buf) - 1, 30000);
		if (x < 0) return(x);
		buf[x] = 0;
		//if (x > 0) fprintf(stderr, "buf - %s\n", buf);
		switch (state) {
			case 0:
				if (x > 0) {
					if (strstr(buf, "OSCCAL")) {
						write(fd, "MON\r\n", 5);
						state = 1;
						x = strlen(buf); z = -1;
						for (y = 0; y < x; y++) {
							if (buf[y] == '=') {
								z = atoi(&buf[y+1]);
								break;
							}
						}
						fprintf(stderr, "\n\ngot OSCCAL (%d)\n", z);
						break;
					}
				}
				else write(fd, " ", 1);
				break;
			case 1:
				if (x > 0) {
					if (strstr(buf, "OK")) {
						fprintf(stderr, "got OK - connected\n");
						return(1);
					}
				}
				break;
		}
	}
	return(0);
}

void avr_cmd_delay(void)
{
	usleep(AVR_CMD_DELAY);
}

// reset avr
void reset_avr(int fd)
{
	unsigned short crc;
	unsigned short *usp;
	command_buffer cmd;

	cmd.command = CMD_RUN;
	cmd.address = 0;
	cmd.nbytes = 0;
	crc = calc_crc((unsigned char *)&cmd, CMD_HEADSIZE);
	usp = (unsigned short *)(&cmd.buffer[0]);
	*usp = crc;
	write(fd, &cmd, CMD_MINSIZE);
	avr_cmd_delay();
}

// restart avr (when running firmware)
void restart_avr(int fd)
{
	write(fd, "\rrestart\r", 9);
}

// read AVR's flash memory
int avr_read_flash_block(int fd, int address, int nbytes, char *buf)
{
	int x, toread;
	unsigned short crc;
	unsigned short *usp;
	command_buffer cmd;

	cmd.command = CMD_READ_FLASH;
	cmd.address = address;
	cmd.nbytes = nbytes;
	crc = calc_crc((unsigned char *)&cmd, CMD_HEADSIZE);
	usp = (unsigned short *)(&cmd.buffer[0]);
	*usp = crc;
	write(fd, &cmd, CMD_MINSIZE);
	toread = cmd.nbytes + CMD_MINSIZE;
	x = timed_read(fd, (char *)&cmd, toread, CMD_MAXTIME);
	if (x != toread) return(-1);
	crc = *(unsigned *)(((char *)&cmd) + (x - 2));
	if (crc != calc_crc((unsigned char *)&cmd, x - 2)) return(-2);
	if (cmd.command != (CMD_READ_FLASH | CMD_OK)) return(-3);
	memcpy(buf, cmd.buffer, nbytes);
	avr_cmd_delay();
	return(0);
}

// read AVR's EE memory
int avr_read_ee_block(int fd, int address, int nbytes, char *buf)
{
	int x, toread;
	unsigned short crc;
	unsigned short *usp;
	command_buffer cmd;

	cmd.command = CMD_READ_EE;
	cmd.address = address;
	cmd.nbytes = nbytes;
	crc = calc_crc((unsigned char *)&cmd, CMD_HEADSIZE);
	usp = (unsigned short *)(&cmd.buffer[0]);
	*usp = crc;
	write(fd, &cmd, CMD_MINSIZE);
	toread = cmd.nbytes + CMD_MINSIZE;
	x = timed_read(fd, (char *)&cmd, toread, CMD_MAXTIME);
	if (x != toread) return(-1);
	crc = *(unsigned *)(((char *)&cmd) + (x - 2));
	if (crc != calc_crc((unsigned char *)&cmd, x - 2)) return(-2);
	if (cmd.command != (CMD_READ_EE | CMD_OK)) return(-3);
	memcpy(buf, cmd.buffer, nbytes);
	avr_cmd_delay();
	return(0);
}

// read AVR's SPM_PAGESIZE memory
int avr_get_spm_pagesize(int fd)
{
	int x, toread;
	unsigned short crc;
	unsigned short *usp;
	command_buffer cmd;

	cmd.command = CMD_GET_SPM_PAGESIZE;
	cmd.address = 0;
	cmd.nbytes = 2;
	crc = calc_crc((unsigned char *)&cmd, CMD_HEADSIZE);
	usp = (unsigned short *)(&cmd.buffer[0]);
	*usp = crc;
	write(fd, &cmd, CMD_MINSIZE);
	toread = cmd.nbytes + CMD_MINSIZE;
	x = timed_read(fd, (char *)&cmd, toread, CMD_MAXTIME);
	if (x != toread) return(-1);
	crc = *(unsigned *)(((char *)&cmd) + (x - 2));
	if (crc != calc_crc((unsigned char *)&cmd, x - 2)) return(-2);
	if (cmd.command != (CMD_GET_SPM_PAGESIZE | CMD_OK)) return(-3);
	x = *(unsigned short *)cmd.buffer;
	avr_cmd_delay();
	return(x);
}

// write AVR's flash memory
int avr_write_flash_block(int fd, int address, int nbytes, char *buf)
{
	int x;
	unsigned short crc;
	unsigned short *usp;
	command_buffer cmd;

	cmd.command = CMD_WRITE_FLASH;
	cmd.address = address;
	cmd.nbytes = nbytes;
	for (x = 0; x < nbytes; x++) {
		cmd.buffer[x] = *(buf + x);
	}
	crc = calc_crc((unsigned char *)&cmd, nbytes + CMD_HEADSIZE);
	usp = (unsigned short *)(&cmd.buffer[nbytes]);
	*usp = crc;
	write(fd, &cmd, nbytes + CMD_MINSIZE);
	x = timed_read(fd, (char *)&cmd, CMD_MINSIZE, CMD_MAXTIME);
	if (x != CMD_MINSIZE) return(-1);
	crc = *(unsigned *)((char *)cmd.buffer);
	if (crc != calc_crc((unsigned char *)&cmd, CMD_HEADSIZE)) return(-2);
	if (cmd.command != (CMD_WRITE_FLASH | CMD_OK)) return(-3);
	avr_cmd_delay();
	return(0);
}

// write AVR's EE memory
int avr_write_ee_block(int fd, int address, int nbytes, char *buf)
{
	int x;
	unsigned short crc;
	unsigned short *usp;
	command_buffer cmd;

	cmd.command = CMD_WRITE_EE;
	cmd.address = address;
	cmd.nbytes = nbytes;
	for (x = 0; x < nbytes; x++) {
		cmd.buffer[x] = *(buf + x);
	}
	crc = calc_crc((unsigned char *)&cmd, nbytes + CMD_HEADSIZE);
	usp = (unsigned short *)(&cmd.buffer[nbytes]);
	*usp = crc;
	write(fd, &cmd, nbytes + CMD_MINSIZE);
	x = timed_read(fd, (char *)&cmd, CMD_MINSIZE, CMD_MAXTIME);
	if (x != CMD_MINSIZE) return(-1);
	crc = *(unsigned *)((char *)cmd.buffer);
	if (crc != calc_crc((unsigned char *)&cmd, CMD_HEADSIZE)) return(-2);
	if (cmd.command != (CMD_WRITE_EE | CMD_OK)) return(-3);
	avr_cmd_delay();
	return(0);
}

void show_usage(void)
{
	fprintf(stderr, "usage: avrboot serial-device -options\n\n");
	fprintf(stderr, "-file filename specifies filename (in intel hex format) for program/verify\n");
	fprintf(stderr, "-program programs chip with specified file\n");
	fprintf(stderr, "-verify verifies chip against specified file\n");
	fprintf(stderr, "-eeread address numbytes reads and displays (stdout) eeprom in hex format\n");
	fprintf(stderr, "-flashread address numwords reads and writes (stdout) flash in binary format\n");
	fprintf(stderr, "-run jumps to application code\n");
	fprintf(stderr, "-restart resets the AVR from within the application code\n");
	fprintf(stderr, "-crc appends ccitt crc to write/verify buffer\n");
}

// parse command line for options
void parse_options(int fv, int argc, char *argv[])
{
	int x, y, z;
		
	for (x = fv; x < argc; x++) {
		if (!strcmp(argv[x], "-program")) program = 1;
		else if (!strcmp(argv[x], "-verify")) verify = 1;
		else if (!strcmp(argv[x], "-run")) run = 1;
		else if (!strcmp(argv[x], "-restart")) restart = 1;
		else if (!strcmp(argv[x], "-crc")) crc = 1;
		else if (!strcmp(argv[x], "-file")) {
			// option -file
			y = x + 1;
			if (y < argc) fname_argno = y;
		}
		else if (!strcmp(argv[x], "-eeread")) {
			// option -eeread
			y = x + 1;
			if (y < argc) {
				z = sscanf(argv[y], "%d", &eeread_addr);
				y = y + 1;
				if (y < argc) {
					z = sscanf(argv[y], "%d", &eeread_nb);
				}
			}
		}
		else if (!strcmp(argv[x], "-flashread")) {
			// option -flashread
			y = x + 1;
			if (y < argc) {
				z = sscanf(argv[y], "%d", &flashread_addr);
				y = y + 1;
				if (y < argc) {
					z = sscanf(argv[y], "%d", &flashread_nw);
				}
			}
		}
	}
}

int main(int argc, char *argv[])
{
	int x, y, z, rv, fd, spm_ps;
	char buf[1024];
			
	if (argc < 2) {
		show_usage();
		return(1);
	}
	parse_options(2, argc, argv);
	fd = open_device(argv[1]);
	if (fd == -1) {
		fprintf(stderr, "open_device() failed - %s\n", strerror(errno));
		return(1);
	}
	if (restart) restart_avr(fd);
	if (establish_connection(fd) != 1) {
		fprintf(stderr, "failed to connect to avr\n");
		return(1);
	}
	// talk to AVR
	x = avr_get_spm_pagesize(fd);
	fprintf(stderr, "avr_get_spm_pagesize() = %d\n", x);
	if ((x <= 0) || (x > sizeof(buf))) {
		fprintf(stderr, "failed to get spm page size\n");
		return(1);
	}
	spm_ps = x; rv = 0;

	if (fname_argno) {		
		x = read_ihex(argv[fname_argno]);
		if (x <= 0) {
			fprintf(stderr, "either filename invalid or wrong format (not intel hex)\n");
			rv = 1;
		}
		else {
			ihex_ok = 1;
		}
	}
	
	if (program && ihex_ok && (rv == 0)) {
		x = 0;
		memset(buf, 0xff, spm_ps); y = -1;
		for (z = 0; z < max_progsize; z++) {
			if (y >= 0) {
				if (((y*2)/spm_ps) != ((z*2)/spm_ps)) {
					y = (y*2)/spm_ps;
					x = avr_write_flash_block(fd, y*spm_ps, spm_ps, buf);
					fprintf(stderr, "avr_write_flash_block(0x%x) = %d\n", y*spm_ps, x);
					if (x) break;
					memset(buf, 0xff, spm_ps); y = -1;
				}
			}
			if (prog_val[z]) {
				y = z;
				buf[((z*2)%spm_ps)+0] = prog[z] & 0xff;
				buf[((z*2)%spm_ps)+1] = (prog[z] >> 8) & 0xff;
			}
		}
		if (y >= 0) {
			if (((y*2)/spm_ps) != ((z*2)/spm_ps)) {
				y = (y*2)/spm_ps;
				x = avr_write_flash_block(fd, y*spm_ps, spm_ps, buf);
				fprintf(stderr, "avr_write_flash_block(0x%x) = %d\n", y*spm_ps, x);
			}
		}
		if (!x) fprintf(stderr, "programmed\n");
		else rv = 1;
	}
	
	if (verify && ihex_ok && (rv == 0)) {
		x = 0;
		memset(buf, 0xff, spm_ps); y = -1;
		for (z = 0; z < max_progsize; z++) {
			if (prog_val[z]) {
				if ((y < 0) || (((y*2)/spm_ps) != ((z*2)/spm_ps))) {
					y = (z*2)/spm_ps;
					x = avr_read_flash_block(fd, y*spm_ps, spm_ps, buf);
					fprintf(stderr, "avr_read_flash_block(0x%x) = %d\n", y*spm_ps, x);
					if (x) break;
					y = z;
				}
				if ((unsigned char)buf[((z*2)%spm_ps)+0] != (prog[z] & 0xff)) {
					x = 1; break;
				}
				if ((unsigned char)buf[((z*2)%spm_ps)+1] != ((prog[z] >> 8) & 0xff)) {
					x = 1; break;
				}
			}
		}
		if (x) {
			fprintf(stderr, "program verify error\n");
			rv = 1;
		}
		else fprintf(stderr, "verify OK\n");
	}
	
	if ((eeread_addr >= 0) && (eeread_nb > 0) && (rv == 0)) {
		//if (eeread_nb <= sizeof(buf)) {
		if (eeread_nb <= 256) {
			x = avr_read_ee_block(fd, eeread_addr, eeread_nb, buf);
			if (!x) {
				for (z = 0; z < eeread_nb; z++) {
					x = (unsigned char)buf[z];
					if (((z + 1) % 16) == 0) fprintf(stdout, "%02X\n", x);
					else fprintf(stdout, "%02X ", x);
				}
				if (eeread_nb % 16) fprintf(stdout, "\n");
			}
		}
	}

	if ((flashread_addr >= 0) && (flashread_nw > 0) && (rv == 0)) {
		x = 0;
		memset(buf, 0xff, spm_ps); y = -1;
		for (z = flashread_addr; z < (flashread_addr + flashread_nw); z++) {
			if ((y < 0) || (((y*2)/spm_ps) != ((z*2)/spm_ps))) {
				y = (z*2)/spm_ps;
				x = avr_read_flash_block(fd, y*spm_ps, spm_ps, buf);
				fprintf(stderr, "avr_read_flash_block(0x%x) = %d\n", y*spm_ps, x);
				if (x) break;
				y = z;
			}
			/*
			if ((unsigned char)buf[((z*2)%spm_ps)+0] != (prog[z] & 0xff)) {
				x = 1; break;
			}
			if ((unsigned char)buf[((z*2)%spm_ps)+1] != ((prog[z] >> 8) & 0xff)) {
				x = 1; break;
			}
			*/
			write(1, &buf[((z*2)%spm_ps)+0], 1);
			write(1, &buf[((z*2)%spm_ps)+1], 1);
		}
	}

	if (run) reset_avr(fd);
	close(fd);
	return(rv);
}
