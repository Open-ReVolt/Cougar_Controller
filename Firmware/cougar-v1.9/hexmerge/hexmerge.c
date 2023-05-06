/*
  Code to merge mutiple intel HEX files
  
  Fran Sabolich
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#define max_progsize 0x80000

unsigned short prog[max_progsize];
unsigned char prog_val[max_progsize];

// by default append CRC to end of input hexfile
int crc = 1;

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

int write_ihex(unsigned short *source_buffer, int numrecords)
{
	int x, y, z;
	int addr, prev_addr, seg, chk;
	unsigned short word;
	unsigned char bb[16];
	char str[128];
	char data[8];

	addr = prev_addr = 0;;
	for (x = 0; x < numrecords; x++) {
		if ((addr & 0xffff0000) != (prev_addr & 0xffff0000)) {
			seg = (addr >> 4);
			sprintf(str, ":02000002%04X", seg);
			chk = 0;
			for (y = 0; y < 6; y++) {
				memset(data, 0, 8);
				memcpy(data, &str[(2 * y) + 1], 2);
				z = -1;
				sscanf(data, "%x", &z);
				if (z >= 0) chk = (chk + z) & 0xff;
			}
			chk = (0 - chk) & 0xff;
			sprintf(&str[13], "%02X", chk);
			fprintf(stdout, "%s\r\n", str);
		}
		prev_addr = addr;
		for (z = y = 0; y < 8; y++) {
			// 8 words (16 bytes) in a record
			word = *source_buffer++;
			bb[z++] = word & 0xff;
			bb[z++] = word >> 8;
		}
		// data record
		sprintf(str, ":%02X%04X00%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X",
			16, (addr & 0xffff),
	   		bb[0], bb[1], bb[2], bb[3], bb[4], bb[5], bb[6], bb[7],
			bb[8], bb[9], bb[10], bb[11], bb[12], bb[13], bb[14], bb[15]);
	   
		chk = 0;
		for (y = 0; y < 20; y++) {
			memset(data, 0, 8);
			memcpy(data, &str[(2 * y) + 1], 2);
			z = -1;
			sscanf(data, "%x", &z);
			if (z >= 0) chk = (chk + z) & 0xff;
		}
		chk = (0 - chk) & 0xff;
		sprintf(&str[41], "%02X", chk);
		fprintf(stdout, "%s\r\n", str);

		addr = addr + 16;
	}
	
	strcpy(str, ":00000001FF");
	fprintf(stdout, "%s\r\n", str);
	return(0);
}

// read intel hex file -- now supports I16HEX segment (02) records
int read_ihex(char *fname)
{
	int x, y, nr, chk, val, adr, segment, adrmax, adrmin;
	FILE *fh;
	char buf[1024+2];
	char data[8];
	char len[8];
	char addr[8];
	char rectype[8];
	char chksum[8];
	int vlen, vaddr, vrectype, vchksum;
	unsigned short sv, crcval;
		
	nr = 0; adrmax = 0; adrmin = max_progsize;
	fh = fopen(fname, "r");
	if (!fh) return(-1);
	segment = 0;
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
								adr = (segment + vaddr) / 2;
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
												if (!prog_val[adr]) {
													prog[adr] = val;
													prog_val[adr] = 1;
													nr = nr + 1;
												}
												else {
													fprintf(stderr, "**** HEX FILES OVERLAP - ABORTING ****\n");
													exit(1);
												}
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
					else if ((vlen == 2) && (vaddr == 0) && (vrectype == 2)) {
						// length zero and segment record
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
							memset(data, 0, 8);
							memcpy(data, &buf[9], 4);
							x =	sscanf(data, "%x", &y);
							if (x == 1) {
								segment = (y << 4);
							}
						}
					}
				}
			}
		}
	}
	// crc here
	if (crc && (adrmax > 0) && (adrmin < max_progsize)) {
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

void show_usage(void)
{
	fprintf(stderr, "usage: avrboot numrecords file1 [file2] [file3] ... [fileN]\n");
	fprintf(stderr, "numrecords specifies number of 16 byte records for output hexfile\n");
	fprintf(stderr, "file1 [file2] [file3] ... [fileN] specifies filenames of input files\n");
}

int main(int argc, char *argv[])
{
	int x, y, numrecords;
	
	if (argc < 2) {
		show_usage();
		return(1);
	}
	numrecords = 0;
	numrecords = atoi(argv[1]);
	fprintf(stderr, "will output %d records (%d bytes)\n", numrecords, numrecords * 16);
	memset(prog, 0xff, sizeof(prog));
	for (x = 2; x < argc; x++) {
		y = read_ihex(argv[x]);
		fprintf(stderr, "read_ihex(%s) = %d\n", argv[x], y);
		if (y <= 0) return(-1);
	}
	write_ihex(prog, numrecords);
	return(0);
}
