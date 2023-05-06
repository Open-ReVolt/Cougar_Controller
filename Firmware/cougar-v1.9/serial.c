/*
serial port support for cougar.c
*/

#ifdef __AVR_ATmega168__
#define MEGA168
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#ifdef MEGA168
#include <avr/iom168.h>
#else
#include <avr/iom8.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cougar.h"

#ifdef MEGA168
#define SIG_UART_RECV SIG_USART_RECV
#define SIG_UART_DATA SIG_USART_DATA
#define UDR UDR0
#define UCSRB UCSR0B
#define RXEN RXEN0
#define TXEN TXEN0
#define RXCIE RXCIE0
#define UDRIE UDRIE0
#define UBRRL UBRR0L
#define UBRRH UBRR0H
#endif

// UART (serial)
typedef struct {
	unsigned char rxbuf[UART_RXBUF_SIZE];
	unsigned char txbuf[UART_TXBUF_SIZE];
	volatile unsigned rxhead;
	volatile unsigned rxtail;
	volatile unsigned txhead;
	volatile unsigned txtail;
} uart_fifo_type;

uart_fifo_type uart;

extern char uart_str[];

/* uart receive interrupt */
SIGNAL(SIG_UART_RECV)
{
	unsigned char c;
	unsigned i;
	
	c = UDR;
	i = uart.rxhead + 1;
	if (i >= UART_RXBUF_SIZE) i = 0;
	if (i != uart.rxtail) {
		// fifo not full
		uart.rxbuf[uart.rxhead] = c;
		uart.rxhead = i;
	}
}

/* uart UDR empty interrupt */
SIGNAL(SIG_UART_DATA)
{
	unsigned i;
	
	i = uart.txtail;
	if (i != uart.txhead) {
		UDR = uart.txbuf[i++];
		if (i >= UART_TXBUF_SIZE) i = 0;
		uart.txtail = i;
	}
	else {
		// disable TX buffer empty interrupt
		UCSRB = (1 << RXEN) | (1 << TXEN) | (1 << RXCIE);
	}
}

// get character from uart fifo, return -1 if fifo empty
int uart_getch(void)
{
	unsigned char c;
	unsigned i, j;
	
	i = uart.rxtail;
	cli(); j = uart.rxhead; sei();
	if (i != j) {
		c = uart.rxbuf[i++];
		if (i >= UART_RXBUF_SIZE) i = 0;
		cli(); uart.rxtail = i; sei();
		return(c);
	}
	return(-1);
}

// put character to uart (return 1 if fifo full, else 0)
unsigned char uart_putch(char c)
{
	unsigned i, j;
	
	i = uart.txhead + 1;
	if (i >= UART_TXBUF_SIZE) i = 0;
	cli(); j = uart.txtail; sei();
	if (i == j) {
		// fifo full
		return(1);
	}
	uart.txbuf[uart.txhead] = c;
	cli(); uart.txhead = i; sei();
	// enable TX buffer empty interrupt
	UCSRB = (1 << RXEN) | (1 << TXEN) | (1 << RXCIE) | (1 << UDRIE);
	return(0);
}

// put string to uart
void uart_putstr(void)
{
	char ch;
	char *str;
	
	str = uart_str;
	while (1) {
		ch = *str++;
		if (ch == 0) break;
		while (uart_putch(ch));
	}
}

// set up UART to 19200,n,8,1
void setup_uart(void)
{
	unsigned int ubrr;

	ubrr = ((unsigned long)F_OSC / ((unsigned long)16 * (unsigned long)19200)) - 1;
	UBRRL = ubrr & 0xff;
	UBRRH = ubrr >> 8;
	#ifdef MEGA168
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
	#else
	UCSRC = (PARITY_NONE << 4) | (BITS_8_1 << 1) | (1 << URSEL);
	#endif
	UCSRB = (1 << RXEN) | (1 << TXEN) | (1 << RXCIE);
}
