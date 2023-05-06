/*
Assembler support functions.

Fran Sabolich
*/

	.file	"misc.asm"
	.arch atmega8

/*
Call-used registers (r18-r27, r30-r31):
May be allocated by gcc for local data. You may use them freely in assembler subroutines.
Calling C subroutines can clobber any of them - the caller is responsible for saving and
restoring.

Call-saved registers (r2-r17, r28-r29):
May be allocated by gcc for local data. Calling C subroutines leaves them unchanged.
Assembler subroutines are responsible for saving and restoring these registers, if
changed. r29:r28 (Y pointer) is used as a frame pointer (points to local data on stack)
if necessary.

Fixed registers (r0, r1):
Never allocated by gcc for local data, but often used for fixed purposes:

r0 - temporary register, can be clobbered by any C code (except interrupt handlers which
save it), may be used to remember something for a while within one piece of assembler code

r1 - assumed to be always zero in any C code, may be used to remember something for a
while within one piece of assembler code, but must then be cleared after use (clr r1).
This includes any use of the [f]mul[s[u]] instructions, which return their result in r1:r0.
Interrupt handlers save and clear r1 on entry, and restore r1 on exit (in case it
was non-zero). 
*/

__SREG__ = 0x3f
__SP_H__ = 0x3e
__SP_L__ = 0x3d
__tmp_reg__ = 0
__zero_reg__ = 1

	.text

;port definitions
#define PIND 0x10
#define DDRD 0x11
#define PORTD 0x12


;software uart input PORT
#define su_PIN PIND
;software uart RxD data direction register
#define su_RxD_ddr DDRD
;software uart RxD bitno
#define su_RxD 0


;software uart output PORT
#define su_PORT PORTD
;software uart TxD data direction register
#define su_TxD_ddr DDRD
;software uart TxD bitno
#define su_TxD 1


;software uart Tx Enable PORT - not needed if TTL<->RS232 chip always enabled
;#define su_EN_PORT PORTD
;software uart Tx Enable data direction register
;#define su_EN_ddr DDRD
;software uart Tx Enable bitno
;#define su_EN 2


; *** 140 = 19200 at 16MHz - WRONG, now fixed, new value 135 ***
;135 = 19200 at 16MHz
;66 = 19200 at 8MHz
;39 = 57600 at 14.746MHz
;124 = 19200 at 14.746MHz
;31 = 4800 at 1MHz
#define su_baud_delay 135

;for 1MHz
;#define SU_GETCHAR_TIMEOUT 1666
;for 8MHz
;#define SU_GETCHAR_TIMEOUT 13328
;for 14.746MHz
;#define SU_GETCHAR_TIMEOUT 24566
;for 16MHz
#define SU_GETCHAR_TIMEOUT 26655

	.global	su_init
	.func	su_init
su_init:
	; RX to input
	cbi		su_RxD_ddr,su_RxD
	; TX to output and high
	sbi		su_PORT,su_TxD
	sbi		su_TxD_ddr,su_TxD
	sbi		su_PORT,su_TxD
	; Tx Enable to output and high - not needed if TTL<->RS232 chip always enabled
	;sbi		su_EN_PORT,su_EN
	;sbi		su_EN_ddr,su_EN
	;sbi		su_EN_PORT,su_EN
	; done, back to caller
	ret
	.endfunc


	.global	do_reboot
	.func	do_reboot
do_reboot:
	; Tx Enable back to input - not needed if TTL<->RS232 chip always enabled
	;cbi		su_EN_ddr,su_EN
	; Tx back to input
	cbi		su_TxD_ddr,su_TxD
	; and jump to application code
	clr		r30
	clr		r31
	ijmp
	.endfunc


#define su_bitcnt r30
#define su_rxbyte r24
#define su_hibyte r25

	.global	su_getchar
	.func	su_getchar
su_getchar:
	ldi		r26,lo8(SU_GETCHAR_TIMEOUT)
	ldi		r27,hi8(SU_GETCHAR_TIMEOUT)
	ldi		su_bitcnt,9				;8 data bit + 1 stop bit
su_getchar1:
	subi	r26,1
	sbci	r27,0
	breq	su_getchar4				;timeout
	sbic	su_PIN,su_RxD			;Wait for start bit
	rjmp	su_getchar1
	rcall	UART_delay				;0.5 bit delay minus
su_getchar2:
	rcall	UART_delay		;1 bit delay
	rcall	UART_delay		
	clc						;clear carry
	sbic	su_PIN,su_RxD	;if RX pin high
	sec
	dec		su_bitcnt		;If bit is stop bit
	breq	su_getchar3		;return
	ror		su_rxbyte		;shift bit into Rxbyte
	rjmp	su_getchar2		;go get next
su_getchar3:
	clr		su_hibyte		;no timeout
	ret
su_getchar4:
	ldi		su_hibyte,-1
	ret
	.endfunc

#undef su_bitcnt
#undef su_rxbyte
#undef su_hibyte


#define su_bitcnt r30
#define su_txbyte r24

	.global	su_putchar
	.func	su_putchar
su_putchar:
	ldi		su_bitcnt,10	;1+8+sb (sb is # of stop bits)
	com		su_txbyte		;Inverte everything
	sec						;Start bit
su_putchar0:
	brcc	su_putchar1		;If carry set
	cbi		su_PORT,su_TxD	;send a '0'
	rjmp	su_putchar2		;else	
su_putchar1:
	sbi		su_PORT,su_TxD	;send a '1'
	nop
su_putchar2:
	rcall	UART_delay		;One bit delay
	rcall	UART_delay
	lsr		su_txbyte		;Get next bit
	dec		su_bitcnt		;If not all bit sent
	brne	su_putchar0		;send next
	ret
	.endfunc

#undef su_bitcnt
#undef su_txbyte
	

; 0.5 bit delay at desired baud rate
; overhead (call, load, untaken branch, return) is 9 cycles
; loop takes 3 cycles per iterration
UART_delay:
	ldi		r31,su_baud_delay
UART_delay1:
	dec		r31
	brne	UART_delay1
	ret


#define wt_lobyte r24
#define wt_hibyte r25

	.global	su_waitpulse
	.func	su_waitpulse
su_waitpulse:
	movw	r30,wt_lobyte
su_waitpulse0:
	sbis	su_PIN,su_RxD			;wait for edge of pulse
	rjmp	su_waitpulse1			;got it
	adiw	r30,1					;increment wait time
	brne	su_waitpulse0			;keep waiting
	movw	wt_lobyte,r30			;clear return value
	ret								;and back to caller
su_waitpulse1:						;leading edge of pulse
	sbic	su_PIN,su_RxD			;wait for edge of pulse
	rjmp	su_waitpulse2			;got it
	adiw	wt_lobyte,1				;increment wait time
	brne	su_waitpulse1			;keep waiting
	ret								;and back to caller (with return value = 0)
su_waitpulse2:
	ret								;back to caller with actual time value
	.endfunc

#undef wt_lobyte
#undef wt_hibyte
