/*
Firmware for cougar open source DC motor controller

version 0.1 - first version tested
version 0.2 - added PW (PWM) to realtime data
version 0.3 - added thermal cutback code (to be checked by Paul)
version 0.4 - added continous check of vref below 2V
version 0.5 - declared some variables volatile to enforce execution within bounds of cli() and sei()
version 0.6 - make ocr1a_lpf time constant longer - V0.5 oscillates if Kp,Ki are low (slow PI loop)
version 0.7 - fixed stupid bug in config_pi() that caused Joe's problems (thanks Joe)
version 0.8 - high pedal lockout added but not yet tested
version 0.9 - store mutiple copies of configuration in EE prom (not yet tested)
version 0.10 - code size reduction, correct thermal cutback starting point, selectable pwm low pass filter
version 1.0 - in pi_loop() added cli() and sei() around update of OCR1A, added cli() after pi_loop
version 1.1 - added battery amps and battery amp hour calculations (not yet tested)
version 1.2 - added motor overspeed logic (experimental and not yet tested)
version 1.3 - added battery amps limit (not yet tested) and changed motor overspeed parameters to 4 digit
version 1.4 - new 8KHz PWM compile to option, added battery_amps_limit, spare, and crc, default_config PROGMEM
version 1.5 - fixed ocr1a_lpf calculation - so battery amps limit should now work, added "restart" command
version 1.6 - added support for ATMEGA168 (not yet tested)
version 1.7 - moved motor overspeed logic (still not yet tested) from main() into interrupt code pi_loop()
version 1.8 - added precharge timer, removed not needed code that measures pi_loop() execution time
version 1.9 - forgot to calculate bat_amp_lim_510 when loading config from EEprom, now fixed (thanks Adam)
version 1.10 - added configurable PWM dead zone and motor overspeed detect time for motor overspeed logic
               configuration menu a little nicer, new watchdog handling (mainly for ATMega168)
               2010-01-10 fixed defines for PIND, DDRD, and PORTD in bootload168/misc.asm (no version change)
version 1.11 - added motor_speed_calc_amps - attempt to avoid overspeed tripping when pedal is "pumped"
               (code no longer fits in ATMega8 with CRC check enabled)
version 1.11b - slight change to "motor_speed_calc_amps" logic; see line 469

Copy either "Makefile.Linux" or "Makefile.Windows" to "Makefile"
For size optimizations (smallest code) use -Os for CPFLAGS in Makefile
For speed optimizations (fastest code) use -O2 for CPFLAGS in Makefile
In Makefile set serial port (first parameter after "avrboot") for your system
*/

#ifdef __AVR_ATmega168__
#define MEGA168
#endif

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#ifdef MEGA168
#include <avr/iom168.h>
#else
#include <avr/iom8.h>
#endif
#include <util/crc16.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cougar.h"

#ifdef MEGA168
#define TIMSK TIMSK1
#define TICIE1 ICIE1
#endif

// if AUTOCRC is defined, use automatically generated CRC file
#ifdef AUTOCRC
#include "autocrc.h"
#endif

typedef struct {
	int throttle_ref;
	int current_ref;
	int current_fb;
	unsigned raw_hs_temp;
	unsigned raw_throttle;
	unsigned battery_amps;
	unsigned long battery_ah;
} realtime_data_type;

typedef struct {
	long K1;
	long K2;
	long error_new;
	long error_old;
	long pwm;
} pi_storage_type;

typedef struct {
	unsigned magic;						// must be 0x12ab
	int Kp;								// PI loop proportional gain
	int Ki;								// PI loop integreal gain
	unsigned throttle_min_raw_counts;	// throttle low voltage (pedal to metal)
	unsigned throttle_max_raw_counts;	// throttle high voltage (foot off pedal)
	unsigned throttle_fault_raw_counts;	// throttle fault voltage (after 200mS)
	unsigned throttle_pos_gain;			// gain for actual throttle position
	unsigned throttle_pwm_gain;			// gain for pwm (voltage)
	int current_ramp_rate;				// current ramp rate
	unsigned rtd_period;				// real time data period
	unsigned pwm_filter;				// filter for ocr1a_lpf
	unsigned motor_os_th;				// motor overspeed threshold
	unsigned motor_os_ft;				// motor overspeed fault time
	unsigned motor_os_dt;				// motor overspeed detect time
	unsigned pwm_deadzone;				// pwm deadzone (before FETs start to conduct)
	unsigned battery_amps_limit;		// battery amps limit
	unsigned precharge_time;			// precharge time in 0.1 second increments
	unsigned motor_sc_amps;				// motor current must be > motor_sc_amps to calculate motor speed
	unsigned spares[5];					// space for future use
	unsigned crc;						// checksum for verification
} config_type;

config_type default_config PROGMEM = {
	0x12ab,								// magic
	2,									// PI loop P gain (Joe's was 1)
	160,								// PI loop I gain (Joe's was 20)
	413,								// throttle low voltage (pedal to metal)
	683,								// throttle high voltage (foot off pedal)
	100,								// throttle fault voltage (after 200mS)
	8,									// throttle pedal position gain
	0,									// throttle pwm (voltage) gain
	6,									// current ramp rate (from throttle)
	0,									// rtd (real time data) period
	0,									// pwm filter
	0,									// motor overspeed threshold
	1000,								// motor overspeed fault time
	10,									// motor overspeed detect time
	5,									// PWM deadzone
	0,									// battery amps limit
	0,									// precharge time
	0,									// motor speed calc amps
	{0,0,0,0,0},						// 5 spares
	0									// crc
};

unsigned char counter_16k = 0;
unsigned char counter_8k = 0;
unsigned char counter_4k = 0;
unsigned char ad_channel = 0;
unsigned char oc_cycles_off_counter = 0;
unsigned char in_pi_loop = 0;

volatile unsigned counter_1k = 0;		// 1KHz (976Hz to be exact) counter for timing

volatile unsigned raw_current_fb;		// AD channel 2
volatile unsigned raw_hs_temp;			// AD channel 1
volatile unsigned raw_throttle;			// AD channel 0

volatile unsigned ocr1a_ghost = 0;		// ocr1a ghost variable (needed for 8KHz PWM)

unsigned vref = 0;						// zero current voltage for LEM current sensor
unsigned ocr1a_lpf = 0;					// ocr1a run through lowpass filter (sort of averaged)
unsigned long ocr1a_lpf_32 = 0;			// ocr1a low pass filter sum
unsigned max_current_ref = 0;			// max_current_ref in variable so controlled by temperature
int throttle_ref = 0;					// reference (desired) throttle
int current_ref = 0;					// reference (desired) current
int current_fb = 0;						// current feedback (actual current)

unsigned battery_amps = 0;				// calculated battery current
unsigned long battery_ah = 0;			// calculated battery AH used by controller

unsigned long bat_amp_lim_510 = 0;		// battery amps limit multiplied by 510 (max PWM)

unsigned long motor_overspeed_threshold = 0;	// motor overspeed threshold
unsigned motor_os_fault_timer = 0;				// motor overspeed fault timer (milliseconds)
unsigned char motor_os_count = 0;				// motor overspeed debounce timer (milliseconds)

unsigned precharge_timer = 0;			// precharge timer in 0.1 second units

unsigned throttle_fault_counts = 0;
volatile unsigned char fault_bits = HPL_FAULT;

unsigned long idle_loopcount;			// how many loops we do while micro is not executing PI

unsigned tm_show_data;					// timer for realtime data display

char uart_str[80];						// string for uart_putstr()

pi_storage_type pi;
config_type config;
realtime_data_type rt_data;

#ifdef crc_address
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
#endif

// calc CRC on block in SRAM
unsigned int calc_block_crc(unsigned nbytes, unsigned char *buf)
{
	unsigned n, crc;
	
	crc = 0xffff;
	for (n = 0; n < nbytes; n++) {
		crc = _crc_ccitt_update (crc, *(buf + n));
	}
	return(crc);
}

#ifdef MEGA168
void watchdog_disable(void)
{
	asm ("cli");
	asm ("wdr");
	MCUSR &= ~(1 << WDRF);
	WDTCSR |= (1 << WDCE) | (1 << WDE);
	WDTCSR = 0x00;
	asm ("sei");
}
void watchdog_enable(void)
{
	asm ("cli");
	asm ("wdr");
	WDTCSR |= (1 << WDCE) | (1 << WDE);
	WDTCSR = (1 << WDE) | (1 << WDP2);
	asm ("sei");
}
#else
void watchdog_disable(void)
{
	wdt_reset();
	wdt_disable();
}
void watchdog_enable(void)
{
	wdt_reset();
	wdt_enable(WDTO_250MS);
}
#endif

// convert val to string (inside body of string) with specified number of digits
// do NOT terminate string
void u16_to_str(char *str, unsigned val, unsigned char digits)
{
	str = str + (digits - 1);
	while (digits-- > 0) {
		*str-- = (unsigned char)(val % 10) + '0';
		val = val / 10;
	}
}

// convert val to hex string (inside body of string) with specified number of digits
// do NOT terminate string
void u16x_to_str(char *str, unsigned val, unsigned char digits)
{
	unsigned char nibble;
	
	str = str + (digits - 1);
	while (digits-- > 0) {
		nibble = val & 0x000f;
		if (nibble >= 10) nibble = (nibble - 10) + 'A';
		else nibble = nibble + '0';
		*str-- = nibble;
		val = val >> 4;
	}
}

inline void clear_oc(void)
{
	PORTB &= ~PB_OC_CLEAR;				// OC clear low (low to clear)
	asm("nop"); asm("nop");				// 4 nops = 1/4th uS - enough for 74HC00
	asm("nop"); asm("nop"); 
	PORTB |= PB_OC_CLEAR;				// OC clear high (high for normal operation)
}

inline unsigned get_time(void)
{
	unsigned t;
	
	cli(); t = counter_1k; sei();
	return(t);
}

inline unsigned diff_time(unsigned before)
{
	unsigned now;
	
	cli(); now = counter_1k; sei();
	return(now - before);
	
}

unsigned long wait_time(unsigned howlong)
{
	unsigned begin;
	unsigned long loopcount;
		
	loopcount = 0;
	begin = get_time();
	while (diff_time(begin) < howlong) loopcount++;
	return(loopcount);
}

// PI loop code - runs at 4Khz
void pi_loop(void)
{
	static unsigned char throttle_counter = 0;
	unsigned loc_current_fb, loc_throttle;
	unsigned uv1, uv2;
	unsigned long luv1;
	int i;
		
	loc_current_fb = raw_current_fb;
	loc_throttle = raw_throttle;
	sei();
	// now we have a snapshot of all ADC readings and interrupts are enabled
	// the timer 1 overflow ISR should re-enter on itself if it needs to
	// we also have a snapshot of the entry time (16KHz counter) so we can measure execution time
	
	// continous Vref fault checking - if below 2V (410 counts) set fault
	if (loc_current_fb < 410) fault_bits |= VREF_FAULT;
	// convert loc_current_fb from raw value to scaled (0 to 511)
	// current starts in [512, 512 + 213]  (if it's in 0 to 500 amps for the LEM 300)
	if (loc_current_fb < vref) loc_current_fb = 0;
	else loc_current_fb -= vref;
	// now current is in the range [0, 213] or so
	current_fb = (loc_current_fb * 19) >> 3;			// (19/8 is almost 2.4)
	// now current is in [0, 506] or so, close to same as current reference range
	pi.error_new = current_ref - current_fb;
	// execute PI loop
	// first, K1 = Kp << 10;
	// second, K2 = Ki - K1;
	if (current_ref == 0) {
		pi.pwm = 0;
		//pi.Kp = 0;
		//pi.Ki = 0;

		// if Kp and Ki = 0, then K1 and K2 = 0;
		// if K1 and K2 = 0, then pwm = pwm + 0, and 0 + 0 = 0
		// so we don't need to run the PI loop, just set error_old to error_new
	}
	else {
		pi.pwm += (pi.K1 * pi.error_new) + (pi.K2 * pi.error_old);
	}
	pi.error_old = pi.error_new;
	if (pi.pwm > (510L << 16)) pi.pwm = (510L << 16);
	else if (pi.pwm < 0L) pi.pwm = 0L;
	uv1 = pi.pwm >> 16;
	if (pi.pwm & 0x8000) uv1++;
	#ifdef PWM8K
	cli(); OCR1A = uv1 << 1; sei();
	ocr1a_ghost = uv1;
	#else
	cli(); OCR1A = uv1; sei();
	ocr1a_ghost = uv1;
	#endif

	// calculate average OCR1A value
	// OCR1A max value is 511, so we can multiply it by up to 127 times
	luv1 = (unsigned long)ocr1a_ghost << 16;
	switch ((unsigned char)config.pwm_filter) {
		case 0:
			ocr1a_lpf_32 = ((ocr1a_lpf_32 * 127) + luv1) >> 7;
			break;
		case 1:
			ocr1a_lpf_32 = ((ocr1a_lpf_32 * 63) + luv1) >> 6;
			break;
		case 2:
			ocr1a_lpf_32 = ((ocr1a_lpf_32 * 31) + luv1) >> 5;
			break;
		case 3:
			ocr1a_lpf_32 = ((ocr1a_lpf_32 * 15) + luv1) >> 4;
			break;
		default:
			ocr1a_lpf_32 = 0;
	}
	ocr1a_lpf = ocr1a_lpf_32 >> 16;

	throttle_counter++;
	if ((throttle_counter & 0x03) == 0x00) {
		// run throttle logic at 1KHz, calculate throttle_ref
		if (loc_throttle > config.throttle_fault_raw_counts) {
			// raw throttle counts > fault value - throttle is OK
			if (throttle_fault_counts > 0) throttle_fault_counts--;

			if (loc_throttle > config.throttle_max_raw_counts)
				loc_throttle = config.throttle_max_raw_counts;
			else if (loc_throttle < config.throttle_min_raw_counts)
				loc_throttle = config.throttle_min_raw_counts;
		
			loc_throttle -= config.throttle_min_raw_counts;
			// now loc_throttle is in [0, (throttle_max_raw_counts - throttle_min_raw_counts)]
			loc_throttle = (config.throttle_max_raw_counts - config.throttle_min_raw_counts) -
				loc_throttle;
			/*
			now, 0 throttle is 0,
			and max throttle is (throttle_max_raw_counts - throttle_min_raw_counts)
			*/
			throttle_ref = (unsigned long)loc_throttle * (unsigned long)MAX_CURRENT_REF /
				(unsigned long)(config.throttle_max_raw_counts - config.throttle_min_raw_counts);
			// now throttle ref in [0 to 511]
		}
		else {
			// raw throttle counts <= fault value - it this persists we will have a throttle fault
			if (throttle_fault_counts < THROTTLE_FAULT_COUNTS) {
				throttle_fault_counts++;
				if (throttle_fault_counts >= THROTTLE_FAULT_COUNTS) fault_bits |= THROTTLE_FAULT;
			}
		}
	}
	else if ((throttle_counter & 0x03) == 0x01) {
		// run throttle logic at 1KHz, calculate current_ref from throttle_ref
		// throttle gain logic
		uv1 = ((unsigned)throttle_ref * config.throttle_pos_gain) >> 3;
		uv2 = (ocr1a_lpf * config.throttle_pwm_gain) >> 3;
		if (uv1 > uv2) loc_throttle = uv1 - uv2;
		else loc_throttle = 0;
		// current_ref ramp rate logic
		if (loc_throttle > max_current_ref) i = (max_current_ref - current_ref);
		else i = (int)loc_throttle - current_ref;
		if (i > config.current_ramp_rate) i = config.current_ramp_rate;
		else if (i < -config.current_ramp_rate) i = -config.current_ramp_rate;
		current_ref += i;
		// to limit battery amps, limit motor amps based on PWM (with low pass filter)
		if (config.battery_amps_limit > 0) {
			// we wish to limit battery amps
			if (ocr1a_lpf > 0) {
				// PWM > 0
				luv1 = bat_amp_lim_510 / (unsigned long)ocr1a_lpf;
				if (luv1 < MAX_CURRENT_REF) {
					uv2 = luv1;
					if (current_ref > uv2) current_ref = uv2;
				}
			}
		}
		// if we have any fault, simply set current_ref to 0
		if (fault_bits) current_ref = 0;
	}
	else if ((throttle_counter & 0x03) == 0x02) {
		// run battery amps and hours logic at 1KHz
		// battery_amps = (current_fb * pwm) / 512;
		battery_amps = ((unsigned long)current_fb * (unsigned long)ocr1a_ghost) >> 8;
		if (battery_amps & 0x0001) battery_amps = (battery_amps >> 1) + 1;
		else battery_amps = battery_amps >> 1;
		// current_fb of 505 counts equals 500 motor amps
		// so for current_fb of 505 counts and pwm 510 (100%) calculated battery_amps = 503
		// the controller will not be 100% efficient so battery amps will be greater anyways
		battery_ah += (unsigned long)battery_amps;
		// now we've added battery_amps to the battery amp hour sum
		if (battery_ah > (unsigned long)4000000000UL) battery_ah = (unsigned long)4000000000UL;
		// clamp to 4E9 to prevent roll-over
	}
	else if ((throttle_counter & 0x03) == 0x03) {
		// run motor overspeed logic at 1KHz
		if (config.motor_os_th > 0) {
			// motor overspeed detection logic enabled
			if (fault_bits & MOTOR_OS_FAULT) {
				// we have a motor overspeed fault
				if (motor_os_fault_timer) {
					motor_os_fault_timer--;
					if (motor_os_fault_timer == 0) {
						// motor overspeed fault expired, reset fault
						fault_bits &= ~MOTOR_OS_FAULT;
					}
				}
			}
			else {
				// no motor overspeed fault, so check for overspeed
				if (ocr1a_ghost > config.pwm_deadzone)
					luv1 = (unsigned long)(ocr1a_ghost - config.pwm_deadzone) << 16;
				else luv1 = 0;

				/*
				// original logic in v1.11
				// if current feedback > motor_speed_calc_amps, then rpm = k * V / current_feedback
				// else rpm = 0
				if (current_fb > config.motor_sc_amps) luv1 = luv1 / (unsigned long)current_fb;
				else luv1 = 0;
				*/

				// logic changed slightly in v1.11b
				// if current feedback > motor_speed_calc_amps, then rpm = k * V / current_feedback
				// else rpm = k * V / motor_speed_calc_amps
				if (current_fb > config.motor_sc_amps) luv1 = luv1 / (unsigned long)current_fb;
				else luv1 = luv1 / (unsigned long)(config.motor_sc_amps + 1);

				if (luv1 > motor_overspeed_threshold) {
					if (motor_os_count < (unsigned char)config.motor_os_dt) motor_os_count++;
					else {
						// we have motor overspeed
						fault_bits |= MOTOR_OS_FAULT;
						motor_os_count = 0;
						motor_os_fault_timer = config.motor_os_ft;
					}
				}
				else motor_os_count = 0;
			}
		}
	}
}

// TIMER1 overflow interrupt
// This occurs center aligned with PWM output - best time to sample current sensor
// Rate is 16KHz (or 8K if PWM8K defined)
ISR(TIMER1_OVF_vect)
{
	unsigned ui;
	
	counter_16k++;
	#ifdef PWM8K
	counter_16k++;
	#else
	if (counter_16k & 0x01) {
	#endif
		// every other time (8KHz)
		counter_8k++;
		if (counter_8k & 0x01) {
			// conversion on throttle or heatsink done - grab result
			ui = ADC;
			ADMUX = ADMUX = (1 << REFS0) | 2;			// start conversion on current fb chan
			ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) | (1 << ADSC);
			if (ad_channel == 0) raw_throttle = ui;
			else if (ad_channel == 1) raw_hs_temp = ui;
			counter_4k++;
			if ((counter_4k & 0x03) == 0) counter_1k++;	// 1 KHz counter for delays, etc.
			// overcurrent trip logic
			if (PINB & PINB_OC_STATE) {
				// overcurrent circuit tripped
				oc_cycles_off_counter++;
			}
			if (oc_cycles_off_counter >= NUM_OC_CYCLES_OFF) {
				// time to reset overcurrent trip circuit
				oc_cycles_off_counter = 0;
				#ifdef OC_CLEAR_ENABLED
				clear_oc();
				#endif
			}
		}
		else {
			// convertion on current sensor reading complete (4KHz)
			raw_current_fb = ADC;						// get conversion result
			ad_channel++;								// next channel channel
			if (ad_channel > 1) ad_channel = 0;			// wrap around logic
			ADMUX = ADMUX = (1 << REFS0) | ad_channel;	// set channel and start conversion
			ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) | (1 << ADSC);
			// execute PI loop with re-entrancy check - in v1.0 added cli() after pi_loop()
			if (!in_pi_loop) {
				in_pi_loop = 1; pi_loop(); cli(); in_pi_loop = 0;
			}
		}
	#ifndef PWM8K
	}
	#endif
}

// timer 1 input capture ISR (1000 hertz)
SIGNAL(SIG_INPUT_CAPTURE1)
{
	counter_1k++;							// 1 KHz counter for delays, etc.	
}

unsigned char measure_vref(void)
{
	unsigned char lp;
	unsigned sum;
	
	sum = 0;
	for (lp = 0; lp < 16; lp++) {
		// do a conversion on channel 2 - current sensor
		ADMUX = ADMUX = (1 << REFS0) | 2;
		ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) | (1 << ADSC);
		while (ADCSRA & (1 << ADSC));
		sum += ADC;
	}
	vref = sum >> 4;
	if ((vref > (512 + 50)) || (vref < (512 - 50))) return(0);
	return(1);
}

void config_pi(void)
{
/* A couple of points:
	We are now doing OCR1A = (pwm >> 16) instead of (pwm >> 15)
	Because of this, both Kp and Ki must double for same loop response
	Also, the PI loop is run at 4KHz instead of 16, so Ki must quadruple for same loop response
	So for same loop response, Kp is 2X and Ki is 8X */
	
	volatile long v1, v2;
	
	v1 = (long)config.Kp << 10;
	v2 = (long)config.Ki - v1;
	cli(); pi.K1 = v1; pi.K2 = v2; sei();
}

void fetch_rt_data(void)
{
	// fetch variable with interrupts off, then re-enable interrupts (interrupts can happen during NOPs)
	cli(); rt_data.throttle_ref = (volatile int)throttle_ref; sei();
	asm("nop"); asm("nop"); asm("nop"); asm("nop");

	// fetch variable with interrupts off, then re-enable interrupts (interrupts can happen during NOPs)
	cli(); rt_data.current_ref = (volatile int)current_ref; sei();
	asm("nop"); asm("nop"); asm("nop"); asm("nop");

	// fetch variable with interrupts off, then re-enable interrupts (interrupts can happen during NOPs)
	cli(); rt_data.current_fb = (volatile int)current_fb; sei();
	asm("nop"); asm("nop"); asm("nop"); asm("nop");

	// fetch variable with interrupts off, then re-enable interrupts (interrupts can happen during NOPs)
	cli(); rt_data.raw_hs_temp = (volatile unsigned)raw_hs_temp; sei();
	asm("nop"); asm("nop"); asm("nop"); asm("nop");
	
	// fetch variable with interrupts off, then re-enable interrupts (interrupts can happen during NOPs)
	cli(); rt_data.raw_throttle = (volatile unsigned)raw_throttle; sei();
	asm("nop"); asm("nop"); asm("nop"); asm("nop");

	// fetch variable with interrupts off, then re-enable interrupts (interrupts can happen during NOPs)
	cli(); rt_data.battery_amps = (volatile unsigned)battery_amps; sei();
	asm("nop"); asm("nop"); asm("nop"); asm("nop");

	// fetch variable with interrupts off, then re-enable interrupts (interrupts can happen during NOPs)
	cli(); rt_data.battery_ah = (volatile unsigned long)battery_ah; sei();
	asm("nop"); asm("nop"); asm("nop"); asm("nop");
}

#if EE_CONFIG_COPIES == 1
void read_config(void)
{
	eeprom_read_block(&config, (void *)EE_CONFIG_ADDRESS, sizeof(config));
	if (config.magic == 0x12ab) {
		// magic OK
		if (calc_block_crc(sizeof(config) - sizeof(unsigned), (unsigned char *)&config) ==
		  config.crc) {
		  	// CRC ok
			motor_overspeed_threshold = (unsigned long)config.motor_os_th << 10;
			bat_amp_lim_510 = (unsigned long)config.battery_amps_limit * (unsigned long)510;
			return;
		}
	}
	memcpy_P(&config, &default_config, sizeof(config));
}

void write_config(void)
{
	config.crc = calc_block_crc(sizeof(config) - sizeof(unsigned), (unsigned char *)&config);
	watchdog_disable();
	eeprom_write_block(&config, (void *)EE_CONFIG_ADDRESS, sizeof(config));
	watchdog_enable();
}
#else
void read_config(void)
{
	unsigned char lp, okmask;
	unsigned address;
	config_type cf;
	
	okmask = 0;
	address = EE_CONFIG_ADDRESS;
	for (lp = 0; lp < EE_CONFIG_COPIES; lp++) {
		okmask = okmask << 1;
		eeprom_read_block(&cf, (void *)address, sizeof(cf));
		address += sizeof(config_type);
		if (cf.magic == 0x12ab) {
			// magic OK
			if (calc_block_crc(sizeof(cf) - sizeof(unsigned), (unsigned char *)&cf) ==
			  cf.crc) {
			  	// CRC ok
				okmask = okmask | 0x01;
				memcpy(&config, &cf, sizeof(config));
				motor_overspeed_threshold = (unsigned long)config.motor_os_th << 10;
				bat_amp_lim_510 = (unsigned long)config.battery_amps_limit * (unsigned long)510;
			}
		}
	}
	if (okmask == 0) {
		// not even one good copy - copy default values and return
		memcpy_P(&config, &default_config, sizeof(config));
		return;
	}
	// we have at least one good copy - repair any bad copies
	address = EE_CONFIG_ADDRESS;
	for (lp = 0; lp < EE_CONFIG_COPIES; lp++) {
		okmask = okmask << 1;
		if ((okmask & (1 << EE_CONFIG_COPIES)) == 0) {
			// copy is bad
			watchdog_disable();
			eeprom_write_block(&config, (void *)address, sizeof(config));
			watchdog_enable();
		}
		address += sizeof(config_type);
	}
}

void write_config(void)
{
	unsigned char lp;
	unsigned address;
	
	config.crc = calc_block_crc(sizeof(config) - sizeof(unsigned), (unsigned char *)&config);
	address = EE_CONFIG_ADDRESS;
	for (lp = 0; lp < EE_CONFIG_COPIES; lp++) {
		watchdog_disable();
		eeprom_write_block(&config, (void *)address, sizeof(config));
		watchdog_enable();
		address += sizeof(config_type);
	}
}
#endif

void show_menu(void)
{
	#ifdef PWM8K
	strcpy_P(uart_str, PSTR("Cougar OS controller firmware v1.11b (8KPWM)\r\n"));
	#else
	strcpy_P(uart_str, PSTR("Cougar OS controller firmware v1.11b\r\n"));
	#endif
	uart_putstr();
}

void show_config(unsigned mask)
{
	if (mask & ((unsigned)1 << 0)) {
		strcpy_P(uart_str, PSTR("Kp=xxx Ki=xxx\r\n"));
		u16_to_str(&uart_str[3], config.Kp, 3);
		u16_to_str(&uart_str[10], config.Ki, 3);
		uart_putstr();
	}

	if (mask & ((unsigned)1 << 1)) {
		strcpy_P(uart_str, PSTR("throttle_min_raw_counts=xxxx throttle_max_raw_counts=xxxx\r\n"));
		u16_to_str(&uart_str[24], config.throttle_min_raw_counts, 4);
		u16_to_str(&uart_str[53], config.throttle_max_raw_counts, 4);
		uart_putstr();
	}

	if (mask & ((unsigned)1 << 2)) {
		strcpy_P(uart_str, PSTR("throttle_fault_raw_counts=xxxx\r\n"));
		u16_to_str(&uart_str[26], config.throttle_fault_raw_counts, 4);
		uart_putstr();
	}

	if (mask & ((unsigned)1 << 3)) {
		strcpy_P(uart_str, PSTR("throttle_pos_gain=xxx throttle_pwm_gain=xxx\r\n"));
		u16_to_str(&uart_str[18], config.throttle_pos_gain, 3);
		u16_to_str(&uart_str[40], config.throttle_pwm_gain, 3);
		uart_putstr();
	}

	if (mask & ((unsigned)1 << 4)) {
		strcpy_P(uart_str, PSTR("current_ramp_rate=xxx\r\n"));
		u16_to_str(&uart_str[18], config.current_ramp_rate, 3);
		uart_putstr();
	}

	if (mask & ((unsigned)1 << 5)) {
		strcpy_P(uart_str, PSTR("rtd_period=xxxxx\r\n"));
		u16_to_str(&uart_str[11], config.rtd_period, 5);
		uart_putstr();
	}

	if (mask & ((unsigned)1 << 6)) {
		strcpy_P(uart_str, PSTR("pwm_filter=x\r\n"));
		u16_to_str(&uart_str[11], config.pwm_filter, 1);
		uart_putstr();
	}

	if (mask & ((unsigned)1 << 7)) {
		strcpy_P(uart_str, PSTR("motor_os_threshold=xxxx motor_os_ftime=xxxx\r\n"));
		u16_to_str(&uart_str[19], config.motor_os_th, 4);
		u16_to_str(&uart_str[39], config.motor_os_ft, 4);
		uart_putstr();
	}

	if (mask & ((unsigned)1 << 8)) {
		strcpy_P(uart_str, PSTR("motor_os_dtime=xx pwm_deadzone=xx\r\n"));
		u16_to_str(&uart_str[15], config.motor_os_dt, 2);
		u16_to_str(&uart_str[31], config.pwm_deadzone, 2);
		uart_putstr();
	}
	if (mask & ((unsigned)1 << 9)) {
		strcpy_P(uart_str, PSTR("motor_speed_calc_amps=xxx\r\n"));
		u16_to_str(&uart_str[22], config.motor_sc_amps, 3);
		uart_putstr();
	}
	if (mask & ((unsigned)1 << 10)) {
		strcpy_P(uart_str, PSTR("battery_amps_limit=xxx\r\n"));
		u16_to_str(&uart_str[19], config.battery_amps_limit, 3);
		uart_putstr();
	}
	if (mask & ((unsigned)1 << 11)) {
		strcpy_P(uart_str, PSTR("precharge_time=xxx\r\n"));
		u16_to_str(&uart_str[15], config.precharge_time, 3);
		uart_putstr();
	}
}

void process_command(char *cmd, int x)
{
	if (!strcmp_P(cmd, PSTR("config"))) {
		show_config(0xffff);
	}
	else if (!strcmp_P(cmd, PSTR("save"))) {
		write_config();
		strcpy_P(uart_str, PSTR("configuration written to EE\r\n"));
		uart_putstr();
	}
	else if (!strcmp_P(cmd, PSTR("idle"))) {
		strcpy_P(uart_str, PSTR("AVR xxx% idle\r\n"));
		u16_to_str(&uart_str[4],
			(unsigned)(wait_time(100) * (unsigned long)100 / idle_loopcount),
			3);		
		uart_putstr();
	}
	else if (!strcmp_P(cmd, PSTR("restart"))) {
		watchdog_enable();
		while(1);
	}
	else if (!strcmp_P(cmd, PSTR("reset-ah"))) {
		cli(); battery_ah = 0; sei();
		strcpy_P(uart_str, PSTR("battery amp hours reset\r\n"));
		uart_putstr();
	}
	else if (!strcmp_P(cmd, PSTR("kp"))) {
		if ((unsigned)x <= 500) {
			config.Kp = x; config_pi();
			show_config((unsigned)1 << 0);
		}
	}
	else if (!strcmp_P(cmd, PSTR("ki"))) {
		if ((unsigned)x <= 500) {
			config.Ki = x; config_pi();
			show_config((unsigned)1 << 0);
		}
	}
	else if (!strcmp_P(cmd, PSTR("t-min-rc"))) {
		if ((unsigned)x <= 1023) {
			cli(); config.throttle_min_raw_counts = x; sei();
			show_config((unsigned)1 << 1);
		}
	}
	else if (!strcmp_P(cmd, PSTR("t-max-rc"))) {
		if ((unsigned)x <= 1023) {
			cli(); config.throttle_max_raw_counts = x; sei();
			show_config((unsigned)1 << 1);
		}
	}
	else if (!strcmp_P(cmd, PSTR("t-fault-rc"))) {
		if ((unsigned)x <= 1023) {
			cli(); config.throttle_fault_raw_counts = x; sei();
			show_config((unsigned)1 << 2);
		}
	}
	else if (!strcmp_P(cmd, PSTR("t-pos-gain"))) {
		if ((unsigned)x <= 128) {
			cli(); config.throttle_pos_gain = x; sei();
			show_config((unsigned)1 << 3);
		}
	}
	else if (!strcmp_P(cmd, PSTR("t-pwm-gain"))) {
		if ((unsigned)x <= 128) {
			cli(); config.throttle_pwm_gain = x; sei();
			show_config((unsigned)1 << 3);
		}
	}
	else if (!strcmp_P(cmd, PSTR("c-rr"))) {
		if ((unsigned)x <= 100) {
			cli(); config.current_ramp_rate = x; sei();
			show_config((unsigned)1 << 4);
		}
	}
	else if (!strcmp_P(cmd, PSTR("rtd-period"))) {
		if ((unsigned)x <= 32000) {
			config.rtd_period = x;
			tm_show_data = get_time();
			show_config((unsigned)1 << 5);
		}
	}
	else if (!strcmp_P(cmd, PSTR("pwm-filter"))) {
		if ((unsigned)x <= 3) {
			cli(); config.pwm_filter = x; sei();
			show_config((unsigned)1 << 6);
		}
	}
	else if (!strcmp_P(cmd, PSTR("motor-os-th"))) {
		if ((unsigned)x <= 9999) {
			config.motor_os_th = x;
			motor_overspeed_threshold = (unsigned long)config.motor_os_th << 10;
			show_config((unsigned)1 << 7);
		}
	}
	else if (!strcmp_P(cmd, PSTR("motor-os-ft"))) {
		if ((unsigned)x <= 9999) {
			config.motor_os_ft = x;
			show_config((unsigned)1 << 7);
		}
	}
	else if (!strcmp_P(cmd, PSTR("motor-os-dt"))) {
		if ((unsigned)x <= 99) {
			config.motor_os_dt = x;
			show_config((unsigned)1 << 8);
		}
	}
	else if (!strcmp_P(cmd, PSTR("pwm-deadzone"))) {
		if ((unsigned)x <= 99) {
			config.pwm_deadzone = x;
			show_config((unsigned)1 << 8);
		}
	}
	else if (!strcmp_P(cmd, PSTR("motor-sc-amps"))) {
		if ((unsigned)x <= MAX_CURRENT_REF) {
			config.motor_sc_amps = x;
			show_config((unsigned)1 << 9);
		}
	}
	else if (!strcmp_P(cmd, PSTR("bat-amps-lim"))) {
		if ((unsigned)x <= MAX_CURRENT_REF) {
			config.battery_amps_limit = x;
			bat_amp_lim_510 = (unsigned long)config.battery_amps_limit * (unsigned long)510;
			show_config((unsigned)1 << 10);
		}
	}
	else if (!strcmp_P(cmd, PSTR("pc-time"))) {
		if ((unsigned)x <= 999) {
			config.precharge_time = x;
			show_config((unsigned)1 << 11);
		}
	}
}

void thermal_cutback(void)
{
	unsigned u;
	
	if (rt_data.raw_hs_temp > THERMAL_CUTBACK_START) {
		// time to do thermal cutback
		u = (rt_data.raw_hs_temp - THERMAL_CUTBACK_START) / 8;
		// Paul's code had steps of 8 ADC counts (7/8, 6/8, 5/8, 4/8, 3/8, 2/8, 1/8, 0/8)
		if (u >= 7) u = 0;						// do not deliver any current (too hot)
		else {
			// u is now 6 to 0 (for 1/8 to 7/8 current)
			u = 7 - u;
			// u is now 1 for 1/8, 2 for 2/8, 7 for 7/8, etc.
			u = (u * MAX_CURRENT_REF) / 8;
			// u should now be maximum current to deliver (same result as Paul's "if-then" sequence)
		}
	}
	else {
		// can deliver full current
		u = MAX_CURRENT_REF;
	}
	cli(); max_current_ref = u; sei();
}

int main(void)
{
	int x;
	unsigned tm_100;
	unsigned char cmdpos, cmdok;
	char cmd[32];
	
#ifdef crc_address
	unsigned crc1, crc2;
	
	crc1 = pgm_read_word(crc_address);		// read program CRC
	crc2 = calc_prog_crc(crc_address);		// read program CRC
	if (crc1 != crc2) {
		// program CRC error
		while(1);							// do nothing for ever
	}
#endif

	PORTD = 0xff & ~PD_LED;					// PORTD weak pullups, LED output pin low (LED off)
	DDRD = PD_LED | PD_CONTACTOR;			// two pins outputs
	PORTC = ~PC_ANALOGS_USED;				// weaks pull ups on _except_ for analog input pins
	PORTB = 0xff & ~PB_PWM;					// PWM output low, other outputs high, weak pullups on
	DDRB = PB_PWM | PB_OC_CLEAR;			// two pins outputs
	
	// for ATMEGA168 disable digital input buffers on pins used for ADC - page 258 of Mega168 PDF
	#ifdef MEGA168
	DIDR0 = (1 << ADC2D) | (1 << ADC1D) | (1 << ADC0D);
	#endif
	
	// External Vcc (5v) for analog reference
	ADMUX = (1 << REFS0);
	// enable ADC, prescale = 128, so conversion clock is 16M / 128 = 125KHz
	// a conversion take 13 cycles, at 125KHz equals 104uS
	// the fastest we can convert is 9.6 KHz
	// if we convert every other PWM cycle, that is 8KHz
	// see page 198 of ATMEG8 manual
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	// do a conversion on channel 2 - the first conversion takes 25 cycles - so do it now
	ADMUX = ADMUX = (1 << REFS0) | 2;
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) | (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
	
	// set up input capture 1 interrupt at 976Hz
	// this is only for temporary timing until timer 1 is used for PWM
	// the reason for 976Hz instead of 1000Hz is explained below
	TCNT1 = 0;								// load 16 bit counter 1
	ICR1 = (long)F_OSC / 976;				// timer at 976 Hz
	TCCR1A = 0;								// no output action on match
	// let counter 1 run at fosc, reset to 0 at ICR1
	TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);
	TIMSK = (1 << TICIE1);					// enable input capture 1 interrupt
	
	read_config();												// read config from EEprom

	if (config.precharge_time > 0) {
		// if precharge timer enabled
		fault_bits |= PRECHARGE_WAIT;
		precharge_timer = config.precharge_time + 5;
	}
	config_pi();							// configure PI loop from config structure
	// interrups are now enabled by config_pi() - sei() instruction in config_pi()

	idle_loopcount = wait_time(100);		// wait 100mS and remember how many loops we did

	watchdog_enable();						// enable watchdog now

	// now that voltages have settled, measure Vref
	if (!measure_vref()) {
		// vref out of range
		fault_bits |= VREF_FAULT;
	}
	// clear overcurrent fault (powerup in unknown state)
	clear_oc();

	// now configure timer 1 for PWM
	TIMSK = 0;								// no timer 1 interrupt
	TCCR1B = 0;								// stop counter 1
	TCNT1 = 0;								// load 16 bit counter 1
    OCR1A = 0;								// set initial PWM duty value to 0
	#ifdef PWM8K
	TCCR1A =  (1 << COM1A1) | (1 << WGM11) | (1 << WGM10);	// Pase Correct PWM mode, 10 bit
	#else
	TCCR1A =  (1 << COM1A1) | (1 << WGM11);					// Pase Correct PWM mode, 9 bit
	#endif
	TCCR1B = (1 << CS10);					// Pre-scaler = 1
    OCR1A = 0;								// again, just to be safe
	TIMSK = (1 << TOIE1);					// enable overflow 1 interrupt
	// now the PWM frequency = 16000000 / (1 << 9) / 2
	// so PWM frequency = 16000000 / 1024 = 15625Hz
	// now, counter_1k is incremented every 16 interrupt, so 15625 / 16 = 976.5625Hz
	// this is why we run SIG_INPUT_CAPTURE1 at 976Hz
	
	setup_uart();									// uart 19200,n,8,1
	show_menu();									// might as well
	// init some time variables
	tm_show_data = tm_100 = get_time();
	// now listen on serial port for commands
	memset(cmd, 0, sizeof(cmd)); cmdpos = 0;
	while (1) {
		wdt_reset();
		x = uart_getch();
		if (x >= 0) {
			if (x != 0x0d) {
				// not a CR
				uart_putch(x);							// echo the character back
				if (cmdpos < (sizeof(cmd) - 1)) {
					cmd[cmdpos++] = x;					// add character to command string
					cmd[cmdpos] = 0;					// and terminate command string
				}
			}
			else {
				// got a CR
				uart_putch(0x0a); uart_putch(0x0d);		// echo back LF and CR
				cmdok = 0;
				if (cmdpos > 0) {
					for (x = 0; x < (int)(cmdpos - 1); x++) {
						if (cmd[x] == ' ') {
							// have a space character, terminate at this position
							// and get numeric value
							cmd[x] = 0; x = atoi(&cmd[x + 1]); cmdok = 1;
							break;
						}
					}
					if (!cmdok) {
						// no space found - command OK but set numeric parameter to -1
						x = -1; cmdok = 1;
					}
				}
				if (cmdok) {
					// cmd is string, x is numeric value
					process_command(cmd, x);
				}
				else show_menu();
				// reset command string
				cmdpos = 0; cmd[0] = 0;
			}
		}
		/* add non time-critical code below */
		// fetch real time data
		fetch_rt_data();
		// do thermal cutback (based on real time data)
		thermal_cutback();
		// if rtd_period not zero display rt data at specified intervals
		if (config.rtd_period) {
			if (diff_time(tm_show_data) >= config.rtd_period) {
				// config.rtd_period mS passed since last time, adjust tm_show_data to trigger again
				tm_show_data += config.rtd_period;
				strcpy_P(uart_str, PSTR("TR=xxx CR=xxx CF=xxx PW=xxx HS=xxxx RT=xxxx FB=xx BA=xxx AH=xxx.x\r\n"));
				u16_to_str(&uart_str[3], rt_data.throttle_ref, 3);
				u16_to_str(&uart_str[10], rt_data.current_ref, 3);
				u16_to_str(&uart_str[17], rt_data.current_fb, 3);
				cli(); x = ocr1a_ghost; sei();
				u16_to_str(&uart_str[24], (unsigned)x, 3);
				u16_to_str(&uart_str[31], rt_data.raw_hs_temp, 4);
				u16_to_str(&uart_str[39], rt_data.raw_throttle, 4);
				u16x_to_str(&uart_str[47], fault_bits, 2);
				u16_to_str(&uart_str[53], rt_data.battery_amps, 3);
				// battery_ah is in amp milliseconds, to convert to Ah divide by 3600000
				// well almost, summation is at 976.56 hertz, so divide by 3515625
				// we will divide by 351562 to get tenths of Ah so we can display a digit after the dp
				x = rt_data.battery_ah / (unsigned long)351562;
				if (x > 9999) x = 9999;
				u16_to_str(&uart_str[60], x / 10, 3);
				u16_to_str(&uart_str[64], x % 10, 1);
				uart_putstr();
			}
		}
		if (diff_time(tm_100) >= 100) {
			// 100 mS passed since last time, adjust tm_100 to trigger again
			tm_100 += 100;
			// if fault, toggle LED, else light LED
			if (fault_bits) {
				PORTD ^= PD_LED;
				if (fault_bits & HPL_FAULT) {
					// high pedal lockout fault set - reset it if current ref is below threshold
					if (rt_data.throttle_ref <= HPL_THROTTLE_THRESHOLD) {
						cli(); fault_bits &= ~HPL_FAULT; sei();
					}
				}
				if (fault_bits & PRECHARGE_WAIT) {
					if (precharge_timer == 0) {
						// remove precharge wait condition to enable power to motor
						cli(); fault_bits &= ~PRECHARGE_WAIT; sei();
					}
					else {
						// with half a second left in precharge, close contactor
						if (precharge_timer <= 5) PORTD &= ~PD_CONTACTOR;
						precharge_timer--;
					}
				}
			}
			else PORTD |= PD_LED;
		}
	}
	return(0);
}
