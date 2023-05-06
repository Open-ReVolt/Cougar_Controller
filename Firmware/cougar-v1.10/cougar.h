#define F_OSC 16000000					// oscillator-frequency in Hz

#define PROGSTART 0x0000				// program start address

#define EE_CONFIG_ADDRESS 0				// address of config in EEprom

#define EE_CONFIG_COPIES 4				// store multiple copies of config in EE prom

#define OC_CLEAR_ENABLED				// defin to enable AVR to clear OC fault

#define NUM_OC_CYCLES_OFF 4				// number of overcurrent cycles off (at 4KHz)

#define THROTTLE_FAULT_COUNTS 200		// number of milliseconds throttle must be bad to raise fault

#define HPL_THROTTLE_THRESHOLD 2		// high pedal lockout threshold (compared to throttle_ref [0 to 511])

#define MAX_CURRENT_REF 511				// max current ref into PI loop

#define MOTOR_OS_DETECT_TIME 10			// number of milliseconds motor must be in overspeed to raise fault

#define PWM_DEADZONE 5					// PWM counts before FETs start conducting (for motor overspeed)

//#define THERMAL_CUTBACK_START 780		// start at 75 degC, end at 85 degC
#define THERMAL_CUTBACK_START 670		// start at 75 degC, end at 85 degC

// PWM8K defined for 8KHz PWM, not defined for 16KHz PWM
// for "buildall" command, this is defined automatically and must be commented out below
//#define PWM8K

#define THROTTLE_FAULT (1 << 0)
#define VREF_FAULT (1 << 1)
#define PRECHARGE_WAIT (1 << 5)
#define MOTOR_OS_FAULT (1 << 6)
#define HPL_FAULT (1 << 7)

#define PINB_OC_STATE (1 << PINB0)		// OC state (high means fault)
#define PB_PWM (1 << PB1)				// PWM output pin (high to turn FETs on)
#define PB_OC_CLEAR (1 << PB2)			// OC clear (low to clear, high for normal operation)

#define PD_LED (1 << PD6)				// IDLE LED - high to light LED
#define PD_CONTACTOR (1 << PD7)			// Contactor Opto LED - low to light LED

// three analog inputs used, these pins must be inputs and weak pullups off
#define PC_ANALOGS_USED ((1 << PC0) | (1 << PC1) | (1 << PC2))

#define PARITY_NONE	0x00
#define PARITY_EVEN	0x02
#define PARITY_ODD	0x03

#define BITS_7_1	0x02
#define BITS_7_2	0x06
#define BITS_8_1	0x03
#define BITS_8_2	0x07

#define UART_RXBUF_SIZE 16
#define UART_TXBUF_SIZE 128

// function prototypes

// get character from uart fifo, return -1 if fifo empty
int uart_getch(void);

// put character to uart (return 1 if fifo full, else 0)
unsigned char uart_putch(char c);

// put uart_str to uart
void uart_putstr(void);

// set up UART to 19200,n,8,1
void setup_uart(void);
