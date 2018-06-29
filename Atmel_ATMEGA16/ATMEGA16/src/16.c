/*Very Important - change F_CPU to match target clock 
  Note: default AVR CLKSEL is 1MHz internal RC
  This program transmits continuously on USART. Interrupt is used for 
	Receive character, which is then transmitted instead. LEDs are used 
	as a test. Normal RX routine is included but not used.
  Change USART_BAUDRATE constant to change Baud Rate
*/
#define F_CPU 8000000UL  // 8 MHz

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define _BV(bit) (1 << (bit))
#define USART_BAUDRATE (9600)   // Define baud rate
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

/* Interrupt Service Routine for Receive Complete 
NOTE: vector name changes with different AVRs see AVRStudio -
Help - AVR-Lib c reference - Library Reference - <avr/interrupt.h>: Interrupts
for vector names other than USART_RXC_vect for ATmega32 */
void USART_Init(void);
uint8_t uart_getc(void);
void uart_putc(uint8_t data);
uint8_t uart_available(void);

void timer0_init(void);
void timer1_init(void);
void timer2_init(void);

void see_buf_RX(void);
void for_blue_led(void);
void for_green_led(void);

/* This variable is volatile so both main and RX interrupt can use it.
It could also be a uint8_t type */
#define UART_RECEIVE_INTERRUPT   USART_RXC_vect
#define UART_TRANSMIT_INTERRUPT  USART_UDRE_vect
#define UART_STATUS   UCSRA
#define UART_CONTROL  UCSRB
#define UART_DATA     UDR
#define UART_UDRIE    UDRIE

#define UART_BUFFER_OVERFLOW  0x040              /**< receive ringbuffer overflow */

#define UART_RX_BUFFER_SIZE (32)
#define UART_RX_BUFFER_MASK (UART_RX_BUFFER_SIZE - 1)

#define UART_TX_BUFFER_SIZE (32)
#define UART_TX_BUFFER_MASK (UART_TX_BUFFER_SIZE - 1)

static volatile uint8_t UART_TxBuf[UART_TX_BUFFER_SIZE];
static volatile uint8_t UART_RxBuf[UART_RX_BUFFER_SIZE];

static volatile uint8_t UART_TxHead;
static volatile uint8_t UART_TxTail;
static volatile uint8_t UART_RxHead;
static volatile uint8_t UART_RxTail;

uint8_t value;  

uint8_t b_start;
uint8_t b_command;
uint8_t b_param;
uint8_t b_checksum;
uint8_t b_end;

uint16_t mul_for_green_timer;
uint16_t new_mul_for_green_timer;
uint8_t flag;

ISR(UART_RECEIVE_INTERRUPT)
/*************************************************************************
Function: UART Receive Complete interrupt
Purpose:  called when the UART has received a character
**************************************************************************/
{
	uint8_t tmphead;
	uint8_t data;
	//uint8_t usr;
	
	/* read UART status register and UART data register */
	//usr  = UART_STATUS;
	data = UART_DATA;
	
	/* calculate buffer index */
	tmphead = (UART_RxHead + 1) & UART_RX_BUFFER_MASK;
	//tmphead = (UART_TxHead + 1) & UART_RX_BUFFER_MASK;
	
	/* store new index */
	UART_RxHead = tmphead;
	//UART_TxHead = tmphead;
	/* store received data in buffer */
	UART_RxBuf[tmphead] = data;
	//UART_TxBuf[tmphead] = data;
}

ISR(UART_TRANSMIT_INTERRUPT)
/*************************************************************************
Function: UART Data Register Empty interrupt
Purpose:  called when the UART is ready to transmit the next byte
**************************************************************************/
{
	uint8_t tmptail;

	if (UART_TxHead != UART_TxTail) 
	{
		/* calculate and store new buffer index */
		tmptail = (UART_TxTail + 1) & UART_TX_BUFFER_MASK;
		UART_TxTail = tmptail;
		/* get one byte from buffer and write it to UART */
		UART_DATA = UART_TxBuf[tmptail];  /* start transmission */
	}
	else
	{
		/* tx buffer empty, disable UDRE interrupt */
		//UART_CONTROL |= _BV(UART_UDRIE); // разрешить прерывание при опустошении буфера передатчика
		UART_CONTROL &= ~_BV(UART_UDRIE); // запретить прерывание при опустошении буфера
	}
}

void USART_Init(void)
{
	UART_TxHead = 0;
	UART_TxTail = 0;
	UART_RxHead = 0;
	UART_RxTail = 0;

	// Set baud rate
	UBRRL = BAUD_PRESCALE;// Load lower 8-bits into the low byte of the UBRR register
	UBRRH = (BAUD_PRESCALE >> 8); 
	/* Load upper 8-bits into the high byte of the UBRR register
    Default frame format is 8 data bits, no parity, 1 stop bit
	to change use UCSRC, see AVR data sheet*/ 

	/* Set frame format: asynchronous, 8data, no parity, 1 stop bit */
	UCSRC = (1 << URSEL) | (3 << UCSZ0);

	// Enable receiver and transmitter and receive complete interrupt 
	UCSRB = (_BV(RXCIE)|_BV(RXEN)|_BV(TXEN));
	// разрешить прерывание при приеме символа
	// TXEN - Transmit Enable Передатчик  
	// RXEN - Receive  Enable Приемник
}

/*************************************************************************
Function: uart_getc()
Purpose:  return byte from ringbuffer (Receive buffer- RX)
Returns:  lower byte:  received byte from ringbuffer
          higher byte: last receive error
**************************************************************************/
uint8_t uart_getc(void)
{
	uint8_t tmptail;
	uint8_t data;

	if (UART_RxHead == UART_RxTail) 
	{
		return 0x00;   /* no data available */
	}
	
	/* calculate / store buffer index */
	tmptail = (UART_RxTail + 1) & UART_RX_BUFFER_MASK;
	
	UART_RxTail = tmptail;
	
	/* get data from receive buffer */
	data = UART_RxBuf[tmptail];

	return data;

} /* uart_getc */

/*************************************************************************
Function: uart_putc()
Purpose:  write byte to ringbuffer for transmitting via UART (Send buffer- TX)
Input:    byte to be transmitted
Returns:  none
**************************************************************************/
void uart_putc(uint8_t data)
{
	uint8_t tmphead;
		
	tmphead = (UART_TxHead + 1) & UART_TX_BUFFER_MASK;
		
	UART_TxBuf[tmphead] = data;
	UART_TxHead = tmphead;

	/* enable UDRE interrupt */
	//UART0_CONTROL |= _BV(UART0_UDRIE);
	//TCNT1 = 0;
} /* uart_putc */

uint8_t uart_available(void)
{
	uint8_t ret;
	
	ret = (UART_RX_BUFFER_SIZE + UART_RxHead - UART_RxTail) & UART_RX_BUFFER_MASK;
	
	return ret;
} /* uart_available */


ISR(TIMER0_COMP_vect)
{
	//
}
// this ISR is fired whenever a match occurs
// hence, toggle led here itself..
ISR (TIMER1_COMPA_vect) // red_led
{
	PORTD |= 0x08;
}

ISR (TIMER2_COMP_vect) // green_led
{
	new_mul_for_green_timer--;

	if (new_mul_for_green_timer < (mul_for_green_timer >> 1) )
		PORTD |= 0x10;
	else
		PORTD &= 0xEC;

	if (new_mul_for_green_timer < 1)
		new_mul_for_green_timer = mul_for_green_timer;
}

void timer0_init(void)
{
	// page 83
	//8 bit timer-counter 0
	TCCR0 |= _BV(CS02) | _BV(CS00);	// 1024 PRESCALER
	TCCR0 |= _BV(WGM01);	// CTC MODE
	TCNT0 = 0;
	OCR0 = 78;  // 10 ms
	TIMSK |= _BV(OCIE0);	// Enable the timer0 compare interrupt

	TCCR0 = 0x00;
}

// initialize timer, interrupt and variable
void timer1_init(void) // red_led
{
	// 113 page data sheet Atmega16
	// set up timer with 
	TCCR1B |= _BV(CS10) | _BV(CS12);	// 1024 PRESCALER
	TCCR1B |= _BV(WGM12);	// CTC(Clear Timer on Compare) MODE
	// initialize counter
	TCNT1 = 0;
	// initialize compare value
	OCR1A = 7813; // 1 sec = ( 8 MHz / 1024 prescaler)
	// Enable the timer1 compare A interrupt
	TIMSK |= _BV(OCIE1A); // start timer
}

void timer2_init(void) // green_led
{
	// page 127
 	//8 bit timer-counter 1
 	TCCR2 |= _BV(CS22) | _BV(CS21) | _BV(CS20);	// 1024 PRESCALER
 	TCCR2 |= _BV(WGM21);	// CTC MODE
 	TCNT2 = 0; 
 	//OCR2 = 78; // 10 ms
	OCR2 = 7; // 1 ms
 	TIMSK |= _BV(OCIE2);	// Enable the timer0 compare interrupt
}

void for_blue_led(void)
{
	if (b_command == 0x30)
	{
		if (b_param == 0x30)
			PORTD &= 0xDC; // off blue;
		else
			PORTD |= 0x20; // on blue;
	}

	else
	{
		b_param = PORTD;
		if (b_param & 0x20)
			b_param = 0x31;
		else
			b_param = 0x30;
	//      param = 0x30 + ((PORTD >> 4) & 0x01);
	}
}

void for_green_led(void)
{
	if (b_command == 0x32)
	{
		if (b_param == 0x30)
		{
			PORTD &= 0xEC; // off green_led
			TCCR2 = 0x00;
			mul_for_green_timer = 0;
		}
		else if (b_param == 0x31)
			mul_for_green_timer = 250;
		else if (b_param == 0x32)
			mul_for_green_timer = 500;
		else if (b_param == 0x33)
			mul_for_green_timer = 1000;
		else // (b_param == 0x34)
			mul_for_green_timer = 2000;

		// PORTD |= 0x10; // on gree_led
	}
	else
	{
		if ( mul_for_green_timer == 0)
			b_param = 0x30;
		else if (mul_for_green_timer == 250)
			b_param = 0x31;
		else if(mul_for_green_timer == 500)
			b_param = 0x32;
		else if(mul_for_green_timer == 1000)
			b_param = 0x33;
		else // mul_for_green_timer == 2000
			b_param = 0x34;
	
	}

	new_mul_for_green_timer = mul_for_green_timer;

	if(b_param != 0x00)
		timer2_init();
}

void see_buf_RX(void)
{
	value = uart_available();
	if (value >= 5)
	{
		b_start = uart_getc();
		if (b_start == 0x02)
		{
			b_checksum = 0;
			b_checksum += b_start;
			
			b_command = uart_getc();
			if ( (b_command != 0x30) && // foo command
			(b_command != 0x31) &&
			(b_command != 0x32) &&
			(b_command != 0x33) )
				return;
			b_checksum += b_command;

			b_param = uart_getc();
			if ( (b_param != 0x30) &&  // foo param
			(b_param != 0x31) &&
			(b_param != 0x32) &&
			(b_param != 0x33) &&
			( (b_command == 0x31) && (b_param != 0x30) ) &&
			( (b_command == 0x32) && (b_param != 0x34) ) )
				return;
			b_checksum += b_param;

			value = uart_getc();
			if (b_checksum != value)  // foo checksum
				return;

			b_end = uart_getc();
			if (b_end != 0x06)
				return;

			if (
				(b_command == 0x30 && 
				(b_param == 0x30 || b_param == 0x31) )
				|| 
				(b_command == 0x31 && b_param == 0x30 ))
				for_blue_led();
			else if ( 
				(b_command == 0x32 && 
				(b_param == 0x30 || b_param == 0x31 || b_param == 0x32 || b_param == 0x33 || b_param == 0x34) ) 
				||
				(b_command == 0x33 && b_param == 0x30 ) )
				for_green_led();
			else
				return;

			TCNT1 = 0; // off red_led
			PORTD &= 0xF4;

			uart_putc(b_start);
			uart_putc(b_command + 0x10);
			uart_putc(b_param);
			uart_putc(b_checksum + 0x10);
			uart_putc(b_end);

			UART_CONTROL |= _BV(UART_UDRIE);
		}
		else
			return;
	}
}

int main(void)
{
   USART_Init();  // Initialize USART
   timer0_init();
   timer1_init();
   timer2_init();
   sei();         // enable all interrupts

   DDRD = 0xFF;
   PORTD = 0;

   flag = 1;
   mul_for_green_timer = 0;
   new_mul_for_green_timer = mul_for_green_timer;

   while(1)
   {
     see_buf_RX();
   }
}