/**
 * Copyright (c) 2020, Taddy G. <fotonix@pm.me>
 * I2C ti UART converter
 */

#include <stdint.h>
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include "uart.h"

// on arduino nano test:
// - SDA on D12
// - SCL on D11
// - LED on D13 (LED_BUILTIN)

#define __inline
#define IS_ATTINY13 (defined(__AVR_ATtiny13__) || defined(__AVR_ATtiny13A__))

#define I2C_SLAVE_ADDR 0x22

#define SCL _BV(PB3)
#define SDA _BV(PB4)
#define MASK_SDA _BV(PCINT4)

#define R_SCL (PINB & SCL)
#define R_SDA (PINB & SDA)
#define R_BOTH (PINB & (SDA | SCL))

#define LED _BV(PB5)
#define LED_L() PORTB &= ~LED;
#define LED_H() PORTB |= LED;

#define BUS_FREE_TIME 5 // in usec

#define nop()                    \
	do                             \
	{                              \
		__asm__ __volatile__("nop"); \
	} while (0)

#if IS_ATTINY13
#define irq_en()       \
	do                   \
	{                    \
		GIFR |= _BV(PCIF); \
		nop();             \
		sei();             \
	} while (0)
#else
#define irq_en()         \
	do                     \
	{                      \
		PCIFR |= _BV(PCIF0); \
		nop();               \
		sei();               \
	} while (0)
#endif

enum
{
	SEQ_BUS_FREE = 0,
	SEQ_START,
	SEQ_DATA,
	SEQ_STOP
};

// Internal I2C status
volatile int8_t status = 0;

void __inline i2c_clk_keep(void)
{
	DDRB |= SCL;	 // SCL out
	PORTB &= ~SCL; // SCL=0
	nop();				 // for sync
}

void __inline i2c_clk_free(void)
{
	DDRB &= ~SCL; // SCL in
	nop();				// for sync
}

// sda go high / idle
void __inline i2c_sda_high()
{
	DDRB &= ~SDA; // SDA as in
	PORTB |= SDA; // SDA pull-up
	nop(); 				// for sync
}

// sda drive low
void __inline i2c_sda_low()
{
	DDRB |= SDA;	 // SDA as out
	PORTB &= ~SDA; // SDA=0
	nop();				 // for sync
}

void __inline i2c_ack(void)
{
	i2c_sda_low();
	while (!R_SCL)
		; // Wait for SCL=1
	while (R_SCL)
		;						// Wait for SCL=0

	i2c_sda_high();
}

// returns R/W bit: 0 = write, 1 = read
uint8_t __inline i2c_detect_addr(void)
{
	uint8_t register bshift = 7;
	uint8_t i;
	uint8_t d = 0;

	for (i = 0; i < 8; i++)
	{
		while (!R_SCL)
			; // Wait for SCL=1
		// Get SDA state
		if (R_SDA)
		{
			d |= 1 << bshift;
			while (R_SCL)
				; // Wait for SCL=0
		}
		else
		{
			while (1)
			{
				uint8_t register x = PINB;
				if ((x & SCL) == 0) // SCL = 0
					break;
				if (x & SDA)
				{ // SDA 0->1 stop
					status = SEQ_STOP;
					return;
				}
			}
		}
		bshift--;
	}

	uint8_t rw = d & 1;
	d >>= 1;

	if (d == I2C_SLAVE_ADDR)
	{
		status = SEQ_DATA;
		i2c_ack(); // ACK on our addr
	}
	else
	{
		status = SEQ_BUS_FREE;
		irq_en();
	}

	wdt_reset();
	return rw;
}

uint8_t __inline i2c_get_byte(void)
{
	uint8_t bshift = 7;
	uint8_t d = 0;
	uint8_t i;

	cli();
	for (i = 0; i < 8; i++)
	{
		while (!R_SCL)
			; // Wait for SCL=1
		// Get SDA state
		if (R_SDA)
		{
			d |= 1 << bshift;
			while (R_SCL)
				; // Wait for SCL=0
		}
		else
		{
			while (1)
			{
				uint8_t register x = PINB;
				if ((x & SCL) == 0)
					break;
				if (x & SDA)
				{
					status = SEQ_STOP;
					return 0;
				}
			}
		}
		bshift--;
	}

	i2c_ack();
	wdt_reset();
	irq_en();

	return d;
}

void __inline i2c_put_bytes(uint8_t *bytes, uint8_t length)
{
	uint8_t i, j;

	cli();
	for (j = 0; j < length; j++)
	{
		uint8_t byte = bytes[j];
		uint8_t bshift = 7;

		for (i = 0; i < 8; i++)
		{
			while (R_SCL)
				; // Wait for SCL=0

			if (byte & (1 << bshift))
			{
				i2c_sda_high();
			}
			else
			{
				i2c_sda_low();
			}
			bshift--;

			while (!R_SCL)
				; // Wait for SCL=1
		}

		while (R_SCL)
			; // Wait for SCL=0

		i2c_sda_high();
		
		while (!R_SCL)
			; // Wait for SCL=1
		while (R_SCL)
			; // Wait for SCL=0
	}
	
	i2c_sda_high();
	while (R_SCL)
		; // Wait for SCL=0

	wdt_reset();
	irq_en();
}

void i2c_wait_for_start(void)
{
	uint8_t register cnt = BUS_FREE_TIME;

	// LED_H();
	DDRB &= ~(SCL | SDA);	 // SDA|SCL in
	PORTB &= ~(SCL | SDA); // Hiz

	wdt_reset();

	// Wait for SCL=1 & SDA=1 stable for BUS_FREE_TIME
	while (cnt)
	{
		if (R_BOTH != (SCL | SDA))
			cnt = BUS_FREE_TIME;
		else
			cnt--;

		_delay_us(1);
	}
	status = SEQ_BUS_FREE;

	irq_en();

	// Wait for detecting START sequence
	while (status != SEQ_START)
	{
		wdt_reset();
	}
	cli();

	// Wait for SLC=0 SDA=0
	while (R_BOTH)
	{
		wdt_reset();
	}
}

ISR(PCINT0_vect)
{
	uint8_t register pin;

	wdt_reset();

	pin = PINB;
	if (pin & SCL)
	{
		if ((pin & SDA) == 0)
		{
			status = SEQ_START;
			// LED_L();
		}
		else
		{
			status = SEQ_STOP;
			// LED_H();
		}
	}
}

ISR(WDT_vect)
{
	uart_puts("\nI2C-UART (WDT reset)\n");
}

int main(void)
{
	uint8_t register byte;

#if IS_ATTINY13
	PCMSK = MASK_SDA;
	GIMSK |= _BV(PCIE);
	GIFR |= _BV(PCIF);
#else
	PCMSK0 = MASK_SDA;
	PCICR |= _BV(PCIE0);
	PCIFR |= _BV(PCIF0);
#endif
	// DDRB |= LED;

	uart_setup(); // Setup UART Tx pin as out

#if IS_ATTINY13
	wdt_enable(WDTO_15MS); // Set prescaler to 15ms
	WDTCR |= _BV(WDTIE);	 // Enable WD irq
#endif
	cli();

	while (1)
	{
		i2c_wait_for_start();
		uint8_t rw = i2c_detect_addr(); // 1 = read, 0 = write

		if (status == SEQ_DATA)
		{
			if (rw == 0)
			{
				while (1)
				{
					byte = i2c_get_byte();
					if (status == SEQ_STOP)
					{
						cli();
						break;
					}
					else
					{
						i2c_clk_keep();

						if (byte == 0xaa)
						{
							//LED_L();
						}
						else if (byte == 0x55)
						{
							//LED_H();
						}

						i2c_clk_free();
					}
				}

				LED_L();
			}
			else
			{
				LED_H();

				uint8_t bytes[] = {0x55, 0x66, 0x77, 0xaa};
				i2c_put_bytes(bytes, 4);
			}
		}
	}
}
