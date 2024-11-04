#include <Arduino.h>

// on arduino nano test:
// - SDA on D12
// - SCL on D11
// - LED on D13 (LED_BUILTIN)

#define I2C_ADDRESS 0x22
#define I2C_SCL_PIN PB3			// D11
#define I2C_SDA_PIN PB4			// D12
#define I2C_BUS_FREE_TIME 5 // in usec
#define I2C_BUFFER_SIZE 4

#include "i2c_sw_target.hpp"

#define LED _BV(PB5) // D13 / LED_BUILTIN
void led_high() { PORTB |= LED; }
void led_low() { PORTB &= ~LED; }

uint8_t int_data[I2C_BUFFER_SIZE];
uint8_t int_data_len = 0;

void on_i2c_event(const bool read, uint8_t *data, uint8_t &len)
{
	if (read)
	{
		// read internal data to controller
		memcpy(data, int_data, int_data_len);
		len = int_data_len;

		Serial.print("[R] len=");
		Serial.println(int_data_len);

		led_high();
	}
	else
	{
		// write controller data to internal
		memcpy(data, int_data, len);
		int_data_len = len;

		Serial.print("[W] len=");
		Serial.println(int_data_len);

		led_low();
	}
}

int main(void)
{
	Serial.begin(115200);
	Serial.println("ready!");

	DDRB |= LED;

	wdt_enable(WDTO_15MS); // enable wdt reset in 15 ms

	WireTarget.listen();
}
