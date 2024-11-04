/**
 * basic software i2c target implementation for the attiny13 and similar avr microcontrollers.
 * uses no hardware i2c peripheral or interrupts, just bit-banging on two GPIO pins.
 * 
 * memory usage:
 * - RAM: 1 + I2C_BUFFER_SIZE bytes
 * - Flash: ~600 bytes
 */

#pragma once
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>

#if !defined(I2C_SDA_PIN)
#define I2C_SDA_PIN PB4
#warning "I2C_SDA_PIN not defined, using default value"
#endif

#if !defined(I2C_SCL_PIN)
#define I2C_SCL_PIN PB3
#warning "I2C_SCL_PIN not defined, using default value"
#endif

#if !defined(I2C_ADDRESS)
#define I2C_ADDRESS 0x22
#warning "I2C_ADDRESS not defined, using default value"
#endif

#if !defined(I2C_BUS_FREE_TIME)
#define I2C_BUS_FREE_TIME 5
#endif

#if !defined(I2C_BUFFER_SIZE)
#define I2C_BUFFER_SIZE 4
#endif

#define __always_inline __attribute__((always_inline)) inline
#define __weak __attribute__((weak))

/**
 * i2c event callback
 *
 * @param read true if controller is reading from the target, false if writing to the target
 * @param data pointer to the buffer containing the data to be sent to the controller (read=true) or the data received from the controller (read=false)
 * @param len the length of the data buffer.
 *            when read=true, this is initially the maximum number of bytes that can be written to the data buffer.
 *            on return, this is the actual number of bytes that should be sent to the controller.
 *            when read=false, this is the number of bytes received from the controller.
 */
__always_inline void on_i2c_event(const bool read, uint8_t *data, uint8_t &len);

/**
 * i2c idle callback
 *
 * called repeatedly while waiting for i2c start condition.
 * this function should not block, as otherwise i2c communication may be interrupted.
 */
__always_inline void on_i2c_idle();
__weak void on_i2c_idle() {}

/**
 * i2c watchdog reset function
 * called regularly to reset the watchdog timer.
 */
__always_inline void i2c_wdt_reset();
__weak void i2c_wdt_reset() { wdt_reset(); }

namespace i2c_sw_target_internal
{

  constexpr uint8_t SDA = _BV(I2C_SDA_PIN);
  constexpr uint8_t SCL = _BV(I2C_SCL_PIN);

  constexpr uint8_t BUS_FREE_TIME = I2C_BUS_FREE_TIME; // in usec

  class I2CTarget
  {
  private:
    enum SEQ_Status : uint8_t
    {
      BusFree,
      Start,
      Data,
      Stop
    };

    SEQ_Status status = BusFree;

  private:
    __always_inline void nop() { asm volatile("nop"); }

    __always_inline bool read_sda() { return PINB & SDA; }
    __always_inline bool read_scl() { return PINB & SCL; }

    __always_inline void sda_high() // = idle
    {
      DDRB &= ~SDA; // sda in
      nop();
    }
    __always_inline void sda_low()
    {
      DDRB |= SDA;   // SDA out
      PORTB &= ~SDA; // SDA low
      nop();
    }

    __always_inline void scl_high() // = idle
    {
      DDRB &= ~SCL; // scl in
    }
    __always_inline void scl_low()
    {
      DDRB |= SCL;   // SCL out
      PORTB &= ~SCL; // SCL low
    }

    __always_inline void clk_keep()
    {
      scl_low();
      nop();
    }
    __always_inline void clk_free()
    {
      DDRB &= ~SCL; // SCL in
      nop();
    }

  private:
    /**
     * send i2c ACK
     */
    __always_inline void ack()
    {
      sda_low();
      while (!read_scl()) // wait until SCL=1
        ;
      while (read_scl()) // wait until SCL=0
        ;
      sda_high();
    }

    /**
     * internal read byte from i2c bus
     * @return the byte read from the bus. only valid if status is not Stop
     * @note updates status to Stop if stop condition is detected
     */
    __always_inline uint8_t _get_byte()
    {
      uint8_t byte = 0;
      for (uint8_t register i = 0; i < 8; i++)
      {
        while (!read_scl()) // wait until SCL=1
          ;

        // get SDA
        if (read_sda())
        {
          byte |= (1 << (7 - i));
          while (read_scl()) // wait until SCL=0
            ;
        }
        else
        {
          while (1)
          {
            uint8_t register pb = PINB;
            if ((pb & SCL) == 0)
            {
              // SCL=0
              break;
            }

            if (pb & SDA)
            {
              // SDA 0->1 = stop condition
              status = Stop;
              return 0;
            }
          }
        }
      }

      return byte;
    }

    /**
     * wait until our i2c address is detected on the bus
     * @return R/W bit: true = read, false = write
     * @note
     * updates status to Data if address is detected,
     * or to BusFree if address is not detected.
     *
     * return value is only valid if status is Data.
     */
    __always_inline bool detect_address()
    {
      uint8_t addr = _get_byte();
      if (status == Stop)
      {
        // STOP detected during address read, stop processing
        return false;
      }

      // get R/W bit and normalize address
      bool rw = addr & 1;
      addr >>= 1;

      // check address match
      if (addr == I2C_ADDRESS)
      {
        status = Data;
        ack(); // ACK on our address
      }
      else
      {
        status = BusFree;
      }

      i2c_wdt_reset();
      return rw;
    }

    /**
     * read bytes from i2c bus
     * @param bytes pointer to the buffer to store the read bytes
     * @param len the maximum number of bytes to read
     * @return the number of bytes read
     *
     * @note updates status to Stop if stop condition is detected
     */
    __always_inline uint8_t get_bytes(uint8_t *bytes, const uint8_t len)
    {
      cli();
      for (uint8_t i = 0; i < len; i++)
      {
        const uint8_t b = _get_byte();
        if (status == Stop)
        {
          // STOP detected during byte read, stop processing and
          // return current number of bytes read
          return i;
        }

        bytes[i] = b;
        ack();
        i2c_wdt_reset();
      }

      return len;
    }

    /**
     * internal put a byte on the i2c bus
     * @param byte the byte to put on the bus
     */
    __always_inline void _put_byte(const uint8_t byte)
    {
      for (uint8_t register i = 0; i < 8; i++)
      {
        while (read_scl()) // wait until SCL=0
          ;

        if (byte & (1 << (7 - i)))
        {
          sda_high();
        }
        else
        {
          sda_low();
        }

        while (!read_scl()) // wait until SCL=1
          ;
      }

      while (read_scl()) // wait until SCL=0
        ;

      sda_high();

      while (!read_scl()) // wait until SCL=1
        ;
      while (read_scl()) // wait until SCL=0
        ;
    }

    /**
     * put multiple bytes on the i2c bus
     * @param bytes pointer to the buffer containing the bytes to put on the bus
     * @param len the number of bytes to put on the bus
     */
    __always_inline void put_bytes(uint8_t *bytes, const uint8_t len)
    {
      cli();
      for (uint8_t i = 0; i < len; i++)
      {
        _put_byte(bytes[i]);
        i2c_wdt_reset();
      }
    }

    /**
     * wait for i2c start condition
     * @note resets status to either Start or Stop on return
     */
    __always_inline void wait_for_start()
    {
      DDRB &= ~(SDA | SCL);  // SDA and SCL in
      PORTB &= ~(SDA | SCL); // HiZ

      i2c_wdt_reset();

      // wait for SCL=1 and SDA=1 stable for BUS_FREE_TIME
      uint8_t register cnt = BUS_FREE_TIME;
      while (cnt)
      {
        if ((PINB & (SDA | SCL)) != (SDA | SCL))
        {
          cnt = BUS_FREE_TIME;
        }
        else
        {
          // SDA=1, SCL=1
          cnt--;
        }

        _delay_us(1);
        i2c_wdt_reset();

        on_i2c_idle();
      }
      status = BusFree;

      // wait until START sequence is detected
      while (status != Start)
      {
        on_i2c_idle();
        i2c_wdt_reset();

        uint8_t register pb = PINB;
        if (pb & SCL)
        {
          if ((pb & SDA) == 0)
          {
            status = Start;
          }
          else
          {
            status = Stop;
          }
        }
      }

      // wait for SCL=0 SDA=0
      while (PINB & (SDA | SCL))
      {
        i2c_wdt_reset();
      }
    }

  private:
    uint8_t buffer[I2C_BUFFER_SIZE];

  public:
    /**
     * begin listening for i2c commands.
     * this function is blocking.
     *
     * @note
     * it is recommended to enable the watchdog timer in reset mode to
     * recover in case of i2c communication lockup.
     */
    void listen()
    {
      while (1)
      {
        wait_for_start();
        const bool is_read = detect_address();

        if (status == Data)
        {
          if (is_read)
          {
            // read from target to controller
            clk_keep();
            uint8_t len = sizeof(buffer);
            on_i2c_event(/*is_read*/ true, buffer, len);
            clk_free();

            if (len > 0)
            {
              put_bytes(buffer, len);
            }
          }
          else
          {
            // write from controller to target
            uint8_t len = get_bytes(buffer, sizeof(buffer));
            if (len > 0)
            {
              clk_keep();
              on_i2c_event(/*is_read*/ false, buffer, len);
              clk_free();
            }
          }
        }
      }
    }
  }; // class I2CTarget
} // namespace i2c_sw_target

/**
 * i2c target instance
 */
i2c_sw_target_internal::I2CTarget WireTarget;
