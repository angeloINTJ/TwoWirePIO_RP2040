/**
 * @file WirePIO_config.h
 * @brief Platform detection, configuration defaults, and error codes for WirePIO.
 *
 * Supports both Arduino-Pico (Earle Philhower core) and Pico SDK (CMake)
 * build environments with zero-overhead compile-time abstraction.
 *
 * @author angeloINTJ
 * @license MIT
 */

#ifndef WIREPIO_CONFIG_H
#define WIREPIO_CONFIG_H

// ─── Platform Detection ───────────────────────────────────────────────

#if defined(ARDUINO)
  // Arduino-Pico (Earle Philhower core)
  #define WIREPIO_PLATFORM_ARDUINO 1
  #include <Arduino.h>
#elif defined(PICO_STDLIB_H) || defined(LIB_PICO_STDLIB_H)
  // Pico SDK (CMake)
  #define WIREPIO_PLATFORM_PICOSDK 1
  #include <pico/stdlib.h>
  #include <hardware/pio.h>
  #include <hardware/dma.h>
  #include <hardware/gpio.h>
  #include <hardware/clocks.h>
  #include <hardware/irq.h>
  #include <hardware/i2c.h>
  #include <hardware/pwm.h>
#else
  #error "WirePIO requires Arduino-Pico or Pico SDK environment"
#endif

// ─── C Standard Library ────────────────────────────────────────────────

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// ─── Timing Abstractions ──────────────────────────────────────────────

#ifdef WIREPIO_PLATFORM_ARDUINO
  #define WIREPIO_DELAY_US(us)     delayMicroseconds(us)
  #define WIREPIO_SLEEP_MS(ms)     delay(ms)
  #define WIREPIO_TIME_US()        time_us_64()
  #define WIREPIO_TIMEOUT_US(us)   (time_us_64() + (uint64_t)(us))
  #define WIREPIO_TIME_REACHED(t)  (time_us_64() >= (t))
#else
  #define WIREPIO_DELAY_US(us)     sleep_us(us)
  #define WIREPIO_SLEEP_MS(ms)     sleep_ms(ms)
  #define WIREPIO_TIME_US()        time_us_64()
  #define WIREPIO_TIMEOUT_US(us)   (time_us_64() + (uint64_t)(us))
  #define WIREPIO_TIME_REACHED(t)  (time_us_64() >= (t))
#endif

// ─── Debug Output ──────────────────────────────────────────────────────

#ifdef WIREPIO_DEBUG
  #ifdef WIREPIO_PLATFORM_ARDUINO
    #define WIREPIO_DBG(fmt, ...)  Serial.printf(fmt, ##__VA_ARGS__)
  #else
    #define WIREPIO_DBG(fmt, ...)  printf(fmt, ##__VA_ARGS__)
  #endif
#else
  #define WIREPIO_DBG(fmt, ...)  ((void)0)
#endif

// ─── Defaults ─────────────────────────────────────────────────────────

#define WIREPIO_BUFFER_SIZE              256
#define WIREPIO_BUFFER_SIZE_MIN          32
#define WIREPIO_DEFAULT_FREQ             100000
#define WIREPIO_DEFAULT_TIMEOUT_US       25000UL
#define WIREPIO_MAX_CMD_WORDS            (3 + WIREPIO_BUFFER_SIZE)

// ─── I2C Bus Speed Presets ────────────────────────────────────────────

#define WIREPIO_FREQ_STANDARD            100000
#define WIREPIO_FREQ_FAST                400000

// ─── Error Codes (compatible with TwoWire return codes) ───────────────

#define WIREPIO_SUCCESS                  0
#define WIREPIO_ERR_DATA_TOO_LONG        1
#define WIREPIO_ERR_NACK_ADDR            2
#define WIREPIO_ERR_NACK_DATA            3
#define WIREPIO_ERR_OTHER                4
#define WIREPIO_ERR_TIMEOUT              5

// ─── Async State ──────────────────────────────────────────────────────

enum WirePIOAsyncState {
    WIREPIO_ASYNC_IDLE = 0,
    WIREPIO_ASYNC_BUSY = 1
};

#endif // WIREPIO_CONFIG_H
