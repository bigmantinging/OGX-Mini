#pragma once
// =============================================================================
// adxl345.h  —  Minimal ADXL345 I2C driver for RP2040-Zero
//
// Pins (must not conflict with OGX-Mini RP_ZERO USB host on GPIO 10/11):
//   SDA → GPIO 4  (i2c0)
//   SCL → GPIO 5  (i2c0)
//   VCC → 3.3V
//   GND → GND
//   SDO → GND   → address 0x53  (most GY-291 boards default)
//   SDO → 3.3V  → address 0x1D  (change ADXL345_ADDR below)
//   CS  → 3.3V  (already pulled high on GY-291 board)
//   INT1, INT2 → leave floating (not used)
//   No external pull-up resistors needed — GY-291 has them onboard.
// =============================================================================

#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <stdint.h>
#include <stdbool.h>

// --- Pin & bus config --------------------------------------------------------
#define ADXL_I2C_PORT    i2c0
#define ADXL_SDA_PIN     4
#define ADXL_SCL_PIN     5
#define ADXL_I2C_FREQ    400000   // 400 kHz

// Change to 0x1D if SDO is tied HIGH on your board
#define ADXL345_ADDR     0x53

// --- Register addresses ------------------------------------------------------
#define ADXL_REG_DEVID        0x00
#define ADXL_REG_BW_RATE      0x2C
#define ADXL_REG_POWER_CTL   0x2D
#define ADXL_REG_DATA_FORMAT  0x31
#define ADXL_REG_DATAX0       0x32   // 6 bytes: X0 X1 Y0 Y1 Z0 Z1

#define ADXL_DEVID_EXPECTED   0xE5
#define ADXL_BW_100HZ         0x0A   // 100 Hz output data rate

// --- Data types --------------------------------------------------------------
typedef struct { int16_t x, y, z; } adxl345_raw_t;

// Normalised: each axis -1.0 … +1.0 in ±2 g mode
typedef struct { float x, y, z;   } adxl345_data_t;

// --- Internal helpers (static so multiple includes don't collide) -------------
static inline void _adxl_write(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    i2c_write_blocking(ADXL_I2C_PORT, ADXL345_ADDR, buf, 2, false);
}

static inline uint8_t _adxl_read(uint8_t reg) {
    uint8_t val = 0;
    i2c_write_blocking(ADXL_I2C_PORT, ADXL345_ADDR, &reg, 1, true);
    i2c_read_blocking (ADXL_I2C_PORT, ADXL345_ADDR, &val, 1, false);
    return val;
}

// --- Public API --------------------------------------------------------------

// Returns true if ADXL345 responds with correct device ID.
// Call once during board init, before the main loop.
static inline bool adxl345_init(void) {
    i2c_init(ADXL_I2C_PORT, ADXL_I2C_FREQ);
    gpio_set_function(ADXL_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(ADXL_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(ADXL_SDA_PIN);
    gpio_pull_up(ADXL_SCL_PIN);
    sleep_ms(10);   // sensor power-on time

    if (_adxl_read(ADXL_REG_DEVID) != ADXL_DEVID_EXPECTED)
        return false;

    _adxl_write(ADXL_REG_BW_RATE,     ADXL_BW_100HZ);
    _adxl_write(ADXL_REG_DATA_FORMAT, 0x00); // ±2 g, right-justified
    _adxl_write(ADXL_REG_POWER_CTL,   0x08); // Measure mode on
    return true;
}

// Read raw 16-bit signed values from all three axes.
// Safe to call from any core — I2C is not shared with anything else.
static inline adxl345_raw_t adxl345_read_raw(void) {
    uint8_t reg = ADXL_REG_DATAX0;
    uint8_t buf[6];
    i2c_write_blocking(ADXL_I2C_PORT, ADXL345_ADDR, &reg, 1, true);
    i2c_read_blocking (ADXL_I2C_PORT, ADXL345_ADDR, buf, 6, false);
    return (adxl345_raw_t){
        .x = (int16_t)(buf[0] | (buf[1] << 8)),
        .y = (int16_t)(buf[2] | (buf[3] << 8)),
        .z = (int16_t)(buf[4] | (buf[5] << 8)),
    };
}

// Convert to normalised float. In ±2 g mode, LSB = 3.9 mg, FS ≈ 256 counts.
#define _ADXL_FS 256.0f
static inline float _clamp1(float v) {
    return v > 1.f ? 1.f : (v < -1.f ? -1.f : v);
}
static inline adxl345_data_t adxl345_read_normalised(void) {
    adxl345_raw_t r = adxl345_read_raw();
    return (adxl345_data_t){
        .x = _clamp1((float)r.x / _ADXL_FS),
        .y = _clamp1((float)r.y / _ADXL_FS),
        .z = _clamp1((float)r.z / _ADXL_FS),
    };
}
