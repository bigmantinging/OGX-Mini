#pragma once
// =============================================================================
// AccelMod.h  —  Inject ADXL345 tilt into OGX-Mini's right stick
//
// Uses the REAL OGX-Mini Gamepad fields (from Gamepad/Gamepad.h):
//   pad_in.joystick_rx  int16_t  right stick X  (-32768 … 32767)
//   pad_in.joystick_ry  int16_t  right stick Y  (-32768 … 32767)
//
// Thread safety: get_pad_in() / set_pad_in() use OGX-Mini's own mutexes.
//
// Physical orientation (board mounted flat, USB toward player):
//   Tilt LEFT  → joystick_rx negative  (look left)
//   Tilt RIGHT → joystick_rx positive  (look right)
//   Tilt UP    → joystick_ry positive  (look up — Y polarity matches XInput)
//   Tilt DOWN  → joystick_ry negative  (look down)
//
// If directions are wrong after install, adjust ACCEL_INVERT_X / _Y below.
// =============================================================================

#include "Gamepad/Gamepad.h"    // OGX-Mini Gamepad class
#include "adxl345.h"            // ADXL345 driver (same folder)
#include <cstdint>
#include <atomic>

// --- Tuning ------------------------------------------------------------------

// Dead-zone (normalised 0–1). Widen to 0.08 if cursor drifts at rest.
#define ACCEL_DEADZONE      0.05f

// Sensitivity multiplier. Increase if small tilts don't move cursor enough.
#define ACCEL_SENSITIVITY   2.2f

// Low-pass alpha: 1.0 = raw (jittery), 0.1 = very smooth (sluggish)
#define ACCEL_ALPHA         0.25f

// Stick-at-rest threshold: only override right stick when physical stick
// is within this many counts of centre (±32767 full scale).
// 3200 ≈ 10% of full scale.
#define ACCEL_STICK_THRESH  3200

// Confirmed from real sensor readings (marek, 2026):
//   Tilt left    → Y strongly negative → needs swap+invert to become forward
//   Tilt forward → X strongly negative → needs swap+invert to become right
#define ACCEL_INVERT_X      1
#define ACCEL_INVERT_Y      1
#define ACCEL_SWAP_XY       1

// -----------------------------------------------------------------------------

class AccelMod {
public:
    // Call once during board_api::init() or equivalent, AFTER i2c is free.
    // Returns false if sensor not found — mod will silently do nothing.
    bool init() {
        enabled_ = adxl345_init();
        return enabled_;
    }

    // Call once per main-loop iteration AFTER the host driver has written
    // new input via gamepad.set_pad_in(), BEFORE the device driver reads it.
    //
    // Suggested placement in OGX-Mini's board_api.cpp main loop:
    //   accel_mod.apply(gamepad_manager.get(0));
    void apply(Gamepad& gamepad) {
        if (!enabled_) return;

        // Read sensor — ~150 µs at 400 kHz, safe on core 0
        adxl345_data_t raw = adxl345_read_normalised();

        // Optional axis swap
        float ax = raw.x;
        float ay = raw.y;
#if ACCEL_SWAP_XY
        float tmp = ax; ax = ay; ay = tmp;
#endif
#if ACCEL_INVERT_X
        ax = -ax;
#endif
#if ACCEL_INVERT_Y
        ay = -ay;
#endif

        // Exponential low-pass filter
        fx_ += ACCEL_ALPHA * (ax - fx_);
        fy_ += ACCEL_ALPHA * (ay - fy_);

        // Dead-zone + sensitivity
        float mx = _dz(fx_) * ACCEL_SENSITIVITY;
        float my = _dz(fy_) * ACCEL_SENSITIVITY;
        mx = mx > 1.f ? 1.f : (mx < -1.f ? -1.f : mx);
        my = my > 1.f ? 1.f : (my < -1.f ? -1.f : my);

        int16_t rx = (int16_t)(mx * 32767.f);
        int16_t ry = (int16_t)(my * 32767.f);

        // Get current pad state (mutex-protected by OGX-Mini)
        Gamepad::PadIn pad = gamepad.get_pad_in();

        // Only replace right stick if physical stick is at centre.
        // This lets the player still override with the physical stick.
        if (pad.joystick_rx > -ACCEL_STICK_THRESH &&
            pad.joystick_rx <  ACCEL_STICK_THRESH &&
            pad.joystick_ry > -ACCEL_STICK_THRESH &&
            pad.joystick_ry <  ACCEL_STICK_THRESH)
        {
            pad.joystick_rx = rx;
            pad.joystick_ry = ry;
            gamepad.set_pad_in(pad);
        }
    }

private:
    bool  enabled_ = false;
    float fx_ = 0.f;
    float fy_ = 0.f;

    static float _dz(float v) {
        if (v > -ACCEL_DEADZONE && v < ACCEL_DEADZONE) return 0.f;
        return v > 0 ? (v - ACCEL_DEADZONE) / (1.f - ACCEL_DEADZONE)
                     : (v + ACCEL_DEADZONE) / (1.f - ACCEL_DEADZONE);
    }
};
