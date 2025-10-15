# Code Review: Rate Mode Control Loop

## Summary
This document captures the major issues I spotted while reviewing the current rate-mode implementation for the STM32F401-based quadcopter. The focus is on safety, control-loop correctness, and robustness of the flight-critical logic in `Core/Src/main.c`.

## Findings

### 1. Loss of RC link leaves the motors at the last commanded output
When the CRSF link times out (`crsf.data.valid` becomes false), the control loop simply skips the motor-mixing block and prints `"waiting CRSF..."`, but it never commands the ESCs back to a safe value. Because PWM outputs stay latched, the motors will keep spinning with the last non-failsafe command, which is a major safety issue. A proper failsafe should explicitly drive all four channels back to 1000 µs (or disarm the ESC timers) as soon as the RC data goes invalid, and keep them there until the link is healthy again.【F:Core/Src/main.c†L201-L247】

### 2. Motors spin before the receiver reports a valid frame
`CRSF_Init` seeds every channel with ~992 µs, `main` immediately starts all PWM channels, and the first loop iteration clamps each motor to the 1000 µs minimum. Because the code does not wait for a valid arming condition (e.g., a low throttle plus an explicit arm switch) before enabling PWM, the props will spool as soon as the board boots—even if no transmitter is on. At minimum the firmware should hold the timers disabled or force 1000 µs until CRSF reports a valid frame and the pilot satisfies an arming sequence.【F:Core/Src/main.c†L159-L247】

### 3. `HAL_GetTick()` wrap can explode the PID math
`dt` is computed by subtracting two `uint32_t` tick counters and dividing by 1000.0. When `HAL_GetTick()` wraps (about every 49 days), `now - last_t` underflows to a huge positive number, which then feeds the integral and derivative terms. The fallback of `0.004f` only applies when the subtraction result is non-positive, so it will not catch this wrap-around; the controller will see a seconds-long timestep and blow up the integral accumulator. Protect against this by computing the delta with modular arithmetic or capping `dt` to a sane maximum before it is used by the PID.【F:Core/Src/main.c†L190-L215】

### 4. PID continues to integrate when throttle is closed
Even with the throttle at its minimum, the PID terms remain active and are mixed straight onto the motor outputs. Because the outputs are clamped to ≥1000 µs, any accumulated integral term can keep the props spinning after a throttle cut, and the controller continues to wind up while the pilot thinks the motors are off. A common mitigation is to reset or freeze the integrators (and optionally bypass the entire rate loop) whenever the throttle is below an arming threshold.【F:Core/Src/main.c†L205-L244】【F:Core/Src/pid.c†L9-L26】

## Recommendations
* Implement an explicit failsafe path that commands 1000 µs (or stops the timers) whenever `crsf.data.valid` is false, and only resume normal control after a healthy frame is received.
* Gate the motor outputs behind an arming state machine so the ESCs stay disarmed until the pilot performs a deliberate arming gesture.
* Harden the timestep calculation against tick wrap and unreasonable `dt` values before calling `PID_Update`.
* Suppress or reset the PID integrators when throttle is low to avoid wind-up and unwanted motor spin.

Addressing these items will materially improve the safety and reliability of the rate-mode implementation.
