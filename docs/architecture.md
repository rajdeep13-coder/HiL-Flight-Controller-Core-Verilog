# Architecture — HiL Quadcopter Flight Controller

## System Overview

This project implements a hardware PID-based flight controller for a quadcopter in Verilog, paired with a Python Hardware-in-the-Loop (HiL) testbench that provides closed-loop rotational physics simulation via cocotb.

All arithmetic uses **signed Q8.8 fixed-point** format — no floating point anywhere in the Verilog design.

---

## Block Diagram

```
                   ┌─────────────────────────────────────────────────────────────────┐
                   │                    flight_controller_top                         │
                   │                                                                 │
  wr_en ───────────┤──┐                                                              │
  wr_addr [3:0] ───┤  ├──► [gain_regs] ──► Kp/Ki/Kd per axis                        │
  wr_data [15:0] ──┤──┘         │                                                    │
                   │            ↓                                                    │
  roll_error ──────┤──► [pid_controller] ──► [saturation_guard] ──► roll_clamped ──┐ │
                   │         ↑                      │                              │ │
                   │         └── integrator_hold ────┘                              │ │
                   │                                                               │ │
  pitch_error ─────┤──► [pid_controller] ──► [saturation_guard] ──► pitch_clamped ─┤ │
                   │         ↑                      │                              │ │
                   │         └── integrator_hold ────┘                              │ │
                   │                                                               │ │
  yaw_error ───────┤──► [pid_controller] ──► [saturation_guard] ──► yaw_clamped ──┤ │
                   │         ↑                      │                              │ │
                   │         └── integrator_hold ────┘                              ↓ │
                   │                                                            ┌──────┐
  throttle ────────┤──────────────────────────────────────────────────────────► │ MIXER │
                   │                                                            └──┬───┘
                   │                 ┌──────────┬──────────┬──────────┐            │
                   │                 ↓          ↓          ↓          ↓            │
                   │            [pwm_gen0] [pwm_gen1] [pwm_gen2] [pwm_gen3]       │
                   │                 │          │          │          │            │
                   │                 ↓          ↓          ↓          ↓            │
                   ├─── pwm_out0 ────┘          │          │          │            │
                   ├─── pwm_out1 ───────────────┘          │          │            │
                   ├─── pwm_out2 ──────────────────────────┘          │            │
                   ├─── pwm_out3 ─────────────────────────────────────┘            │
                   │                                                               │
                   ├─── motor0_duty ◄──────────────────────────────────────────────┤
                   ├─── motor1_duty ◄──────────────────────────────────────────────┤
                   ├─── motor2_duty ◄──────────────────────────────────────────────┤
                   ├─── motor3_duty ◄──────────────────────────────────────────────┤
                   └─────────────────────────────────────────────────────────────────┘
```

---

## Fixed-Point Format: Signed Q8.8

### Bit Layout (16 bits total)

| Bit 15 | Bits 14:8 | Bits 7:0 |
|:------:|:---------:|:--------:|
| Sign   | Integer (7 bits) | Fraction (8 bits) |

### Properties

| Property | Value |
|:---------|:------|
| Total width | 16 bits |
| Signed | Yes (two's complement) |
| Integer range | −128 to +127 |
| Fractional resolution | 1/256 ≈ 0.00390625 |
| Maximum value | +127.99609375 (`0x7FFF`) |
| Minimum value | −128.0 (`0x8000`) |

### Conversion

```
Float → Q8.8:  raw = round(float_value × 256), stored as signed 16-bit
Q8.8 → Float:  float_value = signed_int16 / 256.0
```

### Examples

| Float Value | Q8.8 Hex | Q8.8 Binary |
|:------------|:---------|:------------|
| +10.0 | `0x0A00` | `0000_1010.0000_0000` |
| +1.0 | `0x0100` | `0000_0001.0000_0000` |
| +0.5 | `0x0080` | `0000_0000.1000_0000` |
| +0.1 (≈) | `0x001A` | `0000_0000.0001_1010` |
| −5.0 | `0xFB00` | `1111_1011.0000_0000` |
| −127.0 | `0x8100` | `1000_0001.0000_0000` |

### Multiplication

Multiplying two Q8.8 values produces a Q16.16 (32-bit) intermediate:

```
Q8.8 × Q8.8 = Q16.16

To extract Q8.8 result: take bits [23:8] of the 32-bit product
Check bits [31:23] for overflow:
  - All zeros (positive) or all ones (negative) → no overflow
  - Otherwise → clamp to 0x7FFF (max) or 0x8000 (min)
```

---

## Module Descriptions

### 1. gain_regs (`rtl/gain_regs.v`)

**Purpose:** Runtime-writable register file holding all 9 PID gains (Kp/Ki/Kd × 3 axes). Provides a simple parallel write bus for runtime tuning without recompilation.

**Bus Interface:**

| Signal | Width | Direction | Description |
|:-------|:------|:----------|:------------|
| `wr_en` | 1 | Input | Write enable (single-cycle strobe) |
| `wr_addr` | 4 | Input | Register address (0–8) |
| `wr_data` | 16 | Input | Write data (Q8.8) |

**Register Map:**

| Address | Register | Reset Default (Q8.8) | Float |
|:-------:|:---------|:---------------------|:------|
| `0x0` | `ROLL_Kp` | `0x001A` | ~0.1 |
| `0x1` | `ROLL_Ki` | `0x0001` | ~0.004 |
| `0x2` | `ROLL_Kd` | `0x0033` | ~0.2 |
| `0x3` | `PITCH_Kp` | `0x001A` | ~0.1 |
| `0x4` | `PITCH_Ki` | `0x0001` | ~0.004 |
| `0x5` | `PITCH_Kd` | `0x0033` | ~0.2 |
| `0x6` | `YAW_Kp` | `0x000D` | ~0.05 |
| `0x7` | `YAW_Ki` | `0x0001` | ~0.004 |
| `0x8` | `YAW_Kd` | `0x001A` | ~0.1 |

**Write Protocol:**
1. Assert `wr_en=1` with target `wr_addr` and `wr_data` on a rising clock edge
2. The register updates on that same edge
3. De-assert `wr_en` on the next cycle

**Key Design Decisions:**
- Reset defaults match the original compile-time parameter values — behavior is unchanged out of reset
- Write-only bus (no read path) — keeps hardware minimal; cocotb can read via hierarchical access
- 4-bit address allows future expansion to 16 registers (e.g., adding MAX_OUT/MIN_OUT per axis)

---

### 2. pid_controller (`rtl/pid_controller.v`)

**Purpose:** Implements a discrete PID controller with runtime-configurable gains, all in Q8.8 fixed-point.

**Runtime Inputs (gains from gain_regs):**

| Port | Width | Description |
|:-----|:------|:------------|
| `Kp` | 16 | Proportional gain (Q8.8) |
| `Ki` | 16 | Integral gain (Q8.8) |
| `Kd` | 16 | Derivative gain (Q8.8) |

**Parameters (compile-time):**

| Parameter | Default (Q8.8) | Float Value | Description |
|:----------|:---------------|:------------|:------------|
| `MAX_OUT` | `0x7F00` | +127.0 | Output upper clamp |
| `MIN_OUT` | `0x8100` | −127.0 | Output lower clamp |

**Algorithm (per clock cycle):**

```
P = Kp × error
I = I_prev + Ki × error    (skipped if integrator_hold = 1)
D = Kd × (error − prev_error)

pid_out = clamp(P + I + D, MIN_OUT, MAX_OUT)
prev_error ← error
```

**Key Design Decisions:**
- 32-bit integrator accumulator prevents early saturation of the integral term
- Each multiplication uses a `saturate_q88()` function that extracts `[23:8]` from the 32-bit product and checks for overflow
- The `integrator_hold` input enables anti-windup: when asserted, the integrator freezes its current value
- Kp/Ki/Kd are runtime input ports (not parameters), fed from `gain_regs` — gains can change on any clock cycle without recompilation

---

### 3. saturation_guard (`rtl/saturation_guard.v`)

**Purpose:** Combinational output clamping with anti-windup feedback signal.

**Parameters:** `MAX_OUT`, `MIN_OUT` (same as PID defaults)

**Logic:**

```
if (raw_in > MAX_OUT):
    clamped_out = MAX_OUT, integrator_hold = 1
else if (raw_in < MIN_OUT):
    clamped_out = MIN_OUT, integrator_hold = 1
else:
    clamped_out = raw_in,  integrator_hold = 0
```

**Connection Pattern:** The saturation guard sits **outside** the PID controller. The `integrator_hold` signal feeds back to the PID's `integrator_hold` input for the **next** clock cycle, avoiding combinational loops.

---

### 4. mixer (`rtl/mixer.v`)

**Purpose:** Translates roll/pitch/yaw corrections and throttle into four motor duty cycles using the X-frame mixing matrix.

**X-Frame Motor Layout (top view):**

```
    M2 (FL) \   / M0 (FR)        CW: M0, M1
              \ /                 CCW: M2, M3
               X
              / \
    M1 (BL) /   \ M3 (BR)
```

**Mixing Matrix:**

| Motor | Throttle | Roll | Pitch | Yaw |
|:------|:--------:|:----:|:-----:|:---:|
| M0 (FR) | +1 | −1 | +1 | +1 |
| M1 (BL) | +1 | +1 | −1 | +1 |
| M2 (FL) | +1 | +1 | +1 | −1 |
| M3 (BR) | +1 | −1 | −1 | −1 |

**Output Clamping:** Motor outputs are clamped to `[0, +127.0]` since real motors cannot reverse direction.

---

### 5. pwm_gen (`rtl/pwm_gen.v`)

**Purpose:** Generates a single-bit PWM signal from a Q8.8 duty cycle word.

**Parameters:**

| Parameter | Default | Description |
|:----------|:--------|:------------|
| `CLK_FREQ` | 50,000,000 | System clock frequency (Hz) |
| `PWM_FREQ` | 50 | PWM output frequency (Hz) |

**Operation:**
- Free-running counter: period = `CLK_FREQ / PWM_FREQ` (1,000,000 counts at defaults)
- Threshold: `(duty_word × COUNTER_MAX) >> 8`
- Output: `pwm_out = (counter < threshold)`

**Note:** In HiL simulation, the testbench reads `motor_duty` values (Q8.8) directly from the mixer output, not the PWM bits. The PWM generators are included for hardware completeness.

---

### 6. flight_controller_top (`rtl/flight_controller_top.v`)

**Purpose:** Top-level integration module with runtime gain configuration.

**Instantiation Summary:**
- 1× `gain_regs` — runtime-writable PID gain register file
- 3× `pid_controller` — roll, pitch, yaw (gains from register file)
- 3× `saturation_guard` — one per axis
- 1× `mixer` — combines all axis outputs with throttle
- 4× `pwm_gen` — one per motor

**Gain Register Bus:**

The top module exposes `wr_en`, `wr_addr[3:0]`, and `wr_data[15:0]` input ports. These are wired directly to the internal `gain_regs` instance. From cocotb or any external controller, write a gain register by strobing `wr_en=1` for one clock cycle with the target address and Q8.8 value.

**Default Gains (loaded on reset via gain_regs):**

| Axis | Kp | Ki | Kd |
|:-----|:--:|:--:|:--:|
| Roll | ~0.1 | ~0.004 | ~0.2 |
| Pitch | ~0.1 | ~0.004 | ~0.2 |
| Yaw | ~0.05 | ~0.004 | ~0.1 |

Yaw gains are intentionally lower since yaw authority is typically less than roll/pitch on a quadcopter.

---

## HiL Testbench Architecture

### Physics Model (`sim/tb_flight_controller.py`)

The cocotb testbench implements a simplified rotational dynamics model:

```
State vector: [roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate]

For each simulation step:
  1. Motor duties → forces (F = duty × thrust_coeff)
  2. Forces → torques via X-frame geometry
  3. Torques → angular acceleration (α = τ / I)
  4. Euler integration: rate += α·dt, angle += rate·dt
  5. Compute error: setpoint − angle
  6. Drive Q8.8-encoded errors into DUT
```

### Physical Constants

| Parameter | Value | Description |
|:----------|:------|:------------|
| Ixx, Iyy | 0.01 kg·m² | Roll/pitch moment of inertia |
| Izz | 0.02 kg·m² | Yaw moment of inertia |
| Arm length | 0.225 m | Motor-to-center distance |
| Thrust coeff | 0.04 N/duty | Force per duty cycle unit |
| Physics dt | 0.001 s | Integration time step |

---

## Data Flow Summary

```
Setpoint (Python)   ─────┐
                          ↓
Current Angle (Python) ──[Error = Setpoint − Angle]──► Q8.8 encode
                                                           │
                                                           ↓
                              ┌─── DUT (Verilog) ─────────────────────┐
                              │  error → PID → Sat.Guard → Mixer     │
                              │                               ↓       │
                              │                          motor duties  │
                              └────────────────────────────────┤───────┘
                                                               │
                                                               ↓
                                            Q8.8 decode → [Physics Model]
                                                               │
                                                               ↓
                                                    Updated Angles (Python)
                                                               │
                                                         ┌─────┘
                                                         ↓
                                                    [Next Cycle]
```
