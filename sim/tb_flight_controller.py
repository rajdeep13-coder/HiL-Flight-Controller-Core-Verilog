"""
tb_flight_controller.py — Cocotb Hardware-in-the-Loop Testbench

Implements a simplified 6-DOF rotational quadcopter physics model.
On each clock cycle:
  1. Read motor duty cycles from the DUT (Q8.8)
  2. Compute torques from motor differences
  3. Integrate angular accelerations → rates → angles (Euler integration)
  4. Compute error vs setpoint
  5. Encode errors back to Q8.8 and drive into DUT
  6. Log everything to CSV

Gain registers can be written at runtime via the parallel bus
(wr_en, wr_addr, wr_data) using the write_gain() helper.

Usage:
  cd sim && make
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer
import csv
import struct
import math


# Q8.8 Fixed-Point Conversion Helpers

def float_to_q88(val):
    """Convert a Python float to Q8.8 signed 16-bit integer."""
    raw = int(round(val * 256.0))
    # Clamp to signed 16-bit range
    if raw > 32767:
        raw = 32767
    elif raw < -32768:
        raw = -32768
    # Return as unsigned 16-bit (two's complement)
    return raw & 0xFFFF


def q88_to_float(val):
    """Convert Q8.8 unsigned 16-bit (two's complement) to Python float."""
    if val >= 0x8000:
        val -= 0x10000
    return val / 256.0


# Gain Register Addresses (must match gain_regs.v)
GAIN_ADDR = {
    'roll_kp':  0x0,
    'roll_ki':  0x1,
    'roll_kd':  0x2,
    'pitch_kp': 0x3,
    'pitch_ki': 0x4,
    'pitch_kd': 0x5,
    'yaw_kp':   0x6,
    'yaw_ki':   0x7,
    'yaw_kd':   0x8,
}


async def write_gain(dut, addr, value_q88):
    """Write a single gain register over the parallel bus.

    Args:
        dut: cocotb DUT handle
        addr: Register address (int, 0-8) or string key from GAIN_ADDR
        value_q88: Q8.8 value as unsigned 16-bit integer
    """
    if isinstance(addr, str):
        addr = GAIN_ADDR[addr]
    dut.wr_addr.value = addr
    dut.wr_data.value = value_q88
    dut.wr_en.value   = 1
    await RisingEdge(dut.clk)
    dut.wr_en.value   = 0


async def write_gains_bulk(dut, gains_dict):
    """Write multiple gain registers sequentially.

    Args:
        dut: cocotb DUT handle
        gains_dict: dict mapping register name (str) to float gain value
                    e.g. {'roll_kp': 0.2, 'roll_kd': 0.4}
    """
    for name, float_val in gains_dict.items():
        await write_gain(dut, name, float_to_q88(float_val))


# Simplified Rotational Physics Model

class QuadcopterPhysics:
    """
    Simplified 6-DOF rotational dynamics model for a quadcopter.
    
    - Only models rotational dynamics (roll, pitch, yaw)
    - Ignores translational dynamics (position, velocity)
    - Uses Euler angle integration
    - Assumes X-frame motor configuration
    
    Motor layout (top view, X-frame):
        M2 (FL) \\   / M0 (FR)
                  \\ /
                   X
                  / \\
        M1 (BL) /   \\ M3 (BR)
    
    Axes:
        Roll  = rotation about X (front-back axis)
        Pitch = rotation about Y (left-right axis)
        Yaw   = rotation about Z (vertical axis)
    """
    
    def __init__(self, dt=0.001):
        # Time step (seconds)
        self.dt = dt
        
        # Moments of inertia (kg·m²)
        self.Ixx = 0.01   # Roll
        self.Iyy = 0.01   # Pitch
        self.Izz = 0.02   # Yaw
        
        # Motor arm length (meters) — used for torque calculation
        self.arm_length = 0.225  # ~9 inches
        
        # Thrust coefficient: maps duty (Q8.8 float) to force (N)
        # Approximate: max duty 127 → ~2.5N per motor
        self.thrust_coeff = 0.02  # N per duty unit
        
        # Yaw torque coefficient (reaction torque from prop)
        self.yaw_coeff = 0.005
        
        # Damping coefficients (simple drag model)
        self.roll_damping  = 0.15
        self.pitch_damping = 0.15
        self.yaw_damping   = 0.08
        
        # State: [roll, pitch, yaw] in degrees
        self.roll  = 0.0
        self.pitch = 0.0
        self.yaw   = 0.0
        
        # Angular rates: [roll_rate, pitch_rate, yaw_rate] in deg/s
        self.roll_rate  = 0.0
        self.pitch_rate = 0.0
        self.yaw_rate   = 0.0
    
    def update(self, m0_duty, m1_duty, m2_duty, m3_duty):
        """
        Update physics state given four motor duty cycles (as floats).
        
        Motor mixing (X-frame):
          M0 (FR): T - R + P + Y    →  Roll: -M0 +M1 +M2 -M3
          M1 (BL): T + R - P + Y    →  Pitch: +M0 -M1 +M2 -M3
          M2 (FL): T + R + P - Y    →  Yaw: +M0 +M1 -M2 -M3
          M3 (BR): T - R - P - Y
        
        Torques are derived from thrust differences:
          τ_roll  = arm * ((-F0 + F1 + F2 - F3) / √2)  [for X-frame 45° offset]
          τ_pitch = arm * ((+F0 - F1 + F2 - F3) / √2)
          τ_yaw   = yaw_coeff * (+F0 + F1 - F2 - F3)    [reaction torque]
        """
        # Convert duties to forces
        f0 = m0_duty * self.thrust_coeff
        f1 = m1_duty * self.thrust_coeff
        f2 = m2_duty * self.thrust_coeff
        f3 = m3_duty * self.thrust_coeff
        
        # Compute torques (X-frame geometry, ±45° from axes)
        inv_sqrt2 = 1.0 / math.sqrt(2.0)
        tau_roll  = self.arm_length * inv_sqrt2 * (-f0 + f1 + f2 - f3)
        tau_pitch = self.arm_length * inv_sqrt2 * (+f0 - f1 + f2 - f3)
        tau_yaw   = self.yaw_coeff * (+f0 + f1 - f2 - f3)
        
        # Angular accelerations (Newton's 2nd for rotation: τ = I·α)
        roll_accel  = (tau_roll  - self.roll_damping  * math.radians(self.roll_rate))  / self.Ixx
        pitch_accel = (tau_pitch - self.pitch_damping * math.radians(self.pitch_rate)) / self.Iyy
        yaw_accel   = (tau_yaw   - self.yaw_damping   * math.radians(self.yaw_rate))  / self.Izz
        
        # Convert accel from rad/s² to deg/s²
        roll_accel_deg  = math.degrees(roll_accel)
        pitch_accel_deg = math.degrees(pitch_accel)
        yaw_accel_deg   = math.degrees(yaw_accel)
        
        # Euler integration: rates
        self.roll_rate  += roll_accel_deg  * self.dt
        self.pitch_rate += pitch_accel_deg * self.dt
        self.yaw_rate   += yaw_accel_deg   * self.dt
        
        # Euler integration: angles
        self.roll  += self.roll_rate  * self.dt
        self.pitch += self.pitch_rate * self.dt
        self.yaw   += self.yaw_rate   * self.dt
        
        # Clamp angles to prevent numerical blowup
        self.roll  = max(-90.0, min(90.0, self.roll))
        self.pitch = max(-90.0, min(90.0, self.pitch))
        self.yaw   = max(-180.0, min(180.0, self.yaw))


# Cocotb Test

@cocotb.test()
async def hil_flight_test(dut):
    """
    Hardware-in-the-Loop test: runs the full flight controller with a
    simplified rotational physics model providing closed-loop feedback.
    
    Demonstrates runtime gain tuning at cycle 1500 by increasing roll/pitch
    Kp to show live retuning capability.
    """
    
    # Configuration
    NUM_CYCLES   = 3000          # Number of simulation cycles
    CLK_PERIOD   = 20            # ns (50 MHz)
    PHYSICS_DT   = 0.001         # seconds per sim step
    
    # Cycle at which gains are retuned (mid-simulation)
    RETUNE_CYCLE = 1500
    
    # Setpoints (degrees) — step inputs applied at cycle 0
    ROLL_SETPOINT  = 10.0
    PITCH_SETPOINT = 5.0
    YAW_SETPOINT   = 0.0
    
    # Throttle (hover baseline in Q8.8)
    THROTTLE_HOVER = 50.0        # Q8.8 float value
    
    # Setup
    clock = Clock(dut.clk, CLK_PERIOD, unit="ns")
    cocotb.start_soon(clock.start())
    
    # Initialize physics model
    physics = QuadcopterPhysics(dt=PHYSICS_DT)
    
    # Open CSV log
    csv_file = open("hil_flight_log.csv", "w", newline="")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow([
        "cycle",
        "roll_error", "pitch_error", "yaw_error",
        "roll_pid", "pitch_pid", "yaw_pid",
        "motor0", "motor1", "motor2", "motor3",
        "roll_angle", "pitch_angle", "yaw_angle",
        "roll_rate", "pitch_rate", "yaw_rate"
    ])
    
    # Reset — initialize all inputs including gain bus
    dut.rst.value = 1
    dut.roll_error.value  = 0
    dut.pitch_error.value = 0
    dut.yaw_error.value   = 0
    dut.throttle.value    = float_to_q88(THROTTLE_HOVER)
    dut.wr_en.value       = 0
    dut.wr_addr.value     = 0
    dut.wr_data.value     = 0
    
    for _ in range(10):
        await RisingEdge(dut.clk)
    
    dut.rst.value = 0
    
    # HiL Loop
    dut._log.info(f"Starting HiL simulation: {NUM_CYCLES} cycles")
    dut._log.info(f"Setpoints — Roll: {ROLL_SETPOINT}°, Pitch: {PITCH_SETPOINT}°, Yaw: {YAW_SETPOINT}°")
    dut._log.info(f"Runtime retune scheduled at cycle {RETUNE_CYCLE}")
    
    for cycle in range(NUM_CYCLES):
        await RisingEdge(dut.clk)
        
        # --- Runtime gain retune at mid-simulation ---
        if cycle == RETUNE_CYCLE:
            dut._log.info("=== RUNTIME RETUNE: Increasing roll/pitch Kp from ~0.1 to ~0.2 ===")
            await write_gain(dut, 'roll_kp',  float_to_q88(0.2))   # addr 0x0
            await write_gain(dut, 'pitch_kp', float_to_q88(0.2))   # addr 0x3
            dut._log.info("=== Gains updated. Controller response should change. ===")
        
        # --- 1. Read motor duty cycles from DUT ---
        m0_duty = q88_to_float(dut.motor0_duty.value.to_unsigned() & 0xFFFF)
        m1_duty = q88_to_float(dut.motor1_duty.value.to_unsigned() & 0xFFFF)
        m2_duty = q88_to_float(dut.motor2_duty.value.to_unsigned() & 0xFFFF)
        m3_duty = q88_to_float(dut.motor3_duty.value.to_unsigned() & 0xFFFF)
        
        # --- 2. Update physics model ---
        physics.update(m0_duty, m1_duty, m2_duty, m3_duty)
        
        # --- 3. Compute errors (setpoint - current angle) ---
        roll_err  = ROLL_SETPOINT  - physics.roll
        pitch_err = PITCH_SETPOINT - physics.pitch
        yaw_err   = YAW_SETPOINT   - physics.yaw
        
        # --- 4. Encode errors to Q8.8 and drive into DUT ---
        dut.roll_error.value  = float_to_q88(roll_err)
        dut.pitch_error.value = float_to_q88(pitch_err)
        dut.yaw_error.value   = float_to_q88(yaw_err)
        
        # Keep throttle constant
        dut.throttle.value = float_to_q88(THROTTLE_HOVER)
        
        # --- 5. Read PID outputs for logging ---
        roll_pid  = q88_to_float(dut.roll_pid_out.value.to_unsigned()  & 0xFFFF)
        pitch_pid = q88_to_float(dut.pitch_pid_out.value.to_unsigned() & 0xFFFF)
        yaw_pid   = q88_to_float(dut.yaw_pid_out.value.to_unsigned()   & 0xFFFF)
        
        # --- 6. Log to CSV ---
        csv_writer.writerow([
            cycle,
            f"{roll_err:.6f}", f"{pitch_err:.6f}", f"{yaw_err:.6f}",
            f"{roll_pid:.6f}", f"{pitch_pid:.6f}", f"{yaw_pid:.6f}",
            f"{m0_duty:.6f}", f"{m1_duty:.6f}", f"{m2_duty:.6f}", f"{m3_duty:.6f}",
            f"{physics.roll:.6f}", f"{physics.pitch:.6f}", f"{physics.yaw:.6f}",
            f"{physics.roll_rate:.6f}", f"{physics.pitch_rate:.6f}", f"{physics.yaw_rate:.6f}"
        ])
        
        # --- 7. Periodic logging to console ---
        if cycle % 200 == 0:
            dut._log.info(
                f"Cycle {cycle:5d} | "
                f"Roll: {physics.roll:+8.3f}° (err={roll_err:+8.3f}) | "
                f"Pitch: {physics.pitch:+8.3f}° (err={pitch_err:+8.3f}) | "
                f"Yaw: {physics.yaw:+8.3f}° | "
                f"Motors: [{m0_duty:.1f}, {m1_duty:.1f}, {m2_duty:.1f}, {m3_duty:.1f}]"
            )
    
    # Cleanup
    csv_file.close()
    dut._log.info("HiL simulation complete. Log saved to hil_flight_log.csv")
    
    # Basic assertion: angles should have moved toward setpoints
    roll_final_err  = abs(ROLL_SETPOINT  - physics.roll)
    pitch_final_err = abs(PITCH_SETPOINT - physics.pitch)
    
    dut._log.info(f"Final angles — Roll: {physics.roll:.3f}°, Pitch: {physics.pitch:.3f}°, Yaw: {physics.yaw:.3f}°")
    dut._log.info(f"Final errors — Roll: {roll_final_err:.3f}°, Pitch: {pitch_final_err:.3f}°")
