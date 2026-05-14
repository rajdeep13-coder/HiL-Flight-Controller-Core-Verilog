`timescale 1ns / 1ps

//============================================================================
// Module:      flight_controller_top
// Description: Top-level flight controller integrating runtime-configurable
//              gain registers, 3× PID controllers (roll/pitch/yaw),
//              3× saturation guards, 1× mixer, and 4× PWM generators.
//
//              PID gains are written at runtime via a simple parallel bus
//              (wr_en + wr_addr + wr_data) and stored in the gain_regs
//              register file. See gain_regs.v for the register map.
//
// Architecture:
//   wr_bus → [GAIN_REGS] → gains
//                              ↓
//   error_in → [PID] → raw_out → [SAT_GUARD] → clamped_out → [MIXER] → [PWM]
//                ↑                    |
//                └── integrator_hold ─┘
//
// Fixed-Point: All signals use signed Q8.8 (16-bit).
//============================================================================

module flight_controller_top #(
    // Saturation limits (Q8.8)
    parameter signed [15:0] MAX_OUT  = 16'h7F00,  // +127.0
    parameter signed [15:0] MIN_OUT  = 16'h8100,  // -127.0
    // PWM parameters
    parameter CLK_FREQ = 50_000_000,
    parameter PWM_FREQ = 50
)(
    // ---- System ----
    input  wire        clk,
    input  wire        rst,

    // ---- Gain register write bus ----
    input  wire        wr_en,          // Write enable (single-cycle strobe)
    input  wire [3:0]  wr_addr,        // Register address (see gain_regs.v)
    input  wire [15:0] wr_data,        // Write data (Q8.8)

    // ---- Error inputs (Q8.8 signed) ----
    input  wire [15:0] roll_error,
    input  wire [15:0] pitch_error,
    input  wire [15:0] yaw_error,

    // ---- Throttle command (Q8.8) ----
    input  wire [15:0] throttle,

    // ---- PID outputs for monitoring (Q8.8 signed) ----
    output wire [15:0] roll_pid_out,
    output wire [15:0] pitch_pid_out,
    output wire [15:0] yaw_pid_out,

    // ---- Motor duty cycle outputs (Q8.8) ----
    output wire [15:0] motor0_duty,
    output wire [15:0] motor1_duty,
    output wire [15:0] motor2_duty,
    output wire [15:0] motor3_duty,

    // ---- PWM outputs ----
    output wire        pwm_out0,
    output wire        pwm_out1,
    output wire        pwm_out2,
    output wire        pwm_out3,

    // ---- Overflow flags for monitoring ----
    output wire        roll_pid_overflow,
    output wire        pitch_pid_overflow,
    output wire        yaw_pid_overflow,
    output wire        roll_sat_overflow,
    output wire        pitch_sat_overflow,
    output wire        yaw_sat_overflow
);

    // ================================================================
    // Gain wires from register file
    // ================================================================
    wire [15:0] roll_kp,  roll_ki,  roll_kd;
    wire [15:0] pitch_kp, pitch_ki, pitch_kd;
    wire [15:0] yaw_kp,   yaw_ki,   yaw_kd;

    gain_regs gain_regs_inst (
        .clk      (clk),
        .rst      (rst),
        .wr_en    (wr_en),
        .wr_addr  (wr_addr),
        .wr_data  (wr_data),
        .roll_kp  (roll_kp),
        .roll_ki  (roll_ki),
        .roll_kd  (roll_kd),
        .pitch_kp (pitch_kp),
        .pitch_ki (pitch_ki),
        .pitch_kd (pitch_kd),
        .yaw_kp   (yaw_kp),
        .yaw_ki   (yaw_ki),
        .yaw_kd   (yaw_kd)
    );

    // ================================================================
    // Internal wires
    // ================================================================

    // Raw PID outputs (before saturation)
    wire [15:0] roll_pid_raw;
    wire [15:0] pitch_pid_raw;
    wire [15:0] yaw_pid_raw;

    // Integrator hold signals (from saturation guards)
    wire roll_integrator_hold;
    wire pitch_integrator_hold;
    wire yaw_integrator_hold;

    // Clamped PID outputs (after saturation)
    wire [15:0] roll_clamped;
    wire [15:0] pitch_clamped;
    wire [15:0] yaw_clamped;

    // ================================================================
    // PID Controllers (gains from register file)
    // ================================================================

    pid_controller #(
        .MAX_OUT (MAX_OUT),
        .MIN_OUT (MIN_OUT)
    ) pid_roll (
        .clk             (clk),
        .rst             (rst),
        .Kp              (roll_kp),
        .Ki              (roll_ki),
        .Kd              (roll_kd),
        .error_in        (roll_error),
        .integrator_hold (roll_integrator_hold),
        .pid_out         (roll_pid_raw),
        .overflow        (roll_pid_overflow)
    );

    pid_controller #(
        .MAX_OUT (MAX_OUT),
        .MIN_OUT (MIN_OUT)
    ) pid_pitch (
        .clk             (clk),
        .rst             (rst),
        .Kp              (pitch_kp),
        .Ki              (pitch_ki),
        .Kd              (pitch_kd),
        .error_in        (pitch_error),
        .integrator_hold (pitch_integrator_hold),
        .pid_out         (pitch_pid_raw),
        .overflow        (pitch_pid_overflow)
    );

    pid_controller #(
        .MAX_OUT (MAX_OUT),
        .MIN_OUT (MIN_OUT)
    ) pid_yaw (
        .clk             (clk),
        .rst             (rst),
        .Kp              (yaw_kp),
        .Ki              (yaw_ki),
        .Kd              (yaw_kd),
        .error_in        (yaw_error),
        .integrator_hold (yaw_integrator_hold),
        .pid_out         (yaw_pid_raw),
        .overflow        (yaw_pid_overflow)
    );

    // ================================================================
    // Saturation Guards (combinational anti-windup)
    // ================================================================

    saturation_guard #(
        .MAX_OUT (MAX_OUT),
        .MIN_OUT (MIN_OUT)
    ) sat_roll (
        .raw_in          (roll_pid_raw),
        .clamped_out     (roll_clamped),
        .integrator_hold (roll_integrator_hold),
        .overflow        (roll_sat_overflow)
    );

    saturation_guard #(
        .MAX_OUT (MAX_OUT),
        .MIN_OUT (MIN_OUT)
    ) sat_pitch (
        .raw_in          (pitch_pid_raw),
        .clamped_out     (pitch_clamped),
        .integrator_hold (pitch_integrator_hold),
        .overflow        (pitch_sat_overflow)
    );

    saturation_guard #(
        .MAX_OUT (MAX_OUT),
        .MIN_OUT (MIN_OUT)
    ) sat_yaw (
        .raw_in          (yaw_pid_raw),
        .clamped_out     (yaw_clamped),
        .integrator_hold (yaw_integrator_hold),
        .overflow        (yaw_sat_overflow)
    );

    // Expose clamped PID outputs for monitoring
    assign roll_pid_out  = roll_clamped;
    assign pitch_pid_out = pitch_clamped;
    assign yaw_pid_out   = yaw_clamped;

    // ================================================================
    // Motor Mixer (X-frame configuration)
    // ================================================================

    mixer mixer_inst (
        .throttle  (throttle),
        .roll_out  (roll_clamped),
        .pitch_out (pitch_clamped),
        .yaw_out   (yaw_clamped),
        .motor0    (motor0_duty),
        .motor1    (motor1_duty),
        .motor2    (motor2_duty),
        .motor3    (motor3_duty)
    );

    // ================================================================
    // PWM Generators (one per motor)
    // ================================================================

    pwm_gen #(
        .CLK_FREQ (CLK_FREQ),
        .PWM_FREQ (PWM_FREQ)
    ) pwm_motor0 (
        .clk       (clk),
        .rst       (rst),
        .duty_word (motor0_duty),
        .pwm_out   (pwm_out0)
    );

    pwm_gen #(
        .CLK_FREQ (CLK_FREQ),
        .PWM_FREQ (PWM_FREQ)
    ) pwm_motor1 (
        .clk       (clk),
        .rst       (rst),
        .duty_word (motor1_duty),
        .pwm_out   (pwm_out1)
    );

    pwm_gen #(
        .CLK_FREQ (CLK_FREQ),
        .PWM_FREQ (PWM_FREQ)
    ) pwm_motor2 (
        .clk       (clk),
        .rst       (rst),
        .duty_word (motor2_duty),
        .pwm_out   (pwm_out2)
    );

    pwm_gen #(
        .CLK_FREQ (CLK_FREQ),
        .PWM_FREQ (PWM_FREQ)
    ) pwm_motor3 (
        .clk       (clk),
        .rst       (rst),
        .duty_word (motor3_duty),
        .pwm_out   (pwm_out3)
    );

endmodule
