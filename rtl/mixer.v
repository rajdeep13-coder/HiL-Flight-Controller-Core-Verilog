//============================================================================
// Module:      mixer
// Description: Combinational X-frame quadcopter motor mixer. Translates
//              roll/pitch/yaw PID outputs and throttle command into four
//              individual motor duty cycle commands.
//
// X-Frame Mixing Matrix:
//   Motor 0 (Front Right): Throttle - Roll + Pitch + Yaw
//   Motor 1 (Back Left):   Throttle + Roll - Pitch + Yaw
//   Motor 2 (Front Left):  Throttle + Roll + Pitch - Yaw
//   Motor 3 (Back Right):  Throttle - Roll - Pitch - Yaw
//
// All values in signed Q8.8 fixed-point.
// Motor outputs are clamped to [0, +127.0] (motors can't reverse).
//============================================================================

module mixer (
    input  wire [15:0] throttle,    // Q8.8 base throttle
    input  wire [15:0] roll_out,    // Q8.8 roll PID output
    input  wire [15:0] pitch_out,   // Q8.8 pitch PID output
    input  wire [15:0] yaw_out,     // Q8.8 yaw PID output
    output wire [15:0] motor0,      // Q8.8 front-right motor duty
    output wire [15:0] motor1,      // Q8.8 back-left motor duty
    output wire [15:0] motor2,      // Q8.8 front-left motor duty
    output wire [15:0] motor3       // Q8.8 back-right motor duty
);

    // Motor floor and ceiling (Q8.8)
    localparam signed [15:0] MOTOR_MAX = 16'h7F00;  // +127.0
    localparam signed [15:0] MOTOR_MIN = 16'h0000;  //    0.0

    // Wide intermediate sums (32-bit to catch overflow)
    wire signed [31:0] m0_raw = $signed(throttle) - $signed(roll_out) + $signed(pitch_out) + $signed(yaw_out);
    wire signed [31:0] m1_raw = $signed(throttle) + $signed(roll_out) - $signed(pitch_out) + $signed(yaw_out);
    wire signed [31:0] m2_raw = $signed(throttle) + $signed(roll_out) + $signed(pitch_out) - $signed(yaw_out);
    wire signed [31:0] m3_raw = $signed(throttle) - $signed(roll_out) - $signed(pitch_out) - $signed(yaw_out);

    // Clamp each motor output to [0, +127.0]
    assign motor0 = clamp_motor(m0_raw);
    assign motor1 = clamp_motor(m1_raw);
    assign motor2 = clamp_motor(m2_raw);
    assign motor3 = clamp_motor(m3_raw);

    // Function: clamp_motor
    // Clamps a wide signed value to the motor-safe range [0, MOTOR_MAX].
    function [15:0] clamp_motor;
        input signed [31:0] val;
        begin
            if (val > $signed({16'd0, MOTOR_MAX}))
                clamp_motor = MOTOR_MAX;
            else if (val < $signed(32'd0))
                clamp_motor = MOTOR_MIN;
            else
                clamp_motor = val[15:0];
        end
    endfunction

endmodule
