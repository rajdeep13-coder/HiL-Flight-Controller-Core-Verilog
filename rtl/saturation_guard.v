//============================================================================
// Module:      saturation_guard
// Description: Combinational output saturation / clamping with anti-windup
//              integrator hold signal. When the raw PID output exceeds the
//              configured limits, the output is clamped and integrator_hold
//              is asserted to prevent integral windup.
//
// Fixed-Point: Signed Q8.8 — 16-bit, 1 sign + 7 integer + 8 fractional
//============================================================================

module saturation_guard #(
    parameter signed [15:0] MAX_OUT = 16'h7F00,  // +127.0 in Q8.8
    parameter signed [15:0] MIN_OUT = 16'h8100   // -127.0 in Q8.8
)(
    input  wire [15:0] raw_in,          // Raw PID output (Q8.8 signed)
    output wire [15:0] clamped_out,     // Clamped output (Q8.8 signed)
    output wire        integrator_hold, // High when output is saturated
    output wire        overflow         // High when output is saturated
);

    // Saturation logic (purely combinational)
    wire saturated_high = ($signed(raw_in) > $signed(MAX_OUT));
    wire saturated_low  = ($signed(raw_in) < $signed(MIN_OUT));

    assign clamped_out = saturated_high ? MAX_OUT :
                         saturated_low  ? MIN_OUT :
                         raw_in;

    // Assert integrator hold when output is at either limit
    assign integrator_hold = saturated_high | saturated_low;
    assign overflow = saturated_high | saturated_low;

endmodule
