//============================================================================
// Module:      pid_controller
// Description: Parameterized PID controller using signed Q8.8 fixed-point
//              arithmetic. Implements Proportional, Integral, and Derivative
//              control with anti-windup support via integrator_hold.
//
// Fixed-Point: Signed Q8.8 — 1 sign bit, 7 integer bits, 8 fractional bits
//              Range: -128.0 to +127.99609375
//              Resolution: 1/256 ≈ 0.00390625
//============================================================================

module pid_controller #(
    parameter signed [15:0] MAX_OUT = 16'h7F00,  // +127.0 in Q8.8
    parameter signed [15:0] MIN_OUT = 16'h8100   // -127.0 in Q8.8
)(
    input  wire        clk,
    input  wire        rst,
    // Runtime-configurable PID gains (Q8.8 signed)
    input  wire [15:0] Kp,               // Proportional gain
    input  wire [15:0] Ki,               // Integral gain
    input  wire [15:0] Kd,               // Derivative gain
    input  wire [15:0] error_in,         // Signed Q8.8 error
    input  wire        integrator_hold,  // Freeze integrator when saturated
    output reg  [15:0] pid_out,          // Signed Q8.8 PID output (raw, unclamped)
    output reg         overflow          // High when internal arithmetic overflows
);

    // Internal registers
    reg signed [15:0] prev_error;       // Previous error for derivative term
    reg signed [31:0] integrator;       // Wider accumulator for integral term

    // Intermediate wires for combinational arithmetic
    wire signed [15:0] error_signed = $signed(error_in);

    // --- Proportional term ---
    // Q8.8 × Q8.8 = Q16.16 (32 bits). Extract [23:8] for Q8.8 result.
    wire signed [31:0] p_product = $signed(Kp) * error_signed;
    wire signed [15:0] p_term;
    assign p_term = saturate_q88(p_product);

    // --- Derivative term ---
    // Compute difference using 17 bits to avoid silent 16-bit wraparound
    wire signed [16:0] error_diff_wide = $signed(error_signed) - $signed(prev_error);
    wire signed [15:0] error_diff = clamp_diff(error_diff_wide);
    wire signed [31:0] d_product  = $signed(Kd) * error_diff;
    wire signed [15:0] d_term;
    assign d_term = saturate_q88(d_product);

    // --- Integral term (Ki × error, to be accumulated) ---
    wire signed [31:0] i_product = $signed(Ki) * error_signed;
    wire signed [15:0] i_increment;
    assign i_increment = saturate_q88(i_product);

    // --- Integrator value clamped to Q8.8 range for output ---
    wire signed [15:0] i_term;
    assign i_term = clamp_integrator(integrator);

    // --- PID sum (use wider bus to detect overflow before clamping) ---
    wire signed [31:0] pid_sum_wide = $signed(p_term) + $signed(i_term) + $signed(d_term);
    wire signed [15:0] pid_sum_clamped;
    assign pid_sum_clamped = clamp_to_range(pid_sum_wide, $signed(MAX_OUT), $signed(MIN_OUT));

    // --- Overflow detection logic ---
    wire p_overflow = (p_product[31] == 1'b0 && p_product[31:23] != 9'h000) ||
                      (p_product[31] == 1'b1 && p_product[31:23] != 9'h1FF);
    wire i_overflow = (i_product[31] == 1'b0 && i_product[31:23] != 9'h000) ||
                      (i_product[31] == 1'b1 && i_product[31:23] != 9'h1FF);
    wire d_overflow = (d_product[31] == 1'b0 && d_product[31:23] != 9'h000) ||
                      (d_product[31] == 1'b1 && d_product[31:23] != 9'h1FF);
    wire diff_overflow = (error_diff_wide > 17'sd32767) || (error_diff_wide < -17'sd32768);
    wire sum_overflow = (pid_sum_wide > $signed({{16{MAX_OUT[15]}}, MAX_OUT})) ||
                        (pid_sum_wide < $signed({{16{MIN_OUT[15]}}, MIN_OUT}));
    
    wire any_overflow = p_overflow | i_overflow | d_overflow | diff_overflow | sum_overflow;

    // Clocked process
    always @(posedge clk) begin
        if (rst) begin
            prev_error  <= 16'sd0;
            integrator  <= 32'sd0;
            pid_out     <= 16'sd0;
            overflow    <= 1'b0;
        end else begin
            // Update previous error
            prev_error <= error_signed;

            // Update integrator (with anti-windup hold)
            if (!integrator_hold) begin
                integrator <= integrator + $signed({{16{i_increment[15]}}, i_increment});
            end

            // Output the clamped PID sum
            pid_out <= pid_sum_clamped;
            
            // Update overflow flag
            overflow <= any_overflow;
        end
    end

    // Function: saturate_q88
    // Extracts Q8.8 from a Q16.16 product with overflow saturation.
    function signed [15:0] saturate_q88;
        input signed [31:0] product;
        reg signed [15:0] truncated;
        reg overflow_pos, overflow_neg;
        begin
            truncated    = product[23:8];
            // Check if upper bits [31:23] are all 0 (positive) or all 1 (negative)
            // For no overflow: positive → [31:23] == 9'b0, negative → [31:23] == 9'h1FF
            overflow_pos = (product[31] == 1'b0) && (product[31:23] != 9'h000);
            overflow_neg = (product[31] == 1'b1) && (product[31:23] != 9'h1FF);

            if (overflow_pos)
                saturate_q88 = 16'h7FFF;  // Max positive Q8.8
            else if (overflow_neg)
                saturate_q88 = 16'h8000;  // Max negative Q8.8
            else
                saturate_q88 = truncated;
        end
    endfunction

    // Function: clamp_integrator
    // Clamps the 32-bit integrator accumulator to Q8.8 range.
    function signed [15:0] clamp_integrator;
        input signed [31:0] acc;
        begin
            if (acc > $signed(32'h00007FFF))
                clamp_integrator = 16'h7FFF;
            else if (acc < $signed(32'hFFFF8000))
                clamp_integrator = 16'h8000;
            else
                clamp_integrator = acc[15:0];
        end
    endfunction

    // Function: clamp_to_range
    // Clamps a wide sum to the specified [MIN, MAX] range.
    function signed [15:0] clamp_to_range;
        input signed [31:0] val;
        input signed [15:0] max_val;
        input signed [15:0] min_val;
        begin
            if (val > $signed({{16{max_val[15]}}, max_val}))
                clamp_to_range = max_val;
            else if (val < $signed({{16{min_val[15]}}, min_val}))
                clamp_to_range = min_val;
            else
                clamp_to_range = val[15:0];
        end
    endfunction

    // Function: clamp_diff
    // Clamps a 17-bit difference to 16-bit Q8.8 range.
    function signed [15:0] clamp_diff;
        input signed [16:0] diff;
        begin
            if (diff > 17'sd32767)
                clamp_diff = 16'h7FFF;
            else if (diff < -17'sd32768)
                clamp_diff = 16'h8000;
            else
                clamp_diff = diff[15:0];
        end
    endfunction

endmodule
