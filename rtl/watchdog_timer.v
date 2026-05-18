//============================================================================
// Module:      watchdog_timer
// Description: Monitors a data valid pulse (error_valid). If the pulse is
//              not received for TIMEOUT_CYCLES, motor_enable is pulled low
//              to force a safe disarm state.
//============================================================================

module watchdog_timer #(
    parameter TIMEOUT_CYCLES = 50_000_000 // 1 second at 50 MHz
)(
    input  wire clk,
    input  wire rst,
    input  wire error_valid,
    output reg  motor_enable
);

    // Calculate bit width needed for the counter
    localparam integer COUNTER_WIDTH = $clog2(TIMEOUT_CYCLES + 1);

    reg [COUNTER_WIDTH-1:0] counter;

    always @(posedge clk) begin
        if (rst) begin
            counter <= {COUNTER_WIDTH{1'b0}};
            motor_enable <= 1'b0; // Default to disarmed during reset
        end else if (error_valid) begin
            counter <= {COUNTER_WIDTH{1'b0}};
            motor_enable <= 1'b1; // Arm when valid data is received
        end else begin
            if (counter >= TIMEOUT_CYCLES) begin
                motor_enable <= 1'b0; // Disarm on timeout
                // Keep counter at TIMEOUT_CYCLES to avoid wraparound
                counter <= counter;
            end else begin
                counter <= counter + 1'b1;
                // Keep motor_enable high while counting up to timeout
                // But only if it was already high (handled by state preservation)
            end
        end
    end

endmodule
