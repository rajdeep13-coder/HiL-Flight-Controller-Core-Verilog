//============================================================================
// Module:      pwm_gen
// Description: Counter-based PWM generator. Produces a single-bit PWM
//              output at the configured frequency. The duty cycle is
//              controlled by duty_word (Q8.8 unsigned, interpreted as
//              0–127 mapped to 0–100% duty).
//
// Timing:      Counter period = CLK_FREQ / PWM_FREQ
//              Default: 50 MHz / 50 Hz = 1,000,000 counts per period
//============================================================================

module pwm_gen #(
    parameter CLK_FREQ = 50_000_000,  // System clock frequency (Hz)
    parameter PWM_FREQ = 50           // PWM output frequency (Hz)
)(
    input  wire        clk,
    input  wire        rst,
    input  wire [15:0] duty_word,     // Q8.8 duty cycle value
    output reg         pwm_out        // 1-bit PWM output
);

    // Counter parameters
    localparam integer COUNTER_MAX = CLK_FREQ / PWM_FREQ;

    // Calculate bit width needed for the counter
    // $clog2 gives ceiling of log2
    localparam integer COUNTER_WIDTH = $clog2(COUNTER_MAX + 1);

    // Counter register
    reg [COUNTER_WIDTH-1:0] counter;

    // Simulation start assertion
    localparam integer ACTUAL_PWM_FREQ = CLK_FREQ / COUNTER_MAX;
    initial begin
        $display("PWM Generator: CLK_FREQ=%0d Hz, Target PWM_FREQ=%0d Hz, Actual PWM_FREQ=%0d Hz", 
                 CLK_FREQ, PWM_FREQ, ACTUAL_PWM_FREQ);
    end

    // Threshold calculation
    // duty_word is Q8.8 where the integer part (bits [15:8]) represents
    // the duty percentage (0–127). We scale it to the counter range:
    //   threshold = (duty_word * COUNTER_MAX) >> 8
    // Using wider intermediate to prevent overflow.
    wire [COUNTER_WIDTH+15:0] threshold_wide = duty_word * COUNTER_MAX;
    wire [COUNTER_WIDTH-1:0]  threshold      = threshold_wide[COUNTER_WIDTH+7:8];

    // Counter and PWM output logic
    always @(posedge clk) begin
        if (rst) begin
            counter <= {COUNTER_WIDTH{1'b0}};
            pwm_out <= 1'b0;
        end else begin
            // Free-running counter
            if (counter >= COUNTER_MAX - 1)
                counter <= {COUNTER_WIDTH{1'b0}};
            else
                counter <= counter + 1'b1;

            // PWM comparison
            pwm_out <= (counter < threshold) ? 1'b1 : 1'b0;
        end
    end

endmodule
