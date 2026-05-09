//============================================================================
// Testbench: tb_pid
// Description: Standalone Verilog testbench for pid_controller and
//              saturation_guard modules. Applies a step error input and
//              observes the PID response over 500 clock cycles.
//
// Run with Icarus Verilog:
//   iverilog -o tb_pid rtl/pid_controller.v rtl/saturation_guard.v sim/tb_pid.v
//   vvp tb_pid
//   gtkwave tb_pid.vcd  (optional waveform viewer)
//============================================================================

`timescale 1ns / 1ps

module tb_pid;

    //------------------------------------------------------------------------
    // Parameters — using moderate gains for visible response
    //------------------------------------------------------------------------
    localparam signed [15:0] Kp      = 16'h0100;  // 1.0
    localparam signed [15:0] Ki      = 16'h001A;  // ~0.1 (26/256)
    localparam signed [15:0] Kd      = 16'h0080;  // 0.5
    localparam signed [15:0] MAX_OUT = 16'h7F00;  // +127.0
    localparam signed [15:0] MIN_OUT = 16'h8100;  // -127.0

    //------------------------------------------------------------------------
    // DUT signals
    //------------------------------------------------------------------------
    reg         clk;
    reg         rst;
    reg  [15:0] error_in;

    wire [15:0] pid_raw_out;
    wire [15:0] pid_clamped_out;
    wire        integrator_hold;

    //------------------------------------------------------------------------
    // DUT instantiation: PID → Saturation Guard
    //------------------------------------------------------------------------
    pid_controller #(
        .Kp      (Kp),
        .Ki      (Ki),
        .Kd      (Kd),
        .MAX_OUT (MAX_OUT),
        .MIN_OUT (MIN_OUT)
    ) dut_pid (
        .clk             (clk),
        .rst             (rst),
        .error_in        (error_in),
        .integrator_hold (integrator_hold),
        .pid_out         (pid_raw_out)
    );

    saturation_guard #(
        .MAX_OUT (MAX_OUT),
        .MIN_OUT (MIN_OUT)
    ) dut_sat (
        .raw_in          (pid_raw_out),
        .clamped_out     (pid_clamped_out),
        .integrator_hold (integrator_hold)
    );

    //------------------------------------------------------------------------
    // Clock generation — 20 ns period (50 MHz)
    //------------------------------------------------------------------------
    initial clk = 0;
    always #10 clk = ~clk;

    //------------------------------------------------------------------------
    // Helper function: Convert Q8.8 to real for display
    //------------------------------------------------------------------------
    // Note: Icarus Verilog supports $itor for integer-to-real conversion
    function real q88_to_real;
        input [15:0] val;
        reg signed [15:0] sval;
        begin
            sval = val;
            q88_to_real = $itor(sval) / 256.0;
        end
    endfunction

    //------------------------------------------------------------------------
    // VCD dump setup
    //------------------------------------------------------------------------
    initial begin
        $dumpfile("tb_pid.vcd");
        $dumpvars(0, tb_pid);
    end

    //------------------------------------------------------------------------
    // Test sequence
    //------------------------------------------------------------------------
    integer cycle;

    initial begin
        $display("============================================================");
        $display("  PID Controller + Saturation Guard Testbench");
        $display("  Kp=%.4f  Ki=%.4f  Kd=%.4f", q88_to_real(Kp), q88_to_real(Ki), q88_to_real(Kd));
        $display("============================================================");
        $display("");
        $display("  Cycle | Error(Q8.8) | Error(dec) | PID Raw    | PID Clamp  | Hold");
        $display("  ------|-------------|------------|------------|------------|-----");

        // Initialize
        rst      = 1;
        error_in = 16'h0000;
        cycle    = 0;

        // Hold reset for 5 cycles
        repeat (5) @(posedge clk);
        rst = 0;

        // ---- TEST 1: Positive step error (+10.0 in Q8.8 = 0x0A00) ----
        $display("");
        $display("  [TEST 1] Step error: +10.0 (0x0A00)");
        error_in = 16'h0A00;  // +10.0

        repeat (100) begin
            @(posedge clk);
            cycle = cycle + 1;
            if (cycle <= 20 || cycle % 10 == 0) begin
                $display("  %5d | 0x%04h     | %8.4f   | 0x%04h     | 0x%04h     | %b",
                    cycle, error_in, q88_to_real(error_in),
                    pid_raw_out, pid_clamped_out, integrator_hold);
            end
        end

        // ---- TEST 2: Zero error (should see derivative kick, integral hold) ----
        $display("");
        $display("  [TEST 2] Step error to 0.0 (0x0000)");
        error_in = 16'h0000;

        repeat (100) begin
            @(posedge clk);
            cycle = cycle + 1;
            if (cycle <= 120 || cycle % 10 == 0) begin
                $display("  %5d | 0x%04h     | %8.4f   | 0x%04h     | 0x%04h     | %b",
                    cycle, error_in, q88_to_real(error_in),
                    pid_raw_out, pid_clamped_out, integrator_hold);
            end
        end

        // ---- TEST 3: Negative step error (-5.0 = 0xFB00) ----
        $display("");
        $display("  [TEST 3] Step error: -5.0 (0xFB00)");
        error_in = 16'hFB00;  // -5.0

        repeat (100) begin
            @(posedge clk);
            cycle = cycle + 1;
            if (cycle <= 220 || cycle % 10 == 0) begin
                $display("  %5d | 0x%04h     | %8.4f   | 0x%04h     | 0x%04h     | %b",
                    cycle, error_in, q88_to_real(error_in),
                    pid_raw_out, pid_clamped_out, integrator_hold);
            end
        end

        // ---- TEST 4: Large error to test saturation (+120.0 = 0x7800) ----
        $display("");
        $display("  [TEST 4] Large error: +120.0 (0x7800) — should saturate");
        error_in = 16'h7800;  // +120.0

        repeat (50) begin
            @(posedge clk);
            cycle = cycle + 1;
            $display("  %5d | 0x%04h     | %8.4f   | 0x%04h     | 0x%04h     | %b",
                cycle, error_in, q88_to_real(error_in),
                pid_raw_out, pid_clamped_out, integrator_hold);
        end

        // ---- TEST 5: Reset mid-operation ----
        $display("");
        $display("  [TEST 5] Assert reset — all outputs should zero");
        rst = 1;
        repeat (3) @(posedge clk);
        cycle = cycle + 3;
        $display("  %5d | 0x%04h     | %8.4f   | 0x%04h     | 0x%04h     | %b",
            cycle, error_in, q88_to_real(error_in),
            pid_raw_out, pid_clamped_out, integrator_hold);

        // Verify reset behavior
        if (pid_raw_out == 16'h0000 && pid_clamped_out == 16'h0000)
            $display("  [PASS] Reset cleared PID output correctly.");
        else
            $display("  [FAIL] PID output not zero after reset! raw=0x%04h clamp=0x%04h",
                pid_raw_out, pid_clamped_out);

        rst = 0;

        // ---- Summary ----
        $display("");
        $display("============================================================");
        $display("  Testbench complete. Review tb_pid.vcd for waveforms.");
        $display("============================================================");
        $finish;
    end

endmodule
