//============================================================================
// Module:      gain_regs
// Description: Runtime-writable PID gain register file. Provides 9
//              addressable Q8.8 registers (Kp/Ki/Kd × 3 axes) accessible
//              via a simple parallel write bus (wr_en + addr + data).
//
//              Reset defaults match the original compile-time parameters,
//              so the controller behaves identically out of reset.
//
// Bus Protocol:
//   To write a register, assert wr_en=1 with the target wr_addr and
//   wr_data on the same rising clock edge. The register updates on that
//   edge. De-assert wr_en on the next cycle.
//
// Register Map:
//   0x0  ROLL_Kp     0x3  PITCH_Kp     0x6  YAW_Kp
//   0x1  ROLL_Ki     0x4  PITCH_Ki     0x7  YAW_Ki
//   0x2  ROLL_Kd     0x5  PITCH_Kd     0x8  YAW_Kd
//
// Fixed-Point: All values are signed Q8.8 (16-bit).
//============================================================================

module gain_regs #(
    // Reset defaults — match flight_controller_top original parameters
    parameter signed [15:0] ROLL_Kp_INIT  = 16'h001A,  // ~0.1
    parameter signed [15:0] ROLL_Ki_INIT  = 16'h0001,  // ~0.004
    parameter signed [15:0] ROLL_Kd_INIT  = 16'h0033,  // ~0.2
    parameter signed [15:0] PITCH_Kp_INIT = 16'h001A,  // ~0.1
    parameter signed [15:0] PITCH_Ki_INIT = 16'h0001,  // ~0.004
    parameter signed [15:0] PITCH_Kd_INIT = 16'h0033,  // ~0.2
    parameter signed [15:0] YAW_Kp_INIT   = 16'h000D,  // ~0.05
    parameter signed [15:0] YAW_Ki_INIT   = 16'h0001,  // ~0.004
    parameter signed [15:0] YAW_Kd_INIT   = 16'h001A   // ~0.1
)(
    input  wire        clk,
    input  wire        rst,

    // Write bus
    input  wire        wr_en,        // Write enable (active-high, single-cycle strobe)
    input  wire [3:0]  wr_addr,      // Register address (0–8)
    input  wire [15:0] wr_data,      // Write data (Q8.8)

    // Roll gains
    output reg  [15:0] roll_kp,
    output reg  [15:0] roll_ki,
    output reg  [15:0] roll_kd,

    // Pitch gains
    output reg  [15:0] pitch_kp,
    output reg  [15:0] pitch_ki,
    output reg  [15:0] pitch_kd,

    // Yaw gains
    output reg  [15:0] yaw_kp,
    output reg  [15:0] yaw_ki,
    output reg  [15:0] yaw_kd
);

    // Address constants
    localparam [3:0] ADDR_ROLL_KP  = 4'd0;
    localparam [3:0] ADDR_ROLL_KI  = 4'd1;
    localparam [3:0] ADDR_ROLL_KD  = 4'd2;
    localparam [3:0] ADDR_PITCH_KP = 4'd3;
    localparam [3:0] ADDR_PITCH_KI = 4'd4;
    localparam [3:0] ADDR_PITCH_KD = 4'd5;
    localparam [3:0] ADDR_YAW_KP   = 4'd6;
    localparam [3:0] ADDR_YAW_KI   = 4'd7;
    localparam [3:0] ADDR_YAW_KD   = 4'd8;

    always @(posedge clk) begin
        if (rst) begin
            // Load defaults on reset
            roll_kp  <= ROLL_Kp_INIT;
            roll_ki  <= ROLL_Ki_INIT;
            roll_kd  <= ROLL_Kd_INIT;
            pitch_kp <= PITCH_Kp_INIT;
            pitch_ki <= PITCH_Ki_INIT;
            pitch_kd <= PITCH_Kd_INIT;
            yaw_kp   <= YAW_Kp_INIT;
            yaw_ki   <= YAW_Ki_INIT;
            yaw_kd   <= YAW_Kd_INIT;
        end else if (wr_en) begin
            case (wr_addr)
                ADDR_ROLL_KP:  roll_kp  <= wr_data;
                ADDR_ROLL_KI:  roll_ki  <= wr_data;
                ADDR_ROLL_KD:  roll_kd  <= wr_data;
                ADDR_PITCH_KP: pitch_kp <= wr_data;
                ADDR_PITCH_KI: pitch_ki <= wr_data;
                ADDR_PITCH_KD: pitch_kd <= wr_data;
                ADDR_YAW_KP:   yaw_kp   <= wr_data;
                ADDR_YAW_KI:   yaw_ki   <= wr_data;
                ADDR_YAW_KD:   yaw_kd   <= wr_data;
                default: ;  // Ignore writes to undefined addresses
            endcase
        end
    end

endmodule
