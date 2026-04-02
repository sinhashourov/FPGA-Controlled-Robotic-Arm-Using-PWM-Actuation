// =============================================================================
// Module  : uart_rx
// Project : 3-DOF Robotic Arm Controller
// Board   : Nexys A7 100T  (100 MHz system clock)
// Desc    : Standard 8N1 UART receiver.
//           Asserts rx_valid for one clock cycle when a full byte is ready.
//
// Baud rate : 115200  →  clk_divider = 100_000_000 / 115200 ≈ 868
// =============================================================================

module uart_rx #(
    parameter CLK_FREQ  = 100_000_000,   // Hz
    parameter BAUD_RATE = 115_200
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rx,          // Serial RX line (from ESP32 TX)
    output reg  [7:0] rx_data,     // Received byte
    output reg        rx_valid     // Pulses high for 1 clk when byte is ready
);

    // -------------------------------------------------------------------------
    // Derived parameter – number of clock cycles per bit
    // -------------------------------------------------------------------------
    localparam integer CLK_DIV     = CLK_FREQ / BAUD_RATE;          // 868
    localparam integer CLK_DIV_2   = CLK_DIV / 2;                   // 434

    // -------------------------------------------------------------------------
    // Double-flop the RX line to remove metastability
    // -------------------------------------------------------------------------
    reg rx_meta, rx_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_meta <= 1'b1;
            rx_sync <= 1'b1;
        end else begin
            rx_meta <= rx;
            rx_sync <= rx_meta;
        end
    end

    // -------------------------------------------------------------------------
    // State machine
    // -------------------------------------------------------------------------
    localparam IDLE  = 2'd0,
               START = 2'd1,
               DATA  = 2'd2,
               STOP  = 2'd3;

    reg [1:0]  state;
    reg [15:0] clk_cnt;    // Baud rate counter
    reg [2:0]  bit_idx;    // Current data bit index (0..7)
    reg [7:0]  shift_reg;  // Shift register for incoming bits

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= IDLE;
            clk_cnt   <= 16'd0;
            bit_idx   <= 3'd0;
            shift_reg <= 8'd0;
            rx_data   <= 8'd0;
            rx_valid  <= 1'b0;
        end else begin
            rx_valid <= 1'b0;   // Default – deassert every cycle

            case (state)
                // ---------------------------------------------------------
                // IDLE : wait for falling edge (start bit)
                // ---------------------------------------------------------
                IDLE: begin
                    if (rx_sync == 1'b0) begin
                        state   <= START;
                        clk_cnt <= 16'd0;
                    end
                end

                // ---------------------------------------------------------
                // START : wait half a bit period, then confirm start bit
                // ---------------------------------------------------------
                START: begin
                    if (clk_cnt == CLK_DIV_2 - 1) begin
                        clk_cnt <= 16'd0;
                        if (rx_sync == 1'b0) begin
                            state   <= DATA;
                            bit_idx <= 3'd0;
                        end else begin
                            state   <= IDLE;   // Glitch – abort
                        end
                    end else begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                end

                // ---------------------------------------------------------
                // DATA : sample 8 data bits, LSB first
                // ---------------------------------------------------------
                DATA: begin
                    if (clk_cnt == CLK_DIV - 1) begin
                        clk_cnt            <= 16'd0;
                        shift_reg[bit_idx] <= rx_sync;   // Sample at bit center
                        if (bit_idx == 3'd7) begin
                            state <= STOP;
                        end else begin
                            bit_idx <= bit_idx + 1'b1;
                        end
                    end else begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                end

                // ---------------------------------------------------------
                // STOP : wait one bit period, output the byte
                // ---------------------------------------------------------
                STOP: begin
                    if (clk_cnt == CLK_DIV - 1) begin
                        clk_cnt <= 16'd0;
                        state   <= IDLE;
                        if (rx_sync == 1'b1) begin   // Valid stop bit
                            rx_data  <= shift_reg;
                            rx_valid <= 1'b1;
                        end
                        // If stop bit is 0 (framing error) – silently discard
                    end else begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
