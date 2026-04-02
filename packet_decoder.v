// =============================================================================
// Module  : packet_decoder
// Project : 3-DOF Robotic Arm Controller
//
// Packet format (8 bytes):
//   Byte 0 : 0xAA              – Start marker
//   Byte 1 : {4'b0, S1[11:8]} – Servo 1 high nibble
//   Byte 2 : S1[7:0]          – Servo 1 low byte
//   Byte 3 : {4'b0, S2[11:8]} – Servo 2 high nibble
//   Byte 4 : S2[7:0]          – Servo 2 low byte
//   Byte 5 : {4'b0, S3[11:8]} – Servo 3 high nibble
//   Byte 6 : S3[7:0]          – Servo 3 low byte
//   Byte 7 : XOR of bytes 1..6 – Checksum
//
// FIX: Removed the mid-packet 0xAA resync block that existed after the
//      case statement. That block incorrectly triggered whenever any DATA
//      byte (pkt[2], pkt[4], pkt[6]) happened to equal 0xAA (decimal 170),
//      which is a perfectly valid 8-bit servo value. When it fired, it
//      reset the FSM mid-packet, causing wrong servo values to be latched.
//      The FSM now relies solely on byte counting + checksum validation,
//      which is the correct and robust approach.
// =============================================================================

module packet_decoder (
    input  wire        clk,
    input  wire        rst_n,

    input  wire [7:0]  rx_data,
    input  wire        rx_valid,

    output reg  [11:0] servo1_val,
    output reg  [11:0] servo2_val,
    output reg  [11:0] servo3_val,
    output reg         packet_valid
);

    localparam START_BYTE = 8'hAA;

    localparam S_IDLE  = 3'd0,
               S_S1H   = 3'd1,
               S_S1L   = 3'd2,
               S_S2H   = 3'd3,
               S_S2L   = 3'd4,
               S_S3H   = 3'd5,
               S_S3L   = 3'd6,
               S_CKSUM = 3'd7;

    reg [2:0]  state;
    reg [7:0]  chk_accum;
    reg [11:0] s1_tmp, s2_tmp, s3_tmp;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= S_IDLE;
            chk_accum    <= 8'd0;
            s1_tmp       <= 12'd0;
            s2_tmp       <= 12'd0;
            s3_tmp       <= 12'd0;
            servo1_val   <= 12'd2048;
            servo2_val   <= 12'd2048;
            servo3_val   <= 12'd2048;
            packet_valid <= 1'b0;
        end else begin
            packet_valid <= 1'b0;

            if (rx_valid) begin
                case (state)
                    // ---------------------------------------------------------
                    // IDLE: wait for start byte 0xAA
                    // ---------------------------------------------------------
                    S_IDLE: begin
                        if (rx_data == START_BYTE) begin
                            state     <= S_S1H;
                            chk_accum <= 8'd0;
                        end
                    end

                    // ---------------------------------------------------------
                    // Servo 1 high nibble (bits 11:8)
                    // ---------------------------------------------------------
                    S_S1H: begin
                        s1_tmp    <= {rx_data[3:0], 8'd0};
                        chk_accum <= chk_accum ^ rx_data;
                        state     <= S_S1L;
                    end

                    // ---------------------------------------------------------
                    // Servo 1 low byte (bits 7:0)
                    // NOTE: this byte CAN be 0xAA — do NOT treat as resync
                    // ---------------------------------------------------------
                    S_S1L: begin
                        s1_tmp    <= {s1_tmp[11:8], rx_data};
                        chk_accum <= chk_accum ^ rx_data;
                        state     <= S_S2H;
                    end

                    // ---------------------------------------------------------
                    // Servo 2 high nibble
                    // ---------------------------------------------------------
                    S_S2H: begin
                        s2_tmp    <= {rx_data[3:0], 8'd0};
                        chk_accum <= chk_accum ^ rx_data;
                        state     <= S_S2L;
                    end

                    // ---------------------------------------------------------
                    // Servo 2 low byte
                    // NOTE: this byte CAN be 0xAA — do NOT treat as resync
                    // ---------------------------------------------------------
                    S_S2L: begin
                        s2_tmp    <= {s2_tmp[11:8], rx_data};
                        chk_accum <= chk_accum ^ rx_data;
                        state     <= S_S3H;
                    end

                    // ---------------------------------------------------------
                    // Servo 3 high nibble
                    // ---------------------------------------------------------
                    S_S3H: begin
                        s3_tmp    <= {rx_data[3:0], 8'd0};
                        chk_accum <= chk_accum ^ rx_data;
                        state     <= S_S3L;
                    end

                    // ---------------------------------------------------------
                    // Servo 3 low byte
                    // NOTE: this byte CAN be 0xAA — do NOT treat as resync
                    // ---------------------------------------------------------
                    S_S3L: begin
                        s3_tmp    <= {s3_tmp[11:8], rx_data};
                        chk_accum <= chk_accum ^ rx_data;
                        state     <= S_CKSUM;
                    end

                    // ---------------------------------------------------------
                    // Checksum: validate then latch outputs
                    // ---------------------------------------------------------
                    S_CKSUM: begin
                        state <= S_IDLE;
                        if (rx_data == chk_accum) begin
                            servo1_val   <= s1_tmp;
                            servo2_val   <= s2_tmp;
                            servo3_val   <= s3_tmp;
                            packet_valid <= 1'b1;
                        end
                        // Checksum mismatch: silently discard, return to IDLE
                    end

                    default: state <= S_IDLE;
                endcase
                // NOTE: NO mid-packet resync here — data bytes can equal 0xAA
            end
        end
    end

endmodule
