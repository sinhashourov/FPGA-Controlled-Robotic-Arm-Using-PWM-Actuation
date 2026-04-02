// =============================================================================
// Module  : top
// Project : 3-DOF Robotic Arm Controller
// Board   : Nexys A7 100T
//
// Pin Assignments:
//   uart_rxd_out ← PMOD JB pin 1 (D14) ← ESP32 GPIO17 TX
//   servo1_pwm   → PMOD JA pin 1 (C17) → Servo 1 (Base)
//   servo2_pwm   → PMOD JA pin 2 (D18) → Servo 2 (Shoulder)
//   servo3_pwm   → PMOD JA pin 3 (E18) → Servo 3 (Elbow)
//
// LED status:
//   led[0]    Heartbeat blink  — FPGA running
//   led[1]    Valid packet received (100 ms stretch)
//   led[2]    UART RX byte activity (50 ms stretch)
//   led[3]    Unused
//   led[15:4] Servo 1 ADC value[11:3] (upper 9 bits, lower 3 masked to stop flicker)
// =============================================================================

module top (
    input  wire        clk100mhz,
    input  wire        cpu_resetn,    // Active-low reset (CPU RESET button)

    input  wire        uart_rxd_out,  // PMOD JB1 = D14  ← ESP32 GPIO17

    output wire        servo1_pwm,    // PMOD JA1 = C17
    output wire        servo2_pwm,    // PMOD JA2 = D18
    output wire        servo3_pwm,    // PMOD JA3 = E18

    output wire [15:0] led,

    output wire [6:0]  seg,
    output wire        dp,
    output wire [7:0]  an
);

    // =========================================================================
    // Reset synchroniser (2-FF metastability removal)
    // =========================================================================
    reg rst_sync1, rst_sync2;
    wire rst_n;

    always @(posedge clk100mhz or negedge cpu_resetn) begin
        if (!cpu_resetn) begin
            rst_sync1 <= 1'b0;
            rst_sync2 <= 1'b0;
        end else begin
            rst_sync1 <= 1'b1;
            rst_sync2 <= rst_sync1;
        end
    end
    assign rst_n = rst_sync2;

    // =========================================================================
    // Heartbeat (toggles every ~0.5 s)
    // =========================================================================
    reg [25:0] heartbeat_cnt;
    reg        heartbeat;

    always @(posedge clk100mhz or negedge rst_n) begin
        if (!rst_n) begin
            heartbeat_cnt <= 26'd0;
            heartbeat     <= 1'b0;
        end else begin
            if (heartbeat_cnt == 26'd49_999_999) begin
                heartbeat_cnt <= 26'd0;
                heartbeat     <= ~heartbeat;
            end else begin
                heartbeat_cnt <= heartbeat_cnt + 1'b1;
            end
        end
    end

    // =========================================================================
    // Internal wires
    // =========================================================================
    wire [7:0]  rx_data;
    wire        rx_valid;
    wire [11:0] servo1_val;
    wire [11:0] servo2_val;
    wire [11:0] servo3_val;
    wire        packet_valid;

    // =========================================================================
    // Packet-received LED stretcher (~0.1 s)
    // =========================================================================
    reg [23:0] pkt_led_cnt;
    reg        pkt_led;

    always @(posedge clk100mhz or negedge rst_n) begin
        if (!rst_n) begin
            pkt_led_cnt <= 24'd0;
            pkt_led     <= 1'b0;
        end else begin
            if (packet_valid) begin
                pkt_led_cnt <= 24'd9_999_999;
                pkt_led     <= 1'b1;
            end else if (pkt_led_cnt > 0) begin
                pkt_led_cnt <= pkt_led_cnt - 1'b1;
            end else begin
                pkt_led     <= 1'b0;
            end
        end
    end

    // =========================================================================
    // UART RX activity LED stretcher (~0.05 s)
    // =========================================================================
    reg [22:0] rx_led_cnt;
    reg        rx_led;

    always @(posedge clk100mhz or negedge rst_n) begin
        if (!rst_n) begin
            rx_led_cnt <= 23'd0;
            rx_led     <= 1'b0;
        end else begin
            if (rx_valid) begin
                rx_led_cnt <= 23'd4_999_999;
                rx_led     <= 1'b1;
            end else if (rx_led_cnt > 0) begin
                rx_led_cnt <= rx_led_cnt - 1'b1;
            end else begin
                rx_led     <= 1'b0;
            end
        end
    end

    // =========================================================================
    // UART Receiver
    // =========================================================================
    uart_rx #(
        .CLK_FREQ  (100_000_000),
        .BAUD_RATE (115_200)
    ) u_uart_rx (
        .clk      (clk100mhz),
        .rst_n    (rst_n),
        .rx       (uart_rxd_out),
        .rx_data  (rx_data),
        .rx_valid (rx_valid)
    );

    // =========================================================================
    // Packet Decoder
    // =========================================================================
    packet_decoder u_decoder (
        .clk          (clk100mhz),
        .rst_n        (rst_n),
        .rx_data      (rx_data),
        .rx_valid     (rx_valid),
        .servo1_val   (servo1_val),
        .servo2_val   (servo2_val),
        .servo3_val   (servo3_val),
        .packet_valid (packet_valid)
    );

    // =========================================================================
    // Servo PWM generators
    // PULSE_MIN = 50,000 cycles = 0.5 ms  (0°)
    // PULSE_MAX = 250,000 cycles = 2.5 ms (180°)
    // All three servos use identical parameters.
    // =========================================================================
    servo_pwm #(
        .CLK_FREQ  (100_000_000),
        .PWM_FREQ  (50),
        .PULSE_MIN (50_000),
        .PULSE_MAX (250_000),
        .ADC_BITS  (12)
    ) u_servo1 (
        .clk     (clk100mhz),
        .rst_n   (rst_n),
        .adc_val (servo1_val),
        .pwm_out (servo1_pwm)
    );

    servo_pwm #(
        .CLK_FREQ  (100_000_000),
        .PWM_FREQ  (50),
        .PULSE_MIN (50_000),
        .PULSE_MAX (250_000),
        .ADC_BITS  (12)
    ) u_servo2 (
        .clk     (clk100mhz),
        .rst_n   (rst_n),
        .adc_val (servo2_val),
        .pwm_out (servo2_pwm)
    );

    servo_pwm #(
        .CLK_FREQ  (100_000_000),
        .PWM_FREQ  (50),
        .PULSE_MIN (50_000),
        .PULSE_MAX (250_000),
        .ADC_BITS  (12)
    ) u_servo3 (
        .clk     (clk100mhz),
        .rst_n   (rst_n),
        .adc_val (servo3_val),
        .pwm_out (servo3_pwm)
    );

    // =========================================================================
    // LED assignments
    // =========================================================================
    assign led[0]    = heartbeat;
    assign led[1]    = pkt_led;
    assign led[2]    = rx_led;
    assign led[3]    = 1'b0;
    // Upper 9 bits of servo1 value — lower 3 masked to stop flicker
    assign led[15:4] = {servo1_val[11:3], 3'b000};

    // =========================================================================
    // Seven-segment display — blank all digits
    // =========================================================================
    assign seg = 7'b111_1111;
    assign dp  = 1'b1;
    assign an  = 8'b1111_1111;

endmodule
