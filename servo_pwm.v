// =============================================================================
// Module  : servo_pwm
// Project : 3-DOF Robotic Arm Controller
// Board   : Nexys A7 100T (100 MHz system clock)
//
// Standard RC servo PWM timing:
//   Period    : 20 ms  (50 Hz)  = 2,000,000 cycles @ 100 MHz
//   Min pulse : 0.5 ms (  0°)  =    50,000 cycles
//   Max pulse : 2.5 ms (180°)  =   250,000 cycles
//   Range     : 2.0 ms         =   200,000 cycles over 4096 ADC steps
//
// Design approach — 3-stage registered pipeline:
//
//   [clk 0] adc_val ──reg──> adc_r
//   [clk 1] adc_r   × 200000 ──DSP48reg──> mul_r   (Vivado infers DSP48E1)
//   [clk 2] mul_r[29:12]  + PULSE_MIN ──reg──> pulse_width
//   [clk N] period_cnt compared with pulse_width → pwm_out
//
//   Each stage is register-to-register with at most one operation in between.
//   All paths meet 10 ns timing without any multicycle constraints.
//   The 3-cycle latency (60 ns) is invisible on a 20 ms servo period.
//
// Multiply bit-width:
//   adc_r      : 12 bits, max 4095
//   200,000    : 18 bits
//   product    : 4095 × 200,000 = 819,000,000 → needs 30 bits
//   mul_r      : 32-bit register (30 bits used, 2 spare)
//   mul_r[29:12]: 18-bit result after >> 12 shift (max 199,951)
//   pulse_width : 21-bit (max 250,000 = 0x3D090, needs 18 bits, 21 for safety)
// =============================================================================

module servo_pwm #(
    parameter CLK_FREQ  = 100_000_000,  // Hz
    parameter PWM_FREQ  = 50,           // Hz  — 20 ms period
    parameter PULSE_MIN = 50_000,       // cycles — 0.5 ms — 0°
    parameter PULSE_MAX = 250_000,      // cycles — 2.5 ms — 180°
    parameter ADC_BITS  = 12            // ADC resolution (0 to 4095)
)(
    input  wire                clk,
    input  wire                rst_n,
    input  wire [ADC_BITS-1:0] adc_val, // 0 → 0°,  4095 → 180°
    output reg                 pwm_out
);

    localparam integer PERIOD_CYCLES = CLK_FREQ / PWM_FREQ;    // 2,000,000
    localparam integer PULSE_RANGE   = PULSE_MAX - PULSE_MIN;   // 200,000

    // =========================================================================
    // Stage 0 — Register the ADC input
    //   Gives the multiplier a full clock cycle of setup time.
    // =========================================================================
    reg [11:0] adc_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) adc_r <= 12'd0;
        else        adc_r <= adc_val;
    end

    // =========================================================================
    // Stage 1 — Registered multiply
    //   32-bit × 32-bit → 32-bit (Vivado automatically infers DSP48E1).
    //   4095 × 200,000 = 819,000,000 < 2^30 — no overflow in 32 bits.
    // =========================================================================
    reg [31:0] mul_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) mul_r <= 32'd0;
        else        mul_r <= {20'd0, adc_r} * 32'd200_000;
    end

    // =========================================================================
    // Stage 2 — Shift >> 12, clamp, add PULSE_MIN → pulse_width
    //   mul_r[29:12] extracts the >> 12 result in 18 bits.
    //   Max: 819,000,000 >> 12 = 199,951  (<PULSE_RANGE=200,000, no clamp hit)
    //   pulse_width = PULSE_MIN + mul_r[29:12]
    //              = 50,000 + 199,951 = 249,951 ≈ 2.4995 ms ≈ 180°  ✓
    // =========================================================================
    wire [17:0] scaled = mul_r[29:12];  // 18-bit result of >> 12

    reg [20:0] pulse_width;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pulse_width <= 21'd150_000;          // 1.5 ms = 90° at reset
        end else begin
            if (scaled >= PULSE_RANGE)
                pulse_width <= PULSE_MAX;         // clamp to 2.5 ms
            else
                pulse_width <= PULSE_MIN + {3'b000, scaled}; // normal
        end
    end

    // =========================================================================
    // Free-running PWM counter
    //   period_cnt counts 0 to 1,999,999 (= 20 ms at 100 MHz).
    //   Output is HIGH when period_cnt < pulse_width, LOW otherwise.
    // =========================================================================
    reg [20:0] period_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            period_cnt <= 21'd0;
            pwm_out    <= 1'b0;
        end else begin
            if (period_cnt >= PERIOD_CYCLES - 1)
                period_cnt <= 21'd0;
            else
                period_cnt <= period_cnt + 1'b1;

            pwm_out <= (period_cnt < pulse_width) ? 1'b1 : 1'b0;
        end
    end

endmodule
