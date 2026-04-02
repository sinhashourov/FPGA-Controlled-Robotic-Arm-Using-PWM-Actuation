## =============================================================================
## Nexys A7 100T — Constraints for 3-DOF Robotic Arm Controller
##
## IMPORTANT: Project folder path must have NO spaces or special characters.
##   WRONG: C:/Users/DE/Desktop/robotic arm/project/
##   RIGHT: C:/Users/DE/Desktop/robotic_arm/project/
## =============================================================================

## ─────────────────────────────────────────────────────────────
## System Clock — 100 MHz on-board oscillator
## ─────────────────────────────────────────────────────────────
set_property -dict { PACKAGE_PIN E3  IOSTANDARD LVCMOS33 } [get_ports { clk100mhz }]
create_clock -add -name sys_clk_pin -period 10.000 -waveform {0 5} [get_ports { clk100mhz }]

## ─────────────────────────────────────────────────────────────
## Reset Button (CPU RESET, active-low)
## ─────────────────────────────────────────────────────────────
set_property -dict { PACKAGE_PIN C12 IOSTANDARD LVCMOS33 } [get_ports { cpu_resetn }]

## ─────────────────────────────────────────────────────────────
## UART RX — PMOD JB Pin 1 (D14)
##   ESP32 GPIO17 TX connects here.
## ─────────────────────────────────────────────────────────────
set_property -dict { PACKAGE_PIN D14 IOSTANDARD LVCMOS33 } [get_ports { uart_rxd_out }]

## ─────────────────────────────────────────────────────────────
## Servo PWM outputs — PMOD JA
##   JA top-row rightmost  = JA1 (C17) → Servo 1 (Base)
##   JA top-row 2nd right  = JA2 (D18) → Servo 2 (Shoulder)
##   JA top-row 3rd right  = JA3 (E18) → Servo 3 (Elbow)
## ─────────────────────────────────────────────────────────────
set_property -dict { PACKAGE_PIN C17 IOSTANDARD LVCMOS33 } [get_ports { servo1_pwm }]
set_property -dict { PACKAGE_PIN D18 IOSTANDARD LVCMOS33 } [get_ports { servo2_pwm }]
set_property -dict { PACKAGE_PIN E18 IOSTANDARD LVCMOS33 } [get_ports { servo3_pwm }]

## ─────────────────────────────────────────────────────────────
## On-board LEDs (LD0..LD15)
## ─────────────────────────────────────────────────────────────
set_property -dict { PACKAGE_PIN H17 IOSTANDARD LVCMOS33 } [get_ports { led[0]  }]
set_property -dict { PACKAGE_PIN K15 IOSTANDARD LVCMOS33 } [get_ports { led[1]  }]
set_property -dict { PACKAGE_PIN J13 IOSTANDARD LVCMOS33 } [get_ports { led[2]  }]
set_property -dict { PACKAGE_PIN N14 IOSTANDARD LVCMOS33 } [get_ports { led[3]  }]
set_property -dict { PACKAGE_PIN R18 IOSTANDARD LVCMOS33 } [get_ports { led[4]  }]
set_property -dict { PACKAGE_PIN V17 IOSTANDARD LVCMOS33 } [get_ports { led[5]  }]
set_property -dict { PACKAGE_PIN U17 IOSTANDARD LVCMOS33 } [get_ports { led[6]  }]
set_property -dict { PACKAGE_PIN U16 IOSTANDARD LVCMOS33 } [get_ports { led[7]  }]
set_property -dict { PACKAGE_PIN V16 IOSTANDARD LVCMOS33 } [get_ports { led[8]  }]
set_property -dict { PACKAGE_PIN T15 IOSTANDARD LVCMOS33 } [get_ports { led[9]  }]
set_property -dict { PACKAGE_PIN U14 IOSTANDARD LVCMOS33 } [get_ports { led[10] }]
set_property -dict { PACKAGE_PIN T16 IOSTANDARD LVCMOS33 } [get_ports { led[11] }]
set_property -dict { PACKAGE_PIN V15 IOSTANDARD LVCMOS33 } [get_ports { led[12] }]
set_property -dict { PACKAGE_PIN V14 IOSTANDARD LVCMOS33 } [get_ports { led[13] }]
set_property -dict { PACKAGE_PIN V12 IOSTANDARD LVCMOS33 } [get_ports { led[14] }]
set_property -dict { PACKAGE_PIN V11 IOSTANDARD LVCMOS33 } [get_ports { led[15] }]

## ─────────────────────────────────────────────────────────────
## Seven-Segment Display (blanked in this design)
## ─────────────────────────────────────────────────────────────
set_property -dict { PACKAGE_PIN T10 IOSTANDARD LVCMOS33 } [get_ports { seg[0] }]
set_property -dict { PACKAGE_PIN R10 IOSTANDARD LVCMOS33 } [get_ports { seg[1] }]
set_property -dict { PACKAGE_PIN K16 IOSTANDARD LVCMOS33 } [get_ports { seg[2] }]
set_property -dict { PACKAGE_PIN K13 IOSTANDARD LVCMOS33 } [get_ports { seg[3] }]
set_property -dict { PACKAGE_PIN P15 IOSTANDARD LVCMOS33 } [get_ports { seg[4] }]
set_property -dict { PACKAGE_PIN T11 IOSTANDARD LVCMOS33 } [get_ports { seg[5] }]
set_property -dict { PACKAGE_PIN L18 IOSTANDARD LVCMOS33 } [get_ports { seg[6] }]
set_property -dict { PACKAGE_PIN H15 IOSTANDARD LVCMOS33 } [get_ports { dp     }]

set_property -dict { PACKAGE_PIN J17 IOSTANDARD LVCMOS33 } [get_ports { an[0] }]
set_property -dict { PACKAGE_PIN J18 IOSTANDARD LVCMOS33 } [get_ports { an[1] }]
set_property -dict { PACKAGE_PIN T9  IOSTANDARD LVCMOS33 } [get_ports { an[2] }]
set_property -dict { PACKAGE_PIN J14 IOSTANDARD LVCMOS33 } [get_ports { an[3] }]
set_property -dict { PACKAGE_PIN P14 IOSTANDARD LVCMOS33 } [get_ports { an[4] }]
set_property -dict { PACKAGE_PIN T14 IOSTANDARD LVCMOS33 } [get_ports { an[5] }]
set_property -dict { PACKAGE_PIN K2  IOSTANDARD LVCMOS33 } [get_ports { an[6] }]
set_property -dict { PACKAGE_PIN U13 IOSTANDARD LVCMOS33 } [get_ports { an[7] }]

## =============================================================================
## Timing Exceptions
##
## All external I/O in this design is either:
##   a) Asynchronous (UART RX, reset button) — use set_false_path
##   b) Low-frequency output (50 Hz servo PWM, LEDs) — use set_false_path
##
## The servo_pwm module uses a properly registered 3-stage pipeline.
## All internal register-to-register paths are single-cycle (no multicycle
## path constraints needed). Vivado will infer DSP48E1 for the multiply.
## =============================================================================

## Async input — no clock relationship to system clock
set_false_path -from [get_ports { uart_rxd_out }]
set_false_path -from [get_ports { cpu_resetn   }]

## Slow outputs — no output timing constraint needed
set_false_path -to   [get_ports { servo1_pwm servo2_pwm servo3_pwm }]
set_false_path -to   [get_ports { led[*] }]
set_false_path -to   [get_ports { seg[*] dp an[*] }]

## ─────────────────────────────────────────────────────────────
## Bitstream / Configuration
## ─────────────────────────────────────────────────────────────
set_property CFGBVS         VCCO [current_design]
set_property CONFIG_VOLTAGE  3.3  [current_design]
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
