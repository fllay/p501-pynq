#!/usr/bin/env python3
"""
Minimal AD9361 register-level bring-up driver: BBPLL lock, RX/TX sample-rate
clock chain, RX/TX RFPLL (LO) lock, FDD ENSM enable, manual gain/attenuation.

Ported from ADI's no-OS reference driver (analogdevicesinc/no-OS,
drivers/rf-transceiver/ad9361/ad9361.c) -- NOT a faithful port of the full
ad9361_init() (~7700 lines covering software AGC, TDD timing, automatic
FIR-filter design, automatic clock-rate search). This hand-picks the fixed
subset needed to lock RX/TX LOs to an arbitrary frequency at a fixed
~3.84 MSPS sample rate and enable FDD RX+TX, matching
boards/P501/notebooks/README (plan at ~/.claude/plans/deep-discovering-puppy.md
in the authoring session). Calibration (RX/TX quadrature, DC offset) is
deliberately omitted for v1 -- add if signal quality on real hardware
needs it.

Requires: PYNQ Overlay() already downloaded (AD9361 SPI/control signals
route through PL fabric) and the EMIO reset/enable/txnrx GPIOs released,
both handled by AD9361.__init__ (same sequence as ad9361_spi_test.py).

Reference clock: 40MHz 0.5ppm VCXO (P401/P501 user manual section 2.18.3).
"""
import math
import os
import time

import spidev

REG_TX_ENABLE_FILTER_CTRL = 0x002
REG_RX_ENABLE_FILTER_CTRL = 0x003
REG_CLOCK_ENABLE = 0x009
REG_BBPLL = 0x00A
REG_ENSM_MODE = 0x013
REG_ENSM_CONFIG_1 = 0x014
REG_ENSM_CONFIG_2 = 0x015
REG_CALIBRATION_CTRL = 0x016
REG_STATE = 0x017
REG_TX1_ATTEN_0 = 0x073
REG_TX1_ATTEN_1 = 0x074
REG_CLOCK_CTRL = 0x045
REG_SDM_CTRL_1 = 0x03F
REG_FRACT_BB_FREQ_WORD_1 = 0x041
REG_FRACT_BB_FREQ_WORD_2 = 0x042
REG_FRACT_BB_FREQ_WORD_3 = 0x043
REG_INTEGER_BB_FREQ_WORD = 0x044
REG_CP_CURRENT = 0x046
REG_LOOP_FILTER_1 = 0x048
REG_LOOP_FILTER_2 = 0x049
REG_LOOP_FILTER_3 = 0x04A
REG_VCO_CTRL = 0x04B
REG_VCO_PROGRAM_1 = 0x04C
REG_VCO_PROGRAM_2 = 0x04D
REG_SDM_CTRL = 0x04E
REG_CH_1_OVERFLOW = 0x05E
REG_RX1_MANUAL_LMT_FULL_GAIN = 0x109
REG_REF_DIVIDE_CONFIG_1 = 0x2AB
REG_REF_DIVIDE_CONFIG_2 = 0x2AC
REG_RX_FORCE_VCO_TUNE_1 = 0x238
REG_RX_ALC_VARACTOR = 0x239
REG_RX_VCO_OUTPUT = 0x23A
REG_RX_CP_CURRENT = 0x23B
REG_RX_LOOP_FILTER_1 = 0x23E
REG_RX_LOOP_FILTER_2 = 0x23F
REG_RX_LOOP_FILTER_3 = 0x240
REG_RX_VCO_BIAS_1 = 0x242
REG_RX_VCO_CAL_REF = 0x245
REG_RX_FRACT_BYTE_2 = 0x235
REG_RX_CP_OVERRANGE_VCO_LOCK = 0x247
REG_RX_VCO_VARACTOR_CTRL_0 = 0x250
REG_RX_VCO_VARACTOR_CTRL_1 = 0x251
REG_TX_VCO_OUTPUT = 0x27A
REG_TX_FRACT_BYTE_2 = 0x275
REG_TX_CP_OVERRANGE_VCO_LOCK = 0x287
REG_RFPLL_DIVIDERS = 0x005
REG_TX_ATTEN_OFFSET = 0x077
REG_TX2_DIG_ATTEN = 0x07C
REG_REFERENCE_CLOCK_CYCLES = 0x03A

# -- programmable FIR filter (128 taps max, RX decimation / TX
# interpolation) -- TX block at 0x060-0x065, RX block at an identical
# layout offset by RX_TX_FILTER_OFFSET (0x90), per ad9361_load_fir_filter_coef.
REG_TX_FILTER_COEF_ADDR = 0x060
REG_TX_FILTER_COEF_WRITE_DATA_1 = 0x061
REG_TX_FILTER_COEF_WRITE_DATA_2 = 0x062
REG_TX_FILTER_COEF_READ_DATA_1 = 0x063
REG_TX_FILTER_COEF_READ_DATA_2 = 0x064
REG_TX_FILTER_CONF = 0x065
REG_RX_FILTER_COEF_ADDR = 0x0F0
REG_RX_FILTER_GAIN = 0x0F6
RX_TX_FILTER_OFFSET = REG_RX_FILTER_COEF_ADDR - REG_TX_FILTER_COEF_ADDR

TX_FIR_GAIN_6DB = 1 << 0
FIR_START_CLK = 1 << 1
FIR_WRITE = 1 << 2
FIR_SELECT = lambda x: (x & 0x3) << 3
FIR_NUM_TAPS = lambda x: (x & 0x7) << 5

FIR_IS_RX = 0x80
FIR_TX1 = 0x01
FIR_RX1 = 0x81

REG_RX_MIX_GM_CONFIG = 0x1C0
REG_RX_MIX_LO_CM = 0x1D5
REG_RX_TIA_CONFIG = 0x1DB
REG_TIA1_C_LSB = 0x1DC
REG_TIA1_C_MSB = 0x1DD
REG_TIA2_C_LSB = 0x1DE
REG_TIA2_C_MSB = 0x1DF
REG_RX1_TUNE_CTRL = 0x1E2
REG_RX2_TUNE_CTRL = 0x1E3
REG_RX_BBF_R2346 = 0x1E6
REG_RX_BBF_C3_MSB = 0x1EB
REG_RX_BBF_C3_LSB = 0x1EC
REG_RX_BBF_TUNE_DIVIDE = 0x1F8
REG_RX_BBF_TUNE_CONFIG = 0x1F9
REG_RX_BBBW_MHZ = 0x1FB
REG_RX_BBBW_KHZ = 0x1FC
REG_CALIBRATION_CTRL = 0x016
REG_INPUT_SELECT = 0x004
TX_OUTPUT = 1 << 6
REG_AGC_CONFIG_1 = 0x0FA
DEC_PWR_FOR_LOW_PWR = 1 << 7
DEC_PWR_FOR_LOCK_LEVEL = 1 << 6
DEC_PWR_FOR_GAIN_LOCK_EXIT = 1 << 5
REG_RX1_MANUAL_DIGITALFORCED_GAIN = 0x10B
REG_RX2_MANUAL_DIGITALFORCED_GAIN = 0x10E
RX_BB_TUNE_CAL = 1 << 7
RX_TUNE_RESAMPLE = 1 << 1
RX_PD_TUNE = 1 << 0

TX_RX_VCO_OFFSET = REG_TX_VCO_OUTPUT - REG_RX_VCO_OUTPUT  # applies to the
# whole REG_RX_* block above -- TX uses the same offset relative to each one

FDD_MODE = 1 << 0
FORCE_RX_ON = 1 << 6
FORCE_TX_ON = 1 << 5
BBPLL_LOCK = 1 << 7
VCO_LOCK = 1 << 1
INIT_BB_FO_CAL = 1 << 2
BBPLL_RESET_BAR = 1 << 0
FREQ_CAL_ENABLE = 1 << 7
CLKOUT_ENABLE = 1 << 4
DIGITAL_POWER_UP = 1 << 2
BBPLL_ENABLE = 1 << 0
XO_BYPASS = 1 << 4
RX_REF_RESET_BAR = 1 << 1
TX_REF_RESET_BAR = 1 << 4
POWER_DOWN_RX_SYNTH = 1 << 6
POWER_DOWN_TX_SYNTH = 1 << 5
DUAL_SYNTH_MODE = 1 << 2
RX_CHANNEL_ENABLE = lambda x: (x & 0x3) << 6
TX_CHANNEL_ENABLE = lambda x: (x & 0x3) << 6
RX_1 = 1
TX_1 = 1
BYPASS_LD_SYNTH = 1 << 0
TO_ALERT = 1 << 0
FORCE_ALERT_STATE = 1 << 2
REG_RX_PFD_CONFIG = 0x230
REG_TX_PFD_CONFIG = 0x270

BBPLL_MODULUS = 2088960
RFPLL_MODULUS = 8388593
MIN_VCO_FREQ_HZ = 6_000_000_000
MIN_BBPLL_FREQ = 714_928_500
MAX_BBPLL_FREQ = 1_430_143_000

REG_PRODUCT_ID = 0x037
PRODUCT_ID_MASK = 0xF8
PRODUCT_ID_9361 = 0x08

CONTROL_GPIOS = [640, 641, 642]  # EMIO reset/enable/txnrx

# SynthLUT_TDD[LUT_FTDD_40] from ad9361.c -- VCO band characterization data
# for a 40MHz reference clock (ours), used for RFPLL VCO band init. This is
# the TDD table (not FDD) because ad9361_rfpll_vco_init() falls back to it
# whenever RX LO == TX LO even in FDD mode (reduces VCO pulling) -- our use
# case (single frequency, both LOs equal) always hits this path.
# Columns: VCO_MHz, Output_Level, Varactor, Bias_Ref, Bias_Tcf, Cal_Offset,
#          Varactor_Reference, Charge_Pump_Current, LF_C2, LF_C1, LF_R1,
#          LF_C3, LF_R3
SYNTH_LUT_TDD_40MHZ = [
    (12605, 13, 1, 4, 2, 15, 12, 27, 12, 15, 12, 4, 13),
    (12245, 13, 1, 4, 2, 15, 12, 27, 12, 15, 12, 4, 13),
    (11906, 13, 1, 4, 2, 15, 12, 26, 11, 15, 12, 4, 13),
    (11588, 13, 1, 4, 2, 15, 12, 28, 12, 15, 12, 4, 13),
    (11288, 13, 1, 4, 2, 15, 12, 30, 12, 15, 12, 4, 13),
    (11007, 13, 1, 4, 2, 15, 12, 32, 12, 15, 12, 4, 13),
    (10742, 13, 1, 4, 2, 15, 12, 33, 12, 15, 12, 4, 13),
    (10492, 13, 1, 6, 2, 15, 12, 35, 12, 15, 12, 4, 13),
    (10258, 13, 1, 6, 2, 15, 12, 37, 12, 15, 12, 4, 13),
    (10036, 13, 1, 6, 2, 15, 12, 38, 12, 15, 12, 4, 13),
    (9827, 13, 1, 6, 2, 14, 12, 40, 12, 15, 12, 4, 13),
    (9631, 13, 1, 6, 2, 13, 12, 42, 12, 15, 12, 4, 13),
    (9445, 13, 1, 6, 2, 12, 12, 44, 12, 15, 12, 4, 13),
    (9269, 13, 1, 6, 2, 12, 12, 45, 12, 15, 12, 4, 13),
    (9103, 13, 1, 6, 2, 12, 12, 47, 12, 15, 12, 4, 13),
    (8946, 13, 1, 6, 2, 12, 12, 49, 12, 15, 12, 4, 13),
    (8797, 12, 1, 7, 2, 12, 12, 48, 12, 15, 12, 4, 13),
    (8655, 12, 1, 7, 2, 12, 12, 50, 12, 15, 12, 4, 13),
    (8520, 12, 1, 7, 2, 12, 12, 51, 12, 15, 12, 4, 13),
    (8392, 12, 1, 7, 2, 12, 12, 53, 12, 15, 12, 4, 13),
    (8269, 12, 1, 7, 2, 12, 12, 55, 12, 15, 12, 4, 13),
    (8153, 12, 1, 7, 2, 12, 12, 56, 12, 15, 12, 4, 13),
    (8041, 12, 1, 7, 2, 13, 12, 58, 12, 15, 12, 4, 13),
    (7934, 11, 1, 7, 2, 12, 12, 57, 12, 15, 12, 4, 13),
    (7831, 11, 1, 7, 2, 12, 12, 58, 12, 15, 12, 4, 13),
    (7733, 10, 1, 7, 3, 13, 12, 56, 12, 15, 12, 4, 13),
    (7638, 10, 1, 7, 2, 12, 12, 58, 12, 15, 12, 4, 13),
    (7547, 10, 1, 7, 2, 12, 12, 59, 12, 15, 12, 4, 13),
    (7459, 10, 1, 7, 2, 12, 12, 61, 12, 15, 12, 4, 13),
    (7374, 10, 2, 7, 3, 14, 13, 49, 12, 15, 12, 4, 13),
    (7291, 10, 2, 7, 3, 14, 13, 50, 12, 15, 12, 4, 13),
    (7212, 10, 2, 7, 3, 14, 13, 51, 12, 15, 12, 4, 13),
    (7135, 10, 2, 7, 3, 14, 13, 52, 12, 15, 12, 4, 13),
    (7061, 10, 2, 7, 3, 14, 13, 53, 12, 15, 12, 4, 13),
    (6988, 10, 1, 7, 3, 12, 14, 63, 11, 14, 12, 3, 13),
    (6918, 9, 2, 7, 3, 14, 13, 52, 12, 15, 12, 4, 13),
    (6850, 9, 2, 7, 3, 14, 13, 53, 12, 15, 12, 4, 13),
    (6784, 9, 2, 7, 2, 13, 13, 54, 12, 15, 12, 4, 13),
    (6720, 9, 2, 7, 2, 13, 13, 56, 12, 15, 12, 4, 13),
    (6658, 8, 2, 7, 3, 14, 13, 53, 12, 15, 12, 4, 13),
    (6597, 8, 2, 7, 2, 13, 13, 54, 12, 15, 12, 4, 13),
    (6539, 8, 2, 7, 2, 13, 13, 55, 12, 15, 12, 4, 13),
    (6482, 8, 2, 7, 2, 13, 13, 56, 12, 15, 12, 4, 13),
    (6427, 7, 2, 7, 3, 14, 13, 54, 12, 15, 12, 4, 13),
    (6373, 7, 2, 7, 3, 15, 13, 54, 12, 15, 12, 4, 13),
    (6321, 7, 2, 7, 3, 15, 13, 55, 12, 15, 12, 4, 13),
    (6270, 7, 2, 7, 3, 15, 13, 56, 12, 15, 12, 4, 13),
    (6222, 7, 2, 7, 3, 15, 13, 57, 12, 15, 12, 4, 13),
    (6174, 6, 2, 7, 3, 15, 13, 54, 12, 15, 12, 4, 13),
    (6128, 6, 2, 7, 3, 15, 13, 55, 12, 15, 12, 4, 13),
    (6083, 6, 2, 7, 3, 15, 13, 56, 12, 15, 12, 4, 13),
    (6040, 6, 2, 7, 3, 15, 13, 57, 12, 15, 12, 4, 13),
    (5997, 6, 2, 7, 3, 15, 13, 58, 12, 15, 12, 4, 13),
]


REG_MAX_LMT_FULL_GAIN = 0x0FD
REG_AGC_CONFIG_2 = 0x0FB
AGC_USE_FULL_GAIN_TABLE = 1 << 3
REG_GAIN_TABLE_ADDRESS = 0x130
REG_GAIN_TABLE_WRITE_DATA1 = 0x131
REG_GAIN_TABLE_WRITE_DATA2 = 0x132
REG_GAIN_TABLE_WRITE_DATA3 = 0x133
REG_GAIN_TABLE_READ_DATA1 = 0x134
REG_GAIN_TABLE_CONFIG = 0x137
WRITE_GAIN_TABLE = 1 << 2
START_GAIN_TABLE_CLOCK = 1 << 1
GT_RX1 = 1
GT_RX2 = 2

# RX gain table for 200-1300 MHz (full table), from the vendor kernel driver
# (ad9361.c full_gain_table[TBL_200_1300_MHZ]): (LNA/mixer, TIA/LPF, DC-cal/dig-gain)
RX_GAIN_TABLE_200_1300MHZ = [
    (0x00, 0x00, 0x20), (0x00, 0x00, 0x00), (0x00, 0x00, 0x00), (0x00, 0x01, 0x00),
    (0x00, 0x02, 0x00), (0x00, 0x03, 0x00), (0x00, 0x04, 0x00), (0x00, 0x05, 0x00),
    (0x01, 0x03, 0x20), (0x01, 0x04, 0x00), (0x01, 0x05, 0x00), (0x01, 0x06, 0x00),
    (0x01, 0x07, 0x00), (0x01, 0x08, 0x00), (0x01, 0x09, 0x00), (0x01, 0x0A, 0x00),
    (0x01, 0x0B, 0x00), (0x01, 0x0C, 0x00), (0x01, 0x0D, 0x00), (0x01, 0x0E, 0x00),
    (0x02, 0x09, 0x20), (0x02, 0x0A, 0x00), (0x02, 0x0B, 0x00), (0x02, 0x0C, 0x00),
    (0x02, 0x0D, 0x00), (0x02, 0x0E, 0x00), (0x02, 0x0F, 0x00), (0x02, 0x10, 0x00),
    (0x02, 0x2B, 0x20), (0x02, 0x2C, 0x00), (0x04, 0x28, 0x20), (0x04, 0x29, 0x00),
    (0x04, 0x2A, 0x00), (0x04, 0x2B, 0x00), (0x24, 0x20, 0x20), (0x24, 0x21, 0x00),
    (0x44, 0x20, 0x20), (0x44, 0x21, 0x00), (0x44, 0x22, 0x00), (0x44, 0x23, 0x00),
    (0x44, 0x24, 0x00), (0x44, 0x25, 0x00), (0x44, 0x26, 0x00), (0x44, 0x27, 0x00),
    (0x44, 0x28, 0x00), (0x44, 0x29, 0x00), (0x44, 0x2A, 0x00), (0x44, 0x2B, 0x00),
    (0x44, 0x2C, 0x00), (0x44, 0x2D, 0x00), (0x44, 0x2E, 0x00), (0x44, 0x2F, 0x00),
    (0x44, 0x30, 0x00), (0x44, 0x31, 0x00), (0x44, 0x32, 0x00), (0x64, 0x2E, 0x20),
    (0x64, 0x2F, 0x00), (0x64, 0x30, 0x00), (0x64, 0x31, 0x00), (0x64, 0x32, 0x00),
    (0x64, 0x33, 0x00), (0x64, 0x34, 0x00), (0x64, 0x35, 0x00), (0x64, 0x36, 0x00),
    (0x64, 0x37, 0x00), (0x64, 0x38, 0x00), (0x65, 0x38, 0x20), (0x66, 0x38, 0x20),
    (0x67, 0x38, 0x20), (0x68, 0x38, 0x20), (0x69, 0x38, 0x20), (0x6A, 0x38, 0x20),
    (0x6B, 0x38, 0x20), (0x6C, 0x38, 0x20), (0x6D, 0x38, 0x20), (0x6E, 0x38, 0x20),
    (0x6F, 0x38, 0x20),
]


REG_TX_BBF_TUNE_DIVIDER = 0x0D6
REG_TX_BBF_TUNE_MODE = 0x0D7
REG_TX_TUNE_CTRL = 0x0CA
TX_BB_TUNE_CAL = 1 << 6
TUNER_RESAMPLE = 1 << 1
TX_TUNE_CTRL_1 = 1 << 5          # TUNE_CTRL(1)
TX_PD_TUNE = 1 << 2
REG_CONFIG0 = 0x0D0
REG_RESISTOR = 0x0D1
REG_CAPACITOR = 0x0D2
REG_BB_DC_OFFSET_COUNT = 0x193
REG_BB_DC_OFFSET_SHIFT = 0x190
REG_BB_DC_OFFSET_ATTEN = 0x194
BBDC_CAL = 1 << 0
RFDC_CAL = 1 << 1
TX_QUAD_CAL = 1 << 4
REG_WAIT_COUNT = 0x185
REG_RF_DC_OFFSET_COUNT = 0x186
REG_RF_DC_OFFSET_CONFIG_1 = 0x187
REG_RF_DC_OFFSET_ATTEN = 0x188
REG_INVERT_BITS = 0x189
REG_DC_OFFSET_CONFIG2 = 0x18B
USE_WAIT_COUNTER_FOR_RF_DC_INIT_CAL = 1 << 7
INVERT_RX1_RF_DC_CGOUT_WORD = 1 << 4
INVERT_RX2_RF_DC_CGOUT_WORD = 1 << 5
REG_QUAD_CAL_NCO_FREQ_PHASE_OFFSET = 0x0A0
REG_QUAD_CAL_CTRL = 0x0A1
REG_KEXP_1 = 0x0A2
REG_KEXP_2 = 0x0A3
REG_QUAD_SETTLE_COUNT = 0x0A4
REG_MAG_FTEST_THRESH = 0x0A5
REG_MAG_FTEST_THRESH_2 = 0x0A6
REG_QUAD_CAL_STATUS_TX1 = 0x0A7
REG_QUAD_CAL_COUNT = 0x0A9
REG_TX_QUAD_FULL_LMT_GAIN = 0x0AA
REG_TX_QUAD_LPF_GAIN = 0x0AE
TX1_LO_CONV = 1 << 1
TX1_SSB_CONV = 1 << 0

REG_RX_CP_LEVEL_DETECT = 0x24B
REG_RX_DSM_SETUP_1 = 0x24D
REG_RX_LO_GEN_POWER_MODE = 0x261
REG_RX_VCO_LDO = 0x248
REG_RX_VCO_PD_OVERRIDES = 0x246
REG_RX_CP_CURRENT_ = 0x23B
REG_RX_CP_CONFIG = 0x23D
REG_RX_VCO_CAL = 0x249
REG_RX_CAL_STATUS = 0x244
CP_OFFSET_OFF = 1 << 4
CP_CAL_ENABLE = 1 << 2
CP_CAL_VALID = 1 << 7
VCO_CAL_EN = 1 << 7
REG_GM_SUB_TABLE_ADDRESS = 0x138
REG_GM_SUB_TABLE_GAIN_WRITE = 0x139
REG_GM_SUB_TABLE_BIAS_WRITE = 0x13A
REG_GM_SUB_TABLE_CTRL_WRITE = 0x13B
REG_GM_SUB_TABLE_GAIN_READ = 0x13C
REG_GM_SUB_TABLE_CONFIG = 0x13F
WRITE_GM_SUB_TABLE = 1 << 2
START_GM_SUB_TABLE_CLOCK = 1 << 1
GM_ST_CTRL = [0x0, 0xD, 0x15, 0x1B, 0x21, 0x25, 0x29, 0x2C,
              0x2F, 0x31, 0x33, 0x34, 0x35, 0x3A, 0x3D, 0x3E]
GM_ST_GAIN = [0x78, 0x74, 0x70, 0x6C, 0x68, 0x64, 0x60, 0x5C,
              0x58, 0x54, 0x50, 0x4C, 0x48, 0x30, 0x18, 0x0]

def div_round_closest(a, b):
    return (a + b // 2) // b


class AD9361:
    def __init__(self, bus=1, cs=0, ref_clk_hz=40_000_000):
        self.ref_clk_hz = ref_clk_hz
        self._release_control_gpios()
        self.spi = spidev.SpiDev()
        self.spi.open(bus, cs)
        self.spi.mode = 1  # CPOL=0, CPHA=1 -- confirmed on real hardware
        self.spi.max_speed_hz = 1_000_000
        self.spi.bits_per_word = 8

    def close(self):
        self.spi.close()

    @staticmethod
    def _release_control_gpios():
        """CONTROL_GPIOS[0] (640) is AD9361's RESETB (active-low hardware
        reset). Every previous session only ever drove it high -- fine
        when the chip was already coming from a true power-on state, but
        never actually forces a clean reset if the GPIO was already high
        from a prior boot/driver (e.g. after swapping to a different,
        more complete driver like libiio and back -- found 2026-10-17
        when a stale clock-generation state survived an SD-card swap +
        full reboot, producing a frozen RX capture with CLK_FREQ reading
        ~5kHz instead of tens of MHz despite every SPI-side lock/status
        bit reporting success). Now explicitly pulses RESETB low then
        high for a real, deterministic reset every time, instead of
        relying on whatever state the line happened to already be in."""
        for g in CONTROL_GPIOS:
            path = f"/sys/class/gpio/gpio{g}"
            if not os.path.exists(path):
                with open("/sys/class/gpio/export", "w") as f:
                    f.write(str(g))
        time.sleep(0.2)
        for g in CONTROL_GPIOS:
            path = f"/sys/class/gpio/gpio{g}"
            with open(f"{path}/direction", "w") as f:
                f.write("out")
        reset_gpio = CONTROL_GPIOS[0]
        with open(f"/sys/class/gpio/gpio{reset_gpio}/value", "w") as f:
            f.write("0")
        time.sleep(0.01)
        for g in CONTROL_GPIOS:
            path = f"/sys/class/gpio/gpio{g}"
            with open(f"{path}/value", "w") as f:
                f.write("1")
        time.sleep(0.2)

    # -- raw SPI -----------------------------------------------------
    def reg_read(self, addr):
        cmd = (0 << 15) | (0 << 12) | (addr & 0x3FF)
        tx = [cmd >> 8, cmd & 0xFF, 0x00]
        return self.spi.xfer2(tx)[2]

    def reg_write(self, addr, val):
        cmd = (1 << 15) | (0 << 12) | (addr & 0x3FF)
        tx = [cmd >> 8, cmd & 0xFF, val & 0xFF]
        self.spi.xfer2(tx)

    def reg_write_multi(self, start_addr, values):
        """values[0] goes to start_addr, values[1] to start_addr-1, etc.
        (matches ad9361_spi_writem's descending-address convention for
        multi-byte BBPLL/RFPLL words)."""
        n = len(values)
        cmd = (1 << 15) | (((n - 1) & 0x7) << 12) | (start_addr & 0x3FF)
        tx = [cmd >> 8, cmd & 0xFF] + list(values)
        self.spi.xfer2(tx)

    def reg_writef(self, addr, mask, value):
        cur = self.reg_read(addr)
        shift = (mask & -mask).bit_length() - 1
        new = (cur & ~mask) | ((value << shift) & mask)
        self.reg_write(addr, new)

    def reg_readf(self, addr, mask):
        shift = (mask & -mask).bit_length() - 1
        return (self.reg_read(addr) & mask) >> shift

    def check_cal_done(self, addr, mask, done_state, timeout_iters=20000):
        for _ in range(timeout_iters):
            if self.reg_readf(addr, mask) == done_state:
                return True
            time.sleep(120e-6)
        return False

    # -- identity ------------------------------------------------------
    def product_id(self):
        return self.reg_read(REG_PRODUCT_ID)

    def product_id_ok(self):
        return (self.product_id() & PRODUCT_ID_MASK) == PRODUCT_ID_9361

    # -- clock power-up (must run before set_bbpll_rate) ------------------
    def power_up_clocks(self):
        """Take the reference dividers out of reset and power up
        DIGITAL/BBPLL. Missed on the first pass -- without this,
        REG_CLOCK_ENABLE's BBPLL_ENABLE bit is never set, so BBPLL has
        no power and set_bbpll_rate's lock poll times out. XO_BYPASS is
        set because our reference is an external 40MHz VCXO (P401/P501
        manual 2.18.3), not the chip's internal crystal oscillator
        amplifier -- the external clock drives REFCLK directly."""
        self.reg_writef(REG_REF_DIVIDE_CONFIG_1, RX_REF_RESET_BAR, 1)
        self.reg_writef(REG_REF_DIVIDE_CONFIG_2, TX_REF_RESET_BAR, 1)
        self.reg_writef(REG_REF_DIVIDE_CONFIG_2, 0x3 << 0, 3)  # TX FB delay
        self.reg_writef(REG_REF_DIVIDE_CONFIG_1, 0x3 << 5, 3)  # RX FB delay
        self.reg_write(REG_CLOCK_ENABLE,
                        DIGITAL_POWER_UP | (1 << 1) | BBPLL_ENABLE | XO_BYPASS)
        # RX/TX synths are powered down by default (REG_ENSM_CONFIG_2) --
        # clear both power-down bits or RFPLL VCO_LOCK never asserts no
        # matter how correctly the divider/VCO-band registers are set.
        # DUAL_SYNTH_MODE must also be set for FDD (RX+TX synths active
        # simultaneously) -- without it only one synth can be active at a
        # time, which is why RX alone would lock while TX silently never
        # got a turn (found empirically: byte-identical RX/TX RFPLL
        # register writes, RX locked, TX read back all-zero status).
        self.reg_write(REG_ENSM_CONFIG_2, DUAL_SYNTH_MODE)
        # ENSM must pass through ALERT once after power-up (this is where
        # the reference driver's SLEEP->ALERT transition lives) before any
        # PLL calibration can run.
        self.reg_write(REG_ENSM_CONFIG_1, TO_ALERT | FORCE_ALERT_STATE)
        # VCO lock-detect/calibration circuit is bypassed (disabled) by
        # default -- clear BYPASS_LD_SYNTH on both RX and TX PFD config or
        # VCO_LOCK can never assert regardless of correct RFPLL divider
        # programming.
        self.reg_writef(REG_RX_PFD_CONFIG, BYPASS_LD_SYNTH, 0)
        self.reg_writef(REG_TX_PFD_CONFIG, BYPASS_LD_SYNTH, 0)
        # Calibration/ENSM delay counters are in reference-clock cycles per
        # microsecond; the vendor driver programs this in ad9361_setup()
        # (ad9361_set_ref_clk_cycles) before any calibration.
        self.reg_write(REG_REFERENCE_CLOCK_CYCLES,
                       ((self.ref_clk_hz // 1_000_000) - 1) & 0x7F)

    # -- BBPLL -----------------------------------------------------------
    def set_bbpll_rate(self, rate_hz):
        """Port of ad9361_bbpll_set_rate. rate_hz must be in
        [MIN_BBPLL_FREQ, MAX_BBPLL_FREQ]."""
        parent = self.ref_clk_hz
        tmp = (rate_hz >> 7) * 150
        tmp //= (parent >> 7) * 32
        icp_val = div_round_closest(tmp, 25) - 1
        icp_val = max(1, min(63, icp_val))

        self.reg_write(REG_CP_CURRENT, icp_val)
        self.reg_write(REG_LOOP_FILTER_1, 0x35)
        self.reg_write(REG_LOOP_FILTER_2, 0x5B)
        self.reg_write(REG_LOOP_FILTER_3, 0xE8)

        self.reg_write(REG_VCO_CTRL, FREQ_CAL_ENABLE | ((3 & 0x3) << 5))
        self.reg_write(REG_SDM_CTRL, 0x10)

        integer = rate_hz // parent
        rem = rate_hz % parent
        fract = div_round_closest(rem * BBPLL_MODULUS, parent)

        self.reg_write(REG_INTEGER_BB_FREQ_WORD, integer)
        self.reg_write(REG_FRACT_BB_FREQ_WORD_3, fract & 0xFF)
        self.reg_write(REG_FRACT_BB_FREQ_WORD_2, (fract >> 8) & 0xFF)
        self.reg_write(REG_FRACT_BB_FREQ_WORD_1, (fract >> 16) & 0xFF)

        self.reg_write(REG_SDM_CTRL_1, INIT_BB_FO_CAL | BBPLL_RESET_BAR)
        self.reg_write(REG_SDM_CTRL_1, BBPLL_RESET_BAR)

        self.reg_write(REG_VCO_PROGRAM_1, 0x86)
        self.reg_write(REG_VCO_PROGRAM_2, 0x01)
        self.reg_write(REG_VCO_PROGRAM_2, 0x05)

        return self.check_cal_done(REG_CH_1_OVERFLOW, BBPLL_LOCK, 1)

    # -- sample-rate clock chain -----------------------------------------
    def set_clock_chain(self, adc_div=16, r2_div=2, r1_div=2, clkrf_div=2,
                         rx_fir_div=2, dac_div=1, t2_div=2, t1_div=2,
                         clktf_div=2, tx_fir_div=2):
        """Direct register writes for each divider stage (no auto-search --
        caller picks known-good values). Defaults give BBPLL/16/2/2/2/2 =
        BBPLL/128 sample rate; with set_bbpll_rate(983_040_000) that's
        exactly 3.84 MSPS (ilog2 of each div -> field value, matching
        ad9361_set_clk_scaler's ADC_CLK/DAC_CLK/R2_CLK/.../*_SAMPL_CLK
        cases)."""
        import math

        self.reg_write(REG_CLOCK_CTRL, 0)  # REF_FREQ_SCALER=0, no ref scaling
        self.reg_writef(REG_REF_DIVIDE_CONFIG_1, 1 << 0, 0)  # RX_REF_DIVIDER MSB
        self.reg_writef(REG_REF_DIVIDE_CONFIG_2, (1 << 7) | (0x3 << 2), 0)

        self.reg_writef(REG_BBPLL, 0x7, int(math.log2(adc_div)))
        # DAC_CLK = ADC_CLK / dac_div, dac_div in {1, 2} (REG_BBPLL bit 3 =
        # dac_div-1, per ad9361_set_clk_scaler). It must equal
        # TX_SAMPL * tx_fir_div * clktf_div * t1_div * t2_div or the TX
        # interpolation chain runs at the wrong rate (the old default of 2
        # halved the DAC clock for the default 16x chain -> sideband comb,
        # TX quad cal never converged).
        assert dac_div in (1, 2), "dac_div must be 1 or 2"
        self.reg_writef(REG_BBPLL, 1 << 3, dac_div - 1)

        # div=1 means "bypass the FIR stage entirely" (field value 0), NOT
        # "decimate by 2^0" (which would be field value 1, since the field
        # encoding is bypass=0 / enable-with-log2(div) otherwise -- matches
        # ad9361_set_clk_scaler's RX_SAMPL_CLK/TX_SAMPL_CLK cases in the
        # reference driver, which special-case div==1 via bypass_rx_fir).
        rx_fir_field = 0 if rx_fir_div == 1 else int(math.log2(rx_fir_div)) + 1
        tx_fir_field = 0 if tx_fir_div == 1 else int(math.log2(tx_fir_div)) + 1

        self.reg_writef(REG_RX_ENABLE_FILTER_CTRL, 0x3 << 4, r2_div - 1)
        self.reg_writef(REG_RX_ENABLE_FILTER_CTRL, 1 << 3, r1_div - 1)
        self.reg_writef(REG_RX_ENABLE_FILTER_CTRL, 1 << 2, clkrf_div - 1)
        self.reg_writef(REG_RX_ENABLE_FILTER_CTRL, 0x3 << 0, rx_fir_field)

        self.reg_writef(REG_TX_ENABLE_FILTER_CTRL, 0x3 << 4, t2_div - 1)
        self.reg_writef(REG_TX_ENABLE_FILTER_CTRL, 1 << 3, t1_div - 1)
        self.reg_writef(REG_TX_ENABLE_FILTER_CTRL, 1 << 2, clktf_div - 1)
        self.reg_writef(REG_TX_ENABLE_FILTER_CTRL, 0x3 << 0, tx_fir_field)

    # -- RFPLL -------------------------------------------------------------
    def _rfpll_vco_init(self, tx, vco_freq_hz):
        vco_mhz = vco_freq_hz // 1_000_000
        row = SYNTH_LUT_TDD_40MHZ[-1]
        for candidate in SYNTH_LUT_TDD_40MHZ:
            row = candidate
            if candidate[0] <= vco_mhz:
                break
        (_, out_level, varactor, bias_ref, bias_tcf, cal_offset,
         varactor_ref, cp_current, lf_c2, lf_c1, lf_r1, lf_c3, lf_r3) = row

        off = TX_RX_VCO_OFFSET if tx else 0
        self.reg_write(REG_RX_VCO_OUTPUT + off,
                        ((out_level & 0xF) << 0) | (1 << 6))  # PORB_VCO_LOGIC
        self.reg_writef(REG_RX_ALC_VARACTOR + off, 0xF, varactor)
        self.reg_write(REG_RX_VCO_BIAS_1 + off,
                        ((bias_ref & 0x7) << 0) | ((bias_tcf & 0x3) << 3))
        self.reg_write(REG_RX_FORCE_VCO_TUNE_1 + off, (cal_offset & 0xF) << 3)
        self.reg_write(REG_RX_VCO_VARACTOR_CTRL_1 + off,
                        (varactor_ref & 0xF) << 0)
        self.reg_write(REG_RX_VCO_CAL_REF + off, 0)
        self.reg_write(REG_RX_VCO_VARACTOR_CTRL_0 + off, (7 & 0x7) << 4)
        self.reg_writef(REG_RX_CP_CURRENT + off, 0x3F, cp_current)
        self.reg_write(REG_RX_LOOP_FILTER_1 + off,
                        ((lf_c2 & 0xF) << 4) | ((lf_c1 & 0xF) << 0))
        self.reg_write(REG_RX_LOOP_FILTER_2 + off,
                        ((lf_r1 & 0xF) << 4) | ((lf_c3 & 0xF) << 0))
        self.reg_write(REG_RX_LOOP_FILTER_3 + off, (lf_r3 & 0xF) << 0)

    def set_lo_freq(self, tx, freq_hz, ref_clk_hz=None):
        """Port of ad9361_calc_rfpll_int_divder + the register-write/poll
        portion of ad9361_rfpll_int_set_rate. tx=False for RX LO,
        tx=True for TX LO."""
        ref_clk_hz = ref_clk_hz or self.ref_clk_hz
        freq = freq_hz
        vco_div = -1
        while freq <= MIN_VCO_FREQ_HZ:
            freq <<= 1
            vco_div += 1
        vco_freq = freq

        self._rfpll_vco_init(tx, vco_freq)

        integer = freq // ref_clk_hz
        rem = freq % ref_clk_hz
        fract = div_round_closest(rem * RFPLL_MODULUS, ref_clk_hz)

        if tx:
            reg = REG_TX_FRACT_BYTE_2
            lock_reg = REG_TX_CP_OVERRANGE_VCO_LOCK
            div_mask = 0xF << 4
        else:
            reg = REG_RX_FRACT_BYTE_2
            lock_reg = REG_RX_CP_OVERRANGE_VCO_LOCK
            div_mask = 0xF << 0

        buf0 = (fract >> 16) & 0x7F
        buf1 = (fract >> 8) & 0xFF
        buf2 = fract & 0xFF
        existing_top = self.reg_read(reg - 3) & 0xF8  # preserve non-integer bits
        buf3 = ((integer >> 8) & 0x7) | existing_top
        buf4 = integer & 0xFF
        self.reg_write_multi(reg, [buf0, buf1, buf2, buf3, buf4])

        self.reg_writef(REG_RFPLL_DIVIDERS, div_mask, vco_div)

        return self.check_cal_done(lock_reg, VCO_LOCK, 1)

    def set_rx_lo_freq(self, freq_hz):
        return self.set_lo_freq(False, freq_hz)

    def set_tx_lo_freq(self, freq_hz):
        return self.set_lo_freq(True, freq_hz)

    # -- RX analog bring-up (port of ad9361_setup's RX analog section) -----
    # Without these the RX ADC/baseband filter/TIA are left at power-on
    # defaults and the ADC output is a constant word (found 2026-10-17: BIST
    # PRBS reached the FPGA fine while normal RX data was a flat -1).
    def rx_bb_analog_filter_calib(self, rx_bb_bw_hz, bbpll_hz):
        """Port of ad9361_rx_bb_analog_filter_calib. rx_bb_bw_hz is the
        single-sided baseband bandwidth (RF bandwidth / 2)."""
        rx_bb_bw_hz = max(200_000, min(28_000_000, rx_bb_bw_hz))
        target = 126906 * (rx_bb_bw_hz // 10000)
        self.rxbbf_div = min(511, -(-bbpll_hz // target))  # DIV_ROUND_UP

        self.reg_write(REG_RX_BBF_TUNE_DIVIDE, self.rxbbf_div & 0xFF)
        self.reg_writef(REG_RX_BBF_TUNE_CONFIG, 1 << 0, self.rxbbf_div >> 8)
        self.reg_write(REG_RX_BBBW_MHZ, rx_bb_bw_hz // 1_000_000)
        tmp = div_round_closest((rx_bb_bw_hz % 1_000_000) * 128, 1_000_000)
        self.reg_write(REG_RX_BBBW_KHZ, min(127, tmp))
        self.reg_write(REG_RX_MIX_LO_CM, 0x3F)
        self.reg_write(REG_RX_MIX_GM_CONFIG, 3)
        self.reg_write(REG_RX1_TUNE_CTRL, RX_TUNE_RESAMPLE)
        self.reg_write(REG_RX2_TUNE_CTRL, RX_TUNE_RESAMPLE)

        self.reg_write(REG_CALIBRATION_CTRL, RX_BB_TUNE_CAL)
        ok = self.check_cal_done(REG_CALIBRATION_CTRL, RX_BB_TUNE_CAL, 0)

        self.reg_write(REG_RX1_TUNE_CTRL, RX_TUNE_RESAMPLE | RX_PD_TUNE)
        self.reg_write(REG_RX2_TUNE_CTRL, RX_TUNE_RESAMPLE | RX_PD_TUNE)
        return ok

    def rx_tia_calib(self, bb_bw_hz):
        """Port of ad9361_rx_tia_calib; relies on registers set implicitly
        by rx_bb_analog_filter_calib."""
        reg1eb = self.reg_read(REG_RX_BBF_C3_MSB)
        reg1ec = self.reg_read(REG_RX_BBF_C3_LSB)
        reg1e6 = self.reg_read(REG_RX_BBF_R2346)
        bb_bw_hz = max(200_000, min(20_000_000, bb_bw_hz))

        cbbf = reg1eb * 160 + reg1ec * 10 + 140
        r2346 = 18300 * (reg1e6 & 0x7)
        ctia_ff = (cbbf * r2346 * 560) // 3_500_000

        if bb_bw_hz <= 3_000_000:
            reg1db = 0xE0
        elif bb_bw_hz <= 10_000_000:
            reg1db = 0x60
        else:
            reg1db = 0x20

        if ctia_ff > 2920:
            reg1dc = reg1de = 0x40
            temp = min(127, div_round_closest(ctia_ff - 400, 320))
            reg1dd = reg1df = temp
        else:
            temp = (div_round_closest(ctia_ff - 400, 40) + 0x40) & 0xFF
            reg1dc = reg1de = temp
            reg1dd = reg1df = 0

        self.reg_write(REG_RX_TIA_CONFIG, reg1db)
        self.reg_write(REG_TIA1_C_LSB, reg1dc)
        self.reg_write(REG_TIA1_C_MSB, reg1dd)
        self.reg_write(REG_TIA2_C_LSB, reg1de)
        self.reg_write(REG_TIA2_C_MSB, reg1df)

    def rx_adc_setup(self, bbpll_hz, adc_clk_hz):
        """Port of ad9361_rx_adc_setup: writes 40 ADC config registers
        (0x200-0x227) derived from the BB filter calibration results.
        C unsigned-wrap semantics are reproduced with u64()."""
        M64 = (1 << 64) - 1

        def u64(x):
            return x & M64

        c3_msb = self.reg_read(REG_RX_BBF_C3_MSB)
        c3_lsb = self.reg_read(REG_RX_BBF_C3_LSB)
        r2346 = self.reg_read(REG_RX_BBF_R2346)

        bb_bw_hz = (bbpll_hz * 10000) // (126906 * self.rxbbf_div)
        bb_bw_hz = max(200_000, min(28_000_000, bb_bw_hz))
        scale_snr_1e3 = 1000 if adc_clk_hz < 80_000_000 else 1585

        if bb_bw_hz >= 18_000_000:
            invrc = (160975 * r2346 * (160 * c3_msb + 10 * c3_lsb + 140) *
                     bb_bw_hz * (1000 + (10 * (bb_bw_hz - 18_000_000) // 1_000_000)))
            invrc //= 1000
        else:
            invrc = (160975 * r2346 * (160 * c3_msb + 10 * c3_lsb + 140) *
                     bb_bw_hz)
        invrc //= 1_000_000_000
        assert invrc > 0, "invrc_tconst_1e6 == 0 (BB filter calibration failed?)"

        sqrt_inv_rc_1e3 = math.isqrt(invrc)
        maxsnr = 640 // 160
        scaled_adc_clk_1e6 = div_round_closest(adc_clk_hz, 640)
        inv_scaled_adc_clk_1e3 = div_round_closest(
            640_000_000, div_round_closest(adc_clk_hz, 1000))
        tmp_1e3 = div_round_closest(
            980000 + 20 * max(1000, div_round_closest(inv_scaled_adc_clk_1e3, maxsnr)),
            1000)
        sqrt_term_1e3 = math.isqrt(scaled_adc_clk_1e6)
        min_sqrt_term_1e3 = min(1000, math.isqrt(maxsnr * scaled_adc_clk_1e6))

        d = [0] * 40
        d[3] = 0x24
        d[4] = 0x24
        d[7] = min(124, u64(-50_000_000 + 8 * scale_snr_1e3 * sqrt_inv_rc_1e3 *
                            min_sqrt_term_1e3) // 100_000_000)
        t = (invrc >> 1) + 20 * inv_scaled_adc_clk_1e3 * d[7] // 80 * 1000
        d[8] = min(255, t // invrc)
        d[10] = min(127, u64(-500_000 + 77 * sqrt_inv_rc_1e3 *
                             min_sqrt_term_1e3) // 1_000_000)
        d[9] = min(127, (800 * d[10]) // 1000)
        t = (invrc >> 1) + 20 * inv_scaled_adc_clk_1e3 * d[10] * 1000
        d[11] = min(255, t // (invrc * 77))
        d[12] = min(127, u64(-500_000 + 80 * sqrt_inv_rc_1e3 *
                             min_sqrt_term_1e3) // 1_000_000)
        t = u64(-3 * (invrc >> 1) + inv_scaled_adc_clk_1e3 * d[12] * (1000 * 20 // 80))
        d[13] = min(255, t // invrc)
        d[14] = (21 * (inv_scaled_adc_clk_1e3 // 10000)) & 0xFF
        d[15] = min(127, (500 + 1025 * d[7]) // 1000)
        d[16] = min(127, (d[15] * tmp_1e3) // 1000)
        d[17] = d[15]
        d[18] = min(127, (500 + 975 * d[10]) // 1000)
        d[19] = min(127, (d[18] * tmp_1e3) // 1000)
        d[20] = d[18]
        d[21] = min(127, (500 + 975 * d[12]) // 1000)
        d[22] = min(127, (d[21] * tmp_1e3) // 1000)
        d[23] = d[21]
        d[24] = 0x2E
        d[25] = 128 + min(63000, div_round_closest(63 * scaled_adc_clk_1e6, 1000)) // 1000
        d[26] = min(63, 63 * scaled_adc_clk_1e6 // 1_000_000 *
                    (920 + 80 * inv_scaled_adc_clk_1e3 // 1000) // 1000)
        d[27] = min(63, (32 * sqrt_term_1e3) // 1000)
        d[28] = d[25]
        d[29] = d[26]
        d[30] = d[27]
        d[31] = d[25]
        d[32] = d[26]
        d[33] = min(63, 63 * sqrt_term_1e3 // 1000)
        d[34] = min(127, 64 * sqrt_term_1e3 // 1000)
        d[35] = 0x40
        d[36] = 0x40
        d[37] = 0x2C
        for i, v in enumerate(d):
            self.reg_write(0x200 + i, v & 0xFF)
        return d

    def load_rx_gain_table(self, dest=GT_RX1 | GT_RX2):
        """Port of ad9361_load_gt for the 200-1300 MHz full gain table.
        Without it, manual gain indexes map to nothing (the chip's table RAM
        is undefined at power-up). Valid manual-gain indexes are 0-76."""
        tab = RX_GAIN_TABLE_200_1300MHZ
        self.reg_writef(REG_AGC_CONFIG_2, AGC_USE_FULL_GAIN_TABLE, 1)
        self.reg_write(REG_MAX_LMT_FULL_GAIN, len(tab) - 1)
        sel = (dest & 0x3) << 3
        self.reg_write(REG_GAIN_TABLE_CONFIG, START_GAIN_TABLE_CLOCK | sel)
        self.tx_quad_lpf_tia_match = -1
        for i, (d1, d2, d3) in enumerate(tab):
            if (d2 & 0x3F) == 0x20:
                self.tx_quad_lpf_tia_match = i
            self.reg_write(REG_GAIN_TABLE_ADDRESS, i)
            self.reg_write(REG_GAIN_TABLE_WRITE_DATA1, d1)
            self.reg_write(REG_GAIN_TABLE_WRITE_DATA2, d2)
            self.reg_write(REG_GAIN_TABLE_WRITE_DATA3, d3)
            self.reg_write(REG_GAIN_TABLE_CONFIG,
                           START_GAIN_TABLE_CLOCK | WRITE_GAIN_TABLE | sel)
            self.reg_write(REG_GAIN_TABLE_READ_DATA1, 0)
            self.reg_write(REG_GAIN_TABLE_READ_DATA1, 0)
        self.reg_write(REG_GAIN_TABLE_CONFIG, START_GAIN_TABLE_CLOCK | sel)
        self.reg_write(REG_GAIN_TABLE_READ_DATA1, 0)
        self.reg_write(REG_GAIN_TABLE_READ_DATA1, 0)
        self.reg_write(REG_GAIN_TABLE_CONFIG, 0)

    def rf_port_setup(self, rx_input=0, tx_b=False):
        """Port of ad9361_rf_port_setup (is_out=true path): REG_INPUT_SELECT
        picks the RX input (0=A balanced, 1=B balanced, 2=C balanced, 3-8 =
        single-ended A_N/A_P/B_N/B_P/C_N/C_P) and TX output A/B."""
        assert 0 <= rx_input <= 8
        val = (3 << (rx_input * 2)) if rx_input < 3 else (1 << (rx_input - 3))
        if tx_b:
            val |= TX_OUTPUT
        self.reg_write(REG_INPUT_SELECT, val)

    def gain_control_manual(self):
        """Subset of ad9361_gc_setup: put RX1/RX2 in manual gain control
        (RF_GAIN_MGC=0) with the vendor's default power-measurement bits,
        and zero the digital-gain indexes."""
        self.reg_write(REG_AGC_CONFIG_1,
                       DEC_PWR_FOR_GAIN_LOCK_EXIT | DEC_PWR_FOR_LOCK_LEVEL |
                       DEC_PWR_FOR_LOW_PWR)
        self.reg_write(REG_RX1_MANUAL_DIGITALFORCED_GAIN, 0)
        self.reg_write(REG_RX2_MANUAL_DIGITALFORCED_GAIN, 0)

    def rx_analog_init(self, rf_bandwidth_hz=4_000_000, bbpll_hz=983_040_000,
                       adc_clk_hz=61_440_000, rx_input=0):
        """Run the RX analog bring-up in the vendor's order. Call after LO
        lock, while ENSM is still in ALERT (before enable_fdd)."""
        bb_bw = rf_bandwidth_hz // 2
        self.rf_port_setup(rx_input)
        self.gain_control_manual()
        self.load_rx_gain_table()
        ok = self.rx_bb_analog_filter_calib(bb_bw, bbpll_hz)
        self.rx_tia_calib(bb_bw)
        self.rx_adc_setup(bbpll_hz, adc_clk_hz)
        return ok

    # -- TX analog bring-up + calibrations (ports of the vendor driver) -----
    def run_calibration(self, mask):
        self.reg_write(REG_CALIBRATION_CTRL, mask)
        return self.check_cal_done(REG_CALIBRATION_CTRL, mask, 0)

    def tx_bb_analog_filter_calib(self, tx_bb_bw_hz, bbpll_hz):
        """Port of ad9361_tx_bb_analog_filter_calib (tx_bb_bw = RF bw / 2)."""
        tx_bb_bw_hz = max(625_000, min(20_000_000, tx_bb_bw_hz))
        target = 145036 * (tx_bb_bw_hz // 10000)
        div = min(511, -(-bbpll_hz // target))
        self.reg_write(REG_TX_BBF_TUNE_DIVIDER, div & 0xFF)
        self.reg_writef(REG_TX_BBF_TUNE_MODE, 1 << 0, div >> 8)
        self.reg_write(REG_TX_TUNE_CTRL, TUNER_RESAMPLE | TX_TUNE_CTRL_1)
        ok = self.run_calibration(TX_BB_TUNE_CAL)
        self.reg_write(REG_TX_TUNE_CTRL,
                       TUNER_RESAMPLE | TX_TUNE_CTRL_1 | TX_PD_TUNE)
        return ok

    def tx_bb_second_filter_calib(self, tx_bb_bw_hz):
        """Port of ad9361_tx_bb_second_filter_calib."""
        M64 = (1 << 64) - 1
        tx_bb_bw_hz = max(530_000, min(20_000_000, tx_bb_bw_hz))
        corner = 15708 * (tx_bb_bw_hz // 10000)
        res = 1
        cap = 0
        for _ in range(4):
            div = corner * res
            cap = ((500_000_000 + (div >> 1)) // div - 12) & M64
            if cap < 64:
                break
            res <<= 1
        if cap > 63:
            cap = 63
        if tx_bb_bw_hz <= 4_500_000:
            reg_conf = 0x59
        elif tx_bb_bw_hz <= 12_000_000:
            reg_conf = 0x56
        else:
            reg_conf = 0x57
        reg_res = {1: 0x0C, 2: 0x04, 4: 0x03, 8: 0x01}.get(res, 0x01)
        self.reg_write(REG_CONFIG0, reg_conf)
        self.reg_write(REG_RESISTOR, reg_res)
        self.reg_write(REG_CAPACITOR, cap & 0xFF)

    def bb_dc_offset_calib(self):
        """Port of ad9361_bb_dc_offset_calib."""
        self.reg_write(REG_BB_DC_OFFSET_COUNT, 0x3F)
        self.reg_write(REG_BB_DC_OFFSET_SHIFT, 0xF)
        self.reg_write(REG_BB_DC_OFFSET_ATTEN, 1)
        return self.run_calibration(BBDC_CAL)

    def rf_dc_offset_calib(self, rx_freq_hz, rx2_phase_inverted=False):
        """Port of ad9361_rf_dc_offset_calib using the vendor/no-OS default
        platform data (count low/high = 0x32/0x28, attenuation low/high = 5/6)."""
        self.reg_write(REG_WAIT_COUNT, 0x20)
        if rx_freq_hz <= 4_000_000_000:
            self.reg_write(REG_RF_DC_OFFSET_COUNT, 0x32)
            self.reg_write(REG_RF_DC_OFFSET_CONFIG_1, 4 | (2 << 4))
            self.reg_write(REG_RF_DC_OFFSET_ATTEN, 5)
        else:
            self.reg_write(REG_RF_DC_OFFSET_COUNT, 0x28)
            self.reg_write(REG_RF_DC_OFFSET_CONFIG_1, 4 | (3 << 4))
            self.reg_write(REG_RF_DC_OFFSET_ATTEN, 6)
        self.reg_write(REG_DC_OFFSET_CONFIG2,
                       USE_WAIT_COUNTER_FOR_RF_DC_INIT_CAL | 3)
        inv = INVERT_RX1_RF_DC_CGOUT_WORD
        if not rx2_phase_inverted:
            inv |= INVERT_RX2_RF_DC_CGOUT_WORD
        self.reg_write(REG_INVERT_BITS, inv)
        return self.run_calibration(RFDC_CAL)

    def _tx_quad_calib_once(self, phase, rxnco_word, decim):
        self.reg_write(REG_QUAD_CAL_NCO_FREQ_PHASE_OFFSET,
                       ((rxnco_word & 0x3) << 5) | (phase & 0x1F))
        base = (1 << 6) | (1 << 5) | (1 << 4) | (1 << 3) | (decim & 0x3)
        self.reg_write(REG_QUAD_CAL_CTRL, base | (1 << 2))  # + soft reset
        self.reg_write(REG_QUAD_CAL_CTRL, base)
        if not self.run_calibration(TX_QUAD_CAL):
            return None
        return self.reg_read(REG_QUAD_CAL_STATUS_TX1) & (TX1_LO_CONV | TX1_SSB_CONV)

    def tx_quad_calib(self, bw_rx_hz, bw_tx_hz, rx_phase=-1,
                      clkrf_hz=7_680_000, clktf_hz=7_680_000):
        """Port of ad9361_tx_quad_calib for the RX1/TX1 (1R1T) case with
        no RX2 phase inversion. bw_* are single-sided bandwidths (RF bw/2);
        clkrf/clktf are the RX/TX baseband clocks of the configured chain.
        Returns the final (LO_CONV|SSB_CONV) status bits (3 = both converged)."""
        assert self.tx_quad_lpf_tia_match >= 0, "load_rx_gain_table() first"
        txnco_word = div_round_closest(bw_tx_hz * 8, clktf_hz) - 1
        txnco_word = max(0, min(3, txnco_word))
        rxnco_word = txnco_word
        decim = 2 if clktf_hz <= 4_000_000 else 3
        _rx_phase = 0

        if clkrf_hz // 2 == clktf_hz:
            _rx_phase = 0x0E
            if txnco_word == 0:
                txnco_word += 1
            elif txnco_word == 1:
                rxnco_word -= 1
            elif txnco_word == 2:
                rxnco_word -= 2
                txnco_word -= 1
            elif txnco_word == 3:
                rxnco_word -= 2
                _rx_phase = 0x08
        elif clkrf_hz == clktf_hz:
            if txnco_word in (0, 3):
                _rx_phase = 0x15
            elif txnco_word == 2:
                _rx_phase = 0x1F
            else:
                if self.reg_readf(REG_TX_ENABLE_FILTER_CTRL, 0x3F) == 0x22:
                    _rx_phase = 0x15
                else:
                    _rx_phase = 0x1A
        elif clktf_hz // 2 == clkrf_hz:
            _rx_phase = (-2) & 0xFF
            if txnco_word == 0:
                rxnco_word = 1
            else:
                txnco_word, rxnco_word = 1, 3
        elif clktf_hz // 4 == clkrf_hz:
            _rx_phase = (-2) & 0xFF
            txnco_word, rxnco_word = 0, 3
        elif clkrf_hz // 4 == clktf_hz:
            _rx_phase = (-2) & 0xFF
            txnco_word, rxnco_word = 3, 0
        else:
            raise ValueError(f"unhandled clkrf={clkrf_hz} clktf={clktf_hz}")

        if rx_phase >= 0:
            _rx_phase = rx_phase

        txnco_freq = clktf_hz * (txnco_word + 1) // 32
        assert txnco_freq <= bw_rx_hz // 4 and txnco_freq <= bw_tx_hz // 4, \
            "bandwidth too narrow for TX quad cal (vendor path re-tunes RF bw)"

        self.reg_writef(REG_KEXP_2, 0x3 << 6, txnco_word)
        self.reg_write(REG_QUAD_CAL_COUNT, 0xFF)
        self.reg_write(REG_KEXP_1, (1 << 6) | (3 << 4) | (3 << 2) | 3)
        self.reg_write(REG_MAG_FTEST_THRESH, 0x03)
        self.reg_write(REG_MAG_FTEST_THRESH_2, 0x03)
        self.reg_write(REG_TX_QUAD_FULL_LMT_GAIN, self.tx_quad_lpf_tia_match)
        self.reg_write(REG_QUAD_SETTLE_COUNT, 0xF0)
        self.reg_write(REG_TX_QUAD_LPF_GAIN, 0x00)

        want = TX1_LO_CONV | TX1_SSB_CONV
        val = 0
        if rx_phase != -2:
            val = self._tx_quad_calib_once(_rx_phase, rxnco_word, decim)
            if val != want and getattr(self, "last_tx_quad_cal_phase", 0x7FFFFFFF) < 31:
                val = self._tx_quad_calib_once(self.last_tx_quad_cal_phase,
                                               rxnco_word, decim)
            elif val == want:
                self.last_tx_quad_cal_phase = _rx_phase
        if val != want:
            field = [0] * 64
            for i in range(32):
                v = self._tx_quad_calib_once(i, rxnco_word, decim) or 0
                field[i] = field[i + 32] = int(not ((v & TX1_LO_CONV) and (v & TX1_SSB_CONV)))
            best_len, best_start, cnt, start = 0, 0, 0, -1
            for i, f in enumerate(field):
                if f == 0:
                    if start == -1:
                        start = i
                    cnt += 1
                else:
                    if cnt > best_len:
                        best_len, best_start = cnt, start
                    start, cnt = -1, 0
            if cnt > best_len:
                best_len, best_start = cnt, start
            self.last_tx_quad_cal_phase = (best_start + best_len // 2) & 0x1F
            val = self._tx_quad_calib_once(self.last_tx_quad_cal_phase,
                                           rxnco_word, decim)
        return val

    def synth_cp_calib(self, tx, ref_clk_hz=None):
        """Port of ad9361_txrx_synth_cp_calib (FDD). Vendor order: run for
        RX then TX BEFORE programming the corresponding LO frequency."""
        ref_clk_hz = ref_clk_hz or self.ref_clk_hz
        offs = 0x40 if tx else 0
        self.reg_write(REG_RX_CP_LEVEL_DETECT + offs, 0x17)
        self.reg_write(REG_RX_DSM_SETUP_1 + offs, 0x0)
        self.reg_write(REG_RX_LO_GEN_POWER_MODE + offs, 0x00)
        self.reg_write(REG_RX_VCO_LDO + offs, 0x0B)
        self.reg_write(REG_RX_VCO_PD_OVERRIDES + offs, 0x02)
        self.reg_write(REG_RX_CP_CURRENT_ + offs, 0x80)
        self.reg_write(REG_RX_CP_CONFIG + offs, CP_OFFSET_OFF)
        # FDD: VCO_CAL_COUNT(3) | FB_CLOCK_ADV(2)
        self.reg_write(REG_RX_VCO_CAL + offs, VCO_CAL_EN | (3 << 2) | 2)
        self.reg_write(REG_ENSM_CONFIG_2, DUAL_SYNTH_MODE)
        self.reg_write(REG_ENSM_CONFIG_1, FORCE_ALERT_STATE | TO_ALERT)
        self.reg_write(REG_ENSM_MODE, FDD_MODE)
        self.reg_write(REG_RX_CP_CONFIG + offs, CP_OFFSET_OFF | CP_CAL_ENABLE)
        return self.check_cal_done(REG_RX_CAL_STATUS + offs, CP_CAL_VALID, 1)

    def load_mixer_gm_subtable(self):
        """Port of ad9361_load_mixer_gm_subtable."""
        self.reg_write(REG_GM_SUB_TABLE_CONFIG, START_GM_SUB_TABLE_CLOCK)
        addr = len(GM_ST_CTRL)
        for i in range(len(GM_ST_CTRL)):
            addr -= 1
            self.reg_write(REG_GM_SUB_TABLE_ADDRESS, addr)
            self.reg_write(REG_GM_SUB_TABLE_BIAS_WRITE, 0)
            self.reg_write(REG_GM_SUB_TABLE_GAIN_WRITE, GM_ST_GAIN[i])
            self.reg_write(REG_GM_SUB_TABLE_CTRL_WRITE, GM_ST_CTRL[i])
            self.reg_write(REG_GM_SUB_TABLE_CONFIG,
                           WRITE_GM_SUB_TABLE | START_GM_SUB_TABLE_CLOCK)
            self.reg_write(REG_GM_SUB_TABLE_GAIN_READ, 0)
            self.reg_write(REG_GM_SUB_TABLE_GAIN_READ, 0)
        self.reg_write(REG_GM_SUB_TABLE_CONFIG, START_GM_SUB_TABLE_CLOCK)
        self.reg_write(REG_GM_SUB_TABLE_GAIN_READ, 0)
        self.reg_write(REG_GM_SUB_TABLE_GAIN_READ, 0)
        self.reg_write(REG_GM_SUB_TABLE_CONFIG, 0)

    def analog_init(self, rf_rx_bw_hz=4_000_000, rf_tx_bw_hz=4_000_000,
                    bbpll_hz=983_040_000, adc_clk_hz=61_440_000,
                    rx_freq_hz=700_000_000, rx_input=0,
                    clkrf_hz=7_680_000, clktf_hz=7_680_000):
        """Full RX+TX analog bring-up in the vendor ad9361_setup() order
        (gain control, filter/TIA/ADC setup, DC-offset cals, TX quad cal).
        Call after both LOs are locked, with ENSM still in ALERT. Returns a
        dict of per-step results."""
        rx_bb, tx_bb = rf_rx_bw_hz // 2, rf_tx_bw_hz // 2
        self.rf_port_setup(rx_input)
        self.load_mixer_gm_subtable()
        self.gain_control_manual()
        self.load_rx_gain_table()
        r = {}
        r["rx_bb_filter"] = self.rx_bb_analog_filter_calib(rx_bb, bbpll_hz)
        r["tx_bb_filter"] = self.tx_bb_analog_filter_calib(tx_bb, bbpll_hz)
        self.rx_tia_calib(rx_bb)
        self.tx_bb_second_filter_calib(tx_bb)
        self.rx_adc_setup(bbpll_hz, adc_clk_hz)
        r["bb_dc"] = self.bb_dc_offset_calib()
        r["rf_dc"] = self.rf_dc_offset_calib(rx_freq_hz)
        r["tx_quad"] = self.tx_quad_calib(rx_bb, tx_bb, clkrf_hz=clkrf_hz,
                                           clktf_hz=clktf_hz)
        return r

    # -- ENSM / gain -------------------------------------------------------
    def enable_rx1_tx1(self):
        """RX_CHANNEL_ENABLE/TX_CHANNEL_ENABLE live in the SAME registers
        as the clock-chain decimation bits (REG_RX/TX_ENABLE_FILTER_CTRL)
        but in a different field (bits 7:6, vs bits 5:0 used by
        set_clock_chain()) -- never written, so they sat at their
        power-on-reset default (disabled). This is almost certainly why
        the digital interface could report "locked" while every captured
        sample was genuinely zero: the interface can lock onto a clock/
        frame pattern even with the actual receiver gated off at this
        separate enable bit."""
        self.reg_writef(REG_RX_ENABLE_FILTER_CTRL, RX_CHANNEL_ENABLE(0x3), RX_1)
        self.reg_writef(REG_TX_ENABLE_FILTER_CTRL, TX_CHANNEL_ENABLE(0x3), TX_1)

    def enable_fdd(self):
        """Force ENSM into FDD (RX+TX both active). Must be a full
        register WRITE, not read-modify-write -- power_up_clocks() left
        TO_ALERT|FORCE_ALERT_STATE set in REG_ENSM_CONFIG_1, and
        FORCE_ALERT_STATE overrides FORCE_RX_ON/FORCE_TX_ON if left set,
        which is why an earlier read-modify-write version here left ENSM
        stuck reading back as ALERT (0x5) even after this call."""
        self.reg_write(REG_ENSM_MODE, FDD_MODE)
        self.reg_write(REG_ENSM_CONFIG_1, FORCE_RX_ON | FORCE_TX_ON)
        time.sleep(0.01)

    def ensm_state(self):
        return self.reg_readf(REG_STATE, 0xF)

    def set_rx_manual_gain(self, gain_index=0x58):
        """REG_RX1_MANUAL_LMT_FULL_GAIN -- full gain-table index, manual
        mode. 0x58 (~ -ish mid-table) is a conservative starting point;
        tune empirically once a signal is visible."""
        self.reg_write(REG_RX1_MANUAL_LMT_FULL_GAIN, gain_index)

    def set_tx_atten_db(self, atten_db):
        """REG_TX1_ATTEN_0/1: 0.25dB/LSB, 10-bit value split across two
        registers. atten_db in [0, 89.75]."""
        code = int(atten_db * 4) & 0x3FF
        # Vendor ad9361_set_tx_atten: the new value only takes effect after
        # the IMMEDIATELY_UPDATE_TPC_ATTEN latch (REG_TX2_DIG_ATTEN bit 6) is
        # cleared before and set after the write.
        self.reg_writef(REG_TX_ATTEN_OFFSET, 1 << 6, 0)   # MASK_CLR_ATTEN_UPDATE
        self.reg_writef(REG_TX2_DIG_ATTEN, 1 << 6, 0)
        self.reg_write(REG_TX1_ATTEN_0, code & 0xFF)
        self.reg_writef(REG_TX1_ATTEN_1, 0x3, (code >> 8) & 0x3)
        self.reg_writef(REG_TX2_DIG_ATTEN, 1 << 6, 1)

    # -- programmable FIR filter -------------------------------------------
    def load_fir_filter_coef(self, dest, gain_db, coef):
        """Port of ad9361_load_fir_filter_coef -- loads the AD9361's
        programmable RX/TX FIR filter taps over SPI. Never implemented
        before this session: identified early in the fs/3-artifact
        investigation (AD9361_FS3_INVESTIGATION.md) as a real gap, since
        the chip never actually has real filter coefficients without
        this -- only the bypass/decimation-enable bits were ever set.

        dest: FIR_RX1 or FIR_TX1. coef: list of int16 tap values (length
        16-128, multiple of 16). gain_db: RX supports -12/-6/0/+6dB via
        REG_RX_FILTER_GAIN (same formula as the reference driver); TX
        only has a single -6dB bit (TX_FIR_GAIN_6DB) -- anything else is
        treated as 0dB.

        Unlike the reference driver, this does not save/restore the
        FIR-enable/decimation field in REG_RX/TX_ENABLE_FILTER_CTRL --
        this project always runs one fixed, known clock-chain
        configuration, so the caller must already have set that field
        via set_clock_chain() (e.g. rx_fir_div=2) before calling this."""
        ntaps = len(coef)
        assert coef and ntaps <= 128 and ntaps % 16 == 0, \
            f"ntaps must be 1-128 and a multiple of 16, got {ntaps}"

        self.reg_write(REG_ENSM_CONFIG_1, TO_ALERT | FORCE_ALERT_STATE)

        is_rx = bool(dest & FIR_IS_RX)
        offs = RX_TX_FILTER_OFFSET if is_rx else 0
        fir_conf = 0

        if is_rx:
            gain_field = 3 - (gain_db + 12) // 6
            self.reg_write(REG_RX_FILTER_GAIN, gain_field & 0x3)
        elif gain_db == -6:
            fir_conf = TX_FIR_GAIN_6DB

        fir_conf |= FIR_NUM_TAPS(ntaps // 16 - 1) | FIR_SELECT(dest) | FIR_START_CLK

        self.reg_write(REG_TX_FILTER_CONF + offs, fir_conf)
        for i, c in enumerate(coef):
            c16 = c & 0xFFFF
            self.reg_write(REG_TX_FILTER_COEF_ADDR + offs, i)
            self.reg_write(REG_TX_FILTER_COEF_WRITE_DATA_1 + offs, c16 & 0xFF)
            self.reg_write(REG_TX_FILTER_COEF_WRITE_DATA_2 + offs, (c16 >> 8) & 0xFF)
            self.reg_write(REG_TX_FILTER_CONF + offs, fir_conf | FIR_WRITE)
            # Dummy writes to delay ~1us for the FIR's internal write
            # strobe to settle, matching the reference driver exactly.
            self.reg_write(REG_TX_FILTER_COEF_READ_DATA_2 + offs, 0)
            self.reg_write(REG_TX_FILTER_COEF_READ_DATA_2 + offs, 0)

        self.reg_write(REG_TX_FILTER_CONF + offs, fir_conf)
        fir_conf &= ~FIR_START_CLK
        self.reg_write(REG_TX_FILTER_CONF + offs, fir_conf)

        self.enable_fdd()

    def verify_fir_filter_coef(self, dest, coef):
        """Diagnostic only: read back loaded taps, return a list of
        (index, expected, actual) mismatches (empty = all taps verified)."""
        ntaps = len(coef)
        is_rx = bool(dest & FIR_IS_RX)
        offs = RX_TX_FILTER_OFFSET if is_rx else 0
        conf = FIR_NUM_TAPS(ntaps // 16 - 1) | FIR_SELECT(dest) | FIR_START_CLK
        self.reg_write(REG_TX_FILTER_CONF + offs, conf)
        mismatches = []
        for i, expected in enumerate(coef):
            self.reg_write(REG_TX_FILTER_COEF_ADDR + offs, i)
            lo = self.reg_read(REG_TX_FILTER_COEF_READ_DATA_1 + offs)
            hi = self.reg_read(REG_TX_FILTER_COEF_READ_DATA_2 + offs)
            val = lo | (hi << 8)
            if val != (expected & 0xFFFF):
                mismatches.append((i, expected & 0xFFFF, val))
        self.reg_write(REG_TX_FILTER_CONF + offs, conf & ~FIR_START_CLK)
        return mismatches


if __name__ == "__main__":
    from pynq import Overlay

    ol = Overlay("base/system_top.bit")
    ad = AD9361()
    try:
        print(f"Product ID: 0x{ad.product_id():02X} (ok={ad.product_id_ok()})")
    finally:
        ad.close()
