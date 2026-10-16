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
                         rx_fir_div=2, dac_div=16, t2_div=2, t1_div=2,
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
        self.reg_writef(REG_BBPLL, 1 << 3, dac_div // 16 if dac_div in (1, 2) else 0)
        # NOTE: DAC_CLK div is a single bit (1 or 2 only) per ad9361_set_clk_scaler;
        # pass dac_div=1 or 2 only. Written explicitly below to avoid the
        # confusing expression above:
        self.reg_writef(REG_BBPLL, 1 << 3, 0 if dac_div == 1 else 1)

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
        self.reg_write(REG_TX1_ATTEN_0, code & 0xFF)
        self.reg_writef(REG_TX1_ATTEN_1, 0x3, (code >> 8) & 0x3)

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
