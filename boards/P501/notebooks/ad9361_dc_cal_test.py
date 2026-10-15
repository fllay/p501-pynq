#!/usr/bin/env python3
"""
Both independent digital-interface delay knobs (FPGA-side IDELAY taps,
AD9361 chip-side REG_RX_CLOCK_DATA_DELAY) were swept across their full
range with ZERO effect on the captured pattern -- same exact peak
position, magnitude, and even raw sample values at every single setting.
That rules out a timing/bit-alignment explanation entirely (a genuine
skew issue would show *some* variation across a full sweep).

This points back to the other deliberately-skipped piece from
ad9361_driver.py's own docstring: "Calibration (RX/TX quadrature, DC
offset) is deliberately omitted for v1". An AD9361 that has never run
its baseband + RF DC-offset calibration can genuinely pin its ADC near
full-scale regardless of gain setting (a fixed analog offset upstream of
the variable-gain stage doesn't necessarily shrink with gain reduction),
which fits every symptom seen: full-scale rail values, insensitivity to
RX gain/TX atten, and insensitivity to both delay sweeps (since this
isn't a timing problem at all).

Ported from ad9361_bb_dc_offset_calib()/ad9361_rf_dc_offset_calib() in
no-OS's ad9361.c, using the same default constants as no-OS's own
ad9361 example project (projects/ad9361/src/main.c):
dc_offset_attenuation_high/low = 6/5, dc_offset_count_high/low = 0x28/0x32.
"""
import sys
sys.path.insert(0, '/home/xilinx/jupyter_notebooks')
import time
import numpy as np
from pynq import Overlay, allocate
from ad9361_driver import AD9361
from axi_dmac import AxiDmac
import ad9361_full_bypass_test as fb

REG_CALIBRATION_CTRL = 0x016
BBDC_CAL = 1 << 0
RFDC_CAL = 1 << 1

REG_ENSM_CONFIG_1 = 0x014
TO_ALERT = 1 << 0
FORCE_ALERT_STATE = 1 << 2

REG_WAIT_COUNT = 0x185
REG_RF_DC_OFFSET_COUNT = 0x186
REG_RF_DC_OFFSET_CONFIG_1 = 0x187
REG_RF_DC_OFFSET_ATTEN = 0x188
REG_INVERT_BITS = 0x189
REG_DC_OFFSET_CONFIG2 = 0x18B
REG_BB_DC_OFFSET_SHIFT = 0x190
REG_BB_DC_OFFSET_COUNT = 0x193
REG_BB_DC_OFFSET_ATTEN = 0x194

USE_WAIT_COUNTER_FOR_RF_DC_INIT_CAL = 1 << 7
INVERT_RX1_RF_DC_CGOUT_WORD = 1 << 4
INVERT_RX2_RF_DC_CGOUT_WORD = 1 << 5


def dc_calibration_update(x):
    return x & 0x7


def rf_dc_calibration_count(x):
    return x & 0xF


def dac_fs(x):
    return (x & 0x3) << 4


def rf_dc_offset_atten(x):
    return x & 0x1F


def bb_dc_m_shift(x):
    return x & 0x1F


def bb_dc_offset_atten(x):
    return x & 0xF


def run_calibration(ad, mask, timeout_iters=20000):
    ad.reg_write(REG_CALIBRATION_CTRL, mask)
    for _ in range(timeout_iters):
        if ad.reg_readf(REG_CALIBRATION_CTRL, mask) == 0:
            return True
        time.sleep(0.0012)
    return False


def run_dc_offset_calibration(ad, rx_freq_hz):
    ad.reg_write(REG_ENSM_CONFIG_1, TO_ALERT | FORCE_ALERT_STATE)

    # -- baseband DC offset cal --
    ad.reg_write(REG_BB_DC_OFFSET_COUNT, 0x3F)
    ad.reg_write(REG_BB_DC_OFFSET_SHIFT, bb_dc_m_shift(0xF))
    ad.reg_write(REG_BB_DC_OFFSET_ATTEN, bb_dc_offset_atten(1))
    ok_bb = run_calibration(ad, BBDC_CAL)
    print(f"BBDC_CAL done={ok_bb}")

    # -- RF DC offset cal --
    ad.reg_write(REG_WAIT_COUNT, 0x20)
    if rx_freq_hz <= 4_000_000_000:
        count, atten = 0x32, 5  # *_low_range defaults (no-OS example)
        fs = 2
    else:
        count, atten = 0x28, 6  # *_high_range defaults
        fs = 3
    ad.reg_write(REG_RF_DC_OFFSET_COUNT, count)
    ad.reg_write(REG_RF_DC_OFFSET_CONFIG_1, rf_dc_calibration_count(4) | dac_fs(fs))
    ad.reg_write(REG_RF_DC_OFFSET_ATTEN, rf_dc_offset_atten(atten))
    ad.reg_write(REG_DC_OFFSET_CONFIG2,
                 USE_WAIT_COUNTER_FOR_RF_DC_INIT_CAL | dc_calibration_update(3))
    ad.reg_write(REG_INVERT_BITS,
                 INVERT_RX1_RF_DC_CGOUT_WORD | INVERT_RX2_RF_DC_CGOUT_WORD)
    ok_rf = run_calibration(ad, RFDC_CAL)
    print(f"RFDC_CAL done={ok_rf}")

    ad.enable_fdd()
    return ok_bb and ok_rf


def main():
    ol = Overlay("base/system_top.bit")
    ad = AD9361()
    try:
        ad.power_up_clocks()
        assert ad.set_bbpll_rate(983_040_000), "BBPLL did not lock"
        ad.set_clock_chain()
        assert ad.set_rx_lo_freq(700_000_000), "RX LO did not lock"
        assert ad.set_tx_lo_freq(700_000_000), "TX LO did not lock"

        fb.axi_ad9361_core_enable(ol.axi_ad9361, num_channels=2)
        ad.reg_write(fb.REG_PARALLEL_PORT_CONF_1, fb.PP_CONF_1)
        ad.reg_write(fb.REG_PARALLEL_PORT_CONF_2, fb.PP_CONF_2)
        ad.reg_write(fb.REG_PARALLEL_PORT_CONF_3, fb.PP_CONF_3_LVDS_MODE)
        ad.reg_write(fb.REG_RX_CLOCK_DATA_DELAY, fb.RX_CLOCK_DATA_DELAY)
        ad.reg_write(fb.REG_TX_CLOCK_DATA_DELAY, fb.TX_CLOCK_DATA_DELAY)
        ad.reg_write(fb.REG_LVDS_BIAS_CTRL, fb.LVDS_BIAS_CTRL)
        ad.enable_rx1_tx1()
        ad.set_rx_manual_gain(0x70)
        ad.set_tx_atten_db(20)
        ad.enable_fdd()
        print(f"ENSM before cal = 0x{ad.ensm_state():X}")

        ok = run_dc_offset_calibration(ad, 700_000_000)
        print(f"DC offset calibration overall ok={ok}")
        print(f"ENSM after cal = 0x{ad.ensm_state():X}")

        N_SAMPLES = 8192
        SAMPLE_RATE = 3_840_000
        TONE_FREQ = 1_000_000
        t = np.arange(N_SAMPLES) / SAMPLE_RATE
        tone = np.exp(2j * np.pi * TONE_FREQ * t)
        scale = (2**15 - 1) * 0.5
        i_samples = (np.real(tone) * scale).astype(np.int16)
        q_samples = (np.imag(tone) * scale).astype(np.int16)

        tx_buf = allocate(shape=(N_SAMPLES * 2,), dtype=np.int16)
        tx_buf[0::2] = i_samples
        tx_buf[1::2] = q_samples
        rx_buf = allocate(shape=(N_SAMPLES * 2,), dtype=np.int16)

        tx_dmac = AxiDmac(ol.axi_ad9361_dac_dma)
        rx_dmac = AxiDmac(ol.axi_ad9361_adc_dma)
        tx_buf.flush()
        tx_dmac.transfer_from_memory(tx_buf.physical_address, tx_buf.nbytes, cyclic=True)
        done = rx_dmac.transfer_to_memory(rx_buf.physical_address, rx_buf.nbytes)
        rx_buf.invalidate()
        print(f"RX transfer done={done}")

        rx_i = rx_buf[0::2].astype(np.float64)
        rx_q = rx_buf[1::2].astype(np.float64)
        print(f"First 12 raw I: {rx_i[:12].astype(int).tolist()}")
        print(f"I min/max/mean/std = {rx_i.min():.0f}/{rx_i.max():.0f}/"
              f"{rx_i.mean():.1f}/{rx_i.std():.1f}")
        print(f"Q min/max/mean/std = {rx_q.min():.0f}/{rx_q.max():.0f}/"
              f"{rx_q.mean():.1f}/{rx_q.std():.1f}")

        rx_complex = rx_i + 1j * rx_q
        spectrum = np.fft.fftshift(np.fft.fft(rx_complex))
        freqs = np.fft.fftshift(np.fft.fftfreq(N_SAMPLES, d=1 / SAMPLE_RATE))
        mag_db = 20 * np.log10(np.abs(spectrum) + 1e-9)
        peak_idx = np.argmax(mag_db)
        print(f"Peak at {freqs[peak_idx]/1e3:.1f} kHz, {mag_db[peak_idx]:.1f} dB "
              f"(expect near +{TONE_FREQ/1e3:.0f} kHz)")

        tx_dmac.stop()
    finally:
        ad.close()


if __name__ == "__main__":
    main()
