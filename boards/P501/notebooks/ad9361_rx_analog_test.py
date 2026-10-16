#!/usr/bin/env python3
"""Bring-up + RX analog init (BB filter / TIA / ADC setup ported from the
vendor kernel driver), then TX 1MHz tone / RX capture / FFT. Tests whether
the previously constant RX ADC output (flat -1) becomes real signal."""
import sys
sys.path.insert(0, '/home/xilinx/jupyter_notebooks')
import numpy as np

from pynq import Overlay, allocate

from ad9361_driver import AD9361, FIR_RX1, FIR_TX1
from axi_dmac import AxiDmac
import ad9361_full_bypass_test as fb
from ad9361_fir_test import make_fir_coef

SAMPLE_RATE = 3_840_000
TONE_FREQ = 1_000_000
N_SAMPLES = 8192
TX_ATTEN_DB = 20
RX_GAIN_INDEX = 40


def main():
    ol = Overlay("base/system_top.bit")
    ad = AD9361()
    try:
        ad.power_up_clocks()
        assert ad.set_bbpll_rate(983_040_000), "BBPLL did not lock"
        ad.set_clock_chain()
        assert ad.set_rx_lo_freq(700_000_000), "RX LO did not lock"
        assert ad.set_tx_lo_freq(700_000_000), "TX LO did not lock"

        res = ad.analog_init()
        print(f"analog init: {res} (tx_quad 3 = LO leakage + sideband both converged), "
              f"quad_phase={getattr(ad, 'last_tx_quad_cal_phase', None)}, "
              f"rxbbf_div={ad.rxbbf_div}")

        fb.axi_ad9361_core_enable(ol.axi_ad9361, num_channels=2)
        ad.reg_write(fb.REG_PARALLEL_PORT_CONF_1, fb.PP_CONF_1)
        ad.reg_write(fb.REG_PARALLEL_PORT_CONF_2, fb.PP_CONF_2)
        ad.reg_write(fb.REG_PARALLEL_PORT_CONF_3, fb.PP_CONF_3_LVDS_MODE)
        ad.reg_write(fb.REG_RX_CLOCK_DATA_DELAY, fb.RX_CLOCK_DATA_DELAY)
        ad.reg_write(fb.REG_TX_CLOCK_DATA_DELAY, fb.TX_CLOCK_DATA_DELAY)
        ad.reg_write(fb.REG_LVDS_BIAS_CTRL, fb.LVDS_BIAS_CTRL)
        ad.enable_rx1_tx1()
        ad.set_rx_manual_gain(RX_GAIN_INDEX)
        ad.set_tx_atten_db(TX_ATTEN_DB)
        ad.enable_fdd()
        assert ad.ensm_state() == 0xA, f"ENSM 0x{ad.ensm_state():X}"

        # RX FIR is enabled (decimate-by-2) by set_clock_chain(); with no
        # taps loaded its output is garbage (flat -1 / fs/3 patterns), so
        # real taps must be loaded.
        coef = make_fir_coef()
        ad.load_fir_filter_coef(FIR_RX1, 0, coef)
        ad.load_fir_filter_coef(FIR_TX1, 0, coef)
        fb.axi_ad9361_core_enable(ol.axi_ad9361, num_channels=2)

        t = np.arange(N_SAMPLES) / SAMPLE_RATE
        tone = np.exp(2j * np.pi * TONE_FREQ * t)
        scale = (2**15 - 1) * 0.5
        tx_buf = allocate(shape=(N_SAMPLES * 2,), dtype=np.int16)
        tx_buf[0::2] = (np.real(tone) * scale).astype(np.int16)
        tx_buf[1::2] = (np.imag(tone) * scale).astype(np.int16)
        rx_buf = allocate(shape=(N_SAMPLES * 2,), dtype=np.int16)

        tx_dmac = AxiDmac(ol.axi_ad9361_dac_dma)
        rx_dmac = AxiDmac(ol.axi_ad9361_adc_dma)
        tx_buf.flush()
        tx_dmac.transfer_from_memory(tx_buf.physical_address, tx_buf.nbytes, cyclic=True)
        ok = rx_dmac.transfer_to_memory(rx_buf.physical_address, rx_buf.nbytes)
        rx_buf.invalidate()

        rx_i = rx_buf[0::2].astype(np.float64)
        rx_q = rx_buf[1::2].astype(np.float64)
        print(f"RX done={ok} distinct I values={len(np.unique(rx_i))} "
              f"I std={rx_i.std():.1f} first12={rx_i[:12].astype(int).tolist()}")
        spec = np.fft.fftshift(np.fft.fft(rx_i + 1j * rx_q))
        freqs = np.fft.fftshift(np.fft.fftfreq(N_SAMPLES, d=1 / SAMPLE_RATE))
        mag = 20 * np.log10(np.abs(spec) + 1e-9)
        print(f"Global peak {freqs[np.argmax(mag)]/1e3:.1f} kHz "
              f"{mag.max():.1f} dB (expect +1000 kHz)")
        tx_dmac.stop()
    finally:
        ad.close()


if __name__ == "__main__":
    main()
