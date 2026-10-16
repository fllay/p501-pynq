#!/usr/bin/env python3
"""Isolate which FIR load (RX, TX, both, or neither) causes the frozen
-1 DC capture seen in ad9361_fir_test.py."""
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
TX_ATTEN_DB = 0
RX_GAIN_INDEX = 0xFF

MODE = sys.argv[1] if len(sys.argv) > 1 else "none"  # none|rx|tx|both


def main():
    ol = Overlay("base/system_top.bit")
    ad = AD9361()
    try:
        ad.power_up_clocks()
        assert ad.set_bbpll_rate(983_040_000)
        ad.set_clock_chain()
        assert ad.set_rx_lo_freq(700_000_000)
        assert ad.set_tx_lo_freq(700_000_000)

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
        assert ad.ensm_state() == 0xA

        coef = make_fir_coef()
        print(f"MODE={MODE}")
        if MODE in ("rx", "both"):
            ad.load_fir_filter_coef(FIR_RX1, 0, coef)
            print("RX FIR loaded, ENSM=", hex(ad.ensm_state()))
        if MODE in ("tx", "both"):
            ad.load_fir_filter_coef(FIR_TX1, 0, coef)
            print("TX FIR loaded, ENSM=", hex(ad.ensm_state()))

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
        ok = rx_dmac.transfer_to_memory(rx_buf.physical_address, rx_buf.nbytes)
        rx_buf.invalidate()

        rx_i = rx_buf[0::2].astype(np.float64)
        print(f"ok={ok} I std={rx_i.std():.1f} first8={rx_i[:8].astype(int).tolist()}")
        tx_dmac.stop()
    finally:
        ad.close()


if __name__ == "__main__":
    main()
