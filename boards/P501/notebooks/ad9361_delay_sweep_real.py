#!/usr/bin/env python3
"""
Simpler, more trustworthy alternative to ad9361_dig_tune.py's BIST/PN-based
sweep (which reported 0/256 passes and left the ADC path unable to even
complete a DMA transfer afterward -- likely a bug in how the BIST
injection/PN-checker was engaged, not a real "digital interface totally
broken" result).

Instead of trusting a synthetic BIST pattern, sweep the AD9361 chip-side
REG_RX_CLOCK_DATA_DELAY register (clock_delay x data_delay, both 4-bit
fields, 0-15 each) using the REAL, already-proven TX-tone-and-capture
pipeline (ad9361_tone_test.py's exact bring-up) as the pass/fail signal:
at each delay setting, capture, and check whether the raw samples show
real variation (not the exact frozen 3-value rotating pattern seen at
every setting tested so far) and/or whether the FFT peak lands near the
real +1MHz tone instead of the fs/3 alias.
"""
import sys
sys.path.insert(0, '/home/xilinx/jupyter_notebooks')
import time
import numpy as np
from pynq import Overlay, allocate
from ad9361_driver import AD9361
from axi_dmac import AxiDmac
import ad9361_full_bypass_test as fb

REG_RX_CLOCK_DATA_DELAY = 0x006
REG_ENSM_CONFIG_1 = 0x014
TO_ALERT = 1 << 0
FORCE_ALERT_STATE = 1 << 2

N_SAMPLES = 8192
SAMPLE_RATE = 3_840_000
TONE_FREQ = 1_000_000


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
        ad.reg_write(fb.REG_TX_CLOCK_DATA_DELAY, fb.TX_CLOCK_DATA_DELAY)
        ad.reg_write(fb.REG_LVDS_BIAS_CTRL, fb.LVDS_BIAS_CTRL)
        ad.enable_rx1_tx1()
        ad.set_rx_manual_gain(0x70)
        ad.set_tx_atten_db(20)
        ad.enable_fdd()
        print(f"ENSM = 0x{ad.ensm_state():X}")

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

        print(f"{'cd':>3} {'dd':>3}  {'done':>5} {'std_i':>8} {'peak_kHz':>9} {'peak_dB':>8}  first4_I")
        best = None
        for cd in range(16):
            for dd in range(16):
                ad.reg_write(REG_ENSM_CONFIG_1, TO_ALERT | FORCE_ALERT_STATE)
                ad.reg_write(REG_RX_CLOCK_DATA_DELAY, (cd << 4) | dd)
                ad.enable_fdd()

                ok = rx_dmac.transfer_to_memory(rx_buf.physical_address,
                                                 rx_buf.nbytes, timeout_s=0.5)
                rx_buf.invalidate()
                rx_i = rx_buf[0::2].astype(np.float64)
                rx_q = rx_buf[1::2].astype(np.float64)
                std_i = rx_i.std()

                rx_complex = rx_i + 1j * rx_q
                spectrum = np.fft.fftshift(np.fft.fft(rx_complex))
                freqs = np.fft.fftshift(np.fft.fftfreq(N_SAMPLES, d=1 / SAMPLE_RATE))
                mag_db = 20 * np.log10(np.abs(spectrum) + 1e-9)
                peak_idx = np.argmax(mag_db)
                peak_f = freqs[peak_idx]
                peak_db = mag_db[peak_idx]

                near_tone = abs(peak_f - TONE_FREQ) < 100e3
                marker = " <-- NEAR TONE" if (ok and near_tone) else ""
                print(f"{cd:3d} {dd:3d}  {str(ok):>5} {std_i:8.1f} {peak_f/1e3:9.1f} "
                      f"{peak_db:8.1f}  {rx_i[:4].astype(int).tolist()}{marker}")

                if ok and near_tone:
                    if best is None or peak_db > best[2]:
                        best = (cd, dd, peak_db)

        tx_dmac.stop()

        if best:
            print(f"\nBest match: clock_delay={best[0]}, data_delay={best[1]}, "
                  f"peak {best[2]:.1f} dB near tone")
        else:
            print("\nNo delay setting produced a peak near the real TX tone "
                  "frequency across the full 16x16 sweep.")
    finally:
        ad.close()


if __name__ == "__main__":
    main()
