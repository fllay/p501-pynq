#!/usr/bin/env python3
"""
Loads real AD9361 programmable FIR filter coefficients (never done before
this session -- previously only the FIR enable/bypass decimation field was
set, with the filter itself left at its uninitialized/undefined tap state)
and reruns the standard TX-tone/RX-capture test to check whether the real
1MHz tone becomes the dominant spectral component, matching what the
vendor's own libiio/kernel-driver stack showed (see
AD9361_FS3_INVESTIGATION.md) instead of this project's own fs/3 artifact.

Coefficients: a 128-tap, Hamming-windowed lowpass (scipy.signal.firwin,
cutoff=0.45 of the pre-decimation Nyquist, giving margin below the
decimated Nyquist at 0.5) scaled to Q14 (sum=16384, ADI's standard 0dB-gain
convention) -- a generic, reasonable anti-aliasing filter for the
decimate-by-2 / interpolate-by-2 configuration this project's
set_clock_chain() default already uses (rx_fir_div=2, tx_fir_div=2),
not a vendor-supplied filter file (this project never had access to one).
"""
import sys
sys.path.insert(0, '/home/xilinx/jupyter_notebooks')
import numpy as np

from pynq import Overlay, allocate

from ad9361_driver import AD9361, FIR_RX1, FIR_TX1
from axi_dmac import AxiDmac
import ad9361_full_bypass_test as fb

SAMPLE_RATE = 3_840_000
TONE_FREQ = 1_000_000
N_SAMPLES = 8192
TX_ATTEN_DB = 0
RX_GAIN_INDEX = 0xFF


def make_fir_coef(ntaps=128, cutoff=0.45):
    from scipy.signal import firwin
    h = firwin(ntaps, cutoff, window='hamming')
    coef = np.round(h * 16384).astype(np.int64)
    assert coef.min() >= -32768 and coef.max() <= 32767
    return coef.astype(np.int16).tolist()


def main():
    ol = Overlay("base/system_top.bit")
    ad = AD9361()

    try:
        ad.power_up_clocks()
        assert ad.set_bbpll_rate(983_040_000), "BBPLL did not lock"
        ad.set_clock_chain()  # defaults -> 3.84 MSPS, rx_fir_div=2, tx_fir_div=2
        assert ad.set_rx_lo_freq(700_000_000), "RX LO did not lock"
        assert ad.set_tx_lo_freq(700_000_000), "TX LO did not lock"
        print("AD9361 locked: BBPLL/RFPLL OK")

        axi_ad9361_core_enable = fb.axi_ad9361_core_enable
        axi_ad9361_core_enable(ol.axi_ad9361, num_channels=2)
        print("axi_ad9361 core reset-deasserted, channels enabled, DMA data-select set")

        ad.reg_write(fb.REG_PARALLEL_PORT_CONF_1, fb.PP_CONF_1)
        ad.reg_write(fb.REG_PARALLEL_PORT_CONF_2, fb.PP_CONF_2)
        ad.reg_write(fb.REG_PARALLEL_PORT_CONF_3, fb.PP_CONF_3_LVDS_MODE)
        ad.reg_write(fb.REG_RX_CLOCK_DATA_DELAY, fb.RX_CLOCK_DATA_DELAY)
        ad.reg_write(fb.REG_TX_CLOCK_DATA_DELAY, fb.TX_CLOCK_DATA_DELAY)
        ad.reg_write(fb.REG_LVDS_BIAS_CTRL, fb.LVDS_BIAS_CTRL)
        print("AD9361 digital interface configured")

        ad.enable_rx1_tx1()
        ad.set_rx_manual_gain(RX_GAIN_INDEX)
        ad.set_tx_atten_db(TX_ATTEN_DB)
        ad.enable_fdd()
        assert ad.ensm_state() == 0xA, f"ENSM not FDD, got 0x{ad.ensm_state():X}"
        print(f"ENSM FDD OK, TX atten={TX_ATTEN_DB}dB, RX gain idx=0x{RX_GAIN_INDEX:02X}")

        # FIR coefficients loaded LAST, as close as possible to the proven
        # working pipeline above (minimizes the chance of the ALERT<->FDD
        # cycling inside load_fir_filter_coef disrupting bring-up state) --
        # an earlier attempt loading FIR right after LO lock (before
        # axi_ad9361/digital-interface setup) left RX capture completely
        # frozen at a flat -1 DC value, suggesting the LVDS clock/interface
        # needs to be fully up first.
        coef = make_fir_coef()
        print(f"Loading {len(coef)}-tap FIR (sum={sum(coef)}, "
              f"min={min(coef)}, max={max(coef)})...")
        ad.load_fir_filter_coef(FIR_RX1, 0, coef)
        rx_mismatches = ad.verify_fir_filter_coef(FIR_RX1, coef)
        print(f"RX FIR verify: {len(rx_mismatches)} mismatches"
              + (f" (first: {rx_mismatches[0]})" if rx_mismatches else ""))

        ad.load_fir_filter_coef(FIR_TX1, 0, coef)
        tx_mismatches = ad.verify_fir_filter_coef(FIR_TX1, coef)
        print(f"TX FIR verify: {len(tx_mismatches)} mismatches"
              + (f" (first: {tx_mismatches[0]})" if tx_mismatches else ""))
        print(f"ENSM after FIR load = 0x{ad.ensm_state():X}")

        # Re-sync axi_ad9361's deserializer/cpack2 after the ALERT<->FDD
        # cycling inside load_fir_filter_coef (twice: RX then TX) -- same
        # fix needed after ad9361_dig_tune.py's repeated delay-register
        # cycling left the ADC path unable to complete a DMA transfer.
        # Re-running the same core-enable helper is idempotent/safe and
        # reapplies RSTN + R1_MODE + CHAN_CNTRL + DATA_SELECT together.
        axi_ad9361_core_enable(ol.axi_ad9361, num_channels=2)
        print("axi_ad9361 ADC+DAC re-synced")

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
        print(f"RX transfer done={ok}")

        rx_i = rx_buf[0::2].astype(np.float64)
        rx_q = rx_buf[1::2].astype(np.float64)
        print(f"First 12 raw I: {rx_i[:12].astype(int).tolist()}")
        print(f"I min/max/mean/std = {rx_i.min():.0f}/{rx_i.max():.0f}/"
              f"{rx_i.mean():.1f}/{rx_i.std():.1f}")

        rx_complex = rx_i + 1j * rx_q
        spectrum = np.fft.fftshift(np.fft.fft(rx_complex))
        freqs = np.fft.fftshift(np.fft.fftfreq(N_SAMPLES, d=1 / SAMPLE_RATE))
        mag_db = 20 * np.log10(np.abs(spectrum) + 1e-9)

        peak_idx = np.argmax(mag_db)
        print(f"\nGlobal peak at {freqs[peak_idx]/1e3:.1f} kHz, {mag_db[peak_idx]:.1f} dB")

        near_tone = np.abs(freqs - TONE_FREQ) < 50e3
        ti = np.argmax(mag_db * near_tone - 1e9 * (~near_tone))
        print(f"Near +{TONE_FREQ/1e3:.0f}kHz (real tone): "
              f"{freqs[ti]/1e3:.1f} kHz, {mag_db[ti]:.1f} dB")

        fs3 = SAMPLE_RATE / 3
        near_fs3 = np.abs(np.abs(freqs) - fs3) < 50e3
        fi = np.argmax(mag_db * near_fs3 - 1e9 * (~near_fs3))
        print(f"Near fs/3 (+-{fs3/1e3:.1f}kHz, the old artifact): "
              f"{freqs[fi]/1e3:.1f} kHz, {mag_db[fi]:.1f} dB")

        order = np.argsort(mag_db)[::-1]
        print("\nTop 8 bins:")
        shown, seen = 0, []
        for idx in order:
            f = freqs[idx]
            if any(abs(f - s) < 20e3 for s in seen):
                continue
            print(f"  {f/1e3:9.1f} kHz  {mag_db[idx]:6.1f} dB")
            seen.append(f)
            shown += 1
            if shown >= 8:
                break

        tx_dmac.stop()
    finally:
        ad.close()


if __name__ == "__main__":
    main()
