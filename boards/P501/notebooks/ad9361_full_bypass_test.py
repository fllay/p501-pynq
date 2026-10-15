#!/usr/bin/env python3
"""
Full decimation/FIR bypass diagnostic: R2=R1=CLKRF=FIR=1 on both RX and TX,
giving the maximum RX/TX sample rate (ADC_CLK itself, 61.44 MSPS with
BBPLL=983.04MHz / adc_div=16) with NO decimation stage active anywhere in
the chain. Everything else (LO=700MHz, FDD, R1-mode axi_ad9361 config,
LVDS digital interface, DMA pipeline) is identical to ad9361_tone_test.py's
known-good baseline.

Purpose: across three previously tested configs (3.84/7.68/30.72 MSPS) the
RX FFT peak always lands at exactly (configured_rate)/3, and this is
consistent with a FIXED artifact at exactly ADC_CLK/3 = 61.44MHz/3 =
20.48MHz sitting BEFORE any decimation, which then aliases through
whatever decimation factor is configured down to the observed fs/3
pattern (mod/fold math checked by hand for all three prior rates). This
test runs the one case with NO decimation to alias through: if the
20.48MHz-pre-decimation theory is right, the FFT peak should land at
±20.48MHz (not at anything related to the 1MHz TX tone, and not at
fs/3 = 20.48MHz anyway -- same number, which is itself a useful
coincidence check) and/or TWO peaks should appear (the real 1MHz TX tone
AND the fixed 20.48MHz spur) distinguishable from each other.
"""
import numpy as np

from pynq import Overlay, allocate

from ad9361_driver import AD9361
from axi_dmac import AxiDmac

# ADC_CLK = BBPLL/adc_div = 983.04MHz/16 = 61.44MHz; with r2=r1=clkrf=fir=1
# (all bypassed), RX/TX sample rate = ADC_CLK itself = 61.44 MSPS.
SAMPLE_RATE = 61_440_000
TONE_FREQ = 1_000_000
SPUR_FREQ = 20_480_000  # ADC_CLK/3 -- the suspected fixed pre-decimation artifact
N_SAMPLES = 32768  # more samples than the 8192 baseline for finer FFT bins
TX_ATTEN_DB = 60  # much lower TX power than the 20dB baseline -- testing
# whether the fixed period-3/full-scale pattern is RX front-end saturation
# from TX self-leakage (antennas physically close, same 700MHz LO, and at
# full bypass the receiver has NO channel filtering at all -- wide open to
# the full 61.44MHz band)
RX_GAIN_INDEX = 0x00  # minimum manual gain, same saturation hypothesis


AXI_ADC_REG_RSTN = 0x040
AXI_ADC_MMCM_RSTN = 1 << 1
AXI_ADC_RSTN = 1 << 0
AXI_ADC_REG_CHAN_CNTRL = lambda c: 0x400 + c * 0x40
AXI_ADC_FORMAT_SIGNEXT = 1 << 6
AXI_ADC_FORMAT_ENABLE = 1 << 4
AXI_ADC_ENABLE = 1 << 0

DAC_ADDR_OFFSET = 0x4000
AXI_DAC_REG_RSTN = 0x040 + DAC_ADDR_OFFSET
AXI_DAC_MMCM_RSTN = 1 << 1
AXI_DAC_RSTN = 1 << 0
AXI_DAC_REG_RATECNTRL = 0x04C + DAC_ADDR_OFFSET
AXI_DAC_REG_DATA_SELECT = lambda c: 0x418 + DAC_ADDR_OFFSET + c * 0x40
AXI_DAC_DATA_SEL_DMA = 2

AXI_ADC_REG_CONFIG = 0x044
AXI_ADC_R1_MODE = 1 << 2
AXI_DAC_REG_CONFIG = 0x048 + DAC_ADDR_OFFSET
AXI_DAC_R1_MODE = 1 << 5


def axi_ad9361_core_enable(ip, num_channels=1):
    import time

    ip.write(AXI_ADC_REG_RSTN, 0)
    ip.write(AXI_ADC_REG_RSTN, AXI_ADC_MMCM_RSTN | AXI_ADC_RSTN)
    ip.write(AXI_DAC_REG_RSTN, 0)
    ip.write(AXI_DAC_REG_RSTN, AXI_DAC_MMCM_RSTN | AXI_DAC_RSTN)
    time.sleep(0.1)

    ip.write(AXI_ADC_REG_CONFIG, AXI_ADC_R1_MODE)
    ip.write(AXI_DAC_REG_CONFIG, AXI_DAC_R1_MODE)
    for ch in range(num_channels):
        ip.write(AXI_ADC_REG_CHAN_CNTRL(ch),
                  AXI_ADC_FORMAT_SIGNEXT | AXI_ADC_FORMAT_ENABLE | AXI_ADC_ENABLE)
    for ch in range(num_channels):
        ip.write(AXI_DAC_REG_DATA_SELECT(ch), AXI_DAC_DATA_SEL_DMA)
    ip.write(AXI_DAC_REG_RATECNTRL, 0)


REG_RX_CLOCK_DATA_DELAY = 0x006
REG_TX_CLOCK_DATA_DELAY = 0x007
REG_PARALLEL_PORT_CONF_1 = 0x010
REG_PARALLEL_PORT_CONF_2 = 0x011
REG_PARALLEL_PORT_CONF_3 = 0x012
REG_LVDS_BIAS_CTRL = 0x03C
PP_CONF_1 = 0x80 | 0x40 | 0x08  # PP_TX_SWAP_IQ | PP_RX_SWAP_IQ | RX_FRAME_PULSE_MODE
PP_CONF_2 = 0x00
PP_CONF_3_LVDS_MODE = 0x10
RX_CLOCK_DATA_DELAY = 4
TX_CLOCK_DATA_DELAY = 7 << 4
LVDS_BIAS_CTRL = ((150 - 75) // 75) | (1 << 5)


def main():
    ol = Overlay("base/system_top.bit")
    ad = AD9361()

    try:
        ad.power_up_clocks()
        assert ad.set_bbpll_rate(983_040_000), "BBPLL did not lock"

        # Full bypass: no R2/R1/CLKRF decimation, no FIR decimation/
        # interpolation on either RX or TX.
        ad.set_clock_chain(r2_div=1, r1_div=1, clkrf_div=1, rx_fir_div=1,
                            t2_div=1, t1_div=1, clktf_div=1, tx_fir_div=1)

        REG_RX_ENABLE_FILTER_CTRL = 0x003
        REG_TX_ENABLE_FILTER_CTRL = 0x002
        rx_val = ad.reg_read(REG_RX_ENABLE_FILTER_CTRL)
        tx_val = ad.reg_read(REG_TX_ENABLE_FILTER_CTRL)
        print(f"REG_RX_ENABLE_FILTER_CTRL = 0x{rx_val:02X} "
              f"(FIR={rx_val & 0x3}, R2={(rx_val>>4)&0x3}, R1={(rx_val>>3)&1}, "
              f"CLKRF={(rx_val>>2)&1})")
        print(f"REG_TX_ENABLE_FILTER_CTRL = 0x{tx_val:02X} "
              f"(FIR={tx_val & 0x3}, T2={(tx_val>>4)&0x3}, T1={(tx_val>>3)&1}, "
              f"CLKTF={(tx_val>>2)&1})")

        assert ad.set_rx_lo_freq(700_000_000), "RX LO did not lock"
        assert ad.set_tx_lo_freq(700_000_000), "TX LO did not lock"
        print("AD9361 locked: BBPLL/RFPLL OK, full decimation bypass active")

        axi_ad9361_core_enable(ol.axi_ad9361, num_channels=2)
        print("axi_ad9361 core reset-deasserted, channels enabled, DMA data-select set")

        ad.reg_write(REG_PARALLEL_PORT_CONF_1, PP_CONF_1)
        ad.reg_write(REG_PARALLEL_PORT_CONF_2, PP_CONF_2)
        ad.reg_write(REG_PARALLEL_PORT_CONF_3, PP_CONF_3_LVDS_MODE)
        ad.reg_write(REG_RX_CLOCK_DATA_DELAY, RX_CLOCK_DATA_DELAY)
        ad.reg_write(REG_TX_CLOCK_DATA_DELAY, TX_CLOCK_DATA_DELAY)
        ad.reg_write(REG_LVDS_BIAS_CTRL, LVDS_BIAS_CTRL)
        print("AD9361 digital interface configured")

        ad.enable_rx1_tx1()
        ad.set_rx_manual_gain(RX_GAIN_INDEX)
        ad.set_tx_atten_db(TX_ATTEN_DB)

        ad.enable_fdd()
        assert ad.ensm_state() == 0xA, f"ENSM not FDD, got 0x{ad.ensm_state():X}"
        print(f"ENSM FDD OK, TX atten={TX_ATTEN_DB}dB, RX gain idx=0x{RX_GAIN_INDEX:02X}")

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

        print("Starting continuous TX...")
        tx_buf.flush()
        tx_dmac.transfer_from_memory(tx_buf.physical_address,
                                      tx_buf.nbytes, cyclic=True)

        print("Capturing RX...")
        ok = rx_dmac.transfer_to_memory(rx_buf.physical_address,
                                         rx_buf.nbytes)
        rx_buf.invalidate()
        print(f"RX transfer done={ok}")

        rx_i = rx_buf[0::2].astype(np.float64)
        rx_q = rx_buf[1::2].astype(np.float64)
        print(f"RX sample stats: I min/max/mean = "
              f"{rx_i.min():.0f}/{rx_i.max():.0f}/{rx_i.mean():.1f}, "
              f"Q min/max/mean = {rx_q.min():.0f}/{rx_q.max():.0f}/{rx_q.mean():.1f}")

        rx_complex = rx_i + 1j * rx_q
        spectrum = np.fft.fftshift(np.fft.fft(rx_complex))
        freqs = np.fft.fftshift(np.fft.fftfreq(N_SAMPLES, d=1 / SAMPLE_RATE))
        mag_db = 20 * np.log10(np.abs(spectrum) + 1e-9)

        # Report the single global peak, plus magnitude specifically near
        # the expected TX tone (+-50kHz) and near the suspected spur
        # (+-50kHz), since with no decimation both could be present as
        # distinct peaks rather than aliased onto each other.
        peak_idx = np.argmax(mag_db)
        print(f"\nGlobal peak at {freqs[peak_idx]/1e3:.1f} kHz, {mag_db[peak_idx]:.1f} dB")

        tone_mask = np.abs(freqs - TONE_FREQ) < 50e3
        if tone_mask.any():
            ti = np.argmax(mag_db * tone_mask - 1e9 * (~tone_mask))
            print(f"Near +{TONE_FREQ/1e3:.0f} kHz (TX tone): peak at "
                  f"{freqs[ti]/1e3:.1f} kHz, {mag_db[ti]:.1f} dB")

        for sign, label in [(1, "+"), (-1, "-")]:
            target = sign * SPUR_FREQ
            spur_mask = np.abs(freqs - target) < 50e3
            if spur_mask.any():
                si = np.argmax(mag_db * spur_mask - 1e9 * (~spur_mask))
                print(f"Near {label}{SPUR_FREQ/1e3:.0f} kHz (suspected spur): peak at "
                      f"{freqs[si]/1e3:.1f} kHz, {mag_db[si]:.1f} dB")

        # Top 5 peaks overall (simple local-max free-for-all on the dB
        # array, not true peak-picking, but enough to see if there are
        # two distinct strong components).
        order = np.argsort(mag_db)[::-1]
        print("\nTop 10 bins by magnitude:")
        shown = 0
        shown_freqs = []
        for idx in order:
            f = freqs[idx]
            if any(abs(f - sf) < 20e3 for sf in shown_freqs):
                continue
            print(f"  {f/1e3:9.1f} kHz  {mag_db[idx]:6.1f} dB")
            shown_freqs.append(f)
            shown += 1
            if shown >= 10:
                break

        np.save("/tmp/rx_capture_fullbypass.npy", rx_buf)
        with open("/tmp/rx_spectrum_fullbypass.csv", "w") as f:
            f.write("freq_hz,mag_db\n")
            for fr, m in zip(freqs, mag_db):
                f.write(f"{fr},{m}\n")
        print("\nSaved /tmp/rx_capture_fullbypass.npy and /tmp/rx_spectrum_fullbypass.csv")

        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
        t_us = np.arange(N_SAMPLES) / SAMPLE_RATE * 1e6
        n_show = 200
        ax1.plot(t_us[:n_show], rx_i[:n_show], label="I")
        ax1.plot(t_us[:n_show], rx_q[:n_show], label="Q")
        ax1.set_xlabel("Time (us)")
        ax1.set_ylabel("ADC code")
        ax1.set_title("RX captured I/Q (first 200 samples), full bypass @ 61.44 MSPS")
        ax1.legend()
        ax1.grid(True)

        ax2.plot(freqs / 1e6, mag_db)
        ax2.axvline(TONE_FREQ / 1e6, color="r", linestyle="--", alpha=0.6,
                    label=f"TX tone {TONE_FREQ/1e6:.2f} MHz")
        ax2.axvline(SPUR_FREQ / 1e6, color="g", linestyle="--", alpha=0.6,
                    label=f"suspected spur +{SPUR_FREQ/1e6:.2f} MHz")
        ax2.axvline(-SPUR_FREQ / 1e6, color="g", linestyle="--", alpha=0.6,
                    label=f"suspected spur -{SPUR_FREQ/1e6:.2f} MHz")
        ax2.set_xlabel("Frequency (MHz)")
        ax2.set_ylabel("Magnitude (dB)")
        ax2.set_title(f"RX spectrum, full decimation bypass, peak at "
                       f"{freqs[peak_idx]/1e3:.1f} kHz")
        ax2.legend(fontsize=8)
        ax2.grid(True)

        plt.tight_layout()
        plt.savefig("/tmp/tone_test_fullbypass.png", dpi=100)
        print("Saved /tmp/tone_test_fullbypass.png")

        tx_dmac.stop()
    finally:
        ad.close()


if __name__ == "__main__":
    main()
