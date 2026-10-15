#!/usr/bin/env python3
"""
Full RF test: TX a 1MHz tone at 700MHz center frequency, capture on RX,
plot the spectrum. Saves the plot to tone_test.png since this is driven
non-interactively over SSH, not a live notebook kernel.

Sample format assumption (untested going in -- first real use of the DMA
engines, interpret the captured waveform to confirm/correct): 16-bit
signed I/Q interleaved for channel 0 (I0, Q0, I0, Q0, ...), 4 bytes/sample.
If the FFT looks wrong (e.g. mirrored, wrong scale, or channel-interleaved
garbage), that's the first thing to revisit.
"""
import numpy as np

from pynq import Overlay, allocate

from ad9361_driver import AD9361
from axi_dmac import AxiDmac

SAMPLE_RATE = 3_840_000  # back to the original, most-tested rate for this
# R1-vs-R2 diagnostic (isolate one variable at a time)
TONE_FREQ = 1_000_000
N_SAMPLES = 8192
TX_ATTEN_DB = 20  # raised power (user now using real antennas, not a bare
# close-range test) -- still well short of 0dB (max power) for a first try.
RX_GAIN_INDEX = 0x70  # raised from the earlier conservative 0x58 -- higher
# index = higher gain in AD9361's standard gain table convention.


AXI_ADC_REG_RSTN = 0x040
AXI_ADC_MMCM_RSTN = 1 << 1
AXI_ADC_RSTN = 1 << 0
AXI_ADC_REG_CHAN_CNTRL = lambda c: 0x400 + c * 0x40
AXI_ADC_FORMAT_SIGNEXT = 1 << 6
AXI_ADC_FORMAT_ENABLE = 1 << 4
AXI_ADC_ENABLE = 1 << 0

# DAC-side COMMON_ID is 0x10 (common block) / 0x11 (channel block) in our
# own RTL (library/common/up_dac_common.v / up_dac_channel.v), NOT the
# same address range as the ADC side (0x00/0x01) as originally assumed --
# confirmed by reading the RTL parameter defaults directly. Each DAC
# register sits at exactly ADC_offset + 0x4000 (COMMON_ID=0x10 means the
# whole common block's word-address base is 0x10*256=0x1000 -> byte
# 0x4000; same +0x4000 relationship holds for the channel block's
# 0x11 vs the ADC channel block's 0x01). This was a real, previously
# undiscovered bug: DAC_REG_CONFIG writes at byte 0x048 were silently
# landing on the ADC side's EXT_SYNC control register instead (up_waddr
# 7'h12 in up_adc_common.v), and the DAC core was NEVER actually taken
# out of reset, meaning TX was never transmitting anything at all.
DAC_ADDR_OFFSET = 0x4000
AXI_DAC_REG_RSTN = 0x040 + DAC_ADDR_OFFSET
AXI_DAC_MMCM_RSTN = 1 << 1
AXI_DAC_RSTN = 1 << 0
AXI_DAC_REG_RATECNTRL = 0x04C + DAC_ADDR_OFFSET
AXI_DAC_REG_DATA_SELECT = lambda c: 0x418 + DAC_ADDR_OFFSET + c * 0x40
AXI_DAC_DATA_SEL_DMA = 2


AXI_ADC_REG_CONFIG = 0x044  # up_adc_common.v: up_waddr 7'h11 * 4
AXI_ADC_R1_MODE = 1 << 2
AXI_DAC_REG_CONFIG = 0x048 + DAC_ADDR_OFFSET  # up_dac_common.v: up_waddr 7'h12 * 4
AXI_DAC_R1_MODE = 1 << 5


def axi_ad9361_core_enable(ip, num_channels=1):
    """Reset-deassert + per-channel enable for the axi_ad9361 core itself
    -- completely separate from the AD9361 chip's own SPI registers.
    Ported from ADI no-OS's axi_adc_core/axi_dac_core init (generic
    data-converter core pattern), with DAC-side addresses corrected
    per the DAC_ADDR_OFFSET note above. The 100ms delay after each
    reset-deassert matches axi_adc_init()/axi_dac_init() in no-OS
    (MMCM/PLL lock time) -- also missing on earlier passes."""
    import time

    ip.write(AXI_ADC_REG_RSTN, 0)
    ip.write(AXI_ADC_REG_RSTN, AXI_ADC_MMCM_RSTN | AXI_ADC_RSTN)
    ip.write(AXI_DAC_REG_RSTN, 0)
    ip.write(AXI_DAC_REG_RSTN, AXI_DAC_MMCM_RSTN | AXI_DAC_RSTN)
    time.sleep(0.1)

    # R1 mode (TX1/RX1 only, per up_adc_r1_mode / up_dac_r1_mode in
    # library/common/up_adc_common.v & up_dac_common.v) -- simplifies from
    # the vendor devicetree's 2T2R (adi,2rx-2tx-mode-enable) to a single
    # channel, matching ad9361_driver.py's set_clock_chain() which only
    # configures one RX/TX sample-rate chain anyway. Diagnostic: back to
    # R2 mode (0) to match the vendor's fully-validated reference exactly,
    # ruling in/out whether the fs/3 artifact came from the R1
    # simplification itself.
    ip.write(AXI_ADC_REG_CONFIG, AXI_ADC_R1_MODE)
    ip.write(AXI_DAC_REG_CONFIG, AXI_DAC_R1_MODE)
    for ch in range(num_channels):
        ip.write(AXI_ADC_REG_CHAN_CNTRL(ch),
                  AXI_ADC_FORMAT_SIGNEXT | AXI_ADC_FORMAT_ENABLE | AXI_ADC_ENABLE)

    for ch in range(num_channels):
        ip.write(AXI_DAC_REG_DATA_SELECT(ch), AXI_DAC_DATA_SEL_DMA)

    # Explicitly set (confirmed already 0 by default on this core, so this
    # is a no-op -- included for completeness/clarity rather than relying
    # on an unwritten register). rate=0 means no additional AXI-stream-to-
    # DAC-sample rate scaling beyond what AD9361's own internal FIR
    # interpolation (already configured via set_clock_chain()) provides.
    ip.write(AXI_DAC_REG_RATECNTRL, 0)


REG_RX_CLOCK_DATA_DELAY = 0x006
REG_TX_CLOCK_DATA_DELAY = 0x007
REG_PARALLEL_PORT_CONF_1 = 0x010
REG_PARALLEL_PORT_CONF_2 = 0x011
REG_PARALLEL_PORT_CONF_3 = 0x012
REG_LVDS_BIAS_CTRL = 0x03C
# All values below taken from the vendor's own reference devicetree for
# this exact HDL/axi_ad9361 pairing (inherited ZCU102+FMCOMMS2/3 overlay,
# meta-adi/.../pl-zynqmp-zcu102-rev10-ad9361-fmcomms2-3-overlay.dtsi) --
# NOT guessed. The digital-interface protocol bits (swap/frame-pulse/LVDS/
# 2R2T) are generic to this axi_ad9361 IP, not board-specific pin wiring,
# so they transfer directly; rx-data-delay/tx-fb-clock-delay are the
# board's own tuned timing-skew values.
# PP_TX_SWAP_IQ | PP_RX_SWAP_IQ | RX_FRAME_PULSE_MODE | R2T2_TIMING
# (R2T2_TIMING re-added -- back to full 2T2R/R2 mode for this diagnostic,
# matching the vendor's fully-validated reference devicetree exactly)
PP_CONF_1 = 0x80 | 0x40 | 0x08  # PP_TX_SWAP_IQ | PP_RX_SWAP_IQ |
# RX_FRAME_PULSE_MODE -- tried removing RX_FRAME_PULSE_MODE as a
# frame-misalignment diagnostic; that broke capture entirely (timeout,
# not just wrong data), confirming it's genuinely required. Restored.
PP_CONF_2 = 0x00
PP_CONF_3_LVDS_MODE = 0x10  # LVDS_MODE bit only (full duplex, dual port)
RX_CLOCK_DATA_DELAY = 4  # adi,rx-data-delay = <4>
TX_CLOCK_DATA_DELAY = 7 << 4  # adi,tx-fb-clock-delay = <7> -> FB_CLK_DELAY field
LVDS_BIAS_CTRL = ((150 - 75) // 75) | (1 << 5)  # adi,lvds-bias-mV=150,
# adi,lvds-rx-onchip-termination-enable -- without this, enabling
# LVDS_MODE alone broke the digital interface clock entirely (DMA timeout
# instead of completing with garbage data) -- this termination bit is
# almost certainly why.


def main():
    ol = Overlay("base/system_top.bit")
    ad = AD9361()

    try:
        ad.power_up_clocks()
        assert ad.set_bbpll_rate(983_040_000), "BBPLL did not lock"
        ad.set_clock_chain()  # defaults -> 3.84 MSPS
        assert ad.set_rx_lo_freq(700_000_000), "RX LO did not lock"
        assert ad.set_tx_lo_freq(700_000_000), "TX LO did not lock"
        print("AD9361 locked: BBPLL/RFPLL OK")

        # num_channels=2: axi_ad9361_rx.v/tx.v instantiate 4 channels
        # (CHANNEL_ID 0-3), likely I/Q for RX1 pair + I/Q for RX2 pair --
        # R1 mode may only affect the dual-receiver TIME-MULTIPLEX/framing
        # logic, not eliminate the need to individually enable each of the
        # I and Q sub-channels. Only channel 0 was ever configured before;
        # channel 1 (likely Q) may have been sitting in its default (DDS)
        # data-select mode this whole time -- a strong candidate for the
        # fixed period-3 pattern seen in the raw capture (constant,
        # non-varying values completely independent of the real TX buffer
        # content or chosen sample rate).
        axi_ad9361_core_enable(ol.axi_ad9361, num_channels=2)  # R1 mode: I+Q
        print("axi_ad9361 core reset-deasserted, channels enabled, DMA data-select set")

        # Digital interface config must happen BEFORE the ENSM->FDD
        # transition -- matches the reference driver's own ordering
        # (ad9361_pp_port_setup() runs before the final
        # ad9361_ensm_set_state(FDD) in ad9361_init()). Previously had
        # this after enable_fdd(); reordering to test if sequencing was
        # part of why RX capture reads back all zero.
        ad.reg_write(REG_PARALLEL_PORT_CONF_1, PP_CONF_1)
        ad.reg_write(REG_PARALLEL_PORT_CONF_2, PP_CONF_2)
        ad.reg_write(REG_PARALLEL_PORT_CONF_3, PP_CONF_3_LVDS_MODE)
        ad.reg_write(REG_RX_CLOCK_DATA_DELAY, RX_CLOCK_DATA_DELAY)
        ad.reg_write(REG_TX_CLOCK_DATA_DELAY, TX_CLOCK_DATA_DELAY)
        ad.reg_write(REG_LVDS_BIAS_CTRL, LVDS_BIAS_CTRL)
        print("AD9361 digital interface (LVDS, vendor-devicetree-matched) configured")

        ad.enable_rx1_tx1()
        ad.set_rx_manual_gain(RX_GAIN_INDEX)
        ad.set_tx_atten_db(TX_ATTEN_DB)

        ad.enable_fdd()
        assert ad.ensm_state() == 0xA, f"ENSM not FDD, got 0x{ad.ensm_state():X}"
        print(f"ENSM FDD OK, TX atten={TX_ATTEN_DB}dB, RX gain idx=0x{RX_GAIN_INDEX:02X}")

        # -- build TX tone ---------------------------------------------
        t = np.arange(N_SAMPLES) / SAMPLE_RATE
        tone = np.exp(2j * np.pi * TONE_FREQ * t)
        scale = (2**15 - 1) * 0.5  # back off from full scale
        i_samples = (np.real(tone) * scale).astype(np.int16)
        q_samples = (np.imag(tone) * scale).astype(np.int16)

        tx_buf = allocate(shape=(N_SAMPLES * 2,), dtype=np.int16)
        tx_buf[0::2] = i_samples
        tx_buf[1::2] = q_samples

        rx_buf = allocate(shape=(N_SAMPLES * 2,), dtype=np.int16)

        tx_dmac = AxiDmac(ol.axi_ad9361_dac_dma)
        rx_dmac = AxiDmac(ol.axi_ad9361_adc_dma)

        print("Starting continuous TX...")
        tx_buf.flush()  # CPU-written data -> DRAM, visible to the DMA engine
        tx_dmac.transfer_from_memory(tx_buf.physical_address,
                                      tx_buf.nbytes, cyclic=True)

        print("Capturing RX...")
        ok = rx_dmac.transfer_to_memory(rx_buf.physical_address,
                                         rx_buf.nbytes)
        rx_buf.invalidate()  # DMA-written DRAM -> CPU cache, else stale/zero reads
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

        peak_idx = np.argmax(mag_db)
        print(f"Peak at {freqs[peak_idx]/1e3:.1f} kHz, {mag_db[peak_idx]:.1f} dB "
              f"(expect near +{TONE_FREQ/1e3:.0f} kHz)")

        np.save("/tmp/rx_capture.npy", rx_buf)
        with open("/tmp/rx_spectrum.csv", "w") as f:
            f.write("freq_hz,mag_db\n")
            for fr, m in zip(freqs, mag_db):
                f.write(f"{fr},{m}\n")
        print("Saved /tmp/rx_capture.npy and /tmp/rx_spectrum.csv")

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
        ax1.set_title("RX captured I/Q (first 200 samples)")
        ax1.legend()
        ax1.grid(True)

        ax2.plot(freqs / 1e3, mag_db)
        ax2.axvline(TONE_FREQ / 1e3, color="r", linestyle="--", alpha=0.5,
                    label=f"expected {TONE_FREQ/1e3:.0f} kHz")
        ax2.set_xlabel("Frequency (kHz)")
        ax2.set_ylabel("Magnitude (dB)")
        ax2.set_title(f"RX spectrum (TX 1MHz tone @ 700MHz center, "
                       f"peak at {freqs[peak_idx]/1e3:.1f} kHz)")
        ax2.legend()
        ax2.grid(True)

        plt.tight_layout()
        plt.savefig("/tmp/tone_test.png", dpi=100)
        print("Saved /tmp/tone_test.png")

        tx_dmac.stop()
    finally:
        ad.close()


if __name__ == "__main__":
    main()
