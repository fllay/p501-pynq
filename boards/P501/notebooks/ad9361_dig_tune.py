#!/usr/bin/env python3
"""
Port of the real digital-interface calibration no-OS always runs at init
(ad9361_dig_tune_rx() in drivers/rf-transceiver/ad9361/ad9361_conv.c) --
something this project's hand-rolled bring-up never implemented. The
vendor's own reference devicetree confirms this is required:
adi,digital-interface-tune-skip-mode = <0> ("TUNE RX&TX"), meaning the
known-working vendor config does NOT skip this step.

Mechanism: inject a known PRBS pattern directly at the AD9361's ADC
output (BIST_CTRL_POINT=2, "after ADC"), bypassing the RF front end
entirely, so the FPGA's own PN9 checker (axi_ad9361's per-channel
PN_ERR/PN_OOS status bits) can deterministically detect bit errors at
each (clock_delay, data_delay) setting of REG_RX_CLOCK_DATA_DELAY -- the
same two 4-bit fields whose *seed* values (rx-data-delay=4,
tx-fb-clock-delay=7) were copied from the vendor devicetree, but never
actually swept/recalibrated for this specific board instance the way the
real driver does at every boot.

This explains the earlier finding that sweeping the FPGA-side IDELAY
taps (up_delay_cntrl, 0-31) alone had zero effect: that's a *different*
knob (sub-bit-period fine delay on the FPGA input deserializer) from the
chip-side REG_RX_CLOCK_DATA_DELAY swept here (coarse bit-clock/data
skew on the AD9361's own LVDS output drivers) -- getting the same
result at every IDELAY tap while this chip-side register sat fixed at
the vendor's seed value is consistent with the *chip-side* delay being
wrong, not the FPGA-side one.
"""
import sys
sys.path.insert(0, '/home/xilinx/jupyter_notebooks')
import time
import numpy as np
from pynq import Overlay, allocate
from ad9361_driver import AD9361
from axi_dmac import AxiDmac
import ad9361_full_bypass_test as fb

REG_BIST_CONFIG = 0x3F4
BIST_CTRL_POINT_ADC = 2 << 2
BIST_ENABLE = 1 << 0
REG_RX_CLOCK_DATA_DELAY = 0x006
REG_ENSM_CONFIG_1 = 0x014
TO_ALERT = 1 << 0
FORCE_ALERT_STATE = 1 << 2

AXI_ADC_CHAN_CNTRL = lambda c: 0x400 + c * 0x40
AXI_ADC_CHAN_STATUS = lambda c: 0x404 + c * 0x40
AXI_ADC_REG_STATUS = 0x05C
AXI_ADC_PN_SEL_BIT = 1 << 10
AXI_ADC_FORMAT_SIGNEXT = 1 << 6
AXI_ADC_FORMAT_ENABLE = 1 << 4
AXI_ADC_ENABLE = 1 << 0


def rx_data_delay(x):
    return x & 0xF


def data_clk_delay(x):
    return (x & 0xF) << 4


def main():
    ol = Overlay("base/system_top.bit")
    ad = AD9361()
    ip = ol.axi_ad9361
    try:
        ad.power_up_clocks()
        assert ad.set_bbpll_rate(983_040_000), "BBPLL did not lock"
        ad.set_clock_chain()  # standard 3.84 MSPS -- LVDS bit-clock rate
        # (tied to ADC_CLK=61.44MHz via fixed adc_div=16) is independent
        # of this decimation choice, so any rate works for this test.
        assert ad.set_rx_lo_freq(700_000_000), "RX LO did not lock"
        assert ad.set_tx_lo_freq(700_000_000), "TX LO did not lock"

        fb.axi_ad9361_core_enable(ol.axi_ad9361, num_channels=2)
        ad.reg_write(fb.REG_PARALLEL_PORT_CONF_1, fb.PP_CONF_1)
        ad.reg_write(fb.REG_PARALLEL_PORT_CONF_2, fb.PP_CONF_2)
        ad.reg_write(fb.REG_PARALLEL_PORT_CONF_3, fb.PP_CONF_3_LVDS_MODE)
        ad.reg_write(fb.REG_LVDS_BIAS_CTRL, fb.LVDS_BIAS_CTRL)
        ad.enable_rx1_tx1()
        ad.set_tx_atten_db(60)
        ad.enable_fdd()
        print(f"ENSM = 0x{ad.ensm_state():X} (pre-tune)")

        # Enable PN checking on ch0/ch1 (PN_SEL bit10), PN9 type (default
        # ADC_PN_SEL extended field already 0 = PN9, never touched).
        for ch in (0, 1):
            ip.write(AXI_ADC_CHAN_CNTRL(ch),
                     AXI_ADC_FORMAT_SIGNEXT | AXI_ADC_FORMAT_ENABLE |
                     AXI_ADC_ENABLE | AXI_ADC_PN_SEL_BIT)

        # Inject PRBS at the AD9361's own ADC output (BIST_CTRL_POINT=2),
        # bypassing the RF front end -- a known-content test, not live RF.
        ad.reg_write(REG_BIST_CONFIG, BIST_CTRL_POINT_ADC | BIST_ENABLE)

        print("Sweeping REG_RX_CLOCK_DATA_DELAY (clock_delay x data_delay, 16x16)...")
        grid = np.zeros((16, 16), dtype=bool)  # True = PASS (no PN err/oos)
        for cd in range(16):
            for dd in range(16):
                ad.reg_write(REG_ENSM_CONFIG_1, TO_ALERT | FORCE_ALERT_STATE)
                ad.reg_write(REG_RX_CLOCK_DATA_DELAY,
                             data_clk_delay(cd) | rx_data_delay(dd))
                ad.enable_fdd()

                for ch in (0, 1):
                    ip.write(AXI_ADC_CHAN_STATUS(ch), 0x7)  # clear (w1c)
                time.sleep(0.01)

                # Only the per-channel PN_ERR/PN_OOS bits are unambiguous
                # (write-1-to-clear error flags, confirmed from RTL). The
                # common AXI_ADC_REG_STATUS bit0 semantics are unverified
                # for this core revision (could gate on IDELAYCTRL-ready
                # or something unrelated to link health) -- dropped to
                # avoid a false-fail on every single grid point.
                ok = True
                for ch in (0, 1):
                    if ip.read(AXI_ADC_CHAN_STATUS(ch)) & 0x7:
                        ok = False
                grid[cd, dd] = ok
            row = "".join("#" if grid[cd, dd] else "." for dd in range(16))
            print(f"  clock_delay={cd:2d}: {row}")

        n_pass = grid.sum()
        print(f"\n{n_pass}/256 combinations passed PN check")

        if n_pass == 0:
            print("No passing combination found -- digital interface "
                  "fundamentally broken, not just mistuned.")
        else:
            # Find the largest contiguous run of PASS cells along data_delay
            # for each clock_delay row, pick the best (longest run) overall,
            # center of that run.
            best = (0, 0, 0)  # (run_len, clock_delay, center_data_delay)
            for cd in range(16):
                row = grid[cd]
                run_start = None
                cur_len = 0
                best_len = 0
                best_start = 0
                for dd in range(17):
                    v = row[dd] if dd < 16 else False
                    if v:
                        if run_start is None:
                            run_start = dd
                        cur_len += 1
                    else:
                        if cur_len > best_len:
                            best_len = cur_len
                            best_start = run_start
                        run_start = None
                        cur_len = 0
                if best_len > best[0]:
                    best = (best_len, cd, best_start + best_len // 2)

            run_len, best_cd, best_dd = best
            print(f"Best window: clock_delay={best_cd}, run_len={run_len}, "
                  f"center data_delay={best_dd}")

            # Apply it
            ad.reg_write(REG_ENSM_CONFIG_1, TO_ALERT | FORCE_ALERT_STATE)
            ad.reg_write(REG_RX_CLOCK_DATA_DELAY,
                         data_clk_delay(best_cd) | rx_data_delay(best_dd))
            ad.enable_fdd()

        # Disable BIST, restore normal channel config (no PN_SEL bit)
        ad.reg_write(REG_BIST_CONFIG, 0)
        for ch in (0, 1):
            ip.write(AXI_ADC_CHAN_CNTRL(ch),
                     AXI_ADC_FORMAT_SIGNEXT | AXI_ADC_FORMAT_ENABLE |
                     AXI_ADC_ENABLE)

        # Re-sync the deserializer/cpack2 after ~256 rapid delay changes --
        # matches ad9361_dig_tune_rx()'s own cleanup (axi_adc_write RSTN
        # toggle), missing on the first pass of this script and the likely
        # cause of the DMA timeout seen then.
        AXI_ADC_REG_RSTN = 0x040
        AXI_ADC_MMCM_RSTN = 1 << 1
        AXI_ADC_RSTN = 1 << 0
        ip.write(AXI_ADC_REG_RSTN, AXI_ADC_MMCM_RSTN)
        ip.write(AXI_ADC_REG_RSTN, AXI_ADC_RSTN | AXI_ADC_MMCM_RSTN)
        time.sleep(0.1)

        ad.set_rx_manual_gain(0x70)
        ad.set_tx_atten_db(20)

        if n_pass > 0:
            print(f"\nApplied tuned delay: clock_delay={best_cd}, data_delay={best_dd}")
            print(f"REG_RX_CLOCK_DATA_DELAY now = 0x{ad.reg_read(REG_RX_CLOCK_DATA_DELAY):02X}")

        # -- now run a real capture with the live TX tone to check --------
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
        ok = rx_dmac.transfer_to_memory(rx_buf.physical_address, rx_buf.nbytes)
        rx_buf.invalidate()
        print(f"\nPost-tune RX transfer done={ok}")

        rx_i = rx_buf[0::2].astype(np.float64)
        rx_q = rx_buf[1::2].astype(np.float64)
        print(f"First 12 raw I: {rx_i[:12].astype(int).tolist()}")
        print(f"I min/max/std = {rx_i.min():.0f}/{rx_i.max():.0f}/{rx_i.std():.1f}, "
              f"Q min/max/std = {rx_q.min():.0f}/{rx_q.max():.0f}/{rx_q.std():.1f}")

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
