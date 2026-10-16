#!/usr/bin/env python3
"""2R2T (2x2 MIMO) bring-up and test; sample-rate presets and LO list.

usage: [LOS=433,700,915] [VERBOSE=1] ad9361_mimo_test.py [3.84|30.72|61.44]

2R2T differences vs the 1R1T flow:
  * AD9361: both RX and both TX channels enabled, R2T2_TIMING in PP_CONF_1,
    RX2 manual gain (0x10C), TX2 attenuation (0x075/0x076), TX quad cal must
    converge for TX2 as well.
  * axi_ad9361: R1_MODE cleared (ADC + DAC), 4 channels enabled, l_clk = 4 x fs,
    so DAC DATARATE = l_clk/fs - 1 = 3.
  * DMA word order per sample: I1, Q1, I2, Q2 (16 bit each).

Per LO the script prints one summary line:
  A: chip-internal digital loopback (a different tone per TX channel) - checks
     channel mapping / DMA layout / R2 mode independent of the antennas;
  B: RF TX1/TX2 -> RX1/RX2 coupling matrix (depends on antennas) and noise floors.
"""
import os
import sys
sys.path.insert(0, '/home/xilinx/jupyter_notebooks')
import numpy as np
from pynq import Overlay, allocate

from ad9361_driver import AD9361, R2T2_TIMING
from axi_dmac import AxiDmac
import ad9361_full_bypass_test as fb
import ad9361_rate_test as rt          # rate presets (reads sys.argv[1] = rate)
from ad9361_fir_test import make_fir_coef

CFG = rt.CFG
FS = CFG["fs"]
N = rt.N
LOS = [int(float(x) * 1e6) for x in os.environ.get("LOS", "700").split(",")]
VERBOSE = bool(os.environ.get("VERBOSE"))
TX_ATTEN_DB = 30
RX_GAIN_INDEX = 40
DATARATE = 3
FIR_TX1_2, FIR_RX1_2 = 0x03, 0x83
OBS = 0x3F5


def bring_up(ol, lo):
    ad = AD9361()
    ad.power_up_clocks()
    assert ad.set_bbpll_rate(983_040_000), "BBPLL did not lock"
    ad.set_clock_chain(adc_div=CFG["adc_div"], **CFG["chain"])
    ad.synth_cp_calib(False)
    ad.synth_cp_calib(True)
    rx_ok = ad.set_rx_lo_freq(lo)
    tx_ok = ad.set_tx_lo_freq(lo)
    ad.enable_rx2tx2()
    fb.axi_ad9361_core_enable(ol.axi_ad9361, num_channels=4, dac_datarate=DATARATE,
                              r1_mode=False)
    ad.reg_write(fb.REG_PARALLEL_PORT_CONF_1, fb.PP_CONF_1 | R2T2_TIMING)
    ad.reg_write(fb.REG_PARALLEL_PORT_CONF_2, fb.PP_CONF_2)
    ad.reg_write(fb.REG_PARALLEL_PORT_CONF_3, fb.PP_CONF_3_LVDS_MODE)
    ad.reg_write(fb.REG_RX_CLOCK_DATA_DELAY, fb.RX_CLOCK_DATA_DELAY)
    ad.reg_write(fb.REG_TX_CLOCK_DATA_DELAY, fb.TX_CLOCK_DATA_DELAY)
    ad.reg_write(fb.REG_LVDS_BIAS_CTRL, fb.LVDS_BIAS_CTRL)
    res = ad.analog_init(rf_rx_bw_hz=CFG["bw"], rf_tx_bw_hz=CFG["bw"],
                         adc_clk_hz=CFG["adc_clk"], rx_freq_hz=lo,
                         clkrf_hz=CFG["clk"], clktf_hz=CFG["clk"], rx2tx2=True)
    ad.set_rx_manual_gain(RX_GAIN_INDEX, channels=(1, 2))
    ad.set_tx_atten_db(TX_ATTEN_DB, channels=(1, 2))
    ad.enable_fdd()
    if CFG["fir"]:
        coef = make_fir_coef()
        ad.load_fir_filter_coef(FIR_RX1_2, 0, coef)
        ad.load_fir_filter_coef(FIR_TX1_2, 0, coef)
    fb.axi_ad9361_core_enable(ol.axi_ad9361, num_channels=4, dac_datarate=DATARATE,
                              r1_mode=False)
    return ad, rx_ok, tx_ok, res


def tone(f_hz, amp=0.5):
    cyc = round(f_hz * N / FS)
    t = np.exp(2j * np.pi * cyc * np.arange(N) / N) * amp * (2**15 - 1)
    return t, cyc * FS / N


def tx_buffer(t1, t2):
    tx = allocate(shape=(N * 4,), dtype=np.int16)
    for k, t in enumerate((t1, t2)):
        if t is None:
            tx[2 * k::4] = 0
            tx[2 * k + 1::4] = 0
        else:
            tx[2 * k::4] = t.real.astype(np.int16)
            tx[2 * k + 1::4] = t.imag.astype(np.int16)
    tx.flush()
    return tx


def spectra(raw):
    out = []
    for k in (0, 1):
        x = raw[2 * k::4].astype(float) + 1j * raw[2 * k + 1::4].astype(float)
        x -= x.mean()
        out.append(20 * np.log10(np.abs(np.fft.fftshift(np.fft.fft(x * np.hanning(N)))) + 1e-9))
    return out


def main():
    ol = Overlay("base/system_top.bit")
    txd = AxiDmac(ol.axi_ad9361_dac_dma)
    rxd = AxiDmac(ol.axi_ad9361_adc_dma)
    freqs = np.fft.fftshift(np.fft.fftfreq(N, d=1 / FS))
    w = max(25e3, FS / 512)
    t1, f1 = tone(CFG["tones"][0])
    t2, f2 = tone(-CFG["tones"][0] / 2)
    lc = {3.84: 0x2752, 30.72: 0x13A93 // 2, 61.44: 0x13A93}[RATE_KEY]

    def band(m, f0):
        return float(m[np.abs(freqs - f0) < w].max())

    def capture(a, b):
        tx = tx_buffer(a, b)
        rx = allocate(shape=(N * 4,), dtype=np.int16)
        if a is not None or b is not None:
            txd.transfer_from_memory(tx.physical_address, tx.nbytes, cyclic=True)
        ok = rxd.transfer_to_memory(rx.physical_address, rx.nbytes)
        rx.invalidate()
        if a is not None or b is not None:
            txd.stop()
        return ok, spectra(np.array(rx))

    print(f"2R2T {FS/1e6:.2f} MSPS (l_clk {4*FS/1e6:.2f} MHz), TX1 tone {f1/1e6:+.3f} MHz, "
          f"TX2 tone {f2/1e6:+.3f} MHz, TX atten {TX_ATTEN_DB} dB, RX gain idx {RX_GAIN_INDEX}")
    for lo in LOS:
        ad = None
        try:
            ad, rx_ok, tx_ok, res = bring_up(ol, lo)
            lcnt = ol.axi_ad9361.read(0x54)
            idle_ok, (i1, i2) = capture(None, None)
            ad.reg_write(OBS, ad.reg_read(OBS) | 1)
            ok_l, (l1, l2) = capture(t1, t2)
            ad.reg_write(OBS, ad.reg_read(OBS) & ~1)
            iso1 = band(l1, f1) - band(l1, f2)
            iso2 = band(l2, f2) - band(l2, f1)
            ok_a, (a1, a2) = capture(t1, None)
            ok_b, (b1, b2) = capture(None, t2)
            m11, m12, m21, m22 = band(a1, f1), band(a2, f1), band(b1, f2), band(b2, f2)
            fl1, fl2 = float(np.median(i1)), float(np.median(i2))
            digital_ok = ok_l and iso1 > 60 and iso2 > 60
            rf_ok = ok_a and ok_b and max(m11, m12) > max(fl1, fl2) + 25 and \
                max(m21, m22) > max(fl1, fl2) + 25
            quad = res.get("tx_quad")
            print(f"LO {lo/1e6:7.1f} MHz: locks={rx_ok},{tx_ok} quad={quad} lclk=0x{lcnt:X} | "
                  f"loopback iso RX1 {iso1:5.1f} RX2 {iso2:5.1f} dB {'OK' if digital_ok else 'FAIL'} | "
                  f"RF TX1->RX1 {m11:5.1f} TX1->RX2 {m12:5.1f} TX2->RX1 {m21:5.1f} TX2->RX2 {m22:5.1f} "
                  f"floor {fl1:4.1f}/{fl2:4.1f} {'OK' if rf_ok else 'WEAK'}")
            if VERBOSE:
                print("   cal:", res)
        except Exception as e:
            print(f"LO {lo/1e6:7.1f} MHz: FAILED: {type(e).__name__}: {e}")
        finally:
            if ad is not None:
                ad.close()


RATE_KEY = rt.RATE

if __name__ == "__main__":
    main()
