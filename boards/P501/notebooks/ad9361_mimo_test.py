#!/usr/bin/env python3
"""2R2T (2x2 MIMO) bring-up and test at 3.84 MSPS.

2R2T differences vs the 1R1T flow:
  * AD9361: both RX and both TX channels enabled, R2T2_TIMING in PP_CONF_1,
    RX2 manual gain (0x10C), TX2 attenuation (0x075/0x076), TX quad cal must
    converge for TX2 as well.
  * axi_ad9361: R1_MODE cleared (ADC + DAC), 4 channels enabled, l_clk = 4 x fs,
    so DAC DATARATE = l_clk/fs - 1 = 3.
  * DMA word order per sample: I1, Q1, I2, Q2 (16 bit each).

Stage A  chip-internal digital data-port loopback (no analog): a different tone
         on each TX channel; checks channel mapping / DMA layout / R2 mode.
Stage B  RF: TX1 only, TX2 only, both; which RX sees what (depends on antennas).
usage: ad9361_mimo_test.py [datarate]
"""
import sys
sys.path.insert(0, '/home/xilinx/jupyter_notebooks')
import numpy as np
from pynq import Overlay, allocate

from ad9361_driver import AD9361, R2T2_TIMING
from axi_dmac import AxiDmac
import ad9361_full_bypass_test as fb
from ad9361_fir_test import make_fir_coef

FS = 3_840_000
N = 8192
LO = 700_000_000
RF_BW = 4_000_000
TX_ATTEN_DB = 10
RX_GAIN_INDEX = 45
DATARATE = int(sys.argv[1]) if len(sys.argv) > 1 else 3
FIR_TX1_2, FIR_RX1_2 = 0x03, 0x83
OBS = 0x3F5


def bring_up(ol, datarate=DATARATE):
    ad = AD9361()
    ad.power_up_clocks()
    assert ad.set_bbpll_rate(983_040_000), "BBPLL did not lock"
    ad.set_clock_chain()
    ad.synth_cp_calib(False)
    ad.synth_cp_calib(True)
    rx_ok = ad.set_rx_lo_freq(LO)
    tx_ok = ad.set_tx_lo_freq(LO)
    ad.enable_rx2tx2()
    fb.axi_ad9361_core_enable(ol.axi_ad9361, num_channels=4, dac_datarate=datarate,
                              r1_mode=False)
    ad.reg_write(fb.REG_PARALLEL_PORT_CONF_1, fb.PP_CONF_1 | R2T2_TIMING)
    ad.reg_write(fb.REG_PARALLEL_PORT_CONF_2, fb.PP_CONF_2)
    ad.reg_write(fb.REG_PARALLEL_PORT_CONF_3, fb.PP_CONF_3_LVDS_MODE)
    ad.reg_write(fb.REG_RX_CLOCK_DATA_DELAY, fb.RX_CLOCK_DATA_DELAY)
    ad.reg_write(fb.REG_TX_CLOCK_DATA_DELAY, fb.TX_CLOCK_DATA_DELAY)
    ad.reg_write(fb.REG_LVDS_BIAS_CTRL, fb.LVDS_BIAS_CTRL)
    res = ad.analog_init(rf_rx_bw_hz=RF_BW, rf_tx_bw_hz=RF_BW, rx2tx2=True)
    ad.set_rx_manual_gain(RX_GAIN_INDEX, channels=(1, 2))
    ad.set_tx_atten_db(TX_ATTEN_DB, channels=(1, 2))
    ad.enable_fdd()
    coef = make_fir_coef()
    ad.load_fir_filter_coef(FIR_RX1_2, 0, coef)
    ad.load_fir_filter_coef(FIR_TX1_2, 0, coef)
    fb.axi_ad9361_core_enable(ol.axi_ad9361, num_channels=4, dac_datarate=datarate,
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
    ad, rx_ok, tx_ok, res = bring_up(ol)
    try:
        print(f"2R2T {FS/1e6:.2f} MSPS LO {LO/1e6:.0f} MHz datarate={DATARATE}: locks={rx_ok},{tx_ok} "
              f"cal={res} ensm=0x{ad.ensm_state():X}")
        print(f"  l_clk count: ADC 0x{ol.axi_ad9361.read(0x54):X} DAC 0x{ol.axi_ad9361.read(0x4054):X} "
              f"(expect 4*fs = 15.36 MHz = 0x2752)")

        def band(m, f0):
            return m[np.abs(freqs - f0) < 25e3].max()

        def run(t1, t2, label, f1, f2):
            tx = tx_buffer(t1, t2)
            rx = allocate(shape=(N * 4,), dtype=np.int16)
            txd.transfer_from_memory(tx.physical_address, tx.nbytes, cyclic=True)
            ok = rxd.transfer_to_memory(rx.physical_address, rx.nbytes)
            rx.invalidate()
            txd.stop()
            m1, m2 = spectra(np.array(rx))
            print(f"  {label}: dma_ok={ok}")
            for name, m in (("RX1", m1), ("RX2", m2)):
                i = int(m.argmax())
                print(f"    {name}: peak {freqs[i]/1e3:8.1f} kHz {m[i]:6.1f} dB | at f1={f1/1e3:+.0f}k: "
                      f"{band(m, f1):6.1f}  at f2={f2/1e3:+.0f}k: {band(m, f2):6.1f}  floor {np.median(m):5.1f}")

        t1, f1 = tone(+1.0e6)
        t2, f2 = tone(-0.5e6)

        print("\nStage A: chip-internal digital loopback (TX1: +1.0 MHz, TX2: -0.5 MHz)")
        ad.reg_write(OBS, ad.reg_read(OBS) | 1)
        run(t1, t2, "loopback TX1+TX2", f1, f2)
        run(t1, None, "loopback TX1 only", f1, f2)
        run(None, t2, "loopback TX2 only", f1, f2)
        ad.reg_write(OBS, ad.reg_read(OBS) & ~1)

        print("\nStage B: RF (over the air)")
        run(None, None, "TX idle", f1, f2)
        run(t1, None, "TX1 only", f1, f2)
        run(None, t2, "TX2 only", f1, f2)
        run(t1, t2, "TX1+TX2", f1, f2)
    finally:
        ad.close()


if __name__ == "__main__":
    main()
