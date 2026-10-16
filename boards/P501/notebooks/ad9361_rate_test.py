#!/usr/bin/env python3
"""Tone test at a higher sample rate (default 30.72 MSPS, FIR bypassed).

Chain at 30.72 MSPS with BBPLL=983.04MHz: ADC_CLK=BBPLL/16=61.44MHz,
RX: R2=2 (/2), R1=1, CLKRF=1, FIR bypass => 30.72 MSPS.
TX: FIR bypass, CLKTF=1, T1=1, T2=2 (x2) => DAC_CLK=61.44MHz (dac_div=1).
In R1 mode l_clk = 2 x fs = 61.44 MHz, so DAC datarate stays 1.
usage: ad9361_rate_test.py [3.84|30.72|61.44] [rx_delay tx_delay]  (LVDS clk/data delay bytes)
"""
import sys
sys.path.insert(0, '/home/xilinx/jupyter_notebooks')
import numpy as np
from pynq import Overlay, allocate

from ad9361_driver import AD9361
from axi_dmac import AxiDmac
import ad9361_full_bypass_test as fb

RATES = {
    3.84: dict(fs=3_840_000, adc_div=16, adc_clk=61_440_000, bw=4_000_000, clk=7_680_000, fir=True,
               chain=dict(r2_div=2, r1_div=2, clkrf_div=2, rx_fir_div=2,
                          dac_div=1, t2_div=2, t1_div=2, clktf_div=2, tx_fir_div=2),
               tones=(1e6, -1e6)),
    30.72: dict(fs=30_720_000, adc_div=16, adc_clk=61_440_000, bw=18_000_000,
                clk=30_720_000, fir=False,
                chain=dict(r2_div=2, r1_div=1, clkrf_div=1, rx_fir_div=1,
                           dac_div=1, t2_div=2, t1_div=1, clktf_div=1, tx_fir_div=1),
                tones=(3e6, -3e6, 9e6, -9e6)),
    61.44: dict(fs=61_440_000, adc_div=4, adc_clk=245_760_000, bw=50_000_000,
                clk=61_440_000, fir=False,
                chain=dict(r2_div=2, r1_div=2, clkrf_div=1, rx_fir_div=1,
                           dac_div=2, t2_div=2, t1_div=1, clktf_div=1, tx_fir_div=1),
                tones=(5e6, -5e6, 15e6, -15e6, 25e6, -25e6)),
}
RATE = float(sys.argv[1]) if len(sys.argv) > 1 else 30.72
CFG = RATES[RATE]
FS = CFG["fs"]
N = 8192
LO = 700_000_000
RF_BW = CFG["bw"]
TX_ATTEN_DB = 10
RX_GAIN_INDEX = 45
RX_DELAY = int(sys.argv[2], 0) if len(sys.argv) > 3 else fb.RX_CLOCK_DATA_DELAY
TX_DELAY = int(sys.argv[3], 0) if len(sys.argv) > 3 else fb.TX_CLOCK_DATA_DELAY


def settle(ol, ad, min_rej_db=50.0, max_tries=40):
    """Workaround for a hidden analog TX/RX state that is re-latched at random
    each time the AD9361 enters FDD (levels differ by ~3/9 dB and image
    rejection by 20+ dB; digital data-port loopback is unaffected, so the FPGA
    path and the driver's digital config are not the cause). Measure a test
    tone's image rejection and toggle ALERT -> FDD until it is >= min_rej_db.
    Every 8th failed try also re-runs the TX quad cal. Returns (rejection_dB, tries)."""
    from ad9361_driver import REG_ENSM_CONFIG_1, TO_ALERT, FORCE_ALERT_STATE
    txd = AxiDmac(ol.axi_ad9361_dac_dma)
    rxd = AxiDmac(ol.axi_ad9361_adc_dma)
    freqs = np.fft.fftshift(np.fft.fftfreq(N, d=1 / FS))
    n = np.arange(N)
    tgt = CFG["tones"][0]
    cyc = round(tgt * N / FS)
    f = cyc * FS / N
    tone = np.exp(2j * np.pi * cyc * n / N)
    tx = allocate(shape=(N * 2,), dtype=np.int16)
    sc = (2**15 - 1) * 0.5
    tx[0::2] = (tone.real * sc).astype(np.int16)
    tx[1::2] = (tone.imag * sc).astype(np.int16)
    tx.flush()
    rx = allocate(shape=(N * 2,), dtype=np.int16)
    rej = 0.0
    for tries in range(1, max_tries + 1):
        txd.transfer_from_memory(tx.physical_address, tx.nbytes, cyclic=True)
        rxd.transfer_to_memory(rx.physical_address, rx.nbytes)
        rx.invalidate()
        txd.stop()
        a = np.array(rx)
        x = a[0::2].astype(float) + 1j * a[1::2].astype(float)
        x -= x.mean()
        mag = 20 * np.log10(np.abs(np.fft.fftshift(np.fft.fft(x * np.hanning(N)))) + 1e-9)
        rej = mag[np.abs(freqs - f) < 40e3].max() - mag[np.abs(freqs + f) < 40e3].max()
        if rej >= min_rej_db:
            return rej, tries
        ad.reg_write(REG_ENSM_CONFIG_1, TO_ALERT | FORCE_ALERT_STATE)
        if tries % 8 == 0:
            # toggling alone cannot recover from a poor TX quad-cal solution
            ad.tx_quad_calib(RF_BW // 2, RF_BW // 2, clkrf_hz=CFG["clk"],
                             clktf_hz=CFG["clk"])
        ad.enable_fdd()
        fb.axi_ad9361_core_enable(ol.axi_ad9361, num_channels=2)
    return rej, max_tries


def bring_up(ol, rx_delay=RX_DELAY, tx_delay=TX_DELAY, do_settle=False):
    ad = AD9361()
    ad.power_up_clocks()
    assert ad.set_bbpll_rate(983_040_000), "BBPLL did not lock"
    ad.set_clock_chain(adc_div=CFG["adc_div"], **CFG["chain"])
    ad.synth_cp_calib(False)
    ad.synth_cp_calib(True)
    rx_ok = ad.set_rx_lo_freq(LO)
    tx_ok = ad.set_tx_lo_freq(LO)
    ad.enable_rx1_tx1()
    fb.axi_ad9361_core_enable(ol.axi_ad9361, num_channels=2)
    ad.reg_write(fb.REG_PARALLEL_PORT_CONF_1, fb.PP_CONF_1)
    ad.reg_write(fb.REG_PARALLEL_PORT_CONF_2, fb.PP_CONF_2)
    ad.reg_write(fb.REG_PARALLEL_PORT_CONF_3, fb.PP_CONF_3_LVDS_MODE)
    ad.reg_write(fb.REG_RX_CLOCK_DATA_DELAY, rx_delay)
    ad.reg_write(fb.REG_TX_CLOCK_DATA_DELAY, tx_delay)
    ad.reg_write(fb.REG_LVDS_BIAS_CTRL, fb.LVDS_BIAS_CTRL)
    res = ad.analog_init(rf_rx_bw_hz=RF_BW, rf_tx_bw_hz=RF_BW,
                         adc_clk_hz=CFG["adc_clk"], rx_freq_hz=LO,
                         clkrf_hz=CFG["clk"], clktf_hz=CFG["clk"])
    ad.set_rx_manual_gain(RX_GAIN_INDEX)
    ad.set_tx_atten_db(TX_ATTEN_DB)
    ad.enable_fdd()
    if CFG["fir"]:
        from ad9361_driver import FIR_RX1, FIR_TX1
        from ad9361_fir_test import make_fir_coef
        coef = make_fir_coef()
        ad.load_fir_filter_coef(FIR_RX1, 0, coef)
        ad.load_fir_filter_coef(FIR_TX1, 0, coef)
    fb.axi_ad9361_core_enable(ol.axi_ad9361, num_channels=2)
    if do_settle:
        res['settle_rej_db'], res['settle_tries'] = settle(ol, ad)
    return ad, rx_ok, tx_ok, res


def main():
    ol = Overlay("base/system_top.bit")
    txd = AxiDmac(ol.axi_ad9361_dac_dma)
    rxd = AxiDmac(ol.axi_ad9361_adc_dma)
    freqs = np.fft.fftshift(np.fft.fftfreq(N, d=1 / FS))
    n = np.arange(N)
    import os
    ad, rx_ok, tx_ok, res = bring_up(ol, do_settle=not os.environ.get('NO_SETTLE'))
    try:
        print(f"{FS/1e6:.2f} MSPS, LO {LO/1e6:.0f} MHz, delays rx=0x{RX_DELAY:02X} tx=0x{TX_DELAY:02X}: "
              f"locks={rx_ok},{tx_ok} cal={res} ensm=0x{ad.ensm_state():X}")
        print(f"  axi_ad9361 clk counts: ADC=0x{ol.axi_ad9361.read(0x54):X} DAC=0x{ol.axi_ad9361.read(0x4054):X} "
              f"(freq/100MHz*65536; 7.68MHz=0x13A9)")

        def band(mag, f0, w=40e3):
            return mag[np.abs(freqs - f0) < w].max()

        for tgt in CFG["tones"]:
            cyc = round(tgt * N / FS)
            f = cyc * FS / N
            tone = np.exp(2j * np.pi * cyc * n / N)
            tx = allocate(shape=(N * 2,), dtype=np.int16)
            sc = (2**15 - 1) * 0.5
            tx[0::2] = (tone.real * sc).astype(np.int16)
            tx[1::2] = (tone.imag * sc).astype(np.int16)
            tx.flush()
            rx = allocate(shape=(N * 2,), dtype=np.int16)
            rxd.transfer_to_memory(rx.physical_address, rx.nbytes)
            rx.invalidate()
            off = np.array(rx)
            txd.transfer_from_memory(tx.physical_address, tx.nbytes, cyclic=True)
            ok = rxd.transfer_to_memory(rx.physical_address, rx.nbytes)
            rx.invalidate()
            unf = ol.axi_ad9361.read(0x4088)
            txd.stop()
            on = np.array(rx)

            def spec(a):
                x = a[0::2].astype(float) + 1j * a[1::2].astype(float)
                x -= x.mean()
                return 20 * np.log10(np.abs(np.fft.fftshift(
                    np.fft.fft(x * np.hanning(N)))) + 1e-9), x.std()

            m_on, sd = spec(on)
            m_off, _ = spec(off)
            i = int(m_on.argmax())
            pk, fp = m_on[i], freqs[i]
            img = band(m_on, -f)
            good = abs(fp - f) < 30e3 and pk - img > 20
            print(f"  tone {f/1e6:6.2f}M: dma_ok={ok} peak {fp/1e6:7.3f}M {pk:6.1f} dB "
                  f"(TX off {band(m_off, f):5.1f}) image {img:5.1f} rej {pk-img:5.1f} "
                  f"std {sd:6.1f} floor {np.median(m_on):5.1f} {'OK' if good else 'CHECK'}")
    finally:
        ad.close()


if __name__ == "__main__":
    main()
