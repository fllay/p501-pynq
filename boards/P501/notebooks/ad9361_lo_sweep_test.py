#!/usr/bin/env python3
"""Full bring-up at several LO frequencies (all within the 200-1300 MHz RX
gain table), then transmit +/-1 MHz tones and report where the peak lands,
image rejection and levels. LO-dependent calibrations (RF DC, TX quad) are
re-run for each LO."""
import sys
sys.path.insert(0, '/home/xilinx/jupyter_notebooks')
import numpy as np
from pynq import Overlay, allocate

from ad9361_driver import AD9361, FIR_RX1, FIR_TX1
from axi_dmac import AxiDmac
import ad9361_full_bypass_test as fb
from ad9361_fir_test import make_fir_coef

FS = 3_840_000
N = 8192
LOS = [int(float(a) * 1e6) for a in sys.argv[1:]] or [433_000_000, 700_000_000, 915_000_000]
TX_ATTEN_DB = 10
RX_GAIN_INDEX = 45


def bring_up(ol, lo):
    ad = AD9361()
    ad.power_up_clocks()
    assert ad.set_bbpll_rate(983_040_000), "BBPLL did not lock"
    ad.set_clock_chain()
    ad.synth_cp_calib(False)
    ad.synth_cp_calib(True)
    rx_ok = ad.set_rx_lo_freq(lo)
    tx_ok = ad.set_tx_lo_freq(lo)
    ad.enable_rx1_tx1()
    fb.axi_ad9361_core_enable(ol.axi_ad9361, num_channels=2)
    ad.reg_write(fb.REG_PARALLEL_PORT_CONF_1, fb.PP_CONF_1)
    ad.reg_write(fb.REG_PARALLEL_PORT_CONF_2, fb.PP_CONF_2)
    ad.reg_write(fb.REG_PARALLEL_PORT_CONF_3, fb.PP_CONF_3_LVDS_MODE)
    ad.reg_write(fb.REG_RX_CLOCK_DATA_DELAY, fb.RX_CLOCK_DATA_DELAY)
    ad.reg_write(fb.REG_TX_CLOCK_DATA_DELAY, fb.TX_CLOCK_DATA_DELAY)
    ad.reg_write(fb.REG_LVDS_BIAS_CTRL, fb.LVDS_BIAS_CTRL)
    res = ad.analog_init(rx_freq_hz=lo)
    ad.set_rx_manual_gain(RX_GAIN_INDEX)
    ad.set_tx_atten_db(TX_ATTEN_DB)
    ad.enable_fdd()
    coef = make_fir_coef()
    ad.load_fir_filter_coef(FIR_RX1, 0, coef)
    ad.load_fir_filter_coef(FIR_TX1, 0, coef)
    fb.axi_ad9361_core_enable(ol.axi_ad9361, num_channels=2)
    return ad, rx_ok, tx_ok, res


def main():
    ol = Overlay("base/system_top.bit")
    txd = AxiDmac(ol.axi_ad9361_dac_dma)
    rxd = AxiDmac(ol.axi_ad9361_adc_dma)
    freqs = np.fft.fftshift(np.fft.fftfreq(N, d=1 / FS))
    n = np.arange(N)

    def band(mag, f0, w=15e3):
        return mag[np.abs(freqs - f0) < w].max()

    for lo in LOS:
        ad = None
        try:
            ad, rx_ok, tx_ok, res = bring_up(ol, lo)
            print(f"\nLO {lo/1e6:.1f} MHz: rx_lock={rx_ok} tx_lock={tx_ok} "
                  f"cal={res} quad_phase={getattr(ad, 'last_tx_quad_cal_phase', None)} "
                  f"ensm=0x{ad.ensm_state():X}")
            for tgt in (1.0e6, -1.0e6):
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
                rxd.transfer_to_memory(rx.physical_address, rx.nbytes)
                rx.invalidate()
                txd.stop()
                on = np.array(rx)

                def spec(a):
                    x = a[0::2].astype(float) + 1j * a[1::2].astype(float)
                    x -= x.mean()
                    return 20 * np.log10(np.abs(np.fft.fftshift(
                        np.fft.fft(x * np.hanning(N)))) + 1e-9)

                m_on, m_off = spec(on), spec(off)
                i = int(m_on.argmax())
                pk, fp = m_on[i], freqs[i]
                img = band(m_on, -f)
                ok = abs(fp - f) < 3e3 and pk - img > 20
                print(f"  tone {f/1e3:8.1f}k: peak {fp/1e3:8.1f}k {pk:6.1f} dB "
                      f"(TX off {band(m_off, f):5.1f}), image {img:5.1f} dB, "
                      f"rej {pk-img:5.1f} dB, DC {band(m_on, 0):5.1f}, "
                      f"floor {np.median(m_on):5.1f}  {'OK' if ok else 'CHECK'}")
        except Exception as e:
            print(f"\nLO {lo/1e6:.1f} MHz: FAILED: {type(e).__name__}: {e}")
        finally:
            if ad is not None:
                ad.close()


if __name__ == "__main__":
    main()
