#!/usr/bin/env python3
"""Collect raw captures for the report figures. usage: collect_report.py <part>
A: DAC datarate 0 vs 1 (comb vs clean) @3.84M     B: tone sweep @3.84M (settled)
C: LO sweep 433/700/915 @3.84M                    D: rates 3.84/30.72/61.44 (settled)
E: ENSM-toggle study: analog tone vs digital loopback @3.84M and @30.72M"""
import sys, os, importlib
PART = sys.argv[1]; RATE = sys.argv[2] if len(sys.argv) > 2 else "3.84"
sys.argv = [sys.argv[0], RATE]
sys.path.insert(0, '/home/xilinx/jupyter_notebooks')
import numpy as np
from pynq import Overlay, allocate
from axi_dmac import AxiDmac
import ad9361_full_bypass_test as fb
import ad9361_rate_test as rt
from ad9361_driver import REG_ENSM_CONFIG_1, TO_ALERT, FORCE_ALERT_STATE
OUT = "/tmp/report"; os.makedirs(OUT, exist_ok=True)
ol = Overlay("base/system_top.bit")
txd = AxiDmac(ol.axi_ad9361_dac_dma); rxd = AxiDmac(ol.axi_ad9361_adc_dma)
N = rt.N

def make_tx(tgt, fs):
    cyc = round(tgt * N / fs); n = np.arange(N)
    tone = np.exp(2j * np.pi * cyc * n / N)
    tx = allocate(shape=(N * 2,), dtype=np.int16); sc = (2**15 - 1) * 0.5
    tx[0::2] = (tone.real * sc).astype(np.int16); tx[1::2] = (tone.imag * sc).astype(np.int16); tx.flush()
    return tx, cyc * fs / N

def grab(tx=None):
    rx = allocate(shape=(N * 2,), dtype=np.int16)
    if tx is not None: txd.transfer_from_memory(tx.physical_address, tx.nbytes, cyclic=True)
    rxd.transfer_to_memory(rx.physical_address, rx.nbytes); rx.invalidate()
    if tx is not None: txd.stop()
    return np.array(rx).copy()

def spec_peak_rej(raw, fs, f):
    a = raw; x = a[0::2].astype(float) + 1j * a[1::2].astype(float); x -= x.mean()
    fr = np.fft.fftshift(np.fft.fftfreq(N, 1 / fs))
    m = 20 * np.log10(np.abs(np.fft.fftshift(np.fft.fft(x * np.hanning(N)))) + 1e-9)
    p = m[np.abs(fr - f) < 40e3 * (fs / 3.84e6) ** 0.5].max(); i = m[np.abs(fr + f) < 40e3 * (fs / 3.84e6) ** 0.5].max()
    return float(p), float(p - i)

meta = {}
def clean_off(name):
    """true TX-idle baseline: separate bring-up in which the DMA is never started"""
    a0, *_ = rt.bring_up(ol)
    try: np.save(f"{OUT}/{name}.npy", grab(None))
    finally: a0.close()

if PART == "A":
    ad, *_ = rt.bring_up(ol)
    try:
        tx, f = make_tx(1e6, rt.FS)
        for dr in (0, 1):
            ol.axi_ad9361.write(fb.AXI_DAC_REG_RATECNTRL, dr)
            np.save(f"{OUT}/A_dr{dr}.npy", grab(tx))
        ol.axi_ad9361.write(fb.AXI_DAC_REG_RATECNTRL, 1)
    finally: ad.close()
elif PART == "B":
    clean_off("B_off")
    ad, rx_ok, tx_ok, res = rt.bring_up(ol, do_settle=True)
    try:
        print("settle", res.get("settle_rej_db"), res.get("settle_tries"))
        for tgt in (-1.5e6, -1e6, -0.5e6, 0.5e6, 1e6, 1.5e6):
            tx, f = make_tx(tgt, rt.FS)
            np.save(f"{OUT}/B_{int(round(tgt/1e3)):+05d}k.npy", grab(tx))
    finally: ad.close()
elif PART == "C":
    for lo in (433e6, 700e6, 915e6):
        rt.LO = int(lo)
        ad, rx_ok, tx_ok, res = rt.bring_up(ol, do_settle=True)
        try:
            print("LO", lo / 1e6, "settle", res.get("settle_rej_db"), res.get("settle_tries"))
            for tgt in (1e6, -1e6):
                tx, f = make_tx(tgt, rt.FS)
                np.save(f"{OUT}/C_{int(lo/1e6)}M_{int(tgt/1e3):+05d}k.npy", grab(tx))
        finally: ad.close()
elif PART == "D":
    clean_off(f"D_{RATE}_off")
    ad, rx_ok, tx_ok, res = rt.bring_up(ol, do_settle=True)
    try:
        print("rate", RATE, "settle", res.get("settle_rej_db"), res.get("settle_tries"))
        for tgt in rt.CFG["tones"]:
            tx, f = make_tx(tgt, rt.FS)
            np.save(f"{OUT}/D_{RATE}_{int(round(tgt/1e3)):+06d}k.npy", grab(tx))
    finally: ad.close()
elif PART == "E":
    ad, *_ = rt.bring_up(ol)
    try:
        tx, f = make_tx(rt.CFG["tones"][0], rt.FS)
        rows = []
        for k in range(int(os.environ.get("TOGGLES", "40"))):
            ad.reg_write(REG_ENSM_CONFIG_1, TO_ALERT | FORCE_ALERT_STATE)
            ad.enable_fdd(); fb.axi_ad9361_core_enable(ol.axi_ad9361, num_channels=2)
            p, r = spec_peak_rej(grab(tx), rt.FS, f)
            ad.reg_write(0x3F5, ad.reg_read(0x3F5) | 1)
            lp, lr = spec_peak_rej(grab(tx), rt.FS, f)
            ad.reg_write(0x3F5, ad.reg_read(0x3F5) & ~1)
            rows.append((k, p, r, lp, lr))
        np.save(f"{OUT}/E_{RATE}.npy", np.array(rows))
        print(f"rate {RATE}: {len(rows)} toggles; analog peak {min(r[1] for r in rows):.1f}..{max(r[1] for r in rows):.1f}; "
              f"loopback peak {min(r[3] for r in rows):.1f}..{max(r[3] for r in rows):.1f}")
    finally: ad.close()
