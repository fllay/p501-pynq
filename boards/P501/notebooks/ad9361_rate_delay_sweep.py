#!/usr/bin/env python3
"""30.72 MSPS: sweep AD9361 RX and TX clock/data delay nibbles live and
mark which settings still give a correct +3MHz tone (peak on target, image
>25 dB down). Registers 0x006 (RX) / 0x007 (TX): [7:4]=clock delay, [3:0]=data delay."""
import sys
sys.path.insert(0, '/home/xilinx/jupyter_notebooks')
import numpy as np
from pynq import Overlay, allocate
from axi_dmac import AxiDmac
import ad9361_full_bypass_test as fb
from ad9361_rate_test import bring_up, FS, N

ol = Overlay("base/system_top.bit")
txd = AxiDmac(ol.axi_ad9361_dac_dma); rxd = AxiDmac(ol.axi_ad9361_adc_dma)
ad, rx_ok, tx_ok, res = bring_up(ol)
try:
    freqs = np.fft.fftshift(np.fft.fftfreq(N, d=1 / FS)); n = np.arange(N)
    cyc = round(3e6 * N / FS); f = cyc * FS / N
    tone = np.exp(2j * np.pi * cyc * n / N)
    tx = allocate(shape=(N * 2,), dtype=np.int16); sc = (2**15 - 1) * 0.5
    tx[0::2] = (tone.real * sc).astype(np.int16); tx[1::2] = (tone.imag * sc).astype(np.int16)
    tx.flush()
    rx = allocate(shape=(N * 2,), dtype=np.int16)
    txd.transfer_from_memory(tx.physical_address, tx.nbytes, cyclic=True)
    def score():
        ok = rxd.transfer_to_memory(rx.physical_address, rx.nbytes); rx.invalidate()
        if not ok: return "T"
        a = np.array(rx); x = a[0::2].astype(float) + 1j * a[1::2].astype(float); x -= x.mean()
        if x.std() == 0: return "F"
        mag = 20 * np.log10(np.abs(np.fft.fftshift(np.fft.fft(x * np.hanning(N)))) + 1e-9)
        i = int(mag.argmax()); img = mag[np.abs(freqs + f) < 40e3].max()
        if abs(freqs[i] - f) > 20e3: return "x"
        r = mag[i] - img
        return "#" if r > 25 else ("+" if r > 15 else "-")
    steps = list(range(0, 16, 1))
    for name, reg, mk in (("RX 0x006", 0x006, lambda c, d: (c << 4) | d),
                           ("TX 0x007", 0x007, lambda c, d: (c << 4) | d)):
        base = {0x006: fb.RX_CLOCK_DATA_DELAY, 0x007: fb.TX_CLOCK_DATA_DELAY}
        print(f"\n{name} sweep (rows=clock delay 0..15, cols=data delay 0..15); "
              f"#=good(>25dB rej) +=ok(>15) -=poor x=wrong freq F=frozen T=DMA timeout")
        print("      " + "".join(f"{d:X}" for d in steps))
        for c in steps:
            row = ""
            for d in steps:
                ad.reg_write(reg, mk(c, d)); row += score()
            print(f"  c={c:2d} {row}")
        ad.reg_write(reg, base[reg])
    txd.stop()
finally:
    ad.close()
