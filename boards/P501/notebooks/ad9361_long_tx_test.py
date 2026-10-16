#!/usr/bin/env python3
"""Long continuous cyclic TX; poll the sticky DAC underflow flag (0x4088,
write 1 to clear) and periodically capture RX to check tone stability."""
import sys, time
sys.path.insert(0, '/home/xilinx/jupyter_notebooks')
import numpy as np
from pynq import Overlay, allocate
from axi_dmac import AxiDmac
from ad9361_lo_sweep_test import bring_up, FS, N

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
UNF = 0x4088
ol = Overlay("base/system_top.bit")
txd = AxiDmac(ol.axi_ad9361_dac_dma)
rxd = AxiDmac(ol.axi_ad9361_adc_dma)
ad, rx_ok, tx_ok, res = bring_up(ol, 700_000_000)
try:
    print(f"bring-up: locks={rx_ok},{tx_ok} cal={res}")
    freqs = np.fft.fftshift(np.fft.fftfreq(N, d=1 / FS))
    n = np.arange(N); cyc = round(1e6 * N / FS)
    tone = np.exp(2j * np.pi * cyc * n / N)
    tx = allocate(shape=(N * 2,), dtype=np.int16); sc = (2**15 - 1) * 0.5
    tx[0::2] = (tone.real * sc).astype(np.int16); tx[1::2] = (tone.imag * sc).astype(np.int16)
    tx.flush()

    # baseline: TX idle, does the flag re-assert by itself?
    ol.axi_ad9361.write(UNF, 1)
    time.sleep(0.5)
    print(f"idle baseline (no DMA): unf after clear+0.5s = {ol.axi_ad9361.read(UNF)}")

    txd.transfer_from_memory(tx.physical_address, tx.nbytes, cyclic=True)
    time.sleep(0.2)
    ol.axi_ad9361.write(UNF, 1)           # clear anything from start-up
    t0 = time.time(); next_cap = 0.0
    unf_events = []; polls = 0
    caps = []
    while time.time() - t0 < DURATION:
        t = time.time() - t0
        polls += 1
        if ol.axi_ad9361.read(UNF):
            unf_events.append(round(t, 2)); ol.axi_ad9361.write(UNF, 1)
        if t >= next_cap:
            rx = allocate(shape=(N * 2,), dtype=np.int16)
            rxd.transfer_to_memory(rx.physical_address, rx.nbytes); rx.invalidate()
            a = np.array(rx); x = a[0::2].astype(float) + 1j * a[1::2].astype(float); x -= x.mean()
            mag = 20 * np.log10(np.abs(np.fft.fftshift(np.fft.fft(x * np.hanning(N)))) + 1e-9)
            i = int(mag.argmax()); caps.append((t, freqs[i], mag[i]))
            next_cap += 5.0
        time.sleep(0.25)
    txd.stop()
    print(f"ran {time.time()-t0:.1f}s, {polls} underflow polls, underflow events: {len(unf_events)} {unf_events[:10]}")
    for t, f, p in caps:
        print(f"  t={t:5.1f}s  RX peak {f/1e3:8.1f} kHz  {p:6.1f} dB")
    pk = [c[2] for c in caps]; fr = [c[1] for c in caps]
    print(f"peak level spread {max(pk)-min(pk):.2f} dB, freq spread {(max(fr)-min(fr)):.1f} Hz, "
          f"final unf flag = {ol.axi_ad9361.read(UNF)}")
finally:
    ad.close()
