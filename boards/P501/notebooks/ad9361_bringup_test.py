#!/usr/bin/env python3
from pynq import Overlay
from ad9361_driver import AD9361

ol = Overlay("base/system_top.bit")
ad = AD9361()

try:
    print(f"Product ID: 0x{ad.product_id():02X} (ok={ad.product_id_ok()})")

    print("\n-- power up clocks --")
    ad.power_up_clocks()
    print("REG_CLOCK_ENABLE + ref divider reset-bars written")

    print("\n-- BBPLL --")
    bbpll_rate = 983_040_000  # 983.04 MHz -> /16 ADC div -> 61.44 MHz -> /16 total decim -> 3.84 MSPS
    locked = ad.set_bbpll_rate(bbpll_rate)
    print(f"BBPLL target {bbpll_rate} Hz, locked={locked}")

    print("\n-- clock chain --")
    ad.set_clock_chain()
    print("clock chain dividers written (ADC/DAC div=16, R2=2,R1=2,CLKRF/TF=2,FIR=2 -> 3.84 MSPS)")

    print("\n-- RFPLL RX 700MHz --")
    rx_locked = ad.set_rx_lo_freq(700_000_000)
    print(f"RX LO locked={rx_locked}")

    print("\n-- RFPLL TX 700MHz --")
    tx_locked = ad.set_tx_lo_freq(700_000_000)
    print(f"TX LO locked={tx_locked}")

    print("\n-- ENSM FDD --")
    ad.enable_fdd()
    state = ad.ensm_state()
    print(f"ENSM state=0x{state:X} (want 0xA = FDD)")

    print("\n-- gain/atten --")
    ad.set_rx_manual_gain()
    ad.set_tx_atten_db(10)
    print("RX manual gain + TX 10dB atten set")

finally:
    ad.close()
