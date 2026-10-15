#!/usr/bin/env python3
from pynq import Overlay
from ad9361_driver import (
    AD9361, REG_ENSM_CONFIG_2, REG_RX_PFD_CONFIG, REG_TX_PFD_CONFIG,
    REG_RX_CP_OVERRANGE_VCO_LOCK, REG_TX_CP_OVERRANGE_VCO_LOCK,
    REG_RFPLL_DIVIDERS, POWER_DOWN_RX_SYNTH, POWER_DOWN_TX_SYNTH,
    BYPASS_LD_SYNTH,
)

ol = Overlay("base/system_top.bit")
ad = AD9361()

try:
    ad.power_up_clocks()
    ad.set_bbpll_rate(983_040_000)
    ad.set_clock_chain()

    print(f"REG_ENSM_CONFIG_2 = 0x{ad.reg_read(REG_ENSM_CONFIG_2):02X} "
          f"(PD_RX={bool(ad.reg_read(REG_ENSM_CONFIG_2) & POWER_DOWN_RX_SYNTH)}, "
          f"PD_TX={bool(ad.reg_read(REG_ENSM_CONFIG_2) & POWER_DOWN_TX_SYNTH)})")
    print(f"REG_RX_PFD_CONFIG = 0x{ad.reg_read(REG_RX_PFD_CONFIG):02X} "
          f"(BYPASS_LD={bool(ad.reg_read(REG_RX_PFD_CONFIG) & BYPASS_LD_SYNTH)})")
    print(f"REG_TX_PFD_CONFIG = 0x{ad.reg_read(REG_TX_PFD_CONFIG):02X} "
          f"(BYPASS_LD={bool(ad.reg_read(REG_TX_PFD_CONFIG) & BYPASS_LD_SYNTH)})")

    rx_locked = ad.set_rx_lo_freq(700_000_000)
    print(f"\nRX locked={rx_locked}")
    print(f"REG_RX_CP_OVERRANGE_VCO_LOCK = 0x{ad.reg_read(REG_RX_CP_OVERRANGE_VCO_LOCK):02X}")
    print(f"REG_RFPLL_DIVIDERS = 0x{ad.reg_read(REG_RFPLL_DIVIDERS):02X}")

    tx_locked = ad.set_tx_lo_freq(700_000_000)
    print(f"\nTX locked={tx_locked}")
    print(f"REG_TX_CP_OVERRANGE_VCO_LOCK = 0x{ad.reg_read(REG_TX_CP_OVERRANGE_VCO_LOCK):02X}")
    print(f"REG_RFPLL_DIVIDERS = 0x{ad.reg_read(REG_RFPLL_DIVIDERS):02X}")

    print(f"\nRe-check REG_ENSM_CONFIG_2 after TX attempt = 0x{ad.reg_read(REG_ENSM_CONFIG_2):02X}")
finally:
    ad.close()
