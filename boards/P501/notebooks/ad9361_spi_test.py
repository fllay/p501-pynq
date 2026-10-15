#!/usr/bin/env python3
"""
AD9361 Product ID SPI read, confirmed working end-to-end on real hardware
2026-10-15. Requires the PYNQ overlay to be downloaded first (the AD9361's
SPI/control signals route through PL fabric -- nothing on this path is live
until Overlay() has configured the FPGA via zocl/FPGA Manager).

Two things have to be correct simultaneously, both found by trial on
real hardware:
  1. The EMIO GPIO control lines (640/641/642 -- reset/enable/txnrx) must be
     driven HIGH as outputs. Left as inputs (their boot-time default) they
     float/read low, holding the AD9361 in hardware reset.
  2. SPI mode must be 1 (CPOL=0, CPHA=1), not mode 0. Mode 0 produces a
     stable but systematically 1-bit-shifted readback (e.g. 0x05 instead
     of the correct 0x0A for REG_PRODUCT_ID) -- a classic clock-phase
     mismatch signature.

SPI frame format (from ADI's no-OS driver, drivers/rf-transceiver/ad9361/
ad9361.c -- the authoritative bare-metal reference for this chip):
    16-bit command, MSB first:
        bit 15     = R/W (0 = read, 1 = write)
        bits 14:12 = byte count - 1
        bits 9:0   = register address
    followed by `count` data bytes.

Product ID register (0x037): bits [7:3] == 0x08 for AD9361 silicon
(the low 3 bits are a silicon revision code and vary by part).
"""
import os
import time
import spidev

from pynq import Overlay

REG_PRODUCT_ID = 0x037
PRODUCT_ID_MASK = 0xF8
PRODUCT_ID_9361 = 0x08

AD_READ = 0 << 15
AD_CNT = lambda n: ((n - 1) & 0x7) << 12
AD_ADDR = lambda a: a & 0x3FF

CONTROL_GPIOS = [640, 641, 642]  # EMIO reset/enable/txnrx


def release_control_gpios():
    for g in CONTROL_GPIOS:
        path = f"/sys/class/gpio/gpio{g}"
        if not os.path.exists(path):
            with open("/sys/class/gpio/export", "w") as f:
                f.write(str(g))
    time.sleep(0.2)
    for g in CONTROL_GPIOS:
        path = f"/sys/class/gpio/gpio{g}"
        with open(f"{path}/direction", "w") as f:
            f.write("out")
        with open(f"{path}/value", "w") as f:
            f.write("1")
    time.sleep(0.2)


def ad9361_spi_read(spi, reg):
    cmd = AD_READ | AD_CNT(1) | AD_ADDR(reg)
    tx = [cmd >> 8, cmd & 0xFF, 0x00]
    rx = spi.xfer2(tx)
    return rx[2]


def main():
    ol = Overlay("base/system_top.bit")  # must happen before GPIO/SPI access

    release_control_gpios()

    spi = spidev.SpiDev()
    spi.open(1, 0)  # bus 1, chip-select 0 -- confirm with `ls /dev/spidev*`
    spi.mode = 1  # CPOL=0, CPHA=1 -- NOT mode 0, see module docstring
    spi.max_speed_hz = 1_000_000
    spi.bits_per_word = 8

    try:
        value = ad9361_spi_read(spi, REG_PRODUCT_ID)
        print(f"REG_PRODUCT_ID (0x037) = 0x{value:02X}")
        if (value & PRODUCT_ID_MASK) == PRODUCT_ID_9361:
            print("AD9361 responding correctly over SPI.")
        else:
            print(
                f"Unexpected value -- expected (val & 0x{PRODUCT_ID_MASK:02X}) "
                f"== 0x{PRODUCT_ID_9361:02X}, got 0x{value & PRODUCT_ID_MASK:02X}."
            )
    finally:
        spi.close()


if __name__ == "__main__":
    main()
