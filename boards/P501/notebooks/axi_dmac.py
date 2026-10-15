#!/usr/bin/env python3
"""
Minimal userspace driver for ADI's axi_dmac core (NOT Xilinx's axi_dma --
different IP, different register map; pynq.lib.dma.DMA will not work here).
Register map extracted from the Linux kernel driver source,
drivers/dma/dma-axi-dmac.c, in this project's own kernel build
(sdbuild/build/P501/petalinux_project/.../kernel-source/) -- the
authoritative reference since there's no public register-map datasheet for
this core, only the driver implementation.

Used via a PYNQ MMIO-mapped IP (ol.axi_ad9361_adc_dma / _dac_dma), driving
a pynq.allocate()'d physically-contiguous buffer directly -- no kernel DMA
engine involved, matching this project's register-level driver pattern
(boards/P501/README.md).
"""
import time

REG_CTRL = 0x400
REG_TRANSFER_ID = 0x404
REG_START_TRANSFER = 0x408
REG_FLAGS = 0x40C
REG_DEST_ADDRESS = 0x410
REG_SRC_ADDRESS = 0x414
REG_X_LENGTH = 0x418
REG_Y_LENGTH = 0x41C
REG_DEST_STRIDE = 0x420
REG_SRC_STRIDE = 0x424
REG_TRANSFER_DONE = 0x428
REG_ACTIVE_TRANSFER_ID = 0x42C
REG_STATUS = 0x430

CTRL_ENABLE = 1 << 0
FLAG_CYCLIC = 1 << 0
FLAG_LAST = 1 << 1


class AxiDmac:
    """ip: a PYNQ MMIO-capable IP object, e.g. ol.axi_ad9361_adc_dma."""

    def __init__(self, ip):
        self.ip = ip
        self.ip.write(REG_CTRL, CTRL_ENABLE)

    def _start(self, addr, nbytes, cyclic=False):
        self.ip.write(REG_CTRL, CTRL_ENABLE)
        tid = self.ip.read(REG_TRANSFER_ID)
        self.ip.write(addr, 0)  # placeholder, replaced below by caller
        return tid

    def transfer_to_memory(self, dest_phys_addr, nbytes, timeout_s=3.0):
        """One-shot capture: DMAC writes nbytes starting at dest_phys_addr
        (a pynq.allocate() buffer's .physical_address)."""
        self.ip.write(REG_CTRL, CTRL_ENABLE)
        tid = self.ip.read(REG_TRANSFER_ID)
        self.ip.write(REG_DEST_ADDRESS, dest_phys_addr)
        self.ip.write(REG_X_LENGTH, nbytes - 1)
        self.ip.write(REG_Y_LENGTH, 0)
        self.ip.write(REG_FLAGS, FLAG_LAST)
        self.ip.write(REG_START_TRANSFER, 1)
        return self._wait_done(tid, timeout_s)

    def transfer_from_memory(self, src_phys_addr, nbytes, cyclic=True,
                              timeout_s=3.0):
        """Playback: DMAC reads nbytes starting at src_phys_addr. cyclic=True
        repeats the buffer indefinitely (for a continuous TX tone) -- caller
        must stop() explicitly, there's no "done" to wait for in that mode."""
        self.ip.write(REG_CTRL, CTRL_ENABLE)
        tid = self.ip.read(REG_TRANSFER_ID)
        self.ip.write(REG_SRC_ADDRESS, src_phys_addr)
        self.ip.write(REG_X_LENGTH, nbytes - 1)
        self.ip.write(REG_Y_LENGTH, 0)
        self.ip.write(REG_FLAGS, FLAG_CYCLIC if cyclic else FLAG_LAST)
        self.ip.write(REG_START_TRANSFER, 1)
        if cyclic:
            return True
        return self._wait_done(tid, timeout_s)

    def _wait_done(self, tid, timeout_s):
        end = time.time() + timeout_s
        while time.time() < end:
            done_mask = self.ip.read(REG_TRANSFER_DONE)
            if done_mask & (1 << tid):
                return True
            time.sleep(0.005)
        return False

    def stop(self):
        self.ip.write(REG_CTRL, 0)
