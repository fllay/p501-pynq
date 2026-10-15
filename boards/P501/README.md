# P501 PYNQ board support

PYNQ board definition for the Puzhi PZSDR P501 (Zynq UltraScale+ XCZU5EV-2SFVC784I,
AD9361 2T2R SDR). Builds a Jupyter-ready SD card image via PYNQ-3.1's `sdbuild`
flow (`make BOARDS=P501`), on top of the already-working PetaLinux project at
[fllay/puzi501iio](https://github.com/fllay/puzi501iio) (Vivado HDL + PetaLinux
BSP for this board — start there for the hardware/pin-level bring-up story).

## Status (2026-10-15)

Hardware-verified on real hardware:

- Full PYNQ boot: console, SD card, login.
- `pynq.Overlay('base/system_top.bit')` loads cleanly — `zocl` autoloads, XRT
  enumerates the device, FPGA Manager downloads the bitstream, and the HWH
  metadata parses. `ol.ip_dict` lists `axi_ad9361`, `axi_ad9361_adc_dma`,
  `axi_ad9361_dac_dma`, `axi_sysid_0`, `sys_ps8` — all confirmed alive over
  AXI (`axi_ad9361`'s VERSION register, both DMA engines' `"DMAC"` magic ID
  register).
- AD9361 responds correctly over SPI (`REG_PRODUCT_ID` reads `0x0A`, matching
  the expected `(val & 0xF8) == 0x08` signature) — see
  [`notebooks/ad9361_spi_test.py`](notebooks/ad9361_spi_test.py).

Not yet done: full AD9361 RF bring-up (BBPLL calibration, RX/TX RFPLL LO
lock, ENSM enable, gain control, RX/TX quadrature/DC-offset calibration) —
needed before an actual RF tone/loopback test makes sense. `axi_ad9361` in
this design is ADI's real HDL core but only the data-path/DMA framer; chip
register control goes through the PS's hardware SPI0, routed through PL
fabric (so it's only live once `Overlay()` has actually downloaded a
bitstream).

## Architecture

Built around the pattern from
[dspsandbox/FPGA-Notes-for-Scientists](https://github.com/dspsandbox/FPGA-Notes-for-Scientists):
plain `pynq.Overlay()` + PYNQ's built-in `axi_dma` driver + hand-written
register-level Python driver code, with **no libiio/kernel AD9361 driver
and no `meta-adi` Yocto layer** — AD9361 control is pure userspace SPI
(`spidev`) plus register-level MMIO access to the PL IP via `pynq.MMIO`.

## Building

```
# inside a PYNQ-3.1 v3.1.2 checkout with this branch's changes applied
cd sdbuild
make BOARDS=P501
```

Needs `sdbuild/prebuilt/pynq_rootfs.aarch64.tar.gz` (PYNQ v3.1 release asset,
not included here) and a working PetaLinux 2024.1 + Vivado 2024.1 toolchain.

### Known build-system issue (not yet fixed upstream)

`sdbuild`'s `depmod` step (in `sdbuild/Makefile`, the stage4 rootfs-assembly
rule) can end up generating `modules.dep`/`modules.alias` for the wrong
kernel-module-version directory whenever the rootfs staging area ends up
with more than one version directory under `/lib/modules` — leaving the
*real* running kernel's directory with no depmod output, so **no** kernel
module (not just `zocl`) autoloads. Root mechanism not fully pinned down.
Workaround, needed after every `make BOARDS=P501` run until the Makefile
itself is fixed:

```bash
# find the real running-kernel version and the stale one
tar -tzf sdbuild/build/P501.tar.gz | grep -E 'usr/lib/modules/[^/]+/$'
tar -tzf sdbuild/build/P501.tar.gz | grep zocl   # confirms which dir is real

# patch the output image directly
sudo mount -o loop,offset=<partition2_start_sector * 512> \
    sdbuild/output/P501-3.1.2.img /mnt/p501rootfs
sudo depmod -b /mnt/p501rootfs <real-kernel-version>
sudo rm -rf /mnt/p501rootfs/usr/lib/modules/<stale-kernel-version>
sync && sudo umount /mnt/p501rootfs
```

### Three sdbuild fixes included in this fork

- **`sdbuild/packages/pynq/pre.sh`**: the step that copies a custom board's
  bitstream/`.hwh`/notebooks into the final rootfs was gated behind
  `[ "$BOARDDIR" != "$DEFAULT_BOARDDIR" ]`, but neither variable is
  `export`ed to the shell — the comparison was always `"" != ""` (false),
  so this step silently never ran for *any* board built the normal
  `make BOARDS=<X>` way. Fixed to just check `$PYNQ_BOARD != "Unknown"`,
  plus two related path bugs in the same block.
- **`sdbuild/boot/meta-pynq/recipes-bsp/device-tree/files/pynq_bootargs.dtsi`**:
  added `earlycon` and `cpufreq.off=1` to the shared bootargs string (this
  file wins over both board-specific dtsi and `CONFIG_SUBSYSTEM_EXTRA_BOOTARGS`
  for every PYNQ board, not just P501).
- **`sdbuild/Makefile`**: a generic, currently-inert hook for picking up a
  board's own `meta-adi-xilinx` layer if present (left in from an earlier,
  since-abandoned detour — see project history; harmless no-op for boards
  without that directory).

## AD9361 control path

- `boards/P501/base/base.py`: minimal `pynq.Overlay` subclass.
- `boards/P501/notebooks/ad9361_spi_test.py`: working AD9361 SPI
  connectivity test. Two non-obvious fixes baked in, found by trial on real
  hardware:
  1. EMIO GPIO 640/641/642 (reset/enable/txnrx) default to floating inputs
     at boot, holding the AD9361 in hardware reset — must be driven high as
     outputs first.
  2. SPI mode must be 1 (`CPOL=0, CPHA=1`), not mode 0 — mode 0 produces a
     stable but systematically 1-bit-shifted readback.

## License

Board definition files here follow PYNQ's own BSD-3-Clause license. The
underlying hardware design is at
[fllay/puzi501iio](https://github.com/fllay/puzi501iio) (MIT).
