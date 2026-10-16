# P501 PYNQ board support

PYNQ board definition for the Puzhi PZSDR P501 (Zynq UltraScale+ XCZU5EV-2SFVC784I,
AD9361 2T2R SDR). Builds a Jupyter-ready SD card image via PYNQ-3.1's `sdbuild`
flow (`make BOARDS=P501`), on top of the already-working PetaLinux project at
[fllay/puzi501iio](https://github.com/fllay/puzi501iio) (Vivado HDL + PetaLinux
BSP for this board — start there for the hardware/pin-level bring-up story).

> **Detailed AD9361 bring-up report (root causes, figures, validation, open issue):** [`docs/AD9361_BRINGUP_REPORT.md`](docs/AD9361_BRINGUP_REPORT.md)

## Status (2026-10-15, updated same day in a continuation session)

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
- Full register-level RF bring-up: BBPLL lock (983.04MHz), RX+TX RFPLL lock
  to an arbitrary LO (tested at 700MHz), FDD ENSM state, DMA TX/RX pipeline
  (`axi_ad9361` core reset/enable, LVDS digital interface, cache-coherent
  `pynq.allocate()` buffers) — see [`notebooks/ad9361_driver.py`](notebooks/ad9361_driver.py)
  and [`notebooks/ad9361_tone_test.py`](notebooks/ad9361_tone_test.py). Four
  real register-level bugs found and fixed along the way (missing
  `REG_CLOCK_ENABLE` write, RX/TX synth power-down + lock-detect bypass
  defaults, missing `DUAL_SYNTH_MODE` for FDD, an ENSM force-state
  read-modify-write bug), plus a fifth, bigger one: `axi_ad9361`'s DAC
  sub-block sits at a **`+0x4000` address offset** from the ADC sub-block
  (different `COMMON_ID`, confirmed from this project's own RTL) — every
  DAC register write had been silently landing on the wrong address, so TX
  had never transmitted anything at all until this was found.

**Known open issue — captured RX content does not match the transmitted
tone (the "fs/3 artifact")**: see
[`notebooks/AD9361_FS3_INVESTIGATION.md`](notebooks/AD9361_FS3_INVESTIGATION.md)
for the full writeup. Short version: the FFT peak of a captured signal
lands at exactly 1/3 of whatever sample rate is configured, regardless of
decimation settings, RX gain, TX attenuation, both available digital-
interface delay-tuning mechanisms (full range swept on each), or RX/TX
DC-offset calibration — none of which had any effect.

**Confirmed NOT a hardware issue (2026-10-16)**: booted the same board
into the original `p501_iio` PetaLinux image (ADI kernel driver +
libiio) and ran an equivalent TX-tone/RX-capture test. There, the real
+1MHz tone is the dominant peak in the spectrum, with only a minor (16dB
weaker) fs/3-ish secondary effect — not the dominant artifact this
project's own driver produces. The RF front end and antennas are fine;
this project's hand-rolled driver is still missing something, most
likely RX/TX quadrature calibration (never implemented, only DC-offset
calibration was) and/or real AD9361 FIR coefficient loading (never
ported). See the investigation doc for details and next steps.

Not yet done: RX/TX quadrature calibration (distinct from the DC-offset
calibration above, which *was* tried), AGC, full 2T2R mode (this project
has only exercised R1/single-channel mode).

## Quick operation test (notebook)

`notebooks/P501_AD9361_Test.ipynb` (in `/home/xilinx/jupyter_notebooks/` on the board) brings up the AD9361 and checks TX1 -> RX1 end to end
in about a minute: chip identity, PLL lock/FDD, calibrations, one tone with time-domain and spectrum plots, a -1.5 ... +1.5 MHz tone
sweep (frequency and sign), and TX attenuator linearity, ending with a PASS/FAIL summary. Needs the TX1/RX1 antennas near each other
(or a cable with at least 40 dB attenuation). The copy in the repository is saved with its outputs from a real run.

## USB networking (usb0 gadget)

The board shows up on the host as a USB-Ethernet adapter (CDC-ECM, `usb-gadget-ecm`) and configures its side automatically
(`usb-gadget-net`, modelled on Raspberry Pi OS' `rpi-usb-gadget`; plain systemd-networkd + systemd-resolved, nothing extra installed):

1. **Shared mode (default).** If no DHCP server answers within ~10 s of the link coming up, the board takes `192.168.2.1/24`
   and runs a DHCP server (pool `192.168.2.2-.9`, **no router or DNS advertised**, so the host's default route and DNS are
   never touched). A host with default settings (NetworkManager "Automatic (DHCP)", macOS, Windows) gets `192.168.2.x` with no
   manual commands.
2. **Client mode (host shares its connection).** If the host provides DHCP (NetworkManager `ipv4.method shared`, Windows ICS),
   the board becomes a DHCP client, gets e.g. `10.42.0.x`, and has internet/DNS through the host's NAT.
3. **mDNS/LLMNR.** `systemd-resolved` answers for the hostname in both modes: `ssh xilinx@pynq.local`.

The supervisor re-evaluates when the cable is replugged and also when the host stops answering (e.g. you switch the host between
shared and automatic without unplugging): roughly 20 s for client-to-shared and up to about a minute for shared-to-client (it needs two
failed probes so a live SSH session is never dropped on a false alarm). `journalctl -t usb-gadget-net` shows what it decided.
`DHCP_WAIT` (default 10 s) is configurable in `usb-gadget-net.sh`. The static `192.168.2.1` of earlier images is kept in shared mode.

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
