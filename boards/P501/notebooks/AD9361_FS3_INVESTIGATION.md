> **SUPERSEDED (2026-10-17):** the fs/3 / frozen-capture problem described below was traced to missing driver configuration (unloaded RX FIR, uninitialised RX/TX analog, wrong DAC clock divider and DAC DATARATE). See [`../docs/AD9361_BRINGUP_REPORT.md`](../docs/AD9361_BRINGUP_REPORT.md). Kept for history.

# AD9361 "fs/3 artifact" investigation log

Open issue: a captured RX signal's FFT peak lands at exactly 1/3 of
whatever sample rate is configured, instead of matching the real
transmitted tone. This file is the durable record of everything tried —
written specifically so it survives independent of any particular
machine's local state (session notes, chat history, etc).

## Symptom

With the full RF chain locked (BBPLL, RX+TX RFPLL at 700MHz, FDD ENSM,
LVDS digital interface, DMA pipeline all independently confirmed correct
— see `README.md`'s "Status" section and `ad9361_driver.py`), transmitting
a 1MHz tone and capturing on RX produces an FFT peak that does **not**
land near 1MHz. Instead, it lands at exactly `sample_rate / 3`, confirmed
precisely at three different configured sample rates:

| Configured rate | Peak observed | Ratio |
|---|---|---|
| 3.84 MSPS | 1280.2 kHz | 0.3334 |
| 7.68 MSPS (FIR bypassed) | 2560.3 kHz | 0.3334 |
| 30.72 MSPS (FIR+R1+CLKRF bypassed) | 10241.2 kHz | 0.3333 |

Back-solving "what sample rate would make this the real 1MHz tone"
converges on ~3.000MHz regardless of configured rate — suspiciously
precise but not a clean divisor of the 40MHz reference clock.

## What's been ruled out, with evidence

All of the following were tested across their **full range** and
produced **zero measurable effect** on the artifact (same exact peak
frequency, same magnitude, often bit-for-bit identical raw samples):

1. **RX manual gain** (0x00 to 0x70) and **TX attenuation** (20dB to
   60dB) — rules out front-end saturation/TX-leakage as the cause.
2. **FPGA-side IDELAY taps** (`up_delay_cntrl` core, byte offset
   `0x800 + n*4` for 13 lanes, values 0–31 swept on all lanes
   simultaneously) — a genuine LVDS bit-alignment/skew problem would show
   *some* variation across a full tap sweep; this showed none.
3. **AD9361 chip-side `REG_RX_CLOCK_DATA_DELAY`** (SPI register `0x006`,
   two 4-bit fields: clock_delay and data_delay) — full 16×16 grid swept,
   using the real TX-tone-and-capture pipeline (not a synthetic test) as
   the pass/fail signal. Peak landed at the identical fs/3 alias at every
   one of 254 valid combinations.
4. **RX/TX DC-offset calibration** (`BBDC_CAL`/`RFDC_CAL` via
   `REG_CALIBRATION_CTRL` 0x016, ported from no-OS's
   `ad9361_bb_dc_offset_calib()`/`ad9361_rf_dc_offset_calib()`) — both
   report completing successfully (self-clear within timeout), but the
   captured signal was bit-for-bit identical before and after.
5. **FIR/decimation register field encoding** — direct register readback
   confirmed `REG_RX/TX_ENABLE_FILTER_CTRL` exactly match the intended
   values (R2/R1/CLKRF divider fields, FIR enable/bypass field) at three
   different sample-rate configs. No write-address or field-encoding bug.
6. **`util_cpack2` channel-enable count** — tested with 2 channels
   enabled (R1 mode intent) and all 4 channels enabled; the real (non
   zero-padded) channel data showed the identical artifact either way,
   ruling out a channel-count/packing mismatch as the cause.
7. **BIST/PN-based digital-interface auto-calibration attempt**
   (`ad9361_dig_tune.py`, modeled on no-OS's `ad9361_dig_tune_rx()`) —
   reported 0/256 passing combinations and left the ADC path unable to
   even complete a DMA transfer afterward. This is likely a bug in how
   the BIST PRBS injection or PN-checker enable was wired up in that
   script (not yet root-caused), **not** a reliable negative result —
   superseded by #3 above, which used the proven real-tone pipeline
   instead and is far more trustworthy.

## New structural finding: a fixed spur at exactly ADC_CLK/3

With the entire decimation chain bypassed (R2=R1=CLKRF=FIR=1 on both
RX and TX — i.e. the raw 61.44 MSPS ADC_CLK rate, no filtering anywhere),
the captured I/Q samples are an **exact, zero-variance, repeating 3-sample
sequence pinned at/near 12-bit full scale** (e.g. I cycles through
`-1985, 2047, -2048` forever, every single sample, across 32768+ samples
captured). This is not consistent with real RF/thermal noise, which would
never be perfectly periodic and noise-free at this sample count.

Hand-checked aliasing math: a fixed component at exactly
**ADC_CLK/3 = 61.44MHz/3 = 20.48MHz**, present *before* any decimation,
aliases through each of the three decimation configs in the table above
to land at exactly their observed fs/3 peaks (`20.48 mod rate`, folded
into the ±rate/2 range) — matching all three to within measurement
precision. A direct full-bypass capture confirmed this: a massive
symmetric spur appears at exactly **±20.48MHz** (154dB), about 88dB
*above* the real transmitted 1MHz tone's peak (66dB) in the same capture.
20.48MHz is also exactly `BBPLL/48`.

## RESOLVED (partially): libiio comparison proves this is a software gap, not hardware

**2026-10-16**: booted the same physical board into the original `p501_iio`
PetaLinux image (ADI kernel driver + libiio, `analog`/`analog` login,
hostname `p501iio`, kernel `6.1.70adi-v2023.2`) and ran an equivalent
TX-tone/RX-capture test using `iio_attr`/`iio_readdev` against the real
`ad9361-phy`/`cf-ad9361-lpc`/`cf-ad9361-dds-core-lpc` IIO devices (RX/TX
LO=700MHz, sampling_frequency=3.84MHz, RX manual gain, TX1 DDS tone at
~1MHz via `altvoltage0`/`altvoltage2` scale=1.0).

A TX-on vs TX-off A/B comparison (32768 samples each) is conclusive:
- RX amplitude jumped 25× when TX turned on (I std 45.7 → 1167.4) —
  real over-the-air coupling confirmed, antennas and RF front end work.
- **The real +1MHz tone is the single dominant peak in the whole
  spectrum (138.5 dB)** — correctly placed.
- The fs/3-area bin (~1280kHz) *also* rose somewhat under TX-on (81.3 →
  122.5 dB) — so some genuine fs/3-ish artifact/harmonic does exist on
  this hardware even under the vendor's proven driver — but it's **16 dB
  weaker than the real tone**, a minor secondary effect, not dominant.

**This resolves the (a)/(b) ambiguity in favor of (a): hardware and RF
front end are fine; this project's hand-rolled register driver is still
missing something that lets the same underlying fs/3-ish artifact
*dominate* instead of staying a minor secondary effect.** Two concrete,
still-unimplemented candidates, both previously identified but not yet
built: **RX/TX quadrature calibration** (`RX_QUAD_CAL`/`TX_QUAD_CAL` in
`REG_CALIBRATION_CTRL` — only DC-offset calibration, `BBDC_CAL`/
`RFDC_CAL`, was ever implemented and tested; quadrature imbalance
specifically causes image/sideband leakage of the kind that could
explain why a minor artifact becomes dominant without it) and **real
AD9361 FIR coefficient loading** (`ad9361_load_fir_filter_coef()` was
never ported — only FIR bypass-vs-default-enabled was tested, never
loading real filter taps).

Script: `/tmp/libiio_ab_test.py` (scratch, not persisted in this repo —
recreate from this description if needed: configures via `iio_attr`,
captures via `iio_readdev -b N -s N`, SFTPs the raw `.bin` back, FFTs
locally with numpy). Needs `sudo` on the board (password `analog`) for
both `iio_attr` writes and `iio_readdev`'s buffer allocation.

## Where this leaves it

Confirmed hardware/RF-path is fine (libiio comparison above). The
remaining gap is specifically in this project's hand-rolled register
driver — every *decimation/timing/gain* knob was ruled out before the
libiio comparison, which then separately ruled out hardware, leaving
**calibration/filtering completeness** as the likely remaining area:

- **RX/TX quadrature calibration** — never implemented (only DC-offset
  calibration was). `ad9361_tx_quad_calib()` in no-OS's `ad9361.c` is
  the reference; it's substantially more involved than the DC-offset
  calibration already ported (needs an internal TX test-tone injection
  and iterative phase/gain correction search), which is likely why it
  was deferred earlier in this investigation.
- **Real AD9361 FIR filter coefficient loading** — `ad9361_load_fir_filter_coef()`
  was identified early in this investigation as a real gap (128-tap
  programmable FIR, never loaded) but set aside after a bypass test
  showed the fs/3 artifact persisting identically with FIR enabled vs.
  fully bypassed. That test only proved FIR *presence* doesn't change
  the *dominant* artifact — it doesn't rule out FIR coefficients
  affecting the *relative* strength of the real tone vs. the
  secondary fs/3-ish effect, which is the actual open question now.

Next step: implement one or both and re-run the TX-tone/RX-capture test
on this project's own driver (`ad9361_tone_test.py`) to check whether
the real 1MHz tone becomes dominant, matching the libiio result.

## Scripts (all in this directory)

- `ad9361_full_bypass_test.py` — full decimation bypass + direct
  ±20.48MHz spur check.
- `ad9361_dig_tune.py` — the BIST/PN-based calibration attempt (see
  caveat in finding #7 above).
- `ad9361_delay_sweep_real.py` — the trustworthy real-tone-based 16×16
  chip-side delay sweep (finding #3).
- `ad9361_dc_cal_test.py` — DC-offset calibration attempt (finding #4).
- `ad9361_tone_test.py` — the main, working end-to-end TX/RX/FFT/plot
  test (the one producing the fs/3 symptom in the first place).
- `ad9361_driver.py` — the AD9361 chip-level register driver (BBPLL,
  RFPLL, ENSM, gain/atten, clock chain).
- `axi_dmac.py` — hand-written driver for ADI's `axi_dmac` core.
