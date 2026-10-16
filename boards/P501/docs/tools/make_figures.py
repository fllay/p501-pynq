import numpy as np, matplotlib, sys, glob, os
matplotlib.use("Agg")
import matplotlib.pyplot as plt
D = sys.argv[1] + "/report"; O = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "figures")
N = 8192
def load(name): return np.load(f"{D}/{name}.npy")
def iq(a): x = a[0::2].astype(float) + 1j * a[1::2].astype(float); return x
def spec(a):
    x = iq(a); x = x - x.mean()
    return 20 * np.log10(np.abs(np.fft.fftshift(np.fft.fft(x * np.hanning(N)))) + 1e-9)
def freqs(fs): return np.fft.fftshift(np.fft.fftfreq(N, 1 / fs))
def band(m, fr, f0, w): return m[np.abs(fr - f0) < w].max()
def metrics(a, fs, tgt):
    fr = freqs(fs); cyc = round(tgt * N / fs); f = cyc * fs / N; w = max(40e3, fs / 512)
    m = spec(a); i = int(m.argmax())
    return dict(f=f, peak_f=fr[i], peak=m[i], img=band(m, fr, -f, w), rej=m[i] - band(m, fr, -f, w),
                dc=band(m, fr, 0, w), floor=float(np.median(m)))
plt.rcParams.update({"font.size": 9, "axes.grid": True, "grid.alpha": .3})

# ---- Fig 1: comb (datarate 0) vs clean (datarate 1)
fs = 3.84e6; fr = freqs(fs); fig, ax = plt.subplots(2, 2, figsize=(12, 7))
for c, (dr, lab) in enumerate(((0, "DAC DATARATE = 0 (power-on default)"), (1, "DAC DATARATE = 1 (R1 mode, fixed)"))):
    a = load(f"A_dr{dr}"); x = iq(a); x = x - x.mean(); t = np.arange(160) / fs * 1e6
    ax[0, c].plot(t, x[:160].real, label="I"); ax[0, c].plot(t, x[:160].imag, label="Q")
    ax[0, c].set_title(lab); ax[0, c].set_xlabel("time (us)"); ax[0, c].set_ylabel("RX counts"); ax[0, c].legend(loc="upper right")
    ax[0, c].set_ylim(-250, 250)
    m = spec(a); ax[1, c].plot(fr / 1e3, m, lw=.7); ax[1, c].axvline(1000, color="g", ls="--", lw=.8)
    ax[1, c].set_xlabel("frequency (kHz)"); ax[1, c].set_ylabel("dB"); ax[1, c].set_ylim(30, 125)
    n20 = int((m > m.max() - 20).sum()); ax[1, c].set_title(f"spectrum: {n20} bins within 20 dB of peak")
fig.suptitle("Fig. 1  1 MHz DMA tone as received (700 MHz LO, 3.84 MSPS): impulse comb vs clean sine"); fig.tight_layout()
fig.savefig(f"{O}/fig1_datarate_comb_vs_clean.png", dpi=110); plt.close(fig)
m0, m1 = metrics(load("A_dr0"), fs, 1e6), metrics(load("A_dr1"), fs, 1e6)
print("FIG1", {k: round(float(v), 1) for k, v in m0.items()}, {k: round(float(v), 1) for k, v in m1.items()})

# ---- Fig 2: tone sweep
fig, ax = plt.subplots(2, 1, figsize=(11, 8), gridspec_kw={"height_ratios": [2, 1.2]})
cols = plt.cm.viridis(np.linspace(0, .9, 6)); rows = []
ax[0].plot(fr / 1e3, spec(load("B_off")), color="gray", lw=.6, label="TX off")
for c, tg in zip(cols, (-1500, -1000, -500, 500, 1000, 1500)):
    a = load(f"B_{tg:+05d}k"); ax[0].plot(fr / 1e3, spec(a), color=c, lw=.7, label=f"{tg/1000:+.1f} MHz")
    r = metrics(a, fs, tg * 1e3); rows.append((tg, r))
ax[0].set_title("Fig. 2  Six tones, -1.5 ... +1.5 MHz (3.84 MSPS, 700 MHz LO), overlaid"); ax[0].set_xlabel("frequency (kHz)"); ax[0].set_ylabel("dB"); ax[0].legend(ncol=7, fontsize=8)
x = np.arange(6); ax[1].bar(x - .2, [r["peak"] for _, r in rows], .4, label="tone peak (dB)"); ax[1].bar(x + .2, [r["img"] for _, r in rows], .4, label="image (dB)")
ax[1].set_xticks(x); ax[1].set_xticklabels([f"{tg/1000:+.1f}" for tg, _ in rows]); ax[1].set_xlabel("commanded tone (MHz)")
for i, (_, r) in enumerate(rows): ax[1].text(i, r["peak"] + 1, f"rej {r['rej']:.0f} dB", ha="center", fontsize=8)
ax[1].legend(loc="upper left", ncol=2); ax[1].set_ylim(40, 135); fig.tight_layout(); fig.savefig(f"{O}/fig2_tone_sweep.png", dpi=110); plt.close(fig)
print("FIG2"); [print(f"| {tg/1000:+.1f} | {r['peak_f']/1e3:+.1f} | {r['peak']:.1f} | {r['img']:.1f} | {r['rej']:.1f} | {r['dc']:.1f} | {r['floor']:.1f} |") for tg, r in rows]

# ---- Fig 3: LO sweep
fig, ax = plt.subplots(1, 3, figsize=(13, 4), sharey=True); print("FIG3")
for a_, lo in zip(ax, (433, 700, 915)):
    for tg, col in ((1000, "C3"), (-1000, "C0")):
        a = load(f"C_{lo}M_{tg:+05d}k"); a_.plot(fr / 1e3, spec(a), color=col, lw=.6, label=f"{tg/1000:+.0f} MHz tone")
        r = metrics(a, fs, tg * 1e3); print(f"| {lo} | {tg/1000:+.0f} | {r['peak_f']/1e3:+.1f} | {r['peak']:.1f} | {r['rej']:.1f} | {r['dc']:.1f} |")
    a_.set_title(f"LO {lo} MHz"); a_.set_xlabel("kHz"); a_.legend(fontsize=8)
ax[0].set_ylabel("dB"); fig.suptitle("Fig. 3  LO sweep (3.84 MSPS)"); fig.tight_layout(); fig.savefig(f"{O}/fig3_lo_sweep.png", dpi=110); plt.close(fig)

# ---- Fig 4: sample rates
presets = {"3.84": (3.84e6, (1e6, -1e6)), "30.72": (30.72e6, (3e6, -3e6, 9e6, -9e6)), "61.44": (61.44e6, (5e6, -5e6, 15e6, -15e6, 25e6, -25e6))}
fig, ax = plt.subplots(3, 1, figsize=(11, 10)); print("FIG4")
for a_, (rate, (fs_, tones)) in zip(ax, presets.items()):
    frr = freqs(fs_); a_.plot(frr / 1e6, spec(load(f"D_{rate}_off")), color="gray", lw=.5, label="TX off")
    for k, tg in enumerate(tones):
        a = load(f"D_{rate}_{int(round(tg/1e3)):+0{6 if rate!='3.84' else 6}d}k"); a_.plot(frr / 1e6, spec(a), lw=.5, color=f"C{k}", alpha=.8, label=f"{tg/1e6:+.0f} MHz")
        r = metrics(a, fs_, tg); print(f"| {rate} | {tg/1e6:+.0f} | {r['peak_f']/1e6:+.3f} | {r['peak']:.1f} | {r['rej']:.1f} | {r['floor']:.1f} |")
    a_.set_title(f"{rate} MSPS"); a_.set_xlabel("frequency (MHz)"); a_.set_ylabel("dB"); a_.legend(ncol=7, fontsize=7)
fig.suptitle("Fig. 4  Tones at 3.84 / 30.72 / 61.44 MSPS (700 MHz LO, FIR bypassed above 3.84)"); fig.tight_layout(); fig.savefig(f"{O}/fig4_sample_rates.png", dpi=110); plt.close(fig)

# ---- Fig 5: LVDS delay windows at 30.72 MSPS (from the 16x16 live sweep)
rx = np.zeros((16, 16)); tx = np.zeros((16, 16))   # 1 good, 0.5 poor, 0 fail
for c in range(16):
    for d in range(16):
        rx[c, d] = 0 if (c < 14 and d == 0) or (c == 0 and d == 1) else (0.35 if (c == 0 and d == 2) else 1)
        tx[c, d] = 1 if (c >= 2 and d <= c - 2) else 0.5
fig, ax = plt.subplots(1, 2, figsize=(11, 5))
for a_, g, ttl in ((ax[0], rx, "RX delay (0x006)"), (ax[1], tx, "TX delay (0x007)")):
    a_.imshow(g, origin="lower", cmap="RdYlGn", vmin=0, vmax=1); a_.set_title(ttl); a_.set_xlabel("data delay"); a_.set_ylabel("clock delay")
    a_.set_xticks(range(16)); a_.set_yticks(range(16)); a_.grid(False)
ax[0].plot(4, 0, "ko", ms=8); ax[1].plot(0, 7, "ko", ms=8)
fig.suptitle("Fig. 5  LVDS clock/data delay windows at 30.72 MSPS (l_clk = 61.44 MHz)\nblack dot = default; green = good, yellow = tone present but poor image rejection, red/orange = DMA timeout / wrong frequency", fontsize=10); fig.tight_layout()
fig.savefig(f"{O}/fig5_lvds_delay_windows.png", dpi=110); plt.close(fig)

# ---- Fig 6: random analog state at FDD entry
fig, ax = plt.subplots(2, 2, figsize=(12, 7)); print("FIG6")
for r_, rate in enumerate(("3.84", "30.72")):
    E = np.load(f"{D}/E_{rate}.npy")
    ax[r_, 0].plot(E[:, 0], E[:, 1], "o-", ms=4, label="analog TX->RX tone"); ax[r_, 0].plot(E[:, 0], E[:, 3], "s-", ms=4, color="C3", label="chip digital loopback")
    ax[r_, 0].set_title(f"{rate} MSPS: tone level after each ALERT->FDD entry"); ax[r_, 0].set_xlabel("FDD entry #"); ax[r_, 0].set_ylabel("tone peak (dB)"); ax[r_, 0].legend(fontsize=8)
    ax[r_, 1].scatter(E[:, 1], E[:, 2], c="C0", label="analog"); ax[r_, 1].scatter(E[:, 3], np.minimum(E[:, 4], 100), c="C3", marker="s", label="digital loopback (rejection clipped at 100 dB)")
    ax[r_, 1].set_xlabel("tone peak (dB)"); ax[r_, 1].set_ylabel("image rejection (dB)"); ax[r_, 1].set_title("level vs image rejection"); ax[r_, 1].set_ylim(25, 105); ax[r_, 1].legend(fontsize=8)
    lv = np.round(E[:, 1], 0); u, cn = np.unique(np.round(E[:, 1] / 1.5) * 1.5, return_counts=True)
    print(f"| {rate} | {len(E)} | {E[:,1].min():.1f} .. {E[:,1].max():.1f} | {E[:,2].min():.1f} .. {E[:,2].max():.1f} | {E[:,3].min():.1f} .. {E[:,3].max():.1f} |", dict(zip(u.tolist(), cn.tolist())))
fig.suptitle("Fig. 6  Hidden analog state is re-rolled at every FDD entry; digital path is constant"); fig.tight_layout()
fig.savefig(f"{O}/fig6_random_analog_state.png", dpi=110); plt.close(fig)
