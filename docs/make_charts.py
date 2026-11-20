# สร้างกราฟประกอบ docs/explain.md (SVG + PNG) — รันซ้ำได้เมื่อแก้สูตร/ค่า:
#   cd src && ../.venv/Scripts/python.exe ../docs/make_charts.py      (ต้องมี matplotlib: pip install matplotlib)
# สีตามลำดับ categorical ของ dataviz skill (ฟ้า, ส้ม, เขียวน้ำทะเล) — ทุกเส้นมีป้ายชื่อกำกับบนเส้น
import json
import math
import sys
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

SRC = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SRC))
import config  # noqa: E402
from motion import Mover, min_jerk  # noqa: E402
from singularity import analyze  # noqa: E402

OUT = Path(__file__).resolve().parent / "figures"
C1, C2, C3 = "#2a78d6", "#eb6834", "#1baf7a"
INK, MUTED, GRID = "#0b0b0b", "#52514e", "#e4e3df"
plt.rcParams.update({
    "font.family": ["Tahoma", "DejaVu Sans"], "font.size": 10, "axes.edgecolor": MUTED, "axes.labelcolor": INK,
    "xtick.color": MUTED, "ytick.color": MUTED, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8,
    "axes.spines.top": False, "axes.spines.right": False, "lines.linewidth": 2, "figure.facecolor": "#fcfcfb",
    "axes.facecolor": "#fcfcfb", "savefig.facecolor": "#fcfcfb",
})


def label(ax, x, y, text, color):
    """ป้ายชื่อกำกับบนเส้น (ตัวอักษรสีหมึก + จุดสีของเส้น)"""
    ax.plot([x], [y], "o", color=color, ms=7, mec="#fcfcfb", mew=2)
    ax.annotate(text, (x, y), xytext=(6, 4), textcoords="offset points", color=INK, fontsize=9)


def save(fig, name):
    fig.tight_layout()
    fig.savefig(OUT / f"{name}.svg")
    fig.savefig(OUT / f"{name}.png", dpi=110)
    plt.close(fig)


# ---------- 1) min-jerk: ตำแหน่ง/ความเร็ว/ความเร่ง + 3 ข้อถึงพร้อมกัน ----------
tau = np.linspace(0, 1, 201)
s = np.array([min_jerk(t) for t in tau])
ds = 30 * tau ** 2 - 60 * tau ** 3 + 30 * tau ** 4
dds = 60 * tau - 180 * tau ** 2 + 120 * tau ** 3
fig, (a1, a2) = plt.subplots(1, 2, figsize=(10, 3.8))
a1.plot(tau, s, color=C1)
a1.plot(tau, ds / 1.875, color=C2)
a1.plot(tau, dds / 5.774, color=C3)
label(a1, 0.62, min_jerk(0.62), "ตำแหน่ง s", C1)
label(a1, 0.5, 1.0, "ความเร็ว (สูงสุด 1.875·Δ/T)", C2)
label(a1, 0.21, 1.0, "ความเร่ง", C3)
a1.axhline(0, color=MUTED, lw=0.8)
a1.set_xlabel("เวลา τ = t / T")
a1.set_title("s(τ) = 10τ³ − 15τ⁴ + 6τ⁵  (ปลายทั้งสองข้าง ความเร็ว = ความเร่ง = 0)", fontsize=10)
a1.set_ylim(-1.15, 1.25)
goal = [20, -10, 35]
names = ["ฐาน +20°", "ไหล่ −10°", "ศอก +35°"]
T = 1.875 * max(abs(g) for g in goal) / 30
t = np.linspace(0, T, 200)
for g, n, c in zip(goal, names, (C1, C2, C3)):
    a2.plot(t, [g * min_jerk(x / T) for x in t], color=c)
    label(a2, T * 0.98, g, n, c)
a2.set_xlabel("เวลา (s)")
a2.set_ylabel("มุม (°)")
a2.set_title(f"goto_q: ทุกข้อใช้ s(τ) ร่วมกัน → ถึงพร้อมกันที่ T = 1.875·35/30 = {T:.2f} s", fontsize=10)
a2.set_xlim(0, T * 1.35)
save(fig, "min_jerk")

# ---------- 2) jog: กดค้าง → เร่ง, ปล่อย → ผ่อน, ใกล้ limit → เบรก ----------
dt = config.PERIOD_S
fig, (a1, a2) = plt.subplots(1, 2, figsize=(10, 3.6))
m = Mover([0] * 4)
m.jog_joint(2, +1)
ts, vs = [], []
for k in range(int(1.6 / dt)):
    if k == int(1.0 / dt):
        m.release()
    q0 = m.q[2]
    m.step(dt)
    ts.append(k * dt)
    vs.append((m.q[2] - q0) / dt)
a1.plot(ts, vs, color=C1)
a1.axvline(1.0, color=MUTED, lw=0.8, ls="--")
label(a1, 0.6, config.JOG_V, f"ความเร็ว jog {config.JOG_V:.0f} °/s", C1)
a1.annotate("ปล่อยปุ่ม", (1.0, 3), xytext=(6, 0), textcoords="offset points", color=MUTED, fontsize=9)
a1.set_xlabel("เวลา (s)")
a1.set_ylabel("ความเร็วข้อต่อ (°/s)")
a1.set_title(f"กดค้าง: ความเร็วเพิ่มทีละ a·dt (a = {config.JOG_A:.0f} °/s²) · ปล่อย: ลดทีละ a·dt", fontsize=10)
lim = config.Q_MAX[2]
m = Mover([0, 0, lim - 12, 0])
m.jog_joint(2, +1)
qs, vs = [], []
for _ in range(int(1.5 / dt)):
    q0 = m.q[2]
    m.step(dt)
    qs.append(lim - m.q[2])
    vs.append((m.q[2] - q0) / dt)
room = np.linspace(0, 12, 100)
a2.plot(room, np.minimum(config.JOG_V, np.sqrt(config.JOG_A * room)), color=C2, ls="--")
a2.plot(qs, vs, color=C1)
label(a2, 3, math.sqrt(config.JOG_A * 3), "เพดานเบรก v ≤ √(a·ระยะถึง limit)", C2)
k = int(len(qs) * 0.15)
label(a2, qs[k], vs[k], "jog จริง (เริ่มห่าง 12°)", C1)
a2.invert_xaxis()
a2.set_xlabel("ระยะเหลือถึง limit (°)")
a2.set_ylabel("ความเร็ว (°/s)")
a2.set_title("ใกล้ limit: เบรกให้หยุดพอดี ไม่ชน", fontsize=10)
save(fig, "jog_ramp")

# ---------- 3) ส่งล่วงหน้า: ข้อมูลวัดแขนจริง ก่อน/หลังแก้ ----------
fig, axs = plt.subplots(1, 2, figsize=(10, 3.6), sharey=True)
for ax, (title, f) in zip(axs, (("ก่อนแก้: ส่งท่าปัจจุบัน + เวลาคงที่ 60 ms", "jog.json"),
                                ("หลังแก้: ส่งท่าในอีก 80 ms + เวลา 80 ms", "jog_after.json"))):
    a = np.array(json.load(open(OUT / f)), dtype=float)
    t, cmd, act = a[:, 0], a[:, 1], a[:, 2]
    g = np.arange(t[0], t[-1], 0.08)
    vc, va = np.diff(np.interp(g, t, cmd)) / 0.08, np.diff(np.interp(g, t, act)) / 0.08
    mv = np.abs(vc) > 20
    ax.plot(g[1:], va, color=C2, marker="o", ms=4)
    ax.axhline(0, color=MUTED, lw=0.8)
    label(ax, g[1:][np.argmax(va)], va.max(), "ความเร็วจริงของศอก", C2)
    ax.set_title(f"{title}\nช่วงวิ่ง: {np.abs(va[mv]).mean():.1f} ± {np.abs(va[mv]).std():.1f} °/s", fontsize=10)
    ax.set_xlabel("เวลา (s)")
axs[0].set_ylabel("°/s (เฉลี่ยทุก 80 ms)")
save(fig, "lookahead_measured")

# ---------- 4) singularity: |det J| ตามสูตร vs ตัวเลข ----------
fig, (a1, a2) = plt.subplots(1, 2, figsize=(10, 3.6))
q3s = np.linspace(-120, 120, 61)
num = [analyze([0, 40, q, 20])["det_numeric"] / 1e6 for q in q3s]
form = [analyze([0, 40, q, 20])["det_formula"] / 1e6 for q in q3s]
a1.plot(q3s, form, color=C1)
a1.plot(q3s, num, "o", color=C2, ms=4)
label(a1, 60, form[45], "สูตร r·a2·a3·|sin θ3|", C1)
label(a1, -115, num[2], "Jacobian เชิงตัวเลข", C2)
a1.axvspan(-10, 10, color="#e34948", alpha=0.08)
a1.annotate("แขนเหยียดตรง θ3 = 0", (0, 0.02), xytext=(35, 12), textcoords="offset points",
            color=INK, fontsize=9, arrowprops=dict(arrowstyle="->", color=MUTED))
k0 = int(np.argmin(form[:30]))
a1.annotate("ปลายแขนวนกลับมาบนแกนฐาน r = 0", (q3s[k0], 0.02), xytext=(-20, 150), textcoords="offset points",
            color=INK, fontsize=9, arrowprops=dict(arrowstyle="->", color=MUTED))
a1.set_xlabel("มุมศอก q3 (°)  [ไหล่ 40°, คอ 20°]")
a1.set_ylabel("|det J| (×10⁶)")
a1.set_title("สองวิธีให้ค่าเดียวกัน = หลักฐานว่าสูตรถูก", fontsize=10)
q2s = np.linspace(-60, 60, 61)
rs = [analyze([0, q, 90, 60])["r"] for q in q2s]
dets = [analyze([0, q, 90, 60])["det_numeric"] / 1e6 for q in q2s]
a2.plot(rs, dets, "o", color=C2, ms=4)
a2.plot(rs, np.array(rs) * config.A2 * config.A3 / 1e6, color=C1)
label(a2, rs[45], rs[45] * config.A2 * config.A3 / 1e6, "สูตร r·a2·a3 (sin θ3 = 1)", C1)
label(a2, rs[20], dets[20], "Jacobian เชิงตัวเลข", C2)
a2.axvspan(0, 20, color="#e34948", alpha=0.08)
a2.annotate("ปลายแขนบนแกนฐาน r = 0", (0, 0), xytext=(10, 110), textcoords="offset points",
            color=INK, fontsize=9, arrowprops=dict(arrowstyle="->", color=MUTED))
a2.set_xlabel("r = ระยะปลายแขนจากแกนฐาน (mm)  [ศอก 90°]")
a2.set_ylabel("|det J| (×10⁶)")
a2.set_title("det โตตาม r เป็นเส้นตรง", fontsize=10)
save(fig, "singularity_det")
print("ok")
