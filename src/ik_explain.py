# อธิบายผล IK ให้ผู้ใช้: ไปได้ไหม ถ้าไม่ได้เพราะอะไร และควรแก้ตรงไหน (ไม่แตะ GUI → test ได้)
#
# ไล่ตามขั้นตอน IK ในสไลด์:
#   1) ถอยจากปลายแขนไปข้อมือ ระยะ a4 ตามทิศ φ → ข้อมือต้องห่างแกนไหล่ |a2 − a3| … a2 + a3 (กฎ cosine ถึงมีคำตอบ)
#   2) ลองครบ 4 ท่า: หันหน้า/หันหลัง (θ₁ กับ π + θ₁) × ศอกขึ้น/ลง → ท่าไหนติด limit ข้อไหน เกินกี่องศา
#   3) ที่ตำแหน่งนี้ φ ช่วงไหนไปได้บ้าง (ท่าที่โปรแกรมใช้จริง)
# การเลือกท่า (choose) ใช้กฎเดิมของหน้าต่าง: φ ที่ขอ ศอกขึ้นก่อน → ศอกลง → ไม่ได้ก็หา φ ที่ใกล้ที่สุดที่ไปได้
from collections import Counter
from dataclasses import dataclass, field

import config
from kinematics import REACH_MAX, REACH_MIN, IKGeo, solve_ik

TIP_MAX = REACH_MAX + config.A4            # ปลายแขนห่างแกนไหล่ได้ไกลสุด (แขนเหยียดตรง) = 285.3 mm
PHI_STEP = 1                               # หา φ ที่ไปได้ทีละ 1°


@dataclass
class Option:
    """ท่าหนึ่งในสี่ท่าที่ φ ที่ผู้ใช้ขอ"""
    name: str
    flip: bool
    elbow_up: bool
    geo: IKGeo
    bad: list = field(default_factory=list)   # [(ข้อ i, มุมที่ต้องหมุน, lo, hi)]

    @property
    def ok(self):
        return self.geo.q is not None and not self.bad

    def why(self):
        """เหตุผลสั้น ๆ ว่าทำไมท่านี้ใช้ไม่ได้ ("" = ใช้ได้)"""
        g = self.geo
        if g.q is None:
            if g.dist > REACH_MAX:
                return (f"ข้อมือห่างไหล่ {g.dist:.1f} mm แต่ท่อนแขน a2 + a3 ยืดได้สุด {REACH_MAX:.1f} mm "
                        f"(ขาด {g.dist - REACH_MAX:.1f} mm)")
            return (f"ข้อมือใกล้ไหล่ {g.dist:.1f} mm แต่ท่อนแขนพับได้ใกล้สุด {REACH_MIN:.1f} mm "
                    f"(ใกล้เกิน {REACH_MIN - g.dist:.1f} mm)")
        return " · ".join(
            f"{config.JOINTS[i]['name']} ต้องหมุน {v:.1f}° แต่ทำได้ {lo:.1f}…{hi:.1f}° (เกิน {_excess(v, lo, hi):.1f}°)"
            for i, v, lo, hi in self.bad)


@dataclass
class Diagnosis:
    x: float
    y: float
    z: float
    phi: float
    q: list | None                         # ท่าที่จะใช้ (None = ไปไม่ได้เลย)
    phi_used: float | None                 # φ ของท่าที่จะใช้ (อาจไม่ใช่ φ ที่ขอ)
    options: list                          # 4 ท่าที่ φ ที่ขอ
    phi_ranges: list                       # [(lo, hi)] φ ที่ไปได้ที่ตำแหน่งนี้
    tip_dist: float                        # ปลายแขนห่างแกนไหล่
    headline: str = ""
    reasons: list = field(default_factory=list)
    hints: list = field(default_factory=list)
    warnings: list = field(default_factory=list)

    @property
    def exact(self):
        """ไปได้ด้วย φ ที่ขอพอดี"""
        return self.q is not None and self.phi_used == self.phi


def _excess(v, lo, hi):
    return lo - v if v < lo else v - hi


def _limits(q):
    return [(i, v, lo, hi) for i, (v, lo, hi) in enumerate(zip(q, config.Q_MIN, config.Q_MAX)) if not lo <= v <= hi]


def options_at(x, y, z, phi, theta1_prev=0.0):
    """สี่ท่าตามสไลด์: θ₁ = Atan2 (หันหน้า) / π + Atan2 (หันหลัง) × ศอกขึ้น/ลง"""
    out = []
    for flip, side in ((False, "หันหน้า"), (True, "หันหลัง")):
        for up in (True, False):
            g = solve_ik(x, y, z, phi, up, theta1_prev, flip=flip)
            out.append(Option(f"{side} · ศอก{'ขึ้น' if up else 'ลง'}", flip, up, g, _limits(g.q) if g.q else []))
    return out


def _usable(x, y, z, phi, theta1_prev):
    """ท่าที่โปรแกรมใช้จริง (กฎเดียวกับ inverse_kinematics): ศอกขึ้นก่อน แล้วศอกลง"""
    for up in (True, False):
        g = solve_ik(x, y, z, phi, up, theta1_prev)
        if g.q is not None and not _limits(g.q):
            return g.q
    return None


def phi_ranges(x, y, z, theta1_prev=0.0):
    """ช่วง φ ที่ไปได้ (ทีละ 1°) → [(lo, hi), ...]"""
    ranges, start, prev = [], None, None
    for p in range(-180, 181, PHI_STEP):
        if _usable(x, y, z, p, theta1_prev) is not None:
            if start is None:
                start = p
            prev = p
        elif start is not None:
            ranges.append((start, prev))
            start = None
    if start is not None:
        ranges.append((start, prev))
    return ranges


def fmt_ranges(ranges):
    return ", ".join(f"{lo}°" if lo == hi else f"{lo}…{hi}°" for lo, hi in ranges)


def diagnose(x, y, z, phi, theta1_prev=0.0):
    r = (x * x + y * y) ** 0.5
    tip_dist = (r * r + (z - config.D1) ** 2) ** 0.5
    opts = options_at(x, y, z, phi, theta1_prev)
    ranges = phi_ranges(x, y, z, theta1_prev)
    q, used = _usable(x, y, z, phi, theta1_prev), phi
    if q is None and ranges:                # φ ที่ขอไม่ได้ → φ ที่ใกล้ที่สุดที่ไปได้ (เหมือนหน้าต่างเดิม)
        for p in sorted(range(-180, 181, PHI_STEP), key=lambda p: abs(p - phi)):
            q = _usable(x, y, z, p, theta1_prev)
            if q is not None:
                used = p
                break
    d = Diagnosis(x, y, z, phi, q, used if q is not None else None, opts, ranges, tip_dist)
    front = [o for o in opts if not o.flip]

    if d.exact:
        d.headline = "✅ ไปได้"
    elif q is not None:
        d.headline = f"⚠ φ = {phi:.0f}° ไปไม่ได้ → ใช้ φ = {used}° แทน (ใกล้ที่สุดที่ไปถึง)"
        d.reasons.append(f"ที่ φ = {phi:.0f}°: " + _front_reason(front))
        d.hints.append(f"ที่ตำแหน่งนี้ φ ที่ไปได้: {fmt_ranges(ranges)}")
    else:
        d.headline = "❌ ไปจุดนี้ไม่ได้"
        if tip_dist > TIP_MAX:
            d.reasons.append(f"ไกลเกินไป: ปลายแขนห่างแกนไหล่ {tip_dist:.1f} mm แต่แขนเหยียดสุดได้ "
                             f"{TIP_MAX:.1f} mm (a2 + a3 + a4) — ขาด {tip_dist - TIP_MAX:.1f} mm")
            d.hints.append(f"ขยับเป้าเข้าหาแกนไหล่ (0, 0, {config.D1:.1f}) อย่างน้อย {tip_dist - TIP_MAX:.1f} mm")
        else:
            why, hint = _dominant_block(x, y, z, theta1_prev)
            d.reasons.append(f"ทุกมุม φ ไปไม่ได้ — ส่วนใหญ่เพราะ {why}")
            d.reasons.append(f"ที่ φ = {phi:.0f}°: " + _front_reason(front))
            d.hints.append(hint)
        back = [o for o in opts if o.flip and o.ok]
        if back:
            d.hints.append(f"ท่า{back[0].name} (หมุนฐาน 180° เอื้อมข้ามหัว) ไปได้ แต่โปรแกรมยังไม่ใช้ท่านี้กับแขนจริง")

    if z < 0:
        d.warnings.append("เป้าต่ำกว่าระดับฐาน (z < 0) — limit ยังไม่กันชนโต๊ะ ระวังปลายแขนชน")
    return d


def _front_reason(front):
    """เหตุผลของท่าหันหน้า (ศอกขึ้น/ลง) — ถ้าเหตุผลเหมือนกันรวมเป็นอันเดียว"""
    whys = [(o.name, o.why()) for o in front if not o.ok]
    if len({w for _, w in whys}) == 1:
        return whys[0][1]
    return " | ".join(f"{n}: {w}" for n, w in whys)


def _dominant_block(x, y, z, theta1_prev):
    """ไล่ทุก φ ดูว่าท่าหันหน้าติดอะไรบ่อยสุด → (เหตุผล, คำแนะนำ)"""
    count = Counter()
    for p in range(-180, 181, 5):
        for up in (True, False):
            g = solve_ik(x, y, z, p, up, theta1_prev, flip=False)
            if g.q is None:
                count["reach"] += 1
            else:
                for i, *_ in _limits(g.q):
                    count[i] += 1
    if not count:
        return "ไม่ทราบสาเหตุ", "ลองเป้าอื่น"
    what, _ = count.most_common(1)[0]
    if what == "reach":
        return (f"ข้อมือเอื้อมไม่ถึง (ท่อนแขน a2, a3 ทำระยะได้ {REACH_MIN:.1f}…{REACH_MAX:.1f} mm)",
                "เป้าอยู่ชิดตัวแขนเกินไป ลองขยับเป้าออกห่างฐานหรือสูงขึ้น")
    j, lo, hi = config.JOINTS[what], config.Q_MIN[what], config.Q_MAX[what]
    why = f"{j['name']} เกิน limit ({lo:.1f}…{hi:.1f}°)"
    if what == 0:
        return why, f"เป้าอยู่ด้านหลังฐาน แต่ฐานหมุนได้แค่ {lo:.1f}…{hi:.1f}° — ลองเป้าที่อยู่ด้านหน้าหรือด้านข้าง"
    return why, f"ลองเป้าที่ไม่ต้องให้{j['name']}หมุนเกิน {lo:.1f}…{hi:.1f}° เช่นขยับเป้าเข้าใกล้หรือสูงขึ้น"
