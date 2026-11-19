# ตรวจ singularity ของแขน 4 ข้อ (ฐาน, ไหล่, ศอก, คอ) แบบพิสูจน์ได้
#
# แขนคุมได้ 4 ค่า p = (x, y, z, φ) ด้วย 4 ข้อ → Jacobian J = ∂p/∂θ เป็น 4×4 (จัตุรัส)
# singularity = det J = 0 : มีทิศที่ปลายแขนขยับไปไม่ได้ ไม่ว่าจะหมุนข้อไหน
#
# พิสูจน์สูตรปิด: แยก p เป็น 2 ชั้น
#   1) ฐาน: (x, y) = (r cos θ1, r sin θ1)  →  det ∂(x,y)/∂(θ1,r) = r
#   2) แขนระนาบ 3 ข้อ (θ2, θ3, θ4) → (r, z, φ)  →  det = a2 · a3 · sin θ3   (สูตรแขนระนาบ 3R, d5 ตัดกันหาย)
#   คูณกัน:  |det J| = r · a2 · a3 · |sin θ3|
# ⇒ singularity มี 2 แบบ:
#   • r = 0       ปลายแขนอยู่บนแกนฐาน  → หมุนฐานแล้วปลายแขนไม่ขยับ (เสีย 1 ทิศ)
#   • sin θ3 = 0  แขนเหยียดตรง/พับสุด   → ดึงเข้า-ดันออกตามแนวแขนไม่ได้ (เสีย 1 ทิศ)
# หลักฐาน: det_numeric (คำนวณ J ด้วยการขยับทีละข้อนิดเดียว) ต้องเท่ากับ det_formula — test ตรวจทุกท่า
import math

import numpy as np

import config
from kinematics import forward_kinematics, q_to_theta

R_WARN_MM = 20.0          # ปลายแขนห่างแกนฐานน้อยกว่านี้ = ใกล้ singularity ฐาน
ELBOW_WARN_DEG = 10.0     # ศอกห่างจากเหยียดตรง/พับสุดน้อยกว่านี้ = ใกล้ singularity ศอก
STEP_DEG = 1e-3           # ขยับทีละนิดสำหรับ Jacobian เชิงตัวเลข


def _pose(q):
    f = forward_kinematics(q)
    return np.array([f.x, f.y, f.z, math.radians(f.phi)])


def jacobian(q):
    """J 4×4 = ∂(x, y, z, φ)/∂(θ1..θ4) [mm/rad, rad/rad] ด้วย central difference"""
    J = np.zeros((4, 4))
    for k in range(4):
        dq = [0.0] * 5
        dq[k] = STEP_DEG
        plus = _pose([a + b for a, b in zip(q, dq)])
        minus = _pose([a - b for a, b in zip(q, dq)])
        d = plus - minus
        d[3] = (d[3] + math.pi) % (2 * math.pi) - math.pi        # φ ข้าม ±180°
        J[:, k] = d / math.radians(2 * STEP_DEG)
    return J


def analyze(q):
    """ผลตรวจ: det สองทาง, r, θ3, ค่า singular, และคำเตือน (list ว่าง = ปลอดภัย)"""
    f = forward_kinematics(q)
    r = math.hypot(f.x, f.y)
    theta3 = q_to_theta(q)[2]
    J = jacobian(q)
    sv = np.linalg.svd(J[:3], compute_uv=False)                   # ส่วนตำแหน่ง (3×4) — mm/rad
    out = {
        "det_numeric": float(abs(np.linalg.det(J))),
        "det_formula": r * config.A2 * config.A3 * abs(math.sin(math.radians(theta3))),
        "r": r,
        "theta3": theta3,
        "sigma_min": float(sv[-1]),
        "warnings": [],
    }
    if r < R_WARN_MM:
        out["warnings"].append(f"ปลายแขนเกือบอยู่บนแกนฐาน (r = {r:.1f} mm < {R_WARN_MM:.0f}) — หมุนฐานแล้วปลายแขนไม่ขยับ")
    bend = min(abs(theta3) % 180, 180 - abs(theta3) % 180)
    if bend < ELBOW_WARN_DEG:
        out["warnings"].append(f"แขนเกือบเหยียดตรง/พับสุด (θ3 = {theta3:.1f}°) — ดึงเข้า-ดันออกตามแนวแขนไม่ได้")
    return out


def getting_worse(q_old, q_new):
    """เตือนเมื่อ q_new อยู่ในโซนเตือน และเข้าใกล้ singularity แบบนั้นมากกว่า q_old ("" = ไม่เป็นไร, ถอยออกได้)"""
    a, b = analyze(q_old), analyze(q_new)
    for w in b["warnings"]:
        if "แกนฐาน" in w and b["r"] < a["r"]:
            return w
        if "เหยียด" in w and abs(math.sin(math.radians(b["theta3"]))) < abs(math.sin(math.radians(a["theta3"]))):
            return w
    return ""


def report(q):
    """ข้อความหลายบรรทัดสำหรับ GUI: ผลตรวจ + หลักฐาน"""
    a = analyze(q)
    head = "⚠ ใกล้ SINGULARITY\n" + "\n".join("• " + w for w in a["warnings"]) if a["warnings"] else "✅ ไม่ใกล้ singularity"
    return (f"{head}\n\n"
            f"|det J| (ตัวเลข)         = {a['det_numeric']:12.0f}\n"
            f"r·a2·a3·|sin θ3| (สูตร) = {a['det_formula']:12.0f}\n"
            f"   r = {a['r']:.1f} mm,  θ3 = {a['theta3']:.1f}°\n"
            f"σ_min (ตำแหน่ง)          = {a['sigma_min']:.1f} mm/rad  (≈ 0 = มีทิศที่ขยับไม่ได้)")
