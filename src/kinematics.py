# Kinematics ของแขน 5DOF — เขียนตามสไลด์วิชา 1352205 (Forward / Inverse Kinematics)
# หน่วยที่ขอบฟังก์ชัน: มุมเป็น "องศา" ทั้ง q และ θ, ระยะเป็น mm
#   q = มุม servo (0 = ตั้งตรง)   θ = มุม DH ตามสไลด์   แปลงกันที่ q_to_theta / theta_to_q เท่านั้น
from dataclasses import dataclass

import numpy as np

from config import A2, A3, D1, D5, JOINTS, Q_MAX, Q_MIN

# ---------- ตาราง DH (สไลด์ FK: ตาราง Link | aᵢ | αᵢ | dᵢ | θᵢ) ----------
#            (a,  alpha, d)
DH_TABLE = [
    (0.0, 90.0, D1),   # Link 1: ฐานหมุนรอบ z₀, ยกขึ้น d1, α₁ = 90° พับแกนไปเป็นแนวนอน
    (A2, 0.0, 0.0),    # Link 2: ไหล่ → ศอก ยาว a2
    (A3, 0.0, 0.0),    # Link 3: ศอก → คอ ยาว a3
    (0.0, 90.0, 0.0),  # Link 4: คอ, α₄ = 90° ให้แกน z₄ ชี้ตามหัว
    (0.0, 0.0, D5),    # Link 5: หัวหมุน roll รอบ z₄, ปลายหัวห่าง d5
]


def dh_matrix(theta, d, a, alpha):
    """สไลด์ FK: Aᵢ = Rot(z,θᵢ) · Trans(z,dᵢ) · Trans(x,aᵢ) · Rot(x,αᵢ)  (มุมเป็นองศา)"""
    ct, st = np.cos(np.radians(theta)), np.sin(np.radians(theta))
    ca, sa = np.cos(np.radians(alpha)), np.sin(np.radians(alpha))
    # เมทริกซ์ Aᵢ แบบเต็มตามสไลด์ (คูณ 4 เมทริกซ์ข้างบนแล้ว)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,      sa,       ca,      d],
        [0.0,     0.0,      0.0,    1.0],
    ])


def q_to_theta(q):
    """มุม servo q → มุม DH θ (ตาราง 6.1 ใน design) — offset มุมมีที่นี่ที่เดียว"""
    q1, q2, q3, q4, q5 = q
    return [
        q1,          # θ₁ = q₁       ฐานบวกทวนเข็ม = θ บวกรอบ z₀
        90.0 - q2,   # θ₂ = 90° − q₂ ตั้งตรง = 90° จากแนวนอน, เอนหน้า = θ₂ ลด
        -q3,         # θ₃ = −q₃      ศอกพับไปหน้า = หมุนตามเข็มในระนาบ r–s
        90.0 - q4,   # θ₄ = 90° − q₄ เพราะ α₄ = 90° ต้องบวก 90° ให้หัวชี้ตามท่อนตอนตรง
        q5,          # θ₅ = q₅       roll ไม่มีผลต่อตำแหน่ง
    ]


def theta_to_q(theta):
    """มุม DH θ → มุม servo q (ย้อนตาราง 6.1)"""
    t1, t2, t3, t4, t5 = theta
    return [t1, 90.0 - t2, -t3, 90.0 - t4, t5]


@dataclass
class FK:
    x: float          # ตำแหน่งปลายหัว (mm) ในเฟรมฐาน
    y: float
    z: float
    phi: float        # มุมหัวเทียบแนวนอนในระนาบแขน (องศา): 90 = ชี้ฟ้า, −90 = ชี้พื้น
    T: np.ndarray     # T⁰₅ (4×4)
    frames: list      # [T⁰₀, T⁰₁, ..., T⁰₅] ไว้วาดภาพและหา Jacobian


def wrap180(angle):
    """ทำให้มุมอยู่ในช่วง (−180°, 180°]"""
    return -((-angle + 180.0) % 360.0 - 180.0)


def forward_kinematics(q):
    """สไลด์ FK: T⁰₅ = A₁ A₂ A₃ A₄ A₅"""
    theta = q_to_theta(q)
    T = np.eye(4)                       # T⁰₀ = เมทริกซ์เอกลักษณ์ (เฟรมฐาน)
    frames = [T]
    for (a, alpha, d), th in zip(DH_TABLE, theta):
        T = T @ dh_matrix(th, d, a, alpha)   # T⁰ᵢ = T⁰ᵢ₋₁ · Aᵢ
        frames.append(T)
    x, y, z = T[:3, 3]                  # ตำแหน่งปลายหัว = คอลัมน์ที่ 4 ของ T⁰₅
    # มุมหัว φ: ท่อนแขนเอียง θ₂ + θ₃ จากแนวนอน, แกนหัว z₄ เอียงเพิ่ม θ₄ − 90° (เพราะ α₄ = 90°)
    phi = wrap180(theta[1] + theta[2] + theta[3] - 90.0)
    return FK(x, y, z, phi, T, frames)


# ======================================================================
# Inverse Kinematics — สไลด์ IK: geometric approach + kinematic decoupling
# แขน 5DOF คุมทิศได้แค่มุมก้มเงยของหัว จึงรับ (x, y, z) + φ (มุมหัวเทียบแนวนอน)
# ขอบเขต: θ₁ = atan2(y, x) คำตอบเดียว → ใช้กับท่าที่ข้อมือ+ปลายหัวอยู่ด้านหน้าฐาน
# ======================================================================

@dataclass
class IK:
    ok: bool
    q: list | None    # มุม servo 5 ข้อ (องศา) เมื่อ ok
    reason: str       # เหตุผลภาษาไทยเมื่อไม่ ok ("" เมื่อ ok)


def check_limits(q):
    """คืน None ถ้าทุกข้ออยู่ใน limit ไม่งั้นคืนข้อความบอกทุกข้อที่เกิน"""
    bad = [
        f"{j['name']} เกิน limit: {v:.1f}° (ช่วง {lo:.1f}…{hi:.1f}°)"
        for j, v, lo, hi in zip(JOINTS, q, Q_MIN, Q_MAX)
        if not lo <= v <= hi
    ]
    return " · ".join(bad) if bad else None


def inverse_kinematics(x, y, z, phi=-90.0, elbow_up=True, roll=0.0, theta1_prev=0.0):
    """ตำแหน่งปลายหัว (x, y, z) mm + มุมหัว φ (องศา) → มุม servo q"""
    phi_r = np.radians(phi)

    # θ₁: สไลด์เขียน θ₁ = Atan2(xc, yc) แบบ Spong (อาร์กิวเมนต์ = cos, sin)
    #     = np.arctan2(yc, xc) ของ numpy · wrist center อยู่ในระนาบเดียวกับปลายหัว จึงใช้ x, y ได้เลย
    if x * x + y * y < 1e-9:
        theta1 = theta1_prev      # สไลด์: singular configuration — θ₁ เป็นค่าใดก็ได้ จึงใช้ค่าเดิม
    else:
        theta1 = np.degrees(np.arctan2(y, x))

    r = np.sqrt(x * x + y * y)               # สไลด์: r² = xc² + yc² (ระยะแนวนอนจากแกนฐาน)
    # kinematic decoupling (สไลด์: oc = o − d·R·k): ถอยจากปลายหัวกลับไปที่ข้อมือ ระยะ d5 ตามทิศหัว φ
    rc = r - D5 * np.cos(phi_r)              # ระยะแนวนอนของข้อมือ
    s = z - D5 * np.sin(phi_r) - D1          # สไลด์: s = zc − d1 (ความสูงข้อมือเหนือไหล่)

    # สไลด์: กฎ cosine  D = cos θ₃ = (r² + s² − a2² − a3²) / (2·a2·a3)
    D = (rc * rc + s * s - A2 * A2 - A3 * A3) / (2 * A2 * A3)
    if abs(D) > 1 + 1e-9:
        dist = np.hypot(rc, s)
        return IK(False, None, f"เอื้อมไม่ถึง: ข้อมือห่างจากไหล่ {dist:.1f} mm "
                               f"แต่ท่อนแขนยาวได้ {abs(A2 - A3):.1f}…{A2 + A3:.1f} mm")
    D = float(np.clip(D, -1.0, 1.0))         # ตัดเศษปัดเลขตอนแขนเหยียดตรงพอดี (D = ±1)

    # สไลด์: θ₃ = Atan2(D, ±√(1 − D²)) → numpy: arctan2(±√(1 − D²), D)
    #   elbow up = เครื่องหมายลบ (θ₃ < 0 = ศอกพับไปหน้า = q₃ บวก)
    sign = -1.0 if elbow_up else 1.0
    theta3 = np.arctan2(sign * np.sqrt(1 - D * D), D)

    # สไลด์: θ₂ = Atan2(r, s) − Atan2(a2 + a3·cos θ₃, a3·sin θ₃) → numpy สลับลำดับอาร์กิวเมนต์
    theta2 = np.arctan2(s, rc) - np.arctan2(A3 * np.sin(theta3), A2 + A3 * np.cos(theta3))

    # θ₄: มุมหัว φ = θ₂ + θ₃ + θ₄ − 90° (ดู forward_kinematics) → ย้ายข้าง
    theta4 = phi_r - theta2 - theta3 + np.pi / 2
    theta5 = roll                            # roll ที่ผู้ใช้กำหนด ไม่มีผลต่อตำแหน่ง

    theta = [theta1, np.degrees(theta2), np.degrees(theta3), np.degrees(theta4), theta5]
    q = [wrap180(v) for v in theta_to_q(theta)]
    reason = check_limits(q)
    if reason:
        return IK(False, None, reason)
    return IK(True, [float(v) for v in q], "")
