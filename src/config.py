# ค่าตัวเลขทั้งหมดของแขนอยู่ที่ไฟล์นี้ที่เดียว (copy มาจาก RobotControl/v2 วันที่ 2026-11-19)
# คาลิเบรตใหม่: Calibrate.bat → กดบันทึก → เขียนลงไฟล์นี้

# ---------- ความยาวท่อน (mm) — ชื่อตามตาราง DH ในสไลด์ ----------
D1 = 96.7    # ฐาน → แกนไหล่ (d1)
A2 = 137.0   # ไหล่ → ศอก (a2)
A3 = 98.3    # ศอก → คอ (a3)
D5 = 50.0    # คอ → ปลายแขน (d5) — ถอด servo หัวออกแล้ว วัดใหม่ 2026-11-19 (เดิม 79.7 มีหัว)

# ---------- servo SCS15 ----------
RAW_MAX = 1023
DEG_RANGE = 250.0          # 0..1023 tick ≈ 250° (วัดด้วยตา ±15° — วัดใหม่ในขั้น 6)
TICK_PER_DEG = RAW_MAX / DEG_RANGE

# used: False = ข้อนี้ไม่ได้ต่อ (J5 หัวถอดออก 2026-11-19) → ล็อก q = 0, ไม่ส่ง/ไม่อ่าน servo นี้
#       สมการยังเป็น 5 ข้อ เพราะ J5 หมุนรอบแกนท่อนสุดท้าย ไม่มีผลต่อตำแหน่งปลายแขน
# center: raw ตอนท่อนนั้นตั้งตรงชี้ฟ้า = q 0° (คาลิเบรตด้วย calib_app.py)
# dir: +1/−1 ให้ q บวก = ก้มไปข้างหน้า / ฐานหันซ้าย (ทวนเข็มมองจากบน) / หัวเอียงขวา
JOINTS = [
    {"name": "J1 ฐาน", "id": 4, "dir": -1, "center": 484, "raw_min": 10, "raw_max": 1012, "used": True},
    {"name": "J2 ไหล่", "id": 9, "dir": -1, "center": 506, "raw_min": 107, "raw_max": 883, "used": True},
    {"name": "J3 ศอก", "id": 7, "dir": +1, "center": 527, "raw_min": 10, "raw_max": 1012, "used": True},
    {"name": "J4 คอ", "id": 2, "dir": -1, "center": 522, "raw_min": 202, "raw_max": 821, "used": True},
    {"name": "J5 หัว", "id": 1, "dir": +1, "center": 511, "raw_min": 20, "raw_max": 1003, "used": False},
]


USED = [j["used"] for j in JOINTS]       # ข้อไหนต่ออยู่จริง


def raw_to_q(joint, raw):
    """q = dir × (raw − center) / tick_per_deg  (องศา)"""
    return joint["dir"] * (raw - joint["center"]) / TICK_PER_DEG


def q_to_raw(joint, q):
    """raw = center + dir × q × tick_per_deg แล้วบีบให้อยู่ใน raw limit"""
    raw = round(joint["center"] + joint["dir"] * q * TICK_PER_DEG)
    return max(joint["raw_min"], min(joint["raw_max"], raw))


# ช่วงมุม q ที่สั่งได้ (องศา) คำนวณจาก raw limit
Q_MIN = [min(raw_to_q(j, j["raw_min"]), raw_to_q(j, j["raw_max"])) for j in JOINTS]
Q_MAX = [max(raw_to_q(j, j["raw_min"]), raw_to_q(j, j["raw_max"])) for j in JOINTS]

# ---------- การเชื่อมต่อ ----------
DIRECT_PORT = "COM11"      # Waveshare Bus Servo Adapter (A) จัมเปอร์ B (USB-SERVO)
SERVO_BAUD = 1_000_000     # บัส servo SCS15 (1 Mbps ค่าเริ่มต้น)
SERVO_TIMEOUT_S = 0.02     # รอคำตอบ servo นานสุด (เผื่อ USB หน่วง)
MIN_MOVE_MS = 300          # เวลาเคลื่อนที่ต่ำสุดของคำสั่ง "ค้างที่เดิม"
VOLT_MIN = 6.0             # ไฟ servo ต่ำกว่านี้ไม่ส่งคำสั่ง

# ---------- ป้อนท่าแบบนุ่ม (motion.py / driver.py) ----------
PERIOD_S = 0.04            # ป้อนท่าใหม่ทุก 40 ms (25 ครั้ง/วินาที)
STREAM_MS = 60             # เวลาที่บอก servo ให้ไปถึงแต่ละท่า (ยาวกว่ารอบนิดหน่อย servo จะเกลี่ยต่อเนื่อง)
JUMP_GUARD_DEG = 3.0       # กันบั๊ก: ข้อไหนกระโดดเกินนี้ในคำสั่งที่สั้นกว่า MIN_MOVE_MS → ไม่ส่ง
V_JOINT = 20.0             # ความเร็วข้อต่อสูงสุด (°/s) ค่าเริ่มต้น (ผู้ใช้บอก 10 ช้าไป)
V_MAX_LIMIT = 30.0         # ปรับใน GUI ได้ไม่เกินนี้
A_JOINT = 30.0             # ความเร่งข้อต่อ (°/s²) — เร่งออกตัว/ผ่อนตอนหยุด
V_CART = 20.0              # ความเร็วปลายแขนตอนวิ่งเส้นตรง / jog XYZ (mm/s)
IK_STEP_MM = 1.0           # ความละเอียดการแบ่งเส้นตรงไปคำนวณ IK

# ---------- ชดเชยแขนหยุดไม่ถึงเป้า (driver.py) ----------
CORR_TOL = 0.5             # error ไม่เกินนี้ถือว่าถึง (องศา)
CORR_GAIN = 0.8            # ชดเชย 0.8 × error ต่อรอบ
CORR_ROUNDS = 3            # สูงสุดกี่รอบ
CORR_MAX = 5.0             # ชดเชยรวมไม่เกิน ±5° ต่อข้อ
CORR_GIVE_UP = 5.0         # error เกินนี้ = ไปไม่ถึงเลย (ถูกขวาง/ไฟตก) → ไม่ชดเชย
CORR_WAIT_S = 0.4          # รอแขนนิ่งหลังถึงเป้าก่อนวัด error
