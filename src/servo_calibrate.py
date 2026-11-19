# คาลิเบรตแขนใหม่ทีละข้อผ่าน USB adapter (COM11) — หา center / ทิศ / limit แล้วจดลง config.py
# ไม่เขียนหน่วยความจำถาวร (EEPROM) ของ servo — เขียนแค่ torque และ goal
#
#   read            อ่าน raw / q / torque / ไฟ ทุกข้อที่ต่ออยู่ (อ่านอย่างเดียว)
#   free  j2|all    ปิด torque  ⚠ ข้อนั้นหมดแรงทันที — ประคองแขนก่อน
#   hold  j2|all    ค้างที่ตำแหน่งตอนนี้ แล้วเปิด torque (แขนไม่ขยับ)
#   sweep j2 [วินาที] จด raw ต่ำสุด/สูงสุด ขณะผู้ใช้ขยับข้อนั้นด้วยมือไปสุดสองข้าง (ต้อง free ก่อน)
#
# ขั้นตอน (ทีละข้อ ประคองแขนตลอด):
#   1) free → จับท่อนนั้นตั้งตรงชี้ฟ้า → read  = center
#   2) ขยับไปทางบวก (ก้มหน้า / ฐานหันซ้าย / หัวเอียงขวา) → read : raw เพิ่ม = dir +1, ลด = dir −1
#   3) sweep → ขยับไปสุดสองข้างเท่าที่ไม่ชนอะไร → limit (หดเข้ามา 10 tick)
#   4) hold
import re
import sys
import time
from pathlib import Path

import serial

import config
from scs import ScsBus

LIMIT_MARGIN = 10      # หด limit เข้ามาจากจุดที่ขยับไปถึงจริง กันชน


def pick_joints(name):
    """'j1'..'j5' หรือ 'all' (เฉพาะข้อที่ต่ออยู่) → list ของ joint dict"""
    used = [j for j in config.JOINTS if j["used"]]
    if name == "all":
        return used
    for i, j in enumerate(config.JOINTS):
        if name == f"j{i + 1}" and j["used"]:
            return [j]
    sys.exit(f"ไม่รู้จักข้อ '{name}' — ใช้ j1..j{len(config.JOINTS)} (ที่ต่ออยู่) หรือ all")


def read_all(bus, joints):
    rows = []
    for j in joints:
        got = bus.read_pos_volt(j["id"])
        t = bus.read(j["id"], 40, 1)
        rows.append({
            "name": j["name"], "id": j["id"],
            "raw": got[0] if got else None,
            "q": round(config.raw_to_q(j, got[0]), 1) if got else None,   # ตามค่า center/dir ใน config ตอนนี้
            "volt": got[1] if got else None,
            "torque": bool(t[0]) if t else None,
        })
    return rows


def free(bus, joints):
    for j in joints:
        bus.torque(j["id"], False)


def hold(bus, joints):
    for j in joints:
        got = bus.read_pos_volt(j["id"])
        if got:
            bus.write_pos(j["id"], got[0], config.MIN_MOVE_MS)   # เป้า = ที่อยู่ตอนนี้ → ไม่ขยับ
        bus.torque(j["id"], True)


def sweep(bus, joint, seconds=15.0, period_s=0.05):
    """อ่าน raw ซ้ำ ๆ จนหมดเวลา → (ต่ำสุด, สูงสุด) ที่เห็น"""
    lo, hi = None, None
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        got = bus.read_pos_volt(joint["id"])
        if got:
            lo = got[0] if lo is None else min(lo, got[0])
            hi = got[0] if hi is None else max(hi, got[0])
        time.sleep(period_s)
    return lo, hi


def suggest_limits(lo, hi):
    return lo + LIMIT_MARGIN, hi - LIMIT_MARGIN


# ---------- เขียนค่าที่คาลิเบรตลง config.py ----------
CONFIG_PY = Path(__file__).resolve().parent / "config.py"


def update_config_py(text, values):
    """values = {id: (dir, center, raw_min, raw_max)} → แก้แถว JOINTS ของ id นั้นใน config.py"""
    for sid, (d, c, lo, hi) in values.items():
        pat = rf'"id": {sid}, "dir": [+-]1, "center": \d+, "raw_min": \d+, "raw_max": \d+'
        new = f'"id": {sid}, "dir": {d:+d}, "center": {c}, "raw_min": {lo}, "raw_max": {hi}'
        text, n = re.subn(pat, new, text)
        if n != 1:
            raise ValueError(f"หาแถว ID {sid} ใน config.py ไม่เจอ")
    return text


def save_calibration(values, py_path=CONFIG_PY):
    """ตรวจค่าก่อน แล้วเขียน config.py (ค่าผิดรูป → ValueError ไม่เขียนอะไรเลย)"""
    for sid, (d, c, lo, hi) in values.items():
        if d not in (1, -1) or not 0 <= lo < c < hi <= config.RAW_MAX:
            raise ValueError(f"ID {sid}: ต้องได้ dir ±1 และ 0 ≤ ต่ำสุด < center < สูงสุด ≤ {config.RAW_MAX}")
    py = update_config_py(Path(py_path).read_text(encoding="utf-8"), values)
    Path(py_path).write_text(py, encoding="utf-8")


def save_standby(q, py_path=CONFIG_PY):
    """เขียนท่า Standby ลงบรรทัด STANDBY_Q ใน config.py (ข้อไหนเกิน limit / nan → ValueError ไม่เขียนอะไรเลย)"""
    q = [round(float(v), 1) for v in q]
    for i, (j, v) in enumerate(zip(config.JOINTS, q)):
        if j["used"] and not config.Q_MIN[i] <= v <= config.Q_MAX[i]:      # nan ก็ไม่ผ่านเงื่อนไขนี้
            raise ValueError(f"{j['name']} = {v}° นอกช่วง {config.Q_MIN[i]:.1f}…{config.Q_MAX[i]:.1f}°")
    text = Path(py_path).read_text(encoding="utf-8")
    text, n = re.subn(r"^STANDBY_Q = \[[^\]]*\]", f"STANDBY_Q = {q}", text, flags=re.M)
    if n != 1:
        raise ValueError("หาบรรทัด STANDBY_Q ใน config.py ไม่เจอ")
    Path(py_path).write_text(text, encoding="utf-8")


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__ or "ใช้: servo_calibrate.py read | free j2 | hold j2 | sweep j2 [วินาที]")
    cmd = sys.argv[1]
    target = sys.argv[2] if len(sys.argv) > 2 else "all"
    ser = serial.Serial(config.DIRECT_PORT, config.SERVO_BAUD, timeout=0.002)
    bus = ScsBus(ser)
    try:
        bus.detect_echo()
        if cmd == "read":
            for r in read_all(bus, pick_joints(target)):
                print(f"{r['name']:8} ID {r['id']:2}  raw {r['raw']!s:5} q {r['q']!s:6}  "
                      f"torque {r['torque']!s:5}  ไฟ {r['volt'] and round(r['volt'], 1)} V")
        elif cmd == "free":
            free(bus, pick_joints(target))
            print(f"ปิด torque: {target}")
        elif cmd == "hold":
            hold(bus, pick_joints(target))
            print(f"ค้างที่เดิม + เปิด torque: {target}")
        elif cmd == "sweep":
            seconds = float(sys.argv[3]) if len(sys.argv) > 3 else 15.0
            joint = pick_joints(target)[0]
            print(f"{joint['name']}: ขยับด้วยมือไปสุดสองข้าง ภายใน {seconds:.0f} วินาที ...")
            lo, hi = sweep(bus, joint, seconds)
            if lo is None:
                print("อ่านไม่ได้เลย")
            else:
                print(f"เห็น raw {lo}–{hi} → แนะนำ limit {suggest_limits(lo, hi)}")
        else:
            sys.exit(f"ไม่รู้จักคำสั่ง {cmd}")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
