# คุยกับแขน: DirectLink = PC คุม servo ตรงผ่าน Waveshare Bus Servo Adapter (A) จัมเปอร์ B (COM11)
#             SimLink    = แขนจำลอง (ไม่ต่อแขน) มีเมธอดเหมือนกัน
# เมธอด: send_q(q, time_ms) → bool, stop(), torque(on), online(), close() · ค่า: q_actual, volt, message, simulated
import math
import threading
import time

import serial

import config
from scs import ScsBus

ONLINE_TIMEOUT_S = 0.5                                  # ไม่มี servo ตอบนานเกินนี้ = ออฟไลน์


def jump_too_big(q_from, q_to, time_ms):
    """กันบั๊ก: คำสั่งสั้นกว่า MIN_MOVE_MS แต่ข้อไหนกระโดดเกิน JUMP_GUARD_DEG → True (ไม่ควรส่ง)"""
    if time_ms >= config.MIN_MOVE_MS or q_from is None:
        return False
    return any(abs(b - a) > config.JUMP_GUARD_DEG for a, b in zip(q_from, q_to) if not math.isnan(a))


class SimLink:
    """แขนจำลอง: ไปถึงมุมที่สั่งทันที ไม่ยวบ ไม่มีไฟ"""
    simulated = True

    def __init__(self):
        self.q_actual = [0.0] * len(config.JOINTS)
        self.volt = None
        self.message = ""
        self.torque_on = True
        self.servo = {}

    def send_q(self, q, time_ms):
        if not all(math.isfinite(v) for v in q) or jump_too_big(self.q_actual, q, time_ms):
            self.message = "ไม่ส่ง: มุมผิดรูปหรือกระโดดเร็วเกิน"
            return False
        self.q_actual = [float(v) for v in q]
        return True

    def stop(self):
        pass

    def torque(self, on):
        self.torque_on = on

    def online(self):
        return True

    def close(self):
        pass


class DirectLink:
    """thread อ่านมุมจริง + ไฟทุก period_s · ล็อกบัสทุกครั้งที่ส่ง-รับ (thread กับ GUI ใช้สายเดียวกัน)
    ต่อแล้วแขนไม่ขยับเอง: อ่านตำแหน่งจริง → ตั้งเป็นเป้า → เปิด torque"""
    simulated = False

    def __init__(self, port=config.DIRECT_PORT, serial_obj=None, period_s=0.1):
        if serial_obj is None:
            serial_obj = serial.Serial(port, config.SERVO_BAUD, timeout=0.002)
        self.ser = serial_obj
        self.bus = ScsBus(serial_obj)
        self.lock = threading.Lock()
        self.period_s = period_s
        self.q_actual = None
        self.volt = None
        self.message = ""
        self.last_rx = -math.inf
        self.q_sent = None                        # ท่าล่าสุดที่ส่ง (ใช้กันกระโดด)
        self.servo = {}                           # id → {"raw", "load" %, "temp" °C} ข้อที่ตอบรอบล่าสุด
        try:
            with self.lock:
                self.bus.detect_echo()
                self._hold_here()
                for j in config.JOINTS:
                    self.bus.torque(j["id"], True)
        except OSError:
            self.ser.close()
            raise
        self._running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    # ---------- ภายใน (ต้องถือ lock) ----------
    def _hold_here(self):
        """ทุกข้อ: อ่านตำแหน่งจริงแล้วตั้งเป็นเป้า (= ค้างที่เดิม)"""
        for j in config.JOINTS:
            got = self.bus.read_pos_volt(j["id"])
            if got:
                self.bus.write_pos(j["id"], got[0], config.MIN_MOVE_MS)
        self.q_sent = None

    def _read_all(self):
        """อ่าน addr 56..63 ทุกข้อ: ตำแหน่ง(2) ความเร็ว(2) โหลด(2) ไฟ(1) อุณหภูมิ(1)"""
        q, volt, any_ok, servo = [], None, False, {}
        with self.lock:
            for j in config.JOINTS:
                d = self.bus.read(j["id"], 56, 8)
                if d is None:
                    q.append(math.nan)
                    continue
                raw, load = (d[0] << 8) | d[1], (d[4] << 8) | d[5]
                # โหลด: bit 10 = ทิศ, 10 bit ล่าง = ขนาด (0..1000 = 0..100 %)
                servo[j["id"]] = {"raw": raw, "load": (-1 if load & 0x400 else 1) * (load & 0x3FF) / 10,
                                  "temp": d[7]}
                q.append(config.raw_to_q(j, raw))
                volt, any_ok = d[6] / 10, True
        self.servo = servo
        if any_ok:
            self.q_actual, self.volt = q, volt
            self.last_rx = time.monotonic()

    def _loop(self):
        while self._running:
            try:
                self._read_all()
            except OSError:
                self.last_rx = -math.inf
            time.sleep(self.period_s)

    # ---------- ใช้จากข้างนอก ----------
    def send_q(self, q, time_ms):
        """ส่งท่าให้ทุกข้อ (บีบใน limit) — คืน False ถ้าไม่ส่ง (nan / กระโดดเร็วเกิน / สายหลุด)"""
        if not all(math.isfinite(v) for v in q):
            self.message = "ไม่ส่ง: มีมุมเป็น nan/inf"
            return False
        t = max(20, min(10000, int(time_ms)))
        if jump_too_big(self.q_sent or self.q_actual, q, t):
            self.message = f"ไม่ส่ง: ข้อกระโดดเกิน {config.JUMP_GUARD_DEG}° ใน {t} ms"
            return False
        try:
            with self.lock:
                for j, qi in zip(config.JOINTS, q):
                    self.bus.write_pos(j["id"], config.q_to_raw(j, qi), t)
                self.q_sent = list(q)
        except OSError:
            self.last_rx = -math.inf
            return False
        return True

    def stop(self):
        """หยุด = ทุกข้อค้างที่ตำแหน่งจริงตอนนี้"""
        try:
            with self.lock:
                self._hold_here()
        except OSError:
            self.last_rx = -math.inf

    def torque(self, on):
        """เปิด = ค้างที่เดิมแล้วเปิดแรงบิด · ปิด = ทุกข้อหมดแรงทันที"""
        try:
            with self.lock:
                if on:
                    self._hold_here()
                for j in config.JOINTS:
                    self.bus.torque(j["id"], on)
                self.q_sent = None
        except OSError:
            self.last_rx = -math.inf

    def online(self):
        return time.monotonic() - self.last_rx < ONLINE_TIMEOUT_S

    def close(self):
        self._running = False
        self.thread.join(timeout=1.0)
        try:
            self.ser.close()
        except OSError:
            pass
