# ตัวขับ: ทุก PERIOD_S เดิน Mover หนึ่งก้าวแล้วส่งท่าให้แขน (เวลา STREAM_MS ให้ servo เกลี่ยต่อเนื่อง)
#   ไฟ < 6 V / ออฟไลน์ / แขนไม่รับคำสั่ง → หยุด + ค้างที่มุมจริง
#   ชดเชย: goto จบแล้วรอนิ่ง → ข้อที่ error > 0.5° สั่งเพิ่ม 0.8 × error แบบนุ่ม ≤ 3 รอบ, รวม ≤ ±5°
# GUI เรียกคำสั่งผ่าน do(lambda m: m.goto_q(...)) เพื่อไม่ชนกับ thread (มี lock)
import math
import threading
import time

import config


class Driver:
    def __init__(self, link, mover, period=config.PERIOD_S):
        self.link, self.mover, self.period = link, mover, period
        self.correct = True
        self.message = ""
        self.lock = threading.Lock()
        self._corr = None          # {"target", "offset", "rounds", "check_at", "moving"}
        self._thread = None
        self._running = False

    # ---------- ใช้จาก GUI ----------
    def do(self, fn):
        """สั่ง Mover อย่างปลอดภัย (คำสั่งใหม่ยกเลิกการชดเชยที่ค้างอยู่) → คืนค่าที่ fn คืน"""
        with self.lock:
            self._corr = None
            result = fn(self.mover)
            if self.mover.message:
                self.message = self.mover.message
            return result

    def emergency_stop(self):
        """หยุดทันที: ทิ้งการเคลื่อนที่ → servo ค้างที่ตำแหน่งจริง → ท่าที่สั่ง = มุมจริง"""
        with self.lock:
            self._halt("หยุดแล้ว")

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def close(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)

    # ---------- ภายใน ----------
    def _loop(self):
        next_t = time.monotonic()
        while self._running:
            self.tick()
            next_t += self.period
            time.sleep(max(0.0, next_t - time.monotonic()))
            if time.monotonic() - next_t > 0.5:        # ช้าไปมาก (เครื่องค้าง) → ตั้งจังหวะใหม่ ไม่เร่งตามทัน
                next_t = time.monotonic()

    def _safe(self):
        if not self.link.online():
            return "แขนออฟไลน์ (servo ไม่ตอบ)"
        if not self.link.simulated and (self.link.volt is None or self.link.volt < config.VOLT_MIN):
            return f"ไฟ servo ต่ำ ({self.link.volt} V < {config.VOLT_MIN} V)"
        return ""

    def _halt(self, why):
        self.mover.stop()
        self._corr = None
        self.link.stop()
        if self.link.q_actual is not None:
            self.mover.reset(self.link.q_actual)
        self.message = why

    def tick(self, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            if self.mover.busy:
                why = self._safe()
                if why:
                    self._halt("หยุด: " + why)
                    return
                q = self.mover.step(self.period)
                if not self.link.send_q(q, config.STREAM_MS):
                    self._halt("หยุด: " + (self.link.message or "แขนไม่รับคำสั่ง"))
                    return
                if self.mover.message:
                    self.message = self.mover.message
            self._correction(now)

    def _correction(self, now):
        goal = self.mover.pop_finished()
        if goal is not None:
            if self._corr and self._corr["moving"]:               # รอบชดเชยวิ่งเสร็จ → รอนิ่งแล้ววัดใหม่
                self._corr["moving"], self._corr["check_at"] = False, now + config.CORR_WAIT_S
            elif self.correct and not self.link.simulated:         # goto ของผู้ใช้จบ → เริ่มชดเชย
                self._corr = {"target": goal, "offset": [0.0] * len(goal), "rounds": 0,
                              "check_at": now + config.CORR_WAIT_S, "moving": False}
            else:
                self.message = "ถึงเป้าแล้ว"
        c = self._corr
        if c is None or c["moving"] or self.mover.busy or now < c["check_at"]:
            return
        actual = self.link.q_actual
        if actual is None or self._safe():
            self._corr, self.message = None, "ไม่ชดเชย: อ่านมุม/ไฟไม่ได้"
            return
        err = [t - a if j["used"] and not math.isnan(a) else 0.0
               for j, t, a in zip(config.JOINTS, c["target"], actual)]
        worst = max(abs(e) for e in err)
        if worst <= config.CORR_TOL:
            self._corr = None
            self.message = f"ถึงเป้า (ชดเชย {c['rounds']} รอบ, error สูงสุด {worst:.2f}°)"
            return
        if c["rounds"] == 0 and worst > config.CORR_GIVE_UP:
            self._corr = None
            self.message = f"ไม่ชดเชย: error {worst:.1f}° ใหญ่ผิดปกติ (ถูกขวาง/ไฟตก/แรงไม่พอ?)"
            return
        if c["rounds"] >= config.CORR_ROUNDS:
            self._corr = None
            self.message = f"ชดเชยครบ {config.CORR_ROUNDS} รอบ ยังคลาด {worst:.2f}°"
            return
        c["offset"] = [max(-config.CORR_MAX, min(config.CORR_MAX, o + config.CORR_GAIN * e if abs(e) > config.CORR_TOL else o))
                       for o, e in zip(c["offset"], err)]
        c["rounds"] += 1
        c["moving"] = True
        self.mover.goto_q([t + o for t, o in zip(c["target"], c["offset"])])
        self.message = f"กำลังชดเชย รอบ {c['rounds']} (error สูงสุด {worst:.2f}°)"
