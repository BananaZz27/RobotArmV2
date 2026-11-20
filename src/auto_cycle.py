# วนลูปอัตโนมัติ A → B → C … → A แบบเครื่องจักรในโรงงาน (ไม่มี GUI → test ได้)
#   หน้าต่างเรียก update() ทุกรอบ poll พร้อมบอกว่าแขนยังวิ่งอยู่ไหม (mover.busy) และท่าที่สั่งตอนนี้ (mover.q)
#   update() คืน "จุดถัดไปที่ต้องสั่งไป" (หรือ None = รอ) — การสั่งจริง + ตรวจความปลอดภัยทำที่หน้าต่าง (_ready)
#
#   เริ่ม → วิ่งไปจุดแรก (A) ก่อน → ถึงแล้วรอ dwell → ไปจุดถัดไป → … → กลับถึง A = ครบ 1 รอบ
#   แขนหยุดก่อนถึงจุด (ไฟตก / servo ไม่ตอบ / กด STOP) → หยุดลูป ไม่ไปต่อเอง
import json
from dataclasses import dataclass, field
from pathlib import Path

ARRIVE_TOL = 0.5        # ท่าที่สั่งห่างจุดหมาย ≤ 0.5° ทุกข้อ = ถึงแล้ว (มากกว่านั้น = ถูกหยุดกลางทาง)
POINTS_FILE = Path(__file__).resolve().parent.parent / "auto_points.json"   # จุดที่บันทึกไว้ (ไม่อยู่ใน git)
MAX_POINTS = 8          # จุดสูงสุดในลำดับงาน (A … H)


@dataclass
class AutoCycle:
    points: list                         # [[q1..q4], ...] — จุด A, B, C … (เรียงตามลำดับที่วิ่ง)
    speed: float = 60.0                  # ความเร็วข้อต่อสูงสุด (°/s)
    dwell: float = 0.5                   # หยุดรอที่จุด (s) ค่าเริ่มต้น — ใช้เมื่อจุดนั้นไม่ได้ตั้งเวลาเอง
    dwells: list = field(default_factory=list)   # เวลารอแยกแต่ละจุด (s) เช่น A 0.5, B 2.0 (หยิบ/วางชิ้นงาน)
    cycles: int = 0                      # จำนวนรอบ (0 = ไม่จำกัด จนกด Stop)
    running: bool = False
    phase: str = "idle"                  # idle | move | dwell | done | stopped
    index: int = 0                       # จุดที่กำลังไป / รออยู่
    done: int = 0                        # รอบที่ครบแล้ว
    last_cycle_s: float | None = None    # เวลาของรอบล่าสุด (s)
    reason: str = ""                     # ทำไมหยุด
    _dwell_end: float = 0.0
    _cycle_start: float | None = None
    _first_arrival: bool = field(default=True, repr=False)

    def start(self, now):
        """เริ่มลูป → คืนจุดแรกที่ต้องสั่ง"""
        if len(self.points) < 2 or any(p is None for p in self.points):
            raise ValueError("ต้องบันทึกจุดให้ครบก่อน")
        self.running, self.phase, self.index, self.done = True, "move", 0, 0
        self.last_cycle_s, self.reason, self._cycle_start, self._first_arrival = None, "", None, True
        return self.points[0]

    @property
    def approaching(self):
        """กำลังวิ่งไปจุดแรกครั้งแรก (เริ่มจากท่าไหนก็ได้ — หน้าต่างใช้หมุนข้อต่อเสมอ ไม่วิ่งเส้นตรงผ่านท่าแปลก ๆ)"""
        return self.running and self._first_arrival

    def stop(self, reason="หยุดโดยผู้ใช้"):
        if self.running:
            self.running, self.phase, self.reason = False, "stopped", reason

    def update(self, now, busy, q_now):
        """เรียกทุกรอบ → คืนจุดถัดไปที่ต้องสั่ง หรือ None"""
        if not self.running:
            return None
        if self.phase == "move":
            if busy:
                return None
            goal = self.points[self.index]
            if max(abs(a - b) for a, b in zip(q_now, goal)) > ARRIVE_TOL:
                self.stop("แขนหยุดก่อนถึงจุด " + self.name(self.index))
                return None
            if self.index == 0:                          # ถึง A: ครบรอบ (ยกเว้นครั้งแรกที่เพิ่งวิ่งมาตั้งต้น)
                if self._first_arrival:
                    self._first_arrival = False
                else:
                    self.done += 1
                    self.last_cycle_s = now - self._cycle_start
                    if self.cycles and self.done >= self.cycles:
                        self.running, self.phase = False, "done"
                        return None
                self._cycle_start = now
            self.phase, self._dwell_end = "dwell", now + self.dwell_at(self.index)
            return None
        if self.phase == "dwell" and now >= self._dwell_end:
            self.index = (self.index + 1) % len(self.points)
            self.phase = "move"
            return self.points[self.index]
        return None

    def dwell_at(self, i):
        """เวลารอที่จุด i (ไม่ได้ตั้ง = ค่าเริ่มต้น dwell)"""
        return self.dwells[i] if i < len(self.dwells) and self.dwells[i] is not None else self.dwell

    @staticmethod
    def name(i):
        return chr(ord("A") + i)

    def status(self, now):
        """ข้อความสถานะสั้น ๆ สำหรับหน้าต่าง"""
        if self.phase == "move":
            return f"▶ กำลังไปจุด {self.name(self.index)}"
        if self.phase == "dwell":
            return f"⏸ รอที่จุด {self.name(self.index)}  {max(0.0, self._dwell_end - now):.1f} s"
        if self.phase == "done":
            return f"✅ ครบ {self.done} รอบ"
        if self.phase == "stopped":
            return f"■ หยุด — {self.reason}"
        return "พร้อม (ยังไม่เริ่ม)"


def load_points(path=POINTS_FILE):
    """จุดที่บันทึกไว้ครั้งก่อน → [q, q, ...] (ไม่มีไฟล์/อ่านไม่ได้ = [None, None] — จุด A, B ว่าง)"""
    try:
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        pts = data["points"] if "points" in data else [data.get("A"), data.get("B")]   # รูปแบบเก่ามีแค่ A, B
        pts = [[float(v) for v in p] if p else None for p in pts][:MAX_POINTS]
        return pts + [None] * (2 - len(pts))
    except (OSError, ValueError, TypeError, AttributeError, KeyError):
        return [None, None]


def save_points(points, path=POINTS_FILE, dwells=None):
    data = {"points": points} if dwells is None else {"points": points, "dwells": dwells}
    Path(path).write_text(json.dumps(data, indent=1), encoding="utf-8")


def load_dwells(path=POINTS_FILE):
    """เวลารอแยกแต่ละจุดที่บันทึกไว้ → [s, ...] (ไม่มี/อ่านไม่ได้ = [] → ใช้ค่าเริ่มต้น)"""
    try:
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        return [None if v is None else max(0.0, float(v)) for v in data.get("dwells", [])][:MAX_POINTS]
    except (OSError, ValueError, TypeError, AttributeError):
        return []


# ท่าตัวอย่าง A–G: หยิบชิ้นงานทางซ้าย → วางทางขวา → กลับท่าพร้อม (ปุ่ม ✨ ในแท็บ Auto)
#   คำนวณจาก IK (ศอกขึ้น) · ปลายแขนสูงจากพื้น ≥ 90 mm · ทุกข้อห่าง limit ≥ 13° · ทุกช่วงวิ่งเส้นตรงได้ (test ตรวจ)
DEMO = [   # (ชื่อ, q [°], รอ [s])            (x, y, z) mm, φ
    ("ท่าพร้อม", [0.0, 25.3, 62.1, 32.6], 0.5),          # (200,    0, 200), −30°
    ("เหนือจุดหยิบ", [33.7, 37.4, 57.5, 65.0], 0.2),    # (165,  110, 150), −70°
    ("หยิบ", [33.7, 49.0, 69.7, 51.2], 1.0),            # (165,  110,  90), −80°  รอหนีบชิ้นงาน
    ("ยกขึ้น", [33.7, 33.4, 61.2, 55.4], 0.2),           # (165,  110, 160), −60°
    ("เหนือจุดวาง", [-33.7, 33.4, 61.2, 55.4], 0.2),     # (165, −110, 160), −60°
    ("วาง", [-33.7, 49.0, 69.7, 51.2], 1.0),            # (165, −110,  90), −80°  รอปล่อยชิ้นงาน
    ("ยกออก", [-33.7, 37.4, 57.5, 65.0], 0.2),         # (165, −110, 150), −70°
]
