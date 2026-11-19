# serial จำลองที่ทำตัวเหมือน servo SCS15 หลายตัวบนบัสเดียว (ใช้ test scs.py / DirectLink โดยไม่ต่อแขน)
# ตั้งให้ "ป่วย" ได้: checksum ผิด, ตอบด้วย ID ผิด, ไม่ตอบ, มีขยะนำหน้า, มี echo, สายหลุด
import threading

import serial


def _chk(body):
    return (~sum(body)) & 0xFF


class FakeBus:
    def __init__(self, positions, volt=7.6, echo=False, follow=True):
        self.positions = dict(positions)      # id → raw ตอนนี้
        self.volt = volt
        self.echo = echo
        self.follow = follow                  # True = สั่ง goal แล้วไปถึงทันที
        self.goals = {}                       # id → (raw, time_ms) ล่าสุด
        self.torque = {}
        self.packets = []                     # packet ที่ได้รับ (ถูกต้อง)
        self.bad_packets = 0                  # packet ที่ checksum เพี้ยน (เช่น ส่งซ้อนกัน)
        self.bad_checksum_ids = set()
        self.wrong_id_map = {}
        self.silent_ids = set()
        self.garbage_prefix = b""
        self.fail = False
        self.timeout = 0
        self._out = bytearray()
        self._lock = threading.Lock()

    # ---------- อินเทอร์เฟซแบบ pyserial ----------
    @property
    def in_waiting(self):
        self._check()
        return len(self._out)

    def reset_input_buffer(self):
        self._check()
        with self._lock:
            self._out.clear()

    def read(self, n=1):
        self._check()
        with self._lock:
            data = bytes(self._out[:n])
            del self._out[:n]
        return data

    def write(self, data):
        self._check()
        with self._lock:
            if self.echo:
                self._out += data
            self._handle(bytes(data))
        return len(data)

    def flush(self):
        pass

    def close(self):
        pass

    def _check(self):
        if self.fail:
            raise serial.SerialException("สายหลุด (จำลอง)")

    # ---------- servo ----------
    def memory(self, sid):
        """หน่วยความจำ 70 byte ของ servo: ID ที่ 5, limit 9/11, ตำแหน่ง 56, ไฟ 62, อุณหภูมิ 63"""
        mem = bytearray(70)
        raw = self.positions[sid]
        mem[5] = sid
        mem[9:11] = (20).to_bytes(2, "big")
        mem[11:13] = (1003).to_bytes(2, "big")
        mem[40] = int(self.torque.get(sid, False))
        mem[56:58] = raw.to_bytes(2, "big")
        mem[62] = round(self.volt * 10)
        mem[63] = 31
        return mem

    def _handle(self, pkt):
        if len(pkt) < 6 or pkt[0] != 0xFF or pkt[1] != 0xFF or pkt[3] != len(pkt) - 4 \
                or _chk(pkt[2:-1]) != pkt[-1]:
            self.bad_packets += 1
            return
        sid, inst, params = pkt[2], pkt[4], pkt[5:-1]
        self.packets.append((sid, inst, params))
        if sid not in self.positions or sid in self.silent_ids:
            return
        data = b""
        if inst == 0x02:                                 # READ: ตัดจากหน่วยความจำจำลอง
            mem = self.memory(sid)
            data = bytes(mem[params[0]: params[0] + params[1]])
        elif inst == 0x03 and params[0] == 42:          # WRITE goal
            raw, t = (params[1] << 8) | params[2], (params[3] << 8) | params[4]
            self.goals[sid] = (raw, t)
            if self.follow:
                self.positions[sid] = raw
        elif inst == 0x03 and params[0] == 40:          # WRITE torque
            self.torque[sid] = bool(params[1])
        rid = self.wrong_id_map.get(sid, sid)
        body = bytes([rid, len(data) + 2, 0]) + data
        chk = _chk(body) ^ (0x01 if sid in self.bad_checksum_ids else 0)
        self._out += self.garbage_prefix + b"\xff\xff" + body + bytes([chk])
