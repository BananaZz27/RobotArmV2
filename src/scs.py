# คุยกับ servo Feetech SCS15 บนบัส half-duplex ตรงจาก PC (ย้ายมาจาก v2/firmware/arm_v2/scs.cpp)
# ใช้กับ Waveshare Bus Servo Adapter (A) จัมเปอร์ B (USB-SERVO) — design 6.9
#
# packet ที่ PC ส่ง (instruction):
#   FF FF  ID  LEN  INST  PARAM...  CHK
#   │  │   │   │    │     │         └ checksum = ~(ID + LEN + INST + PARAM ทุกตัว) เก็บแค่ 8 bit ล่าง
#   │  │   │   │    │     └ ข้อมูล: byte แรก = address ในหน่วยความจำ servo, ที่เหลือ = ค่า
#   │  │   │   │    └ คำสั่ง: 01 ping, 02 อ่าน, 03 เขียน
#   │  │   │   └ LEN = จำนวน PARAM + 2
#   │  │   └ หมายเลข servo
#   └──┴ หัว packet สองไบต์
# packet ที่ servo ตอบ (status):  FF FF  ID  LEN  ERR  DATA...  CHK   (LEN = จำนวน DATA + 2)
# ค่า 2 byte (ตำแหน่ง, เวลา) ส่งแบบ high byte ก่อน (big-endian)
import time

import config

HEADER = 0xFF
INST_PING, INST_READ, INST_WRITE = 0x01, 0x02, 0x03
ADDR_TORQUE = 40        # 1 byte: 1 = เปิดแรงบิด
ADDR_GOAL = 42          # 6 byte: ตำแหน่งเป้า(2) เวลา ms(2) ความเร็ว(2)
ADDR_PRESENT = 56       # 7 byte: ตำแหน่ง(2) ความเร็ว(2) โหลด(2) ไฟ(1) — ไฟอยู่ที่ addr 62
PROBE_ID = 253          # ID ที่ไม่มี servo ใช้ ไว้ตรวจ echo


def checksum(body):
    """body = ID, LEN, INST/ERR, params → ~ผลรวม เก็บ 8 bit"""
    return (~sum(body)) & 0xFF


def make_packet(sid, inst, params=b""):
    body = bytes([sid, len(params) + 2, inst]) + bytes(params)   # ID, LEN, INST, PARAM...
    return bytes([HEADER, HEADER]) + body + bytes([checksum(body)])


class ScsBus:
    """ส่ง/รับ packet ทีละคำสั่ง (ไม่ thread-safe — ผู้เรียกต้องล็อกเอง)"""

    def __init__(self, ser, timeout_s=config.SERVO_TIMEOUT_S):
        self.ser = ser
        self.timeout_s = timeout_s
        self.echo = False       # วงจรบางแบบได้ยินข้อมูลที่ตัวเองส่งวนกลับมา
        self._buf = bytearray()

    # ---------- ระดับ byte ----------
    def _byte(self, deadline):
        """อ่าน 1 byte ก่อนหมดเวลา (คืน None ถ้าหมดเวลา)"""
        while not self._buf:
            chunk = self.ser.read(self.ser.in_waiting or 1)
            if chunk:
                self._buf += chunk
            elif time.monotonic() > deadline:
                return None
        return self._buf.pop(0)

    def _send(self, sid, inst, params=b""):
        pkt = make_packet(sid, inst, params)
        self.ser.reset_input_buffer()           # ทิ้งขยะค้างก่อนส่ง
        self._buf.clear()
        self.ser.write(pkt)
        self.ser.flush()                        # รอส่งครบก่อนฟังคำตอบ
        if self.echo:                           # ทิ้ง echo ของตัวเองให้ครบความยาว packet
            deadline = time.monotonic() + self.timeout_s
            for _ in pkt:
                if self._byte(deadline) is None:
                    break
        return pkt

    def _receive(self, sid, n):
        """หา FF FF แล้วตรวจ ID / ความยาว / checksum — ไม่ตรงทิ้งแล้วหาต่อจนหมดเวลา
        คืน DATA n byte หรือ None (บทเรียนโคมน้อย: ไม่เช็ค ID → เจอ "ID ผี")"""
        deadline = time.monotonic() + self.timeout_s
        while True:
            # 1) หาหัว FF FF
            prev = None
            while True:
                b = self._byte(deadline)
                if b is None:
                    return None
                if prev == HEADER and b == HEADER:
                    break
                prev = b
            # 2) ข้าม FF ที่เกินมา → byte ถัดไปคือ ID
            while b == HEADER:
                b = self._byte(deadline)
                if b is None:
                    return None
            rid = b
            # 3) LEN, ERR
            length, err = self._byte(deadline), self._byte(deadline)
            if length is None or err is None or length < 2:
                return None
            # 4) DATA (LEN − 2 byte)
            data = bytearray()
            for _ in range(length - 2):
                v = self._byte(deadline)
                if v is None:
                    return None
                data.append(v)
            # 5) CHK
            chk = self._byte(deadline)
            if chk is None:
                return None
            # 6) ต้องเป็น ID ที่ถาม, ยาวตามที่ขอ, checksum ถูก — ไม่งั้นหา packet ถัดไป
            if rid == sid and len(data) == n and checksum(bytes([rid, length, err]) + data) == chk:
                return bytes(data)

    def _write(self, sid, addr, data):
        self._send(sid, INST_WRITE, bytes([addr]) + bytes(data))
        return self._receive(sid, 0) is not None

    # ---------- คำสั่ง ----------
    def detect_echo(self):
        """ping ID ที่ไม่มีใครใช้ ถ้าได้ byte เดิมวนกลับมาครบ = วงจรมี echo"""
        self.echo = False
        pkt = self._send(PROBE_ID, INST_PING)
        deadline = time.monotonic() + self.timeout_s
        got = 0
        while got < len(pkt) and self._byte(deadline) is not None:
            got += 1
        self.echo = got == len(pkt)
        return self.echo

    def ping(self, sid):
        self._send(sid, INST_PING)
        return self._receive(sid, 0) is not None

    def torque(self, sid, on):
        return self._write(sid, ADDR_TORQUE, [1 if on else 0])

    def write_pos(self, sid, raw, time_ms):
        # ตำแหน่ง(2) + เวลา(2) + ความเร็ว(2) = 0 → ให้ servo คุมด้วยเวลา (แบบเดียวกับ firmware)
        return self._write(sid, ADDR_GOAL, [raw >> 8, raw & 0xFF, time_ms >> 8, time_ms & 0xFF, 0, 0])

    def read(self, sid, addr, n):
        """อ่านหน่วยความจำ servo n byte เริ่มที่ addr → bytes หรือ None"""
        self._send(sid, INST_READ, bytes([addr, n]))             # PARAM = address เริ่ม, จำนวน byte
        return self._receive(sid, n)

    def read_pos_volt(self, sid):
        """อ่าน addr 56..62 → (raw, ไฟ V) หรือ None"""
        d = self.read(sid, ADDR_PRESENT, 7)
        if d is None:
            return None
        return (d[0] << 8) | d[1], d[6] * 0.1                     # ตำแหน่ง big-endian, ไฟหน่วย 0.1 V
