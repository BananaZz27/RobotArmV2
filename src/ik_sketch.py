# รูปมุมมองข้าง (ระนาบแขน r–z) ในแท็บ IK: วาดว่าทำไมไปได้/ไม่ได้
#   วงเขียว = ที่ที่ข้อมือไปได้ (ห่างแกนไหล่ |a2 − a3| … a2 + a3) · วงประ = ปลายแขนไกลสุด a2 + a3 + a4
#   เป้า → ถอยตามทิศ φ ไป a4 = ข้อมือ · ข้อมืออยู่นอกวง = เอื้อมไม่ถึง (เส้นแดงบอกระยะที่ขาด)
#   ส้ม = ท่าที่จะใช้ · แดงประ = ท่าที่คำนวณได้แต่ติด limit (วงแดงที่ข้อที่เกิน)
import math

from PyQt6.QtCore import QPointF, QRectF, Qt
from PyQt6.QtGui import QBrush, QColor, QPainter, QPainterPath, QPen
from PyQt6.QtWidgets import QWidget

import config
from kinematics import REACH_MAX, REACH_MIN, forward_kinematics
from ik_explain import TIP_MAX

ORANGE, RED, GREEN, GREY = QColor("#ff8c42"), QColor("#d1453b"), QColor(46, 160, 120), QColor("#8a9499")
JOINT_AT = {0: 0, 1: 1, 2: 2, 3: 3}          # ข้อ i หมุนที่จุดกำเนิดเฟรม i (J1 ที่ฐาน, J2 ไหล่, J3 ศอก, J4 คอ)


class IKSketch(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.d = None
        self.setMinimumHeight(260)

    def set_diagnosis(self, d):
        self.d = d
        self.update()

    # ---------- แปลงพิกัด ----------
    def _setup(self):
        d = self.d
        self.u = (d.x, d.y) if math.hypot(d.x, d.y) > 1e-6 else (1.0, 0.0)   # ทิศระนาบแขน (หันหน้าเข้าเป้า)
        n = math.hypot(*self.u)
        self.u = (self.u[0] / n, self.u[1] / n)
        r = math.hypot(d.x, d.y)
        lo_x, hi_x = min(-TIP_MAX * 0.75, r - 40), max(TIP_MAX + 20, r + 40)
        lo_z, hi_z = min(-40, d.z - 40), max(config.D1 + TIP_MAX + 10, d.z + 40)
        m = 14
        s = min((self.width() - 2 * m) / (hi_x - lo_x), (self.height() - 2 * m) / (hi_z - lo_z))
        self.s = s
        self.ox = m + (self.width() - 2 * m - s * (hi_x - lo_x)) / 2 - s * lo_x
        self.oy = self.height() - m - (self.height() - 2 * m - s * (hi_z - lo_z)) / 2 + s * lo_z

    def P(self, r, z):
        return QPointF(self.ox + self.s * r, self.oy - self.s * z)

    def _proj(self, q):
        """จุดข้อต่อของท่า q ฉายลงระนาบแขน → [(r, z), ...]"""
        return [(T[0, 3] * self.u[0] + T[1, 3] * self.u[1], T[2, 3]) for T in forward_kinematics(q).frames]

    # ---------- วาด ----------
    def paintEvent(self, _ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        ink = self.palette().color(self.foregroundRole())
        p.fillRect(self.rect(), self.palette().color(self.backgroundRole()))
        if self.d is None:
            p.setPen(GREY)
            p.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, "กด Preview หรือ Go เพื่อดูว่าแขนไปจุดนี้ได้ไหม และเพราะอะไร")
            return
        self._setup()
        d, s = self.d, self.s
        shoulder = self.P(0, config.D1)

        # พื้น + ฐาน
        p.setPen(QPen(GREY, 1.5))
        p.drawLine(self.P(-TIP_MAX, 0), self.P(TIP_MAX * 1.3, 0))
        p.fillRect(QRectF(self.P(-25, 14), self.P(25, 0)), QColor(150, 155, 160, 80))

        # วงที่ข้อมือไปได้ + วงปลายแขนไกลสุด
        ring = QPainterPath()
        ring.addEllipse(shoulder, REACH_MAX * s, REACH_MAX * s)
        hole = QPainterPath()
        hole.addEllipse(shoulder, REACH_MIN * s, REACH_MIN * s)
        p.fillPath(ring.subtracted(hole), QColor(46, 160, 120, 38))
        p.setPen(QPen(GREEN, 1))
        p.drawPath(ring.subtracted(hole))
        p.setPen(QPen(GREY, 1, Qt.PenStyle.DashLine))
        p.drawEllipse(shoulder, TIP_MAX * s, TIP_MAX * s)

        front = [o for o in d.options if not o.flip]
        g = front[0].geo                                   # ข้อมือของท่าหันหน้า (ศอกขึ้น/ลงใช้ข้อมือจุดเดียวกัน)
        tip = self.P(math.hypot(d.x, d.y), d.z)
        wrist = self.P(g.rc, g.s + config.D1)
        wrist_ok = REACH_MIN - 1e-6 <= g.dist <= REACH_MAX + 1e-6

        # ท่าที่ติด limit (ถ้าไปไม่ได้ด้วย φ ที่ขอ แต่คำนวณมุมได้)
        if not d.exact:
            blocked = next((o for o in front if o.geo.q is not None), None)
            if blocked:
                pts = self._proj(blocked.geo.q)
                p.setPen(QPen(RED, 3, Qt.PenStyle.DashLine))
                for a, b in zip(pts, pts[1:]):
                    p.drawLine(self.P(*a), self.P(*b))
                p.setPen(QPen(RED, 2.5))
                for k, (i, v, lo, hi) in enumerate(blocked.bad):
                    p.drawEllipse(self.P(*pts[JOINT_AT[i]]), 9, 9)
                    p.drawText(QPointF(10, 52 + 16 * k), f"○ ติด limit: {config.JOINTS[i]['name']} ต้องหมุน {v:.1f}° "
                                                         f"แต่ได้ {lo:.1f}…{hi:.1f}° ({blocked.name})")

        # ท่าที่จะใช้
        if d.q is not None:
            pts = self._proj(d.q)
            p.setPen(QPen(ORANGE, 6, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
            for a, b in zip(pts, pts[1:]):
                p.drawLine(self.P(*a), self.P(*b))
            p.setBrush(QBrush(self.palette().color(self.backgroundRole())))
            p.setPen(QPen(ORANGE, 2))
            for pt in pts[1:4]:
                p.drawEllipse(self.P(*pt), 4, 4)
            p.setBrush(Qt.BrushStyle.NoBrush)

        # เป้า → ข้อมือ ตามทิศ φ ที่ขอ
        bad_phi = not d.exact
        p.setPen(QPen(RED if bad_phi else ORANGE, 2, Qt.PenStyle.DashLine if bad_phi else Qt.PenStyle.SolidLine))
        p.drawLine(tip, wrist)
        if not wrist_ok:                                   # ข้อมือนอกวง: เส้นจากไหล่ + ระยะที่ขาด/เกิน
            p.setPen(QPen(RED, 1.5, Qt.PenStyle.DashLine))
            p.drawLine(shoulder, wrist)
            gap = g.dist - REACH_MAX if g.dist > REACH_MAX else REACH_MIN - g.dist
            word = "ขาด" if g.dist > REACH_MAX else "ใกล้เกิน"
            p.drawText(wrist + QPointF(-44, 36), f"ข้อมือ{word} {gap:.1f} mm")
        p.setPen(QPen(RED if not wrist_ok else GREEN, 2))
        p.drawEllipse(wrist, 5, 5)
        p.setPen(QPen(ORANGE, 2.5))
        p.drawEllipse(tip, 7, 7)

        # ป้าย
        p.setPen(ink)
        p.drawText(tip + QPointF(10, -12), f"เป้า φ {d.phi:.0f}°")
        p.drawText(wrist + QPointF(-44, 20), "ข้อมือ")
        p.setPen(GREY)
        p.drawText(QPointF(10, 18), "มุมมองข้าง (ระนาบแขน) · วงเขียว = ข้อมือไปได้ · วงประ = ปลายแขนไกลสุด 285 mm")
        p.drawText(QPointF(10, 34), "ส้ม = ท่าที่จะใช้ · แดง = ส่วนที่ไปไม่ได้")
