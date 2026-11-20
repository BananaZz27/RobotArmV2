# หน้าต่างควบคุมแขน หน้าตาแบบ ws-control (PyQt6 + 3D PyVista) — แต่ต่อ servo ตรงผ่าน adapter (ไม่มี ESP32)
#   และกดส่งแล้วแขนวิ่งนุ่ม (Mover/Driver) · RobotArm.bat  หรือ  cd src && ../.venv/Scripts/python.exe qt_app.py
#
#  ┌ [Connect COM11] ● สถานะ ──────────────────────── [Torque OFF] [■ STOP (Space)] ┐
#  │ ┌──────── 3D ────────┐ ┌ แท็บ: Joint | Inverse Kinematics | Status ┐          │
#  │ └────────────────────┘ └───────────────────────────────────────────┘          │
#  │ Forward Kinematics (ท่า Target): X Y Z φ Yaw + T⁰₄                            │
#  │ Log                                                                           │
#  └───────────────────────────────────────────────────────────────────────────────┘
import math
import sys
import time
from datetime import datetime
from html import escape

import serial
from PyQt6.QtCore import QLocale, Qt, QTimer
from PyQt6.QtGui import QKeySequence, QShortcut
from PyQt6.QtWidgets import (QApplication, QCheckBox, QDoubleSpinBox, QFrame, QGridLayout, QHBoxLayout, QHeaderView,
                             QLabel, QMainWindow, QMessageBox, QPlainTextEdit, QSlider, QSplitter, QTableWidget,
                             QTableWidgetItem, QTabWidget, QTextEdit, QVBoxLayout, QWidget)

import config
import theme
from driver import Driver
from ik_explain import TIP_MAX, diagnose, fmt_ranges
from ik_sketch import IKSketch
from kinematics import check_limits, forward_kinematics
from link import DirectLink, SimLink
from motion import Mover, clamp_q
from singularity import analyze, report
from servo_calibrate import save_standby

USED = range(len(config.JOINTS))     # ทุกข้อ (ฐาน ไหล่ ศอก คอ)
SLIDER_SCALE = 10      # slider เป็น int → ×10 ให้ละเอียด 0.1°


def _spin(lo, hi, step, suffix, value=0.0, decimals=1):
    s = QDoubleSpinBox(decimals=decimals, minimum=lo, maximum=hi, singleStep=step, suffix=suffix)
    s.setValue(value)
    return s


def _ik_html(d):
    """ผล IK + เหตุผล (ik_explain.Diagnosis) → HTML สำหรับกล่องผลลัพธ์"""
    color = "#2a9d8f" if d.exact else ("#c77d14" if d.q is not None else "#d1453b")
    out = [f"<p style='font-size:15px;font-weight:600;color:{color};margin:0'>{escape(d.headline)}</p>"]
    for title, items, c in (("สาเหตุ", d.reasons, "#d1453b"), ("ลองแก้", d.hints, "#2a6fdb"), ("ระวัง", d.warnings, "#c77d14")):
        if items:
            out.append(f"<p style='margin:6px 0 0'><b style='color:{c}'>{title}</b></p><ul style='margin:0'>" +
                       "".join(f"<li>{escape(t)}</li>" for t in items) + "</ul>")
    rows = []
    for o in d.options:
        chosen = d.exact and o.ok and d.q is not None and all(abs(a - b) < 1e-6 for a, b in zip(o.geo.q, d.q))
        mark, c = ("★", "#2a9d8f") if chosen else (("✓", "#2a9d8f") if o.ok else ("✗", "#d1453b"))
        q = "  ".join(f"{v:6.1f}" for v in o.geo.q) if o.geo.q is not None else "—"
        rows.append(f"<tr><td style='color:{c}'>{mark}</td><td>{o.name}</td><td style='font-family:Consolas'>{q}</td>"
                    f"<td style='color:#d1453b'>{escape(o.why())}</td></tr>")
    out.append(f"<p style='margin:8px 0 2px'><b>4 ท่าที่ φ = {d.phi:.0f}°</b> (สไลด์ IK: θ₁ = Atan2 / π + Atan2 × ศอกขึ้น/ลง"
               " · ★ = ที่ใช้ · โปรแกรมใช้เฉพาะท่าหันหน้า)</p>"
               "<table cellspacing='0' cellpadding='3'><tr><th></th><th align='left'>ท่า</th>"
               "<th align='left'>q1 … q4 (°)</th><th align='left'>ทำไมใช้ไม่ได้</th></tr>" + "".join(rows) + "</table>")
    out.append(f"<p style='margin:6px 0 0'>φ ที่ไปได้ที่ตำแหน่งนี้: <b>{fmt_ranges(d.phi_ranges) or 'ไม่มี'}</b>"
               f" · ปลายแขนห่างแกนไหล่ {d.tip_dist:.1f} mm (ไกลสุด {TIP_MAX:.1f})</p>")
    if d.q is not None:
        out.append(f"<pre style='font-family:Consolas'>{escape(report(d.q))}</pre>")
    return "".join(out)


def _set_quiet(widget, value):
    """ตั้งค่าโดยไม่ยิง signal (กันวนกลับมาสั่งแขน)"""
    widget.blockSignals(True)
    widget.setValue(value)
    widget.blockSignals(False)


class MainWindow(QMainWindow):
    def __init__(self, link=None, connect_fn=None, viewer=None, save_standby_fn=save_standby):
        super().__init__()
        self.connect_fn = connect_fn or (lambda: DirectLink(config.DIRECT_PORT))
        self.save_standby = save_standby_fn          # test ส่งตัวปลอมมา ไม่เขียน config.py จริง
        self.link, self.driver = None, None
        self.q_target = [0.0] * len(config.JOINTS)
        self.torque_on = True
        self.setWindowTitle("RobotArm 4DOF — Control GUI (USB adapter)")
        self.resize(1400, 900)

        if viewer is None:
            from viewer3d import Viewer3D            # import ช้า (VTK) — test ส่ง viewer ปลอมมาแทน
            viewer = Viewer3D()
        self.viewer = viewer
        self.tabs = QTabWidget()
        self.tabs.addTab(self._build_joint_tab(), "Joint")
        self.tabs.addTab(self._build_jog_tab(), "Jog")
        self.tabs.addTab(self._build_ik_tab(), "Inverse Kinematics")
        self.tabs.addTab(self._build_status_tab(), "Status")
        self.tabs.setDocumentMode(True)
        self.log_view = QPlainTextEdit(readOnly=True, maximumBlockCount=500)
        self.log_view.setObjectName("log")
        self.log_view.setFont(theme.mono(9))
        view, vl = theme.card("มุมมอง 3D", "ส้ม = Target · ฟ้าโปร่ง = ท่าจริง")
        vl.addWidget(self.viewer, 1)
        split = QSplitter(Qt.Orientation.Horizontal)
        split.addWidget(view)
        split.addWidget(self.tabs)
        split.setSizes([780, 620])
        log, ll = theme.card("Log")
        ll.addWidget(self.log_view)
        log.setMaximumHeight(170)

        central = QWidget()
        lay = QVBoxLayout(central)
        lay.setContentsMargins(12, 12, 12, 12)
        lay.setSpacing(10)
        lay.addWidget(self._build_toolbar())
        lay.addWidget(split, stretch=1)
        lay.addWidget(self._build_fk_box())
        lay.addWidget(log)
        self.setCentralWidget(central)
        QShortcut(QKeySequence("Space"), self, activated=self.stop)

        self._use_link(link or SimLink())
        self.set_target(self.q_target)
        self.log("พร้อมใช้งาน (โหมด Simulation) — กด Connect เพื่อต่อแขนผ่าน COM11, กด Space = STOP")
        self.timer = QTimer(self, interval=50)
        self.timer.timeout.connect(self.poll)
        self.timer.start()

    # ================= สร้างหน้าต่าง =================
    def _build_toolbar(self):
        """แถบหัว: ชื่อ + พอร์ต + สถานะเชื่อมต่อ (ซ้าย) · Standby / Torque / STOP (ขวา)"""
        self.connect_btn = theme.button("Connect", "primary")
        self.connect_btn.clicked.connect(self.toggle_connect)
        self.conn_label = QLabel()
        self.torque_btn = theme.button("Torque OFF", tip="ปิด torque เพื่อจับแขนขยับด้วยมือ — ท่าใน GUI จะตามแขนจริง")
        self.torque_btn.clicked.connect(lambda: self.set_torque(not self.torque_on))
        stop = theme.button("■  STOP  (Space)", "danger")
        stop.clicked.connect(self.stop)
        standby = theme.button("⌂ Standby", tip=f"ไปท่า Standby ช้า ๆ ({config.STANDBY_V:.0f} °/s) — "
                                                "ตั้งท่าได้ที่ปุ่ม 📌 ในแท็บ Joint")
        standby.clicked.connect(self.go_standby)

        title = QVBoxLayout()
        title.setSpacing(0)
        title.addWidget(theme.label("RobotArm 4DOF", "app"))
        title.addWidget(theme.label(f"Port {config.DIRECT_PORT} · USB adapter → Feetech SCS15", "hint"))
        frame = QFrame()
        frame.setObjectName("header")
        bar = QHBoxLayout(frame)
        bar.setContentsMargins(16, 10, 12, 10)
        bar.setSpacing(10)
        bar.addLayout(title)
        bar.addSpacing(18)
        bar.addWidget(self.connect_btn)
        bar.addWidget(self.conn_label)
        bar.addStretch(1)
        for w in (standby, self.torque_btn, stop):
            bar.addWidget(w)
        return frame

    @staticmethod
    def _joint_name(i):
        """ชื่อข้อ + ช่วงที่หมุนได้ (ตัวเล็กใต้ชื่อ)"""
        box = QVBoxLayout()
        box.setSpacing(0)
        box.addWidget(theme.label(config.JOINTS[i]["name"], "name"))
        box.addWidget(theme.label(f"{config.Q_MIN[i]:.1f} … {config.Q_MAX[i]:.1f}°", "range"))
        return box

    @staticmethod
    def _page(*widgets):
        """หน้าในแท็บ: การ์ดเรียงลงล่าง"""
        w = QWidget()
        lay = QVBoxLayout(w)
        lay.setContentsMargins(0, 10, 0, 0)
        lay.setSpacing(10)
        for x in widgets:
            lay.addWidget(x) if isinstance(x, QWidget) else lay.addLayout(x)
        lay.addStretch(1)
        return w

    def _build_joint_tab(self):
        frame, lay = theme.card("ท่า Target ของแต่ละข้อ", "ลาก slider / พิมพ์ตัวเลข → แขนตามทันที")
        grid = QGridLayout()
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(10)
        for col, text in enumerate(("ข้อต่อ", "", "Target", "Actual")):
            grid.addWidget(theme.label(text, "hint"), 0, col)
        self.rows = {}
        for r, i in enumerate(USED, start=1):
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(int(config.Q_MIN[i] * SLIDER_SCALE), int(config.Q_MAX[i] * SLIDER_SCALE))
            spin = _spin(config.Q_MIN[i], config.Q_MAX[i], 1.0, " °")
            spin.setKeyboardTracking(False)             # พิมพ์เสร็จกด Enter ค่อยใช้
            spin.setMinimumWidth(92)
            actual = theme.label("—", "actual")
            actual.setFont(theme.mono(10, bold=True))
            actual.setMinimumWidth(78)
            actual.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            grid.addLayout(self._joint_name(i), r, 0)
            for col, w in enumerate((slider, spin, actual), start=1):
                grid.addWidget(w, r, col)
            slider.valueChanged.connect(lambda v, i=i: self.on_joint(i, v / SLIDER_SCALE))
            spin.valueChanged.connect(lambda v, i=i: self.on_joint(i, v))
            self.rows[i] = (slider, spin, actual)
        grid.setColumnStretch(1, 1)
        lay.addLayout(grid)

        zero = theme.button("Zero (ท่าตั้งตรง)")
        zero.clicked.connect(lambda: (self.set_target([0.0] * len(config.JOINTS)), self.send()))
        pin = theme.button("📌 ตั้งท่านี้เป็น Standby", "ghost",
                           "บันทึกท่า Target ตอนนี้ลง config.py — กด Connect ครั้งต่อไปแขนจะไปท่านี้เอง")
        pin.clicked.connect(self.set_standby_here)
        bar = QHBoxLayout()
        bar.addWidget(pin)
        bar.addStretch(1)
        bar.addWidget(zero)
        lay.addSpacing(4)
        lay.addLayout(bar)
        lay.addWidget(theme.label("ใน 3D: สีส้ม = Target · สีฟ้าโปร่ง = ท่าจริง (Actual)", "hint"))
        return self._page(frame)

    def _build_jog_tab(self):
        self.jog_buttons = {}

        def hold(grid, text, row, col, fn):
            b = theme.button(text, "jog")
            b.setAutoRepeat(False)
            b.pressed.connect(lambda: self.jog(fn))          # กดค้าง = เร่งขึ้น
            b.released.connect(self.jog_release)             # ปล่อย = ผ่อนจนหยุด
            grid.addWidget(b, row, col)
            return b

        joints, jl = theme.card("ข้อต่อ", "+ = ก้มหน้า / ฐานหันซ้าย")
        grid = QGridLayout()
        grid.setColumnStretch(0, 1)
        for r, i in enumerate(USED):
            grid.addLayout(self._joint_name(i), r, 0)
            for col, d in ((1, -1), (2, +1)):
                self.jog_buttons[(i, d)] = hold(grid, "−" if d < 0 else "+", r, col,
                                                lambda m, i=i, d=d: m.jog_joint(i, d))
        jl.addLayout(grid)

        tip, tl = theme.card("ปลายแขน", "เส้นตรง, φ คงที่")
        grid = QGridLayout()
        grid.setColumnStretch(0, 1)
        for k, axis in enumerate("XYZ"):
            grid.addWidget(theme.label(f"{axis}  (mm)", "name"), k, 0)
            for col, d in ((1, -1), (2, +1)):
                self.jog_buttons[(axis, d)] = hold(grid, f"{axis}{'−' if d < 0 else '+'}", k, col,
                                                   lambda m, k=k, d=d: m.jog_xyz(k, d))
        tl.addLayout(grid)
        hint = theme.label("กดค้าง = แขนเคลื่อนต่อเนื่อง · ปล่อย = หยุดนุ่ม ๆ\n"
                           "jog ปลายแขนหยุดเองก่อนเข้า singularity (ปลายแขนเหนือแกนฐาน / แขนเหยียดสุด)\n"
                           "— หลักฐาน |det J| ดูที่การ์ด Singularity ด้านล่าง · ถอยออกจาก singularity ได้เสมอ", "hint")
        return self._page(joints, tip, hint)

    def jog(self, fn):
        if self._ready(self.driver.mover.q):
            self.driver.do(fn)

    def jog_release(self):
        self.driver.do(lambda m: m.release())

    def _build_ik_tab(self):
        reach = config.D1 + config.A2 + config.A3 + config.A4
        self.ik_x = _spin(-reach, reach, 5, " mm", 110)
        self.ik_y = _spin(-reach, reach, 5, " mm", 10)
        self.ik_z = _spin(-reach / 2, reach, 5, " mm", 350)
        self.ik_phi = _spin(-180, 180, 5, " °", 40)
        target, tl = theme.card("เป้าหมายปลายแขน", "φ: 0° = ชี้ไปข้างหน้า · −90° = ชี้ลงพื้น · +90° = ชี้ฟ้า")
        form = QGridLayout()
        form.setHorizontalSpacing(10)
        for k, (name, w) in enumerate((("X", self.ik_x), ("Y", self.ik_y), ("Z", self.ik_z),
                                        ("φ มุมปลายแขน", self.ik_phi))):
            form.addWidget(theme.label(name, "name"), k // 2, (k % 2) * 2)
            form.addWidget(w, k // 2, (k % 2) * 2 + 1)
        form.setColumnStretch(1, 1)
        form.setColumnStretch(3, 1)
        tl.addLayout(form)
        btns = QHBoxLayout()
        for text, kind, fn in (("ดึงค่าจากท่าปัจจุบัน", "ghost", self.ik_copy),
                               ("👁 Preview (ไม่ส่ง)", None, lambda: self.ik_solve(False)),
                               ("▶ Go (ส่งไปแขน)", "primary", lambda: self.ik_solve(True))):
            b = theme.button(text, kind)
            b.clicked.connect(fn)
            btns.addWidget(b)
        tl.addLayout(btns)
        tl.addWidget(theme.label("แขน 4 ข้อคุมทิศได้แค่ φ (yaw ถูกกำหนดโดยตำแหน่ง X, Y เอง)", "hint"))

        self.ik_sketch = IKSketch()                  # รูปมุมมองข้าง: ทำไมไปได้/ไม่ได้
        sketch, sl = theme.card("ไปได้ไหม — มุมมองข้าง")
        sl.addWidget(self.ik_sketch, 1)
        self.ik_result = QTextEdit(readOnly=True)
        result, rl = theme.card("ผลลัพธ์ IK")
        rl.addWidget(self.ik_result, 1)
        split = QSplitter(Qt.Orientation.Vertical)
        split.addWidget(sketch)
        split.addWidget(result)
        split.setSizes([300, 330])
        w = self._page(target, split)
        w.layout().takeAt(w.layout().count() - 1)    # ไม่เว้นท้าย: ให้รูป + ผลลัพธ์ยืดเต็ม
        return w

    def _build_status_tab(self):
        cols = ["Joint", "Online", "Raw", "Actual °", "Error °", "Temp °C", "Load %"]
        self.status_table = QTableWidget(len(USED), len(cols))
        self.status_table.setHorizontalHeaderLabels(cols)
        self.status_table.verticalHeader().setVisible(False)
        self.status_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.status_table.setAlternatingRowColors(True)
        self.status_table.setShowGrid(False)
        self.status_table.setFont(theme.mono(10))
        self.status_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        for r, i in enumerate(USED):
            self.status_table.setRowHeight(r, 32)
            self.status_table.setItem(r, 0, QTableWidgetItem(config.JOINTS[i]["name"]))
        self.status_table.setFixedHeight(32 * len(USED) + 36)
        self.summary = theme.label("", "title")
        self.correct_box = QCheckBox("ชดเชยแขนยวบหลังถึงเป้า")
        self.correct_box.setChecked(True)
        self.correct_box.toggled.connect(lambda on: setattr(self.driver, "correct", on))
        box, bl = theme.card("Servo")
        for x in (self.summary, self.status_table, self.correct_box,
                  theme.label("Error = Target − ท่าจริง · หลังถึงเป้าโปรแกรมชดเชยแขนยวบให้อัตโนมัติ (≤ ±5°)", "hint")):
            bl.addWidget(x)
        return self._page(box)

    def _build_fk_box(self):
        """แถวล่าง: ตำแหน่งปลายแขน (FK) · เมทริกซ์ T⁰₄ · ผลตรวจ singularity — ของท่า Target"""
        pos, pl = theme.card("ปลายแขน (FK ของท่า Target)")
        grid = QGridLayout()
        grid.setHorizontalSpacing(18)
        self.fk_values = {}
        for k, (name, unit) in enumerate([("X", "mm"), ("Y", "mm"), ("Z", "mm"), ("φ", "°"), ("Yaw", "°")]):
            val = theme.label("0.0", "tile-value")
            val.setFont(theme.mono(15, bold=True))
            val.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            val.setMinimumWidth(84)
            grid.addWidget(theme.label(f"{name}  ({unit})", "tile-name"), 0, k, Qt.AlignmentFlag.AlignRight)
            grid.addWidget(val, 1, k)
            self.fk_values[name] = val
        pl.addLayout(grid)
        pl.addStretch(1)

        mat, ml = theme.card("T⁰₄", "ท่าปลายแขนเทียบฐาน")
        self.fk_table = QTableWidget(4, 4)
        self.fk_table.setFont(theme.mono(10))
        self.fk_table.horizontalHeader().setVisible(False)
        self.fk_table.verticalHeader().setVisible(False)
        self.fk_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.fk_table.setShowGrid(False)
        self.fk_table.setFixedSize(4 * 70 + 4, 4 * 23 + 4)
        self.fk_table.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.fk_table.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        for c in range(4):
            self.fk_table.setColumnWidth(c, 70)
            self.fk_table.setRowHeight(c, 23)
        ml.addWidget(self.fk_table)

        sing, sl = theme.card("Singularity")
        self.sing_label = QLabel()                    # ผลตรวจ singularity ของท่า Target + หลักฐาน (singularity.py)
        self.sing_label.setFont(theme.mono(9))
        self.sing_label.setToolTip("|det J| = r·a2·a3·|sin θ3| — คำนวณ 2 ทาง (ตัวเลข/สูตร) ต้องเท่ากัน\n"
                                   "= 0 เมื่อปลายแขนอยู่บนแกนฐาน (r = 0) หรือแขนเหยียด/พับสุด (θ3 = 0°/180°)")
        sl.addWidget(self.sing_label)
        sl.addStretch(1)

        w = QWidget()
        lay = QHBoxLayout(w)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(10)
        lay.addWidget(pos, 3)
        lay.addWidget(mat, 0)
        lay.addWidget(sing, 2)
        return w

    # ================= ท่า Target =================
    def set_target(self, q):
        """ตั้งท่าที่จะสั่ง → อัปเดต slider/ช่องตัวเลข, FK, ภาพ 3D (ยังไม่ส่ง)"""
        # ข้อที่อ่านไม่ได้ (nan — servo ไม่ตอบ) ใช้ค่าเดิม: เดิม slider รับ nan แล้วโปรแกรมพังตอน Connect
        self.q_target = clamp_q([old if v is None or math.isnan(v) else float(v) for old, v in zip(self.q_target, q)])
        for i, (slider, spin, _a) in self.rows.items():
            _set_quiet(slider, int(round(self.q_target[i] * SLIDER_SCALE)))
            _set_quiet(spin, self.q_target[i])
        fk = forward_kinematics(self.q_target)
        yaw = math.degrees(math.atan2(fk.y, fk.x)) if math.hypot(fk.x, fk.y) > 1e-6 else self.q_target[0]
        for name, v in (("X", fk.x), ("Y", fk.y), ("Z", fk.z), ("φ", fk.phi), ("Yaw", yaw)):
            self.fk_values[name].setText(f"{v:.1f}")
        for r in range(4):
            for c in range(4):
                item = QTableWidgetItem(f"{fk.T[r, c]:.1f}" if c == 3 else f"{fk.T[r, c]:.3f}")
                item.setTextAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
                self.fk_table.setItem(r, c, item)
        warn = bool(analyze(self.q_target)["warnings"])
        self.sing_label.setText(report(self.q_target))
        self.sing_label.setStyleSheet("color:#e63946; font-weight:bold" if warn else f"color:{theme.OK}")
        self.viewer.show_target(self.q_target)

    def on_joint(self, i, deg):
        q = list(self.q_target)
        q[i] = deg
        self.set_target(q)
        if self._ready():                              # slider ขับแขนทันที (ไม่ต้องกด Send)
            target = list(self.q_target)
            self.driver.do(lambda m: m.follow(target))

    # ================= สั่งแขน =================
    def _ready(self, q=None):
        """เช็คก่อนสั่ง — คืน True ถ้าส่งได้ (ไม่ได้ → เขียนเหตุผลลง log)"""
        why = check_limits(q or self.q_target)
        if not why and not self.torque_on:
            why = "Torque ปิดอยู่ — กด Torque ON ก่อน"
        if not why and not self.link.online():
            why = "แขนออฟไลน์: servo ไม่ตอบ"
        if not why and not self.link.simulated and not (self.link.volt or 0) >= config.VOLT_MIN:
            why = f"ไฟ servo ต่ำ ({self.link.volt} V < {config.VOLT_MIN} V) — ไม่ส่ง"
        if why:
            self.log("❌ " + why)
        return not why

    def send(self):
        if self._ready():
            target = list(self.q_target)
            self.driver.do(lambda m: m.goto_q(target))
            self.log("▶ Send: " + "  ".join(f"{target[i]:.1f}" for i in USED))

    def stop(self):
        self.driver.emergency_stop()
        self.set_target(self.driver.mover.q)
        self.log("■ STOP — แขนค้างที่ตำแหน่งจริง")

    def set_torque(self, on):
        self.driver.emergency_stop()
        self.link.torque(on)
        self.torque_on = on
        self.torque_btn.setText("Torque OFF" if on else "Torque ON")
        if on and self.link.q_actual is not None:
            self.driver.do(lambda m: m.reset(self.link.q_actual))
            self.set_target(self.link.q_actual)
        self.log("Torque ON — แขนค้างท่านี้" if on else "Torque OFF — แขนหมดแรง ประคองไว้ (ท่าใน GUI ตามแขนจริง)")

    # ================= IK =================
    def ik_copy(self):
        fk = forward_kinematics(self.q_target)
        for w, v in ((self.ik_x, fk.x), (self.ik_y, fk.y), (self.ik_z, fk.z), (self.ik_phi, fk.phi)):
            w.setValue(float(v))

    def ik_solve(self, send):
        x, y, z, phi = self.ik_x.value(), self.ik_y.value(), self.ik_z.value(), self.ik_phi.value()
        self.viewer.show_ik_marker((x, y, z))
        d = diagnose(x, y, z, phi, theta1_prev=self.q_target[0])     # ไปได้ไหม + เหตุผล (ik_explain.py)
        self.ik_sketch.set_diagnosis(d)
        self.ik_result.setHtml(_ik_html(d))
        best = d.q
        if best is None:
            self.log(f"❌ IK ({x:.0f}, {y:.0f}, {z:.0f}) φ {phi:.0f}°: " + d.reasons[0])
            return
        if not d.exact:
            self.log(d.headline + " — " + d.reasons[0])
        for w in d.warnings:
            self.log("⚠ " + w)
        if analyze(best)["warnings"]:
            self.log(f"⚠ จุด ({x:.0f}, {y:.0f}, {z:.0f}) ใกล้ singularity — ดูหลักฐานในแท็บ IK")
        self.set_target(best)
        if send:
            self.send()

    # ================= เชื่อมต่อ =================
    def _use_link(self, link):
        if self.driver:
            self.driver.close()
        if self.link:
            self.link.close()
        self.link = link
        q0 = link.q_actual if link.q_actual is not None else [0.0] * len(config.JOINTS)
        self.driver = Driver(link, Mover(q0))
        self.driver.correct = self.correct_box.isChecked()
        self.driver.start()
        self.torque_on = True
        self.torque_btn.setText("Torque OFF")
        self.connect_btn.setText("Connect" if link.simulated else "Disconnect")
        self._label("● Simulation" if link.simulated else "● Connected", "#888" if link.simulated else "#2a9d8f")

    def startup(self):
        """เปิดโปรแกรม: ต่อแขนเอง → ไปท่า Standby (ต่อไม่ได้ → แขนจำลองไปท่า Standby แทน)"""
        self.log(f"เปิดโปรแกรม: กำลังต่อแขนผ่าน {config.DIRECT_PORT} แล้วไปท่า Standby ...")
        self.toggle_connect()                        # ต่อได้ → go_standby() ถูกเรียกในนั้นแล้ว
        if self.link.simulated:
            self.log("ต่อแขนไม่ได้ → แสดงแขนจำลองที่ท่า Standby แทน (กด Connect เพื่อลองใหม่)")
            self.go_standby()

    def toggle_connect(self):
        if not self.link.simulated:
            self._use_link(SimLink())
            self.log("ตัดการเชื่อมต่อ — กลับโหมด Simulation (แขนค้างท่าสุดท้าย)")
            return
        try:
            link = self.connect_fn()
        except (serial.SerialException, OSError) as e:
            self._label("● ต่อไม่ได้", "#e63946", str(e))
            self.log(f"❌ เปิด {config.DIRECT_PORT} ไม่ได้ (เสียบ adapter / ปิดโปรแกรมอื่นที่ใช้พอร์ต?): {e}")
            return
        end = time.monotonic() + 2.0
        while link.q_actual is None and time.monotonic() < end:       # รอค่าแรกจาก servo
            QApplication.processEvents()
            time.sleep(0.02)
        if link.q_actual is None:
            link.close()
            self._label("● servo ไม่ตอบ", "#e63946")
            self.log("❌ ต่อพอร์ตได้แต่ servo ไม่ตอบ — เปิดไฟ servo 7.4 V / จัมเปอร์ B ?")
            return
        self._use_link(link)
        self.set_target(link.q_actual)
        self.log(f"เชื่อมต่อ {config.DIRECT_PORT} แล้ว — แขนค้างที่เดิม, Target = ท่าจริง")
        self.go_standby()                            # เปิดใช้งานทุกครั้ง → ไปท่า Standby เอง (ช้า ๆ, เช็คไฟ/servo ก่อน)

    # ================= ท่า Standby =================
    def go_standby(self):
        """วิ่งไปท่า Standby ด้วย STANDBY_V (ช้า) — ไม่ไปถ้าไฟต่ำ / servo ตอบไม่ครบ / torque ปิด"""
        q = config.STANDBY_Q
        act = self.link.q_actual
        if act is None or any(math.isnan(act[i]) for i in USED):
            self.log("❌ ไม่ไปท่า Standby: servo ตอบไม่ครบทุกข้อ")
            return False
        if not self._ready(q):
            return False
        self.set_target(q)
        self.driver.do(lambda m: m.goto_q(q, v=config.STANDBY_V))
        self.log(f"⌂ กำลังไปท่า Standby ({config.STANDBY_V:.0f} °/s): "
                 + "  ".join(f"{q[i]:.1f}" for i in USED) + " — กด STOP / Space เพื่อหยุด")
        return True

    def set_standby_here(self):
        """บันทึกท่า Target ตอนนี้เป็นท่า Standby (เขียน config.py + ใช้ทันที)"""
        q = list(self.q_target)
        text = "  ".join(f"{config.JOINTS[i]['name']} {q[i]:.1f}°" for i in USED)
        if QMessageBox.question(self, "ตั้งท่า Standby", f"{text}\n\nใช้ท่านี้เป็นท่า Standby?") \
                != QMessageBox.StandardButton.Yes:
            return
        try:
            self.save_standby(q)
        except ValueError as e:
            self.log(f"❌ ไม่บันทึกท่า Standby: {e}")
            return
        config.STANDBY_Q = [round(v, 1) for v in q]
        self.log("📌 บันทึกท่า Standby แล้ว: " + text)

    def _label(self, text, color, tip=""):
        self.conn_label.setText(text)
        self.conn_label.setToolTip(tip)
        self.conn_label.setStyleSheet(theme.pill(color))

    # ================= อัปเดตทุก 50 ms =================
    def poll(self):
        link, m = self.link, self.driver.mover
        if self.driver.message:
            self.log(self.driver.message)
            self.driver.message = ""
        if m.mode in ("jog_joint", "jog_xyz"):                      # jog: Target/slider ตามท่าที่กำลังวิ่ง
            self.set_target(m.q)
        q_act = link.q_actual if not link.simulated else m.q        # จำลอง: แสดงท่าที่กำลังวิ่ง
        if not self.torque_on and link.q_actual is not None:          # ปลด torque: Target ตามแขนจริง
            self.set_target(link.q_actual)
        self.viewer.show_actual(q_act)
        for r, i in enumerate(USED):
            a = None if q_act is None or math.isnan(q_act[i]) else q_act[i]
            self.rows[i][2].setText("—" if a is None else f"{a:6.1f}°")
            s = getattr(link, "servo", {}).get(config.JOINTS[i]["id"])
            cells = ["✅" if (a is not None and (link.simulated or s)) else "❌",
                     str(s["raw"]) if s else "-",
                     "-" if a is None else f"{a:.1f}",
                     "-" if a is None or m.busy else f"{self.q_target[i] - a:+.2f}",
                     str(s["temp"]) if s else "-",
                     f"{s['load']:+.1f}" if s else "-"]
            for c, text in enumerate(cells, start=1):
                self.status_table.setItem(r, c, QTableWidgetItem(text))
        if link.simulated:
            self.summary.setText("โหมด Simulation (ไม่ได้ต่อแขน)")
        else:
            v = link.volt
            low = v is None or v < config.VOLT_MIN
            self.summary.setText(f"ไฟเลี้ยง {v if v is None else round(v, 1)} V   •   Torque: "
                                 f"{'ON' if self.torque_on else 'OFF (ขยับด้วยมือได้)'}   •   "
                                 f"{'ออฟไลน์' if not link.online() else 'กำลังวิ่ง' if m.busy else 'นิ่ง'}")
            if not link.online():
                self._label("● ออฟไลน์ (servo ไม่ตอบ)", "#e63946")
            elif low:
                self._label(f"● Connected — ไฟ servo ต่ำ {v} V (ต้อง ≥ {config.VOLT_MIN:.0f} V)", "#f4a261")
            else:
                self._label(f"● Connected  {v:.1f} V", "#2a9d8f")

    def log(self, text):
        self.log_view.appendPlainText(f"[{datetime.now():%H:%M:%S}] {text}")

    def closeEvent(self, event):
        self.timer.stop()
        self.driver.close()
        self.link.close()
        self.viewer.close()
        super().closeEvent(event)


def main():
    # Windows ของเครื่องนี้ตั้ง Region เป็นไทย → Qt แสดงเลขไทย (๒๐.๐) ในช่องตัวเลข → บังคับเลขอารบิก
    QLocale.setDefault(QLocale(QLocale.Language.English, QLocale.Country.UnitedStates))
    app = QApplication(sys.argv)
    theme.apply(app)
    w = MainWindow()
    w.show()
    QTimer.singleShot(300, w.startup)                # รอหน้าต่างขึ้นก่อน (เห็นปุ่ม STOP) แล้วค่อยต่อแขน + ไปท่า Standby
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
