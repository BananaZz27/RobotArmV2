# หน้าต่างควบคุมแขน หน้าตาแบบ ws-control (PyQt6 + 3D PyVista) — แต่ต่อ servo ตรงผ่าน adapter (ไม่มี ESP32)
#   และกดส่งแล้วแขนวิ่งนุ่ม (Mover/Driver) · RobotArm.bat  หรือ  cd src && ../.venv/Scripts/python.exe qt_app.py
#
#  ┌ [Connect COM11] ● สถานะ ──────────────────────── [Torque OFF] [■ STOP (Space)] ┐
#  │ ┌──────── 3D ────────┐ ┌ แท็บ: Joint | Inverse Kinematics | Status ┐          │
#  │ └────────────────────┘ └───────────────────────────────────────────┘          │
#  │ Forward Kinematics (ท่า Target): X Y Z φ Yaw + T⁰₅                            │
#  │ Log                                                                           │
#  └───────────────────────────────────────────────────────────────────────────────┘
import math
import sys
import time
from datetime import datetime

import serial
from PyQt6.QtCore import QLocale, Qt, QTimer
from PyQt6.QtGui import QFont, QKeySequence, QShortcut
from PyQt6.QtWidgets import (QApplication, QCheckBox, QDoubleSpinBox, QFormLayout, QGridLayout, QGroupBox,
                             QHBoxLayout, QLabel, QMainWindow, QPlainTextEdit, QPushButton, QSlider, QSplitter,
                             QTableWidget, QTableWidgetItem, QTabWidget, QTextEdit, QVBoxLayout, QWidget)

import config
from driver import Driver
from kinematics import check_limits, forward_kinematics, inverse_kinematics
from link import DirectLink, SimLink
from motion import Mover, clamp_q

USED = [i for i, j in enumerate(config.JOINTS) if j["used"]]
SLIDER_SCALE = 10      # slider เป็น int → ×10 ให้ละเอียด 0.1°
MONO = QFont("Consolas")


def _spin(lo, hi, step, suffix, value=0.0, decimals=1):
    s = QDoubleSpinBox(decimals=decimals, minimum=lo, maximum=hi, singleStep=step, suffix=suffix)
    s.setValue(value)
    return s


def _set_quiet(widget, value):
    """ตั้งค่าโดยไม่ยิง signal (กันวนกลับมาสั่งแขน)"""
    widget.blockSignals(True)
    widget.setValue(value)
    widget.blockSignals(False)


class MainWindow(QMainWindow):
    def __init__(self, link=None, connect_fn=None, viewer=None):
        super().__init__()
        self.connect_fn = connect_fn or (lambda: DirectLink(config.DIRECT_PORT))
        self.link, self.driver = None, None
        self.q_target = [0.0] * 5
        self.torque_on = True
        self.setWindowTitle("RobotArm 4DOF — Control GUI (USB adapter)")
        self.resize(1400, 900)

        if viewer is None:
            from viewer3d import Viewer3D            # import ช้า (VTK) — test ส่ง viewer ปลอมมาแทน
            viewer = Viewer3D()
        self.viewer = viewer
        self.tabs = QTabWidget()
        self.tabs.addTab(self._build_joint_tab(), "Joint")
        self.tabs.addTab(self._build_ik_tab(), "Inverse Kinematics")
        self.tabs.addTab(self._build_status_tab(), "Status")
        self.log_view = QPlainTextEdit(readOnly=True, maximumBlockCount=500)
        self.log_view.setMaximumHeight(130)
        split = QSplitter(Qt.Orientation.Horizontal)
        split.addWidget(self.viewer)
        split.addWidget(self.tabs)
        split.setSizes([800, 600])

        central = QWidget()
        lay = QVBoxLayout(central)
        lay.addLayout(self._build_toolbar())
        lay.addWidget(split, stretch=1)
        lay.addWidget(self._build_fk_box())
        lay.addWidget(self.log_view)
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
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connect)
        self.conn_label = QLabel()
        self.torque_btn = QPushButton("Torque OFF")
        self.torque_btn.setToolTip("ปิด torque เพื่อจับแขนขยับด้วยมือ — ท่าใน GUI จะตามแขนจริง")
        self.torque_btn.clicked.connect(lambda: self.set_torque(not self.torque_on))
        stop = QPushButton("■ STOP  (Space)")
        stop.setStyleSheet("background:#e63946; color:white; font-weight:bold; padding:6px 18px")
        stop.clicked.connect(self.stop)
        bar = QHBoxLayout()
        for w in (QLabel(f"Port {config.DIRECT_PORT}"), self.connect_btn, self.conn_label):
            bar.addWidget(w)
        bar.addStretch(1)
        bar.addWidget(self.torque_btn)
        bar.addWidget(stop)
        return bar

    def _build_joint_tab(self):
        grid = QGridLayout()
        for col, text in enumerate(("Joint", "", "Target", "Actual")):
            grid.addWidget(QLabel(f"<b>{text}</b>"), 0, col)
        self.rows = {}
        for r, i in enumerate(USED, start=1):
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(int(config.Q_MIN[i] * SLIDER_SCALE), int(config.Q_MAX[i] * SLIDER_SCALE))
            spin = _spin(config.Q_MIN[i], config.Q_MAX[i], 1.0, " °")
            spin.setKeyboardTracking(False)             # พิมพ์เสร็จกด Enter ค่อยใช้
            actual = QLabel("—")
            actual.setMinimumWidth(60)
            actual.setStyleSheet("color:#3a86ff")
            for col, w in enumerate((QLabel(config.JOINTS[i]["name"]), slider, spin, actual)):
                grid.addWidget(w, r, col)
            slider.valueChanged.connect(lambda v, i=i: self.on_joint(i, v / SLIDER_SCALE))
            spin.valueChanged.connect(lambda v, i=i: self.on_joint(i, v))
            self.rows[i] = (slider, spin, actual)
        grid.setColumnStretch(1, 1)

        zero = QPushButton("Zero (ท่าตั้งตรง)")
        zero.clicked.connect(lambda: (self.set_target([0.0] * 5), self.send()))
        bar = QHBoxLayout()
        bar.addStretch(1)
        bar.addWidget(zero)
        hint = QLabel("ลาก slider / พิมพ์ตัวเลข → แขนตามทันที · สีส้มใน 3D = Target, สีฟ้าโปร่ง = ท่าจริง\n"
                      "J5 หัวถอดแล้ว")
        hint.setStyleSheet("color:#666")
        w = QWidget()
        lay = QVBoxLayout(w)
        lay.addLayout(grid)
        lay.addLayout(bar)
        lay.addWidget(hint)
        lay.addStretch(1)
        return w

    def _build_ik_tab(self):
        reach = config.D1 + config.A2 + config.A3 + config.D5
        self.ik_x = _spin(-reach, reach, 5, " mm", 110)
        self.ik_y = _spin(-reach, reach, 5, " mm", 10)
        self.ik_z = _spin(-reach / 2, reach, 5, " mm", 350)
        self.ik_phi = _spin(-180, 180, 5, " °", 40)
        form = QFormLayout()
        for name, w in (("X", self.ik_x), ("Y", self.ik_y), ("Z", self.ik_z), ("φ (มุมปลายแขน)", self.ik_phi)):
            form.addRow(name, w)
        btns = QHBoxLayout()
        for text, fn in (("⟲ ดึงค่าจากท่าปัจจุบัน", self.ik_copy), ("👁 Preview (ไม่ส่ง)", lambda: self.ik_solve(False)),
                         ("▶ Go (ส่งไปแขน)", lambda: self.ik_solve(True))):
            b = QPushButton(text)
            b.clicked.connect(fn)
            btns.addWidget(b)
        self.ik_result = QTextEdit(readOnly=True)
        self.ik_result.setFont(MONO)
        help_ = QLabel("φ: 0° = ปลายแขนชี้ออกไปข้างหน้า, −90° = ชี้ลงพื้น, +90° = ชี้ฟ้า\n"
                       "แขน 4 ข้อคุมทิศได้แค่ φ (yaw ถูกกำหนดโดยตำแหน่ง X, Y เอง)")
        help_.setStyleSheet("color:#666")
        w = QWidget()
        lay = QVBoxLayout(w)
        lay.addLayout(form)
        lay.addLayout(btns)
        lay.addWidget(help_)
        lay.addWidget(QLabel("<b>ผลลัพธ์ IK</b>"))
        lay.addWidget(self.ik_result)
        return w

    def _build_status_tab(self):
        cols = ["Joint", "Online", "Raw", "Actual °", "Error °", "Temp °C", "Load %"]
        self.status_table = QTableWidget(len(USED), len(cols))
        self.status_table.setHorizontalHeaderLabels(cols)
        self.status_table.verticalHeader().setVisible(False)
        self.status_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        for r, i in enumerate(USED):
            self.status_table.setItem(r, 0, QTableWidgetItem(config.JOINTS[i]["name"]))
        self.summary = QLabel()
        note = QLabel("Error = Target − ท่าจริง · หลังถึงเป้าโปรแกรมชดเชยแขนยวบให้อัตโนมัติ (≤ ±5°)")
        note.setStyleSheet("color:#666")
        self.correct_box = QCheckBox("ชดเชยแขนยวบหลังถึงเป้า")
        self.correct_box.setChecked(True)
        self.correct_box.toggled.connect(lambda on: setattr(self.driver, "correct", on))
        box = QGroupBox("Servo")
        bl = QVBoxLayout(box)
        for x in (self.summary, self.status_table, note, self.correct_box):
            bl.addWidget(x)
        w = QWidget()
        QVBoxLayout(w).addWidget(box)
        return w

    def _build_fk_box(self):
        box = QGroupBox("Forward Kinematics (ท่า Target)")
        grid = QGridLayout()
        self.fk_values = {}
        for k, (name, unit) in enumerate([("X", "mm"), ("Y", "mm"), ("Z", "mm"), ("φ", "°"), ("Yaw", "°")]):
            lbl = QLabel("0.0")
            lbl.setFont(MONO)
            lbl.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            lbl.setMinimumWidth(70)
            grid.addWidget(QLabel(name), k % 3, (k // 3) * 3)
            grid.addWidget(lbl, k % 3, (k // 3) * 3 + 1)
            grid.addWidget(QLabel(unit), k % 3, (k // 3) * 3 + 2)
            self.fk_values[name] = lbl
        self.fk_table = QTableWidget(4, 4)
        self.fk_table.setFont(MONO)
        self.fk_table.horizontalHeader().setVisible(False)
        self.fk_table.verticalHeader().setVisible(False)
        self.fk_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.fk_table.setFixedSize(4 * 72 + 4, 4 * 24 + 4)
        for c in range(4):
            self.fk_table.setColumnWidth(c, 72)
            self.fk_table.setRowHeight(c, 24)
        lay = QHBoxLayout(box)
        lay.addLayout(grid)
        lay.addSpacing(12)
        lay.addWidget(QLabel("T⁰₅ ="))
        lay.addWidget(self.fk_table)
        lay.addStretch(1)
        return box

    # ================= ท่า Target =================
    def set_target(self, q):
        """ตั้งท่าที่จะสั่ง → อัปเดต slider/ช่องตัวเลข, FK, ภาพ 3D (ยังไม่ส่ง)"""
        self.q_target = clamp_q([float(v) for v in q])
        for i, (slider, spin, _a) in self.rows.items():
            _set_quiet(slider, int(round(self.q_target[i] * SLIDER_SCALE)))
            _set_quiet(spin, self.q_target[i])
        fk = forward_kinematics(self.q_target)
        yaw = math.degrees(math.atan2(fk.y, fk.x)) if math.hypot(fk.x, fk.y) > 1e-6 else self.q_target[0]
        for name, v in (("X", fk.x), ("Y", fk.y), ("Z", fk.z), ("φ", fk.phi), ("Yaw", yaw)):
            self.fk_values[name].setText(f"{v:8.1f}")
        for r in range(4):
            for c in range(4):
                item = QTableWidgetItem(f"{fk.T[r, c]:.1f}" if c == 3 else f"{fk.T[r, c]:.3f}")
                item.setTextAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
                self.fk_table.setItem(r, c, item)
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
        lines, best = [], None
        for elbow_up, name in ((True, "elbow up"), (False, "elbow down")):
            ik = inverse_kinematics(x, y, z, phi, elbow_up, theta1_prev=self.q_target[0])
            if ik.ok:
                lines.append(f"{'★' if best is None else '✓'} {name:<10} [" +
                             "  ".join(f"{ik.q[i]:7.1f}" for i in USED) + "]")
                best = best or ik.q
            else:
                lines.append(f"✗ {name:<10} {ik.reason}")
        head = "✅ ได้คำตอบ (★ = ที่ใช้)" if best else "❌ ไปจุดนี้ไม่ได้"
        self.ik_result.setPlainText(head + "\n\n" + "\n".join(lines))
        if best:
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
        q0 = link.q_actual if link.q_actual is not None else [0.0] * 5
        self.driver = Driver(link, Mover(q0))
        self.driver.correct = self.correct_box.isChecked()
        self.driver.start()
        self.torque_on = True
        self.torque_btn.setText("Torque OFF")
        self.connect_btn.setText("Connect" if link.simulated else "Disconnect")
        self._label("● Simulation" if link.simulated else "● Connected", "#888" if link.simulated else "#2a9d8f")

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

    def _label(self, text, color, tip=""):
        self.conn_label.setText(text)
        self.conn_label.setToolTip(tip)
        self.conn_label.setStyleSheet(f"color:{color}; font-weight:bold")

    # ================= อัปเดตทุก 50 ms =================
    def poll(self):
        link, m = self.link, self.driver.mover
        if self.driver.message:
            self.log(self.driver.message)
            self.driver.message = ""
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
    w = MainWindow()
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
