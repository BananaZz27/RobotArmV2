# หน้าต่างควบคุมแขน (หน้าตาแบบ RobotControl v2) — แต่กด "ส่ง" แล้วแขนวิ่งนุ่มผ่าน Mover/Driver
#   ซ้าย: slider 5 ข้อ, FK (X Y Z φ + เมทริกซ์ T⁰₅), IK (X Y Z φ), การเชื่อมต่อ, ปุ่มหยุด
#   ขวา: ภาพจำลอง 2D สองมุม (sim_view.py)
#   RobotArm.bat  หรือ  cd src && ../.venv/Scripts/python.exe app.py
# เริ่มที่โหมดจำลองเสมอ — ต่อแขนจริงโดยเลือก "แขนตรง (USB adapter)" (แขนค้างที่เดิม ไม่ขยับเอง)
import tkinter as tk
from tkinter import ttk

import serial
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

import config
from driver import Driver
from kinematics import check_limits, forward_kinematics, inverse_kinematics
from link import DirectLink, SimLink
from motion import Mover
from sim_view import SimView

SIM = "จำลอง"
DIRECT = "แขนตรง (USB adapter)"
TICK_MS = 50


class App:
    def __init__(self, root, link=None, connect_fn=None):
        self.root = root
        self.connect_fn = connect_fn or (lambda: DirectLink(config.DIRECT_PORT))
        self.link, self.driver = None, None
        self.q_cmd = [0.0] * 5
        self.status = tk.StringVar()
        root.title("แขน 4 ข้อ — แบบนุ่ม")

        left = ttk.Frame(root, padding=8)
        left.pack(side="left", fill="y")
        self._build_joint_box(left)
        self._build_fk_box(left)
        self._build_ik_box(left)
        self._build_status_bar(left)

        figure = Figure(figsize=(9, 4.5))
        self.view = SimView(figure)
        FigureCanvasTkAgg(figure, master=root).get_tk_widget().pack(side="right", fill="both", expand=True)
        self._last_drawn = None
        self._use_link(link or SimLink())
        self.set_q(self.q_cmd)
        root.after(TICK_MS, self._tick_loop)

    # ---------- สร้างหน้าต่าง ----------

    def _build_joint_box(self, parent):
        box = ttk.LabelFrame(parent, text="มุม servo q (องศา)", padding=6)
        box.pack(fill="x")
        self.slider_vars, self.sliders = [], []
        for i, joint in enumerate(config.JOINTS):
            var = tk.DoubleVar(value=0.0)
            ttk.Label(box, text=joint["name"], width=8).grid(row=i, column=0, sticky="w")
            scale = tk.Scale(box, variable=var, from_=config.Q_MIN[i], to=config.Q_MAX[i],
                             resolution=0.1, orient="horizontal", length=260,
                             command=lambda _v: self.on_slider())
            scale.grid(row=i, column=1)
            if not joint["used"]:
                scale.config(state="disabled")    # J5 ถอดแล้ว → ล็อก 0
            self.slider_vars.append(var)
            self.sliders.append(scale)
        buttons = ttk.Frame(box)
        buttons.grid(row=5, column=0, columnspan=2, pady=4)
        ttk.Button(buttons, text="ส่งไปแขน", command=self.on_send).pack(side="left", padx=4)
        ttk.Button(buttons, text="ตั้งตรง (q = 0)", command=lambda: self.set_q([0] * 5)).pack(side="left", padx=4)
        ttk.Label(box, text="ความเร็ว °/s").grid(row=6, column=0, sticky="w")
        self.speed = tk.DoubleVar(value=config.V_JOINT)
        tk.Scale(box, variable=self.speed, from_=2, to=config.V_MAX_LIMIT, orient="horizontal", length=260,
                 command=lambda _v: self.on_speed()).grid(row=6, column=1)

    def _build_fk_box(self, parent):
        box = ttk.LabelFrame(parent, text="Forward Kinematics", padding=6)
        box.pack(fill="x", pady=6)
        self.fk_text = tk.StringVar()
        self.matrix_text = tk.StringVar()
        ttk.Label(box, textvariable=self.fk_text).pack(anchor="w")
        ttk.Label(box, textvariable=self.matrix_text, font=("Consolas", 9)).pack(anchor="w")

    def _build_ik_box(self, parent):
        box = ttk.LabelFrame(parent, text="Inverse Kinematics", padding=6)
        box.pack(fill="x")
        defaults = {"x": "110", "y": "10", "z": "350", "phi": "40"}
        labels = {"x": "X (mm)", "y": "Y (mm)", "z": "Z (mm)", "phi": "φ (องศา)"}
        self.ik_entries = {}
        for col, key in enumerate(defaults):
            var = tk.StringVar(value=defaults[key])
            ttk.Label(box, text=labels[key]).grid(row=0, column=col)
            ttk.Entry(box, textvariable=var, width=8).grid(row=1, column=col, padx=2)
            self.ik_entries[key] = var
        self.elbow_up = tk.BooleanVar(value=True)
        ttk.Checkbutton(box, text="elbow up", variable=self.elbow_up).grid(row=2, column=0, columnspan=2, sticky="w")
        ttk.Button(box, text="ดูก่อน", command=self.on_ik_preview).grid(row=2, column=2)
        ttk.Button(box, text="ส่ง", command=self.on_ik_send).grid(row=2, column=3)

    def _build_status_bar(self, parent):
        box = ttk.LabelFrame(parent, text="การเชื่อมต่อ", padding=6)
        box.pack(fill="x", pady=6)
        self.port_var = tk.StringVar(value=SIM)
        combo = ttk.Combobox(box, textvariable=self.port_var, values=[SIM, DIRECT], width=20, state="readonly")
        combo.grid(row=0, column=0)
        combo.bind("<<ComboboxSelected>>", lambda _e: self.connect(self.port_var.get()))
        self.volt_text = tk.StringVar(value=SIM)
        self.volt_label = tk.Label(box, textvariable=self.volt_text, width=16)
        self.volt_label.grid(row=0, column=1, padx=6)
        tk.Button(box, text="หยุด", bg="#d33", fg="white", width=8, command=self.on_stop).grid(row=0, column=2)
        self.actual_text = tk.StringVar()
        ttk.Label(box, textvariable=self.actual_text).grid(row=1, column=0, columnspan=3, sticky="w")
        self.corr_on = tk.BooleanVar(value=True)
        ttk.Checkbutton(box, text="ชดเชยแขนยวบ (แขนจริง)", variable=self.corr_on,
                        command=lambda: setattr(self.driver, "correct", self.corr_on.get())).grid(
            row=2, column=0, columnspan=3, sticky="w")
        self.corr_text = tk.StringVar()
        ttk.Label(box, textvariable=self.corr_text, wraplength=330).grid(row=3, column=0, columnspan=3, sticky="w")
        ttk.Label(box, textvariable=self.status, foreground="#a00", wraplength=330).grid(
            row=4, column=0, columnspan=3, sticky="w")

    # ---------- มุมที่จะสั่ง ----------

    def set_q(self, q):
        """ตั้งมุมที่จะสั่ง (เก็บละเอียดใน q_cmd, slider แค่แสดง) แล้วอัปเดต FK + ภาพ"""
        self.q_cmd = [float(v) if used else 0.0 for v, used in zip(q, config.USED)]
        for var, v in zip(self.slider_vars, self.q_cmd):
            var.set(round(v, 1))
        self.update_fk()

    def on_slider(self):
        # แก้เฉพาะข้อที่ผู้ใช้เลื่อนจริง (กัน slider ปัดเศษทับค่าละเอียดจาก IK)
        for i, var in enumerate(self.slider_vars):
            if abs(var.get() - round(self.q_cmd[i], 1)) > 0.05:
                self.q_cmd[i] = var.get()
        self.update_fk()

    def update_fk(self):
        fk = forward_kinematics(self.q_cmd)
        self.fk_text.set(f"X {fk.x:.1f}   Y {fk.y:.1f}   Z {fk.z:.1f} mm   φ {fk.phi:.1f}°")
        rows = ["[" + " ".join(f"{v:8.3f}" for v in row) + " ]" for row in fk.T]
        self.matrix_text.set("T⁰₅ =\n" + "\n".join(rows))

    # ---------- ปุ่ม ----------

    def on_send(self):
        reason = check_limits(self.q_cmd)
        if reason is None and not self.link.online():
            reason = "แขนออฟไลน์: servo ไม่ตอบ"
        if reason is None and not volt_ok(self.link.volt):
            reason = f"ไฟ servo ต่ำหรืออ่านไม่ได้ ({self.link.volt} V, ต้อง ≥ {config.VOLT_MIN} V) — ไม่ส่งคำสั่ง"
        if reason:
            self.status.set(reason)
            return False
        q = list(self.q_cmd)
        self.driver.do(lambda m: m.goto_q(q))         # วิ่งนุ่ม: ออกตัวช้า ผ่อนก่อนถึง ทุกข้อถึงพร้อมกัน
        self.corr_text.set("")
        self.status.set("กำลังไป")
        return True

    def on_ik_preview(self):
        """คำนวณ IK แล้วตั้ง slider + ภาพ (ยังไม่ส่ง) — คืน True ถ้าได้คำตอบ"""
        try:
            x, y, z, phi = (float(self.ik_entries[k].get()) for k in ("x", "y", "z", "phi"))
        except ValueError:
            self.status.set("กรอก X Y Z φ เป็นตัวเลขให้ครบ")
            return False
        if not -180 <= phi <= 180:
            self.status.set("φ ต้องอยู่ระหว่าง −180° ถึง 180°")
            return False
        ik = inverse_kinematics(x, y, z, phi, self.elbow_up.get(), roll=self.q_cmd[4], theta1_prev=self.q_cmd[0])
        if not ik.ok:
            self.status.set(ik.reason)
            return False
        self.set_q(ik.q)
        self.status.set("IK: " + "  ".join(f"{v:.1f}" for v in ik.q))
        return True

    def on_ik_send(self):
        if self.on_ik_preview():
            self.on_send()

    def on_speed(self):
        v = self.speed.get()

        def apply(m):
            m.v, m.v_cart = v, 2 * v
        self.driver.do(apply)

    def on_stop(self):
        self.driver.emergency_stop()
        self.status.set("หยุดแล้ว (ค้างท่าปัจจุบัน)")

    def _use_link(self, link):
        if self.driver:
            self.driver.close()
        if self.link:
            self.link.close()
        self.link = link
        q0 = link.q_actual if link.q_actual is not None else [0.0] * 5
        v = self.speed.get()
        self.driver = Driver(link, Mover(q0, v=v, v_cart=2 * v))
        self.driver.correct = self.corr_on.get()
        self.driver.start()

    def connect(self, name):
        try:
            link = SimLink() if name == SIM else self.connect_fn()
        except (serial.SerialException, OSError) as e:
            self.port_var.set(SIM)
            self.status.set(f"เปิดพอร์ต {config.DIRECT_PORT} ไม่ได้ (ปิดโปรแกรมอื่นที่ใช้พอร์ตอยู่หรือยัง?): {e}")
            return
        for _ in range(40):                               # รอค่าแรกจาก servo (≤ 2 s)
            if link.q_actual is not None:
                break
            self.root.after(50)
        if link.q_actual is None:
            link.close()
            self.port_var.set(SIM)
            self.status.set("ต่อพอร์ตได้แต่ servo ไม่ตอบ — เปิดไฟ servo / จัมเปอร์ B ?")
            return
        self._use_link(link)
        self.set_q(link.q_actual)                         # slider เริ่มที่ท่าจริง → กดส่งทันทีก็ไม่ขยับ
        self.status.set(f"เชื่อมต่อ {name}")

    # ---------- อัปเดตเป็นระยะ ----------

    def tick(self):
        if self.driver.message:
            self.corr_text.set(self.driver.message)
            self.driver.message = ""
            if self.status.get() == "กำลังไป":
                self.status.set("")
        online = self.link.online()
        q_act = self.link.q_actual
        if self.link.simulated:
            self.volt_text.set(SIM)
            self.volt_label.config(foreground="black")
        elif not online:
            self.volt_text.set("ออฟไลน์")
            self.volt_label.config(foreground="red")
        else:
            self.volt_text.set(f"ไฟ servo {self.link.volt:.1f} V")
            self.volt_label.config(foreground="green" if volt_ok(self.link.volt) else "red")
        if q_act is not None:
            self.actual_text.set("มุมจริง: " + "  ".join(f"{v:.1f}" for v in q_act))
        key = (tuple(self.q_cmd), None if q_act is None else tuple(q_act))
        if key != self._last_drawn:
            self.view.draw(self.q_cmd, q_act)
            self._last_drawn = key

    def _tick_loop(self):
        try:
            self.tick()
        finally:                        # ถึง tick พังก็ต้องวนต่อ ไม่งั้นหน้าจอค้าง
            self.root.after(TICK_MS, self._tick_loop)

    def close(self):
        self.driver.close()
        self.link.close()


def volt_ok(volt):
    """ไฟ servo พอไหม — None = โหมดจำลอง (ไม่มีไฟ), nan = อ่านไม่ได้ ถือว่าไม่พอ"""
    return volt is None or volt >= config.VOLT_MIN


def main():
    root = tk.Tk()
    app = App(root)
    root.protocol("WM_DELETE_WINDOW", lambda: (app.close(), root.destroy()))
    root.mainloop()


if __name__ == "__main__":
    main()
