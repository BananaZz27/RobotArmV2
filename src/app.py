# หน้าต่างควบคุมแขนแบบนุ่ม: Jog (กดค้าง) · เป้า (มุม / XYZ เส้นตรง) · Slider (แขนตามสด)
#   RobotArm.bat  หรือ  cd src && ../.venv/Scripts/python.exe app.py
# เปิดมาเป็นโหมดจำลองเสมอ → กด "เชื่อมต่อแขน" เมื่อพร้อม (ต่อแล้วแขนค้างที่เดิม ไม่ขยับเอง)
import tkinter as tk
from tkinter import ttk

import serial
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

import config
from driver import Driver
from kinematics import forward_kinematics
from link import DirectLink, SimLink
from motion import Mover
from sim_view import SimView

USED = [i for i, j in enumerate(config.JOINTS) if j["used"]]
AXES = ["X", "Y", "Z"]


class App:
    def __init__(self, root, link=None, connect_fn=None):
        self.root = root
        self.connect_fn = connect_fn or (lambda: DirectLink(config.DIRECT_PORT))
        self.link, self.driver = None, None
        self.torque_on = True
        root.title("RobotArmSmooth — ควบคุมแขนแบบนุ่ม")
        self._build()
        self._use_link(link or SimLink())
        self.tick()

    # ---------- สร้างหน้าต่าง ----------
    def _build(self):
        top = ttk.Frame(self.root, padding=6)
        top.pack(fill="x")
        tk.Button(top, text="■ หยุด", bg="#d33", fg="white", font=("", 12, "bold"), width=8,
                  command=self.on_stop).pack(side="left", padx=(0, 10))      # ซ้ายสุด เห็นเสมอ
        ttk.Button(top, text="เชื่อมต่อแขน (COM11)", command=self.on_connect).pack(side="left")
        ttk.Button(top, text="จำลอง", command=lambda: self._use_link(SimLink())).pack(side="left", padx=4)
        self.mode_text = tk.StringVar()
        ttk.Label(top, textvariable=self.mode_text, width=10).pack(side="left", padx=6)
        self.volt_text = tk.StringVar(value="ไฟ —")
        self.volt_label = tk.Label(top, textvariable=self.volt_text, width=11)
        self.volt_label.pack(side="left")
        ttk.Button(top, text="ปลด torque", command=lambda: self.on_torque(False)).pack(side="left", padx=(12, 2))
        ttk.Button(top, text="ล็อก torque", command=lambda: self.on_torque(True)).pack(side="left")

        body = ttk.Frame(self.root, padding=6)
        body.pack(fill="both", expand=True)
        left = ttk.Frame(body)
        left.pack(side="left", fill="y")
        tabs = ttk.Notebook(left)
        tabs.pack(fill="x")
        tabs.add(self._build_jog(tabs), text="Jog (กดค้าง)")
        tabs.add(self._build_goal(tabs), text="ใส่เป้า")
        tabs.add(self._build_slider(tabs), text="Slider ตามสด")
        self._build_speed(left)
        self.read_text = tk.StringVar()
        ttk.Label(left, textvariable=self.read_text, font=("Consolas", 9), justify="left").pack(anchor="w", pady=6)
        self.status = tk.StringVar()
        ttk.Label(left, textvariable=self.status, foreground="#185FA5", wraplength=380).pack(anchor="w")

        fig = Figure(figsize=(6, 3.2), dpi=80)
        self.view = SimView(fig)
        self.canvas = FigureCanvasTkAgg(fig, master=body)
        self.canvas.get_tk_widget().pack(side="left", fill="both", expand=True)

    def _hold_button(self, parent, text, press):
        """ปุ่มกดค้าง: กด = press(), ปล่อย = release()"""
        b = ttk.Button(parent, text=text, width=6)
        b.bind("<ButtonPress-1>", lambda _e: self.on_jog(press))
        b.bind("<ButtonRelease-1>", lambda _e: self.on_release())
        return b

    def _build_jog(self, parent):
        f = ttk.Frame(parent, padding=8)
        ttk.Label(f, text="ข้อต่อ (+ = ก้มหน้า / ฐานหันซ้าย)").grid(row=0, column=0, columnspan=3, sticky="w")
        self.jog_buttons = {}
        for r, i in enumerate(USED, start=1):
            ttk.Label(f, text=config.JOINTS[i]["name"], width=8).grid(row=r, column=0, sticky="w")
            for c, d in ((1, -1), (2, +1)):
                b = self._hold_button(f, "−" if d < 0 else "+", lambda m, i=i, d=d: m.jog_joint(i, d))
                b.grid(row=r, column=c, padx=2, pady=1)
                self.jog_buttons[(i, d)] = b
        ttk.Label(f, text="ปลายแขน (φ คงที่)").grid(row=len(USED) + 1, column=0, columnspan=3, sticky="w", pady=(8, 0))
        for k, name in enumerate(AXES):
            r = len(USED) + 2 + k
            ttk.Label(f, text=name).grid(row=r, column=0, sticky="w")
            for c, d in ((1, -1), (2, +1)):
                b = self._hold_button(f, f"{name}{'−' if d < 0 else '+'}", lambda m, k=k, d=d: m.jog_xyz(k, d))
                b.grid(row=r, column=c, padx=2, pady=1)
                self.jog_buttons[(name, d)] = b
        return f

    def _build_goal(self, parent):
        f = ttk.Frame(parent, padding=8)
        ttk.Label(f, text="มุมข้อต่อ (องศา)").grid(row=0, column=0, columnspan=4, sticky="w")
        self.q_entries = {}
        for c, i in enumerate(USED):
            ttk.Label(f, text=config.JOINTS[i]["name"]).grid(row=1, column=c)
            var = tk.StringVar(value="0")
            ttk.Entry(f, textvariable=var, width=7).grid(row=2, column=c, padx=2)
            self.q_entries[i] = var
        ttk.Button(f, text="ไปมุมนี้", command=self.on_goto_q).grid(row=3, column=0, columnspan=2, pady=4, sticky="w")
        ttk.Button(f, text="ตั้งตรง (q = 0)", command=lambda: self.on_goto_q([0.0] * 5)).grid(row=3, column=2, columnspan=2)
        ttk.Label(f, text="ปลายแขน X Y Z (mm), φ (องศา) — วิ่งเป็นเส้นตรง").grid(row=4, column=0, columnspan=4, sticky="w", pady=(8, 0))
        self.xyz_entries = {}
        for c, name in enumerate(["X", "Y", "Z", "φ"]):
            ttk.Label(f, text=name).grid(row=5, column=c)
            var = tk.StringVar(value="0")
            ttk.Entry(f, textvariable=var, width=7).grid(row=6, column=c, padx=2)
            self.xyz_entries[name] = var
        ttk.Button(f, text="ไปจุดนี้", command=self.on_goto_xyz).grid(row=7, column=0, columnspan=2, pady=4, sticky="w")
        ttk.Button(f, text="ใส่ค่าท่าตอนนี้", command=self.fill_from_current).grid(row=7, column=2, columnspan=2)
        return f

    def _build_slider(self, parent):
        f = ttk.Frame(parent, padding=8)
        ttk.Label(f, text="เลื่อนแล้วแขนตามทันที (ไม่เกินความเร็วที่ตั้ง)").grid(row=0, column=0, columnspan=2, sticky="w")
        self.slider_vars, self.sliders = {}, {}
        for r, i in enumerate(USED, start=1):
            ttk.Label(f, text=config.JOINTS[i]["name"], width=8).grid(row=r, column=0, sticky="w")
            var = tk.DoubleVar(value=0.0)
            s = tk.Scale(f, variable=var, from_=config.Q_MIN[i], to=config.Q_MAX[i], resolution=0.5,
                         orient="horizontal", length=260)
            s.grid(row=r, column=1)
            s.bind("<B1-Motion>", lambda _e: self.on_slider())          # เฉพาะผู้ใช้ลาก ไม่ใช่ตอนโปรแกรมตั้งค่า
            s.bind("<ButtonRelease-1>", lambda _e: self.on_slider())
            self.slider_vars[i], self.sliders[i] = var, s
        self._dragging = False
        return f

    def _build_speed(self, parent):
        f = ttk.LabelFrame(parent, text="ความเร็ว", padding=6)
        f.pack(fill="x", pady=6)
        self.v_joint = tk.DoubleVar(value=config.V_JOINT)
        self.v_cart = tk.DoubleVar(value=config.V_CART)
        ttk.Label(f, text="ข้อต่อ °/s").grid(row=0, column=0, sticky="w")
        tk.Scale(f, variable=self.v_joint, from_=1, to=config.V_MAX_LIMIT, orient="horizontal", length=200,
                 command=lambda _v: self.on_speed()).grid(row=0, column=1)
        ttk.Label(f, text="ปลายแขน mm/s").grid(row=1, column=0, sticky="w")
        tk.Scale(f, variable=self.v_cart, from_=2, to=60, orient="horizontal", length=200,
                 command=lambda _v: self.on_speed()).grid(row=1, column=1)
        self.corr_on = tk.BooleanVar(value=True)
        ttk.Checkbutton(f, text="ชดเชยแขนยวบหลังถึงเป้า", variable=self.corr_on,
                        command=self.on_speed).grid(row=2, column=0, columnspan=2, sticky="w")

    # ---------- เชื่อมต่อ ----------
    def _use_link(self, link):
        if self.driver:
            self.driver.close()
        if self.link:
            self.link.close()
        self.link = link
        q0 = link.q_actual if link.q_actual is not None else [0.0] * 5
        self.driver = Driver(link, Mover(q0, v=self.v_joint.get(), v_cart=self.v_cart.get()))
        self.driver.correct = self.corr_on.get()
        self.driver.mover.reset(q0)
        self.driver.start()
        self.torque_on = True
        self.mode_text.set("จำลอง" if link.simulated else "แขนจริง")
        self.status.set("โหมดจำลอง (ไม่ต่อแขน)" if link.simulated else "ต่อแขนแล้ว — แขนค้างที่เดิม")
        self.fill_from_current()

    def on_connect(self):
        try:
            link = self.connect_fn()
        except (serial.SerialException, OSError) as e:
            self.status.set(f"ต่อแขนไม่ได้ (เสียบ adapter / เปิดไฟ / ปิดโปรแกรมอื่นที่ใช้ COM11?): {e}")
            return
        for _ in range(40):                               # รอค่าแรกจาก servo (≤ 2 s)
            if link.q_actual is not None:
                break
            self.root.after(50)
        if link.q_actual is None:
            link.close()
            self.status.set("ต่อพอร์ตได้แต่ servo ไม่ตอบ — เปิดไฟ servo / จัมเปอร์ B ?")
            return
        self._use_link(link)

    # ---------- ปุ่ม ----------
    def _can_move(self):
        if not self.torque_on:
            self.status.set("torque ปลดอยู่ — กด \"ล็อก torque\" ก่อน")
            return False
        return True

    def on_jog(self, press):
        if self._can_move():
            self.driver.do(press)

    def on_release(self):
        self.driver.do(lambda m: m.release())

    def on_goto_q(self, q=None):
        if not self._can_move():
            return
        try:
            if q is None:
                q = [float(self.q_entries[i].get()) if i in self.q_entries else 0.0 for i in range(5)]
        except ValueError:
            self.status.set("กรอกมุมเป็นตัวเลข")
            return
        self.driver.do(lambda m: m.goto_q(q))
        self.status.set("กำลังไปมุมเป้า")

    def on_goto_xyz(self):
        if not self._can_move():
            return
        try:
            x, y, z, phi = (float(self.xyz_entries[k].get()) for k in ("X", "Y", "Z", "φ"))
        except ValueError:
            self.status.set("กรอก X Y Z φ เป็นตัวเลข")
            return
        msg = self.driver.do(lambda m: m.goto_xyz(x, y, z, phi))
        self.status.set(msg or self.driver.mover.message or "กำลังวิ่งเส้นตรงไปจุดเป้า")

    def on_slider(self):
        if not self._can_move():
            return
        q = [self.slider_vars[i].get() if i in self.slider_vars else 0.0 for i in range(5)]
        self.driver.do(lambda m: m.follow(q))

    def on_speed(self):
        v, vc = self.v_joint.get(), self.v_cart.get()

        def apply(m):
            m.v, m.v_cart = v, vc
        self.driver.do(apply)
        self.driver.correct = self.corr_on.get()

    def on_stop(self):
        self.driver.emergency_stop()
        self.status.set("หยุดแล้ว — แขนค้างที่ตำแหน่งจริง")

    def on_torque(self, on):
        self.driver.emergency_stop()
        self.link.torque(on)
        self.torque_on = on
        if on:
            self.root.after(300, lambda: self.driver.do(lambda m: m.reset(self.link.q_actual or m.q)))
            self.status.set("ล็อกแล้ว — แขนค้างท่านี้")
        else:
            self.status.set("ปลด torque แล้ว — แขนหมดแรง ประคองไว้ (ขยับด้วยมือได้)")

    def fill_from_current(self):
        q = self.driver.mover.q
        for i, var in self.q_entries.items():
            var.set(f"{q[i]:.1f}")
        f = forward_kinematics(q)
        for k, v in zip(("X", "Y", "Z", "φ"), (f.x, f.y, f.z, f.phi)):
            self.xyz_entries[k].set(f"{v:.1f}")

    # ---------- อัปเดตจอทุก 100 ms ----------
    def tick(self):
        m, link = self.driver.mover, self.link
        q_act = link.q_actual
        if self.driver.message:
            self.status.set(self.driver.message)
            self.driver.message = ""
        if m.mode != "follow":                            # slider แสดงท่าที่สั่ง (ยกเว้นตอนผู้ใช้กำลังลาก)
            for i, var in self.slider_vars.items():
                var.set(round(m.q[i] * 2) / 2)
        v = link.volt
        if link.simulated:
            self.volt_text.set("ไฟ — (จำลอง)")
            self.volt_label.config(foreground="gray")
        else:
            self.volt_text.set("ไฟ ?" if v is None else f"ไฟ {v:.1f} V")
            self.volt_label.config(foreground="green" if v is not None and v >= config.VOLT_MIN else "red")
        f = forward_kinematics(m.q)
        names = "  ".join(f"{config.JOINTS[i]['name'][3:]:>6}" for i in USED)
        cmd = "  ".join(f"{m.q[i]:6.1f}" for i in USED)
        act = "  ".join("     —" if q_act is None else f"{q_act[i]:6.1f}" for i in USED)
        self.read_text.set(f"          {names}\nสั่ง     {cmd}\nจริง     {act}\n"
                           f"ปลายแขน X {f.x:.1f}  Y {f.y:.1f}  Z {f.z:.1f} mm  φ {f.phi:.1f}°")
        self.view.draw(m.q, None if link.simulated else q_act)
        self.root.after(100, self.tick)

    def close(self):
        self.driver.close()
        self.link.close()


def main():
    root = tk.Tk()
    app = App(root)
    root.protocol("WM_DELETE_WINDOW", lambda: (app.close(), root.destroy()))
    root.mainloop()


if __name__ == "__main__":
    main()
