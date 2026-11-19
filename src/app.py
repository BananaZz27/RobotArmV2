# หน้าต่างควบคุมแขนแบบง่าย: slider 4 ข้อ (แขนตามทันที แบบนุ่ม) · ใส่ X Y Z แล้วกดไป · ภาพ 3D หมุนดูได้
#   RobotArm.bat  หรือ  cd src && ../.venv/Scripts/python.exe app.py
# เปิดมาเป็นโหมดจำลองเสมอ → กด "เชื่อมต่อแขน" (แขนค้างที่เดิม ไม่ขยับเอง)
import tkinter as tk
from tkinter import ttk

import serial
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

import config
from driver import Driver
from kinematics import forward_kinematics, inverse_kinematics
from link import DirectLink, SimLink
from motion import Mover
from sim3d import Sim3D

USED = [i for i, j in enumerate(config.JOINTS) if j["used"]]


def pick_phi(q_now, x, y, z):
    """φ ที่ใกล้ท่าตอนนี้ที่สุดที่ IK ไปถึง (ผู้ใช้ไม่ต้องกรอก φ) — None = ไปไม่ถึงเลย"""
    phi_now = forward_kinematics(q_now).phi
    for phi in sorted(range(-90, 91, 5), key=lambda p: abs(p - phi_now)):
        if inverse_kinematics(x, y, z, phi, q_now[2] >= 0, theta1_prev=q_now[0]).ok:
            return phi
    return None


class App:
    def __init__(self, root, link=None, connect_fn=None):
        self.root = root
        self.connect_fn = connect_fn or (lambda: DirectLink(config.DIRECT_PORT))
        self.link, self.driver = None, None
        root.title("แขนกล 4 ข้อ")
        self._build()
        self._use_link(link or SimLink())
        self.tick()

    def _build(self):
        top = ttk.Frame(self.root, padding=6)
        top.pack(fill="x")
        tk.Button(top, text="■ หยุด", bg="#d33", fg="white", font=("", 12, "bold"), width=8,
                  command=self.on_stop).pack(side="left", padx=(0, 10))
        ttk.Button(top, text="เชื่อมต่อแขน", command=self.on_connect).pack(side="left")
        self.mode_text = tk.StringVar()
        ttk.Label(top, textvariable=self.mode_text).pack(side="left", padx=8)
        self.volt_text = tk.StringVar()
        self.volt_label = tk.Label(top, textvariable=self.volt_text)
        self.volt_label.pack(side="left")

        body = ttk.Frame(self.root, padding=6)
        body.pack(fill="both", expand=True)
        left = ttk.Frame(body)
        left.pack(side="left", fill="y", padx=(0, 8))

        self.slider_vars = {}
        for i in USED:
            ttk.Label(left, text=config.JOINTS[i]["name"][3:]).pack(anchor="w")
            var = tk.DoubleVar(value=0.0)
            s = tk.Scale(left, variable=var, from_=config.Q_MIN[i], to=config.Q_MAX[i], resolution=0.5,
                         orient="horizontal", length=280)
            s.pack(anchor="w")
            s.bind("<B1-Motion>", lambda _e: self.on_slider())          # เฉพาะตอนผู้ใช้ลาก
            s.bind("<ButtonRelease-1>", lambda _e: self.on_slider())
            self.slider_vars[i] = var

        xyz = ttk.LabelFrame(left, text="ไปจุด (mm)", padding=6)
        xyz.pack(fill="x", pady=8)
        self.xyz_vars = {}
        for c, name in enumerate("XYZ"):
            ttk.Label(xyz, text=name).grid(row=0, column=2 * c)
            var = tk.StringVar(value="0")
            ttk.Entry(xyz, textvariable=var, width=7).grid(row=0, column=2 * c + 1, padx=(2, 8))
            self.xyz_vars[name] = var
        ttk.Button(xyz, text="ไปจุดนี้", command=self.on_goto_xyz).grid(row=1, column=0, columnspan=6, pady=(6, 0))

        ttk.Label(left, text="ความเร็ว (°/s)").pack(anchor="w")
        self.speed = tk.DoubleVar(value=config.V_JOINT)
        tk.Scale(left, variable=self.speed, from_=2, to=config.V_MAX_LIMIT, orient="horizontal", length=280,
                 command=lambda _v: self.on_speed()).pack(anchor="w")
        self.read_text = tk.StringVar()
        ttk.Label(left, textvariable=self.read_text, font=("Consolas", 10)).pack(anchor="w", pady=6)
        self.status = tk.StringVar()
        ttk.Label(left, textvariable=self.status, foreground="#185FA5", wraplength=290).pack(anchor="w")

        fig = Figure(figsize=(5.5, 5), dpi=80)
        self.view = Sim3D(fig)
        FigureCanvasTkAgg(fig, master=body).get_tk_widget().pack(side="left", fill="both", expand=True)

    # ---------- เชื่อมต่อ ----------
    def _use_link(self, link):
        if self.driver:
            self.driver.close()
        if self.link:
            self.link.close()
        self.link = link
        q0 = link.q_actual if link.q_actual is not None else [0.0] * 5
        v = self.speed.get()
        self.driver = Driver(link, Mover(q0, v=v, v_cart=2 * v))
        self.driver.mover.reset(q0)
        self.driver.start()
        self.mode_text.set("โหมดจำลอง" if link.simulated else "แขนจริง")
        self.status.set("" if link.simulated else "ต่อแล้ว — แขนค้างที่เดิม")
        self._fill_xyz()

    def on_connect(self):
        try:
            link = self.connect_fn()
        except (serial.SerialException, OSError) as e:
            self.status.set(f"ต่อไม่ได้ (เสียบ adapter / เปิดไฟ / ปิดโปรแกรมอื่นที่ใช้ COM11?): {e}")
            return
        for _ in range(40):                               # รอค่าแรกจาก servo (≤ 2 s)
            if link.q_actual is not None:
                break
            self.root.after(50)
        if link.q_actual is None:
            link.close()
            self.status.set("servo ไม่ตอบ — เปิดไฟ servo / จัมเปอร์ B ?")
            return
        self._use_link(link)

    # ---------- ปุ่ม ----------
    def on_slider(self):
        q = [self.slider_vars[i].get() if i in self.slider_vars else 0.0 for i in range(5)]
        self.driver.do(lambda m: m.follow(q))

    def on_goto_xyz(self):
        try:
            x, y, z = (float(self.xyz_vars[k].get()) for k in "XYZ")
        except ValueError:
            self.status.set("กรอก X Y Z เป็นตัวเลข")
            return
        phi = pick_phi(self.driver.mover.q, x, y, z)
        if phi is None:
            self.status.set(f"ไปจุด ({x:.0f}, {y:.0f}, {z:.0f}) ไม่ได้ — เอื้อมไม่ถึงหรือเกิน limit")
            return
        self.status.set(self.driver.do(lambda m: m.goto_xyz(x, y, z, phi)) or "กำลังไป")

    def on_speed(self):
        v = self.speed.get()

        def apply(m):
            m.v, m.v_cart = v, 2 * v
        self.driver.do(apply)

    def on_stop(self):
        self.driver.emergency_stop()
        self.status.set("หยุดแล้ว")

    def _fill_xyz(self):
        f = forward_kinematics(self.driver.mover.q)
        for k, v in zip("XYZ", (f.x, f.y, f.z)):
            self.xyz_vars[k].set(f"{v:.0f}")

    # ---------- อัปเดตจอทุก 100 ms ----------
    def tick(self):
        m, link = self.driver.mover, self.link
        if self.driver.message:
            self.status.set(self.driver.message)
            self.driver.message = ""
        if m.mode != "follow":                            # ไม่ทับค่าตอนผู้ใช้กำลังลาก
            for i, var in self.slider_vars.items():
                var.set(round(m.q[i] * 2) / 2)
        v = link.volt
        if link.simulated:
            self.volt_text.set("")
        else:
            self.volt_text.set("ไฟ ?" if v is None else f"ไฟ {v:.1f} V")
            self.volt_label.config(foreground="green" if v is not None and v >= config.VOLT_MIN else "red")
        f = forward_kinematics(m.q)
        self.read_text.set(f"ปลายแขน X {f.x:.0f}  Y {f.y:.0f}  Z {f.z:.0f} mm")
        self.view.draw(m.q, None if link.simulated else link.q_actual)
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
