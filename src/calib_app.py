# หน้าต่างคาลิเบรตแขน (ผ่าน USB adapter COM11) — หา center / ทิศ / limit ทีละข้อด้วยมือ แล้วบันทึกลง config.py
#   Calibrate.bat  หรือ  cd src && ../.venv/Scripts/python.exe calib_app.py
# ไม่เขียนหน่วยความจำถาวร (EEPROM) ของ servo — สั่งแค่ torque เปิด/ปิด และ goal = ตำแหน่งตอนนี้ (ตอนล็อก)
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

import serial

import config
from scs import ScsBus
from servo_calibrate import save_calibration, suggest_limits

DIR_MIN_TICK = 20          # ต้องขยับอย่างน้อยเท่านี้ (≈5°) ถึงจะบอกทิศได้
STEPS = ("ขั้นตอน (ประคองแขนตลอด — ข้อที่ปลด torque หมดแรงทันที):\n"
         "1) ปลด torque → จับท่อนตั้งตรงชี้ฟ้า (ฐาน: หันไปด้านหน้า) → กด \"ใช้ค่านี้\" = center\n"
         "2) ขยับไปทางบวก ≥ 5° (ไหล่/ศอก/คอ ก้มไปข้างหน้า · ฐานหันซ้าย) → กด \"หาทิศ\"\n"
         "3) กด \"เริ่มจด\" → ขยับไปสุดสองข้างเท่าที่ไม่ชน → \"หยุดจด\" (limit หดเข้า 10 tick)\n"
         "4) ล็อก torque → บันทึกลง config.py → ปิดหน้าต่างนี้แล้วเปิด RobotArm ใหม่")


class BusReader:
    """thread อ่าน raw ทุกข้อที่ต่ออยู่ + สั่ง torque/ล็อก ผ่านบัสเดียวกัน (ล็อกบัส)"""

    def __init__(self, bus, joints, period_s=0.05):
        self.bus, self.joints, self.period_s = bus, joints, period_s
        self.lock = threading.Lock()
        self.raw = {j["id"]: None for j in joints}
        self.torque = {j["id"]: None for j in joints}
        self.volt = None
        self.error = ""
        self._running = True
        with self.lock:
            bus.detect_echo()
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        while self._running:
            try:
                with self.lock:
                    for j in self.joints:
                        got = self.bus.read_pos_volt(j["id"])
                        self.raw[j["id"]] = got[0] if got else None
                        if got:
                            self.volt = got[1]
                        t = self.bus.read(j["id"], 40, 1)
                        self.torque[j["id"]] = bool(t[0]) if t else None
                self.error = ""
            except OSError as e:
                self.error = f"อ่านไม่ได้ (สายหลุด?): {e}"
            time.sleep(self.period_s)

    def set_torque(self, sid, on):
        try:
            with self.lock:
                if on:                                    # ล็อก: เป้า = ที่อยู่ตอนนี้ → ไม่ขยับ
                    got = self.bus.read_pos_volt(sid)
                    if got:
                        self.bus.write_pos(sid, got[0], config.MIN_MOVE_MS)
                self.bus.torque(sid, on)
        except OSError as e:
            self.error = f"สั่งไม่ได้: {e}"

    def close(self):
        self._running = False
        self.thread.join(timeout=1.0)


class CalibApp:
    def __init__(self, root, reader, save=save_calibration):
        self.root, self.reader, self.save = root, reader, save
        self.joints = reader.joints
        self.rec = {}                                    # id → [ต่ำสุด, สูงสุด] ขณะจด limit
        root.title("คาลิเบรตแขน v2 (USB adapter)")
        frame = ttk.Frame(root, padding=10)
        frame.pack(fill="both", expand=True)
        ttk.Label(frame, text=STEPS, justify="left").grid(row=0, column=0, columnspan=12, sticky="w", pady=(0, 8))
        heads = ["ข้อ", "raw สด", "torque", "", "", "center", "", "dir", "", "limit ต่ำ", "สูง", ""]
        for c, h in enumerate(heads):
            ttk.Label(frame, text=h, font=("", 9, "bold")).grid(row=1, column=c, padx=3)
        self.rows = {}
        for r, j in enumerate(self.joints, start=2):
            sid = j["id"]
            w = {
                "raw": tk.StringVar(value="—"), "torque": tk.StringVar(value="—"),
                "center": tk.StringVar(value=str(j["center"])), "dir": tk.StringVar(value=f"{j['dir']:+d}"),
                "min": tk.StringVar(value=str(j["raw_min"])), "max": tk.StringVar(value=str(j["raw_max"])),
            }
            ttk.Label(frame, text=f"{j['name']} (ID {sid})").grid(row=r, column=0, sticky="w")
            ttk.Label(frame, textvariable=w["raw"], width=6).grid(row=r, column=1)
            w["torque_label"] = tk.Label(frame, textvariable=w["torque"], width=6)
            w["torque_label"].grid(row=r, column=2)
            ttk.Button(frame, text="ปลด", width=5, command=lambda s=sid: self.on_torque(s, False)).grid(row=r, column=3)
            ttk.Button(frame, text="ล็อก", width=5, command=lambda s=sid: self.on_torque(s, True)).grid(row=r, column=4)
            ttk.Entry(frame, textvariable=w["center"], width=6).grid(row=r, column=5)
            ttk.Button(frame, text="ใช้ค่านี้", command=lambda s=sid: self.on_center(s)).grid(row=r, column=6)
            ttk.Combobox(frame, textvariable=w["dir"], values=["+1", "-1"], width=3, state="readonly").grid(row=r, column=7)
            ttk.Button(frame, text="หาทิศ", command=lambda s=sid: self.on_dir(s)).grid(row=r, column=8)
            ttk.Entry(frame, textvariable=w["min"], width=6).grid(row=r, column=9)
            ttk.Entry(frame, textvariable=w["max"], width=6).grid(row=r, column=10)
            w["rec_btn"] = ttk.Button(frame, text="เริ่มจด", width=8, command=lambda s=sid: self.on_record(s))
            w["rec_btn"].grid(row=r, column=11)
            self.rows[sid] = w
        bottom = ttk.Frame(frame)
        bottom.grid(row=len(self.joints) + 2, column=0, columnspan=12, sticky="we", pady=(10, 0))
        self.volt_text = tk.StringVar(value="ไฟ —")
        self.volt_label = tk.Label(bottom, textvariable=self.volt_text, width=12)
        self.volt_label.pack(side="left")
        ttk.Button(bottom, text="ปลดทุกข้อ", command=lambda: self.on_all(False)).pack(side="left", padx=4)
        ttk.Button(bottom, text="ล็อกทุกข้อ", command=lambda: self.on_all(True)).pack(side="left", padx=4)
        ttk.Button(bottom, text="บันทึกลง config", command=self.on_save).pack(side="right")
        self.status = tk.StringVar(value="")
        ttk.Label(frame, textvariable=self.status, foreground="#185FA5").grid(
            row=len(self.joints) + 3, column=0, columnspan=12, sticky="w", pady=(6, 0))
        self.tick()

    # ---------- ปุ่ม ----------
    def on_torque(self, sid, on):
        self.reader.set_torque(sid, on)
        self.status.set(f"ID {sid}: " + ("ล็อก (ค้างที่เดิม)" if on else "ปลด torque — ประคองไว้"))

    def on_all(self, on):
        for j in self.joints:
            self.reader.set_torque(j["id"], on)
        self.status.set("ล็อกทุกข้อ (ค้างที่เดิม)" if on else "ปลดทุกข้อ — แขนหมดแรง ประคองไว้")

    def on_center(self, sid):
        raw = self.reader.raw[sid]
        if raw is None:
            self.status.set(f"ID {sid} อ่านไม่ได้")
            return
        self.rows[sid]["center"].set(str(raw))
        self.status.set(f"ID {sid}: center = {raw}")

    def on_dir(self, sid):
        raw = self.reader.raw[sid]
        try:
            center = int(self.rows[sid]["center"].get())
        except ValueError:
            self.status.set("ตั้ง center ก่อน")
            return
        if raw is None or abs(raw - center) < DIR_MIN_TICK:
            self.status.set(f"ID {sid}: ขยับไปทางบวกจาก center ให้เกิน ~5° ก่อนกด")
            return
        d = 1 if raw > center else -1
        self.rows[sid]["dir"].set(f"{d:+d}")
        self.status.set(f"ID {sid}: raw {center} → {raw} ⇒ dir {d:+d}")

    def on_record(self, sid):
        w = self.rows[sid]
        if sid in self.rec:                              # หยุดจด
            lo, hi = self.rec.pop(sid)
            w["rec_btn"].config(text="เริ่มจด")
            if lo is None:
                self.status.set(f"ID {sid}: ไม่ได้ค่า")
                return
            lo, hi = suggest_limits(lo, hi)
            w["min"].set(str(lo))
            w["max"].set(str(hi))
            self.status.set(f"ID {sid}: limit {lo}–{hi} (หดเข้า 10 tick แล้ว)")
        else:
            self.rec[sid] = [None, None]
            w["rec_btn"].config(text="หยุดจด")
            self.status.set(f"ID {sid}: กำลังจด — ขยับไปสุดสองข้างช้า ๆ แล้วกด \"หยุดจด\"")

    def values(self):
        """ค่าที่จะบันทึก {id: (dir, center, min, max)} (ValueError ถ้ากรอกไม่ใช่ตัวเลข)"""
        out = {}
        for sid, w in self.rows.items():
            out[sid] = (int(w["dir"].get()), int(w["center"].get()), int(w["min"].get()), int(w["max"].get()))
        return out

    def on_save(self):
        try:
            vals = self.values()
            text = "\n".join(f"ID {s}: dir {d:+d}, center {c}, limit {lo}–{hi}" for s, (d, c, lo, hi) in vals.items())
            if not messagebox.askyesno("บันทึกลง config", text + "\n\nเขียนลง config.py ?", parent=self.root):
                return
            self.save(vals)
        except ValueError as e:
            self.status.set(f"ไม่บันทึก: {e}")
            return
        self.status.set("บันทึกแล้ว — ปิดหน้าต่างนี้แล้วเปิด RobotArm ใหม่")

    # ---------- อัปเดตเป็นระยะ ----------
    def tick(self):
        for sid, w in self.rows.items():
            raw = self.reader.raw[sid]
            w["raw"].set("—" if raw is None else str(raw))
            t = self.reader.torque[sid]
            w["torque"].set({True: "ล็อก", False: "ปลด", None: "—"}[t])
            w["torque_label"].config(foreground={True: "green", False: "#A32D2D", None: "gray"}[t])
            if sid in self.rec and raw is not None:
                lo, hi = self.rec[sid]
                self.rec[sid] = [raw if lo is None else min(lo, raw), raw if hi is None else max(hi, raw)]
                w["min"].set(str(self.rec[sid][0]))
                w["max"].set(str(self.rec[sid][1]))
        v = self.reader.volt
        self.volt_text.set("ไฟ —" if v is None else f"ไฟ {v:.1f} V")
        self.volt_label.config(foreground="green" if v is not None and v >= config.VOLT_MIN else "red")
        if self.reader.error:
            self.status.set(self.reader.error)
        self.root.after(100, self.tick)


def main():
    try:
        ser = serial.Serial(config.DIRECT_PORT, config.SERVO_BAUD, timeout=0.002)
    except serial.SerialException as e:
        messagebox.showerror("เปิดพอร์ตไม่ได้", f"{config.DIRECT_PORT}: {e}\nปิดโปรแกรมอื่นที่ใช้พอร์ตอยู่หรือยัง?")
        return
    reader = BusReader(ScsBus(ser), [j for j in config.JOINTS if j["used"]])
    root = tk.Tk()
    CalibApp(root, reader)

    def close():
        reader.close()
        ser.close()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", close)
    root.mainloop()


if __name__ == "__main__":
    main()
