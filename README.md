# RobotArmSmooth — ควบคุมแขน 4 ข้อแบบนุ่ม

แขน SCS15 × 4 (ฐาน 4, ไหล่ 9, ศอก 7, คอ 2) ผ่าน Waveshare Bus Servo Adapter (A) จัมเปอร์ B → **COM11** · ไฟ servo **7.4 V เท่านั้น**
ต่อยอดจาก `D:\PIM\RobotControl\v2` · design: `docs/design.md`

หน้าตาแบบ ws-control (PyQt6 + ภาพ 3D PyVista) แต่ **ไม่ใช้ ESP32** (PC คุม servo ตรง) และ **กดส่งแล้วแขนวิ่งนุ่ม**

```
[Port COM11][Connect] ● Connected 7.8 V                     [Torque OFF] [■ STOP (Space)]
┌────────── 3D (ส้ม = Target, ฟ้าโปร่ง = จริง) ──────────┐ ┌ Joint | Inverse Kinematics | Status ┐
└───────────────────────────────────────────────────────┘ └──────────────────────────────────────┘
Forward Kinematics (ท่า Target): X Y Z φ Yaw + T⁰₅            Log
```

- **เปิด:** `RobotArm.bat` → โหมด Simulation → **Connect** (แขนค้างที่เดิม, Target = ท่าจริง)
- **Joint:** ลาก slider / พิมพ์ตัวเลข → แขนตามทันที (ไม่มีลิมิตความเร็วให้ตั้ง — วิ่งเต็มกำลัง servo มีช่วงเร่ง/ผ่อนสั้น ๆ) · Zero = กลับท่าตั้งตรง
- **Jog:** กดค้าง = เคลื่อนต่อเนื่อง, ปล่อย = หยุดนุ่ม · ต่อข้อ (J1–J4 ±) หรือปลายแขนเป็นเส้นตรง (X Y Z ±, φ คงที่) — jog ปลายแขนหยุดเองก่อนเข้า singularity
- **Singularity (กล่องขวาของ FK):** แดง = ท่า Target ใกล้ singularity · พิสูจน์ด้วย |det J| คำนวณ 2 ทางต้องเท่ากัน
  (Jacobian 4×4 เชิงตัวเลข vs สูตร **r·a2·a3·|sin θ3|** → = 0 เมื่อปลายแขนอยู่บนแกนฐาน หรือแขนเหยียด/พับสุด) + σ_min · test ตรวจ 300 ท่าสุ่ม (`singularity.py`)
- **Inverse Kinematics:** X Y Z φ → Preview (ดูคำตอบ elbow up/down) / Go
- **Status:** ต่อข้อ: online, raw, มุมจริง, error, อุณหภูมิ, โหลด · ไฟเลี้ยง · ติ๊กชดเชยแขนยวบ
- **STOP / Space:** แขนค้างที่ตำแหน่งจริงทันที · **Torque OFF:** แขนหมดแรง จับขยับด้วยมือได้ (Target ตามแขนจริง)
- ไฟ < 6 V / servo ไม่ตอบ → ไม่ส่ง/หยุดเอง · **คาลิเบรต:** `Calibrate.bat` → `src/config.py`

โค้ด: `qt_app.py` (หน้าต่าง) · `viewer3d.py` (3D) · `motion.py` (ตัวป้อนท่า) → `driver.py` (40 ms) → `link.py` (adapter / จำลอง)

ติดตั้ง: `py -3.11 -m venv .venv && .venv\Scripts\python.exe -m pip install -r requirements.txt`
test: `cd src && ..\.venv\Scripts\python.exe -m pytest -q`
