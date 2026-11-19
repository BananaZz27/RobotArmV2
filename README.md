# RobotArmSmooth — ควบคุมแขน 4 ข้อแบบนุ่ม

แขน SCS15 × 4 (ฐาน 4, ไหล่ 9, ศอก 7, คอ 2) ผ่าน Waveshare Bus Servo Adapter (A) จัมเปอร์ B → **COM11** · ไฟ servo **7.4 V เท่านั้น**
ต่อยอดจาก `D:\PIM\RobotControl\v2` · design: `docs/design.md`

หน้าตาแบบ RobotControl v2 (slider + ส่ง, FK + เมทริกซ์ T⁰₅, IK ดูก่อน/ส่ง, ภาพ 2D สองมุม) แต่ **กดส่งแล้วแขนวิ่งนุ่ม**:
ออกตัวช้า ผ่อนก่อนถึง ทุกข้อถึงพร้อมกัน (PC ป้อนท่าทุก 40 ms — `motion.py` → `driver.py`)

- **เปิด:** ดับเบิลคลิก `RobotArm.bat` → โหมดจำลอง → ช่องการเชื่อมต่อเลือก **"แขนตรง (USB adapter)"** (แขนค้างที่เดิม, slider ตั้งเป็นท่าจริง)
- เลื่อน slider = ดูก่อน (ภาพ + FK) → **ส่งไปแขน** · IK: ใส่ X Y Z φ → ดูก่อน / ส่ง · ความเร็ว °/s ปรับได้ 2–30
- **หยุด:** แขนค้างที่ตำแหน่งจริงทันที · ไฟ < 6 V / servo ไม่ตอบ → ไม่ส่ง/หยุดเอง · ชดเชยแขนยวบหลังถึงเป้า (ปิดได้)
- **คาลิเบรต:** `Calibrate.bat` → บันทึกลง `src/config.py` (ค่าอยู่ที่เดียว) แล้วเปิด RobotArm ใหม่
- `motion.py` ยังมี jog / follow / เส้นตรง XYZ (มี test) แต่ไม่ได้ใส่ปุ่มในหน้าต่าง

ติดตั้ง: `py -3.11 -m venv .venv && .venv\Scripts\python.exe -m pip install -r requirements.txt`
test: `cd src && ..\.venv\Scripts\python.exe -m pytest -q`
