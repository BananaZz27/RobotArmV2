# RobotArmSmooth

ผู้ใช้สื่อสารภาษาไทย · ภาพรวม `README.md` · design `docs/design.md` · โปรเจคพี่: `D:\PIM\RobotControl` (v2)

## กฎความปลอดภัย (แขนจริง)
- **ถามผู้ใช้ก่อนสั่งแขนขยับทุกครั้ง** ให้ผู้ใช้ประคองแขน · ครั้งแรกใช้ความเร็ว ≤ 5 °/s ระยะสั้น
- เช็คไฟ ≥ 6 V และ servo ตอบครบ 4 ตัวก่อน-หลังขยับ · ท่าแขนยื่นหน้าไกล ไฟตก/ไหล่ยกไม่ขึ้น (แหล่งจ่ายกระแสไม่พอ)
- COM11 เปิดได้ทีละโปรแกรม · ปลด torque = แขนหมดแรงทันที
- ผู้ใช้ชอบให้ผู้ช่วยรันคำสั่งทีละขั้นแล้วรายงานผล มากกว่าให้กด GUI เอง

## โค้ด
- ค่าคาลิเบรตอยู่ `src/config.py` ที่เดียว (J5 หัวถอดแล้ว `used=False`, d5 = 50 mm)
- `qt_app.py` (PyQt6 แบบ ws-control) + `viewer3d.py` (PyVista) → `motion.py` (Mover ไม่แตะฮาร์ดแวร์) → `driver.py` (thread 40 ms) → `link.py` (DirectLink / SimLink)
- Windows Region เป็นไทย → `qt_app.main()` บังคับ QLocale en-US ไม่งั้นช่องตัวเลขเป็นเลขไทย
- test: `cd src && ../.venv/Scripts/python.exe -m pytest -q`
- Windows: รอบ thread 40 ms แกว่งจริง 27–52 ms → driver ส่ง "ท่าล่วงหน้า STREAM_MS (80 ms) + เวลา 80 ms" (`Mover.peek`) ไม่งั้น jog กระตุก (วัดแขนจริง 2026-11-19: ความเร็วศอกแกว่ง ±10.8 → ±5.8 °/s, ตามหลัง 4.4° → 2.5°)
- GitHub: `BananaZz27/RobotArmV2` (private) branch `main` · เอกสารอธิบายสูตร: `docs/explain.md` (กราฟสร้างจาก `docs/make_charts.py`)
- **English version:** `BananaZz27/RobotArmV2-EN` (local `D:\PIM\RobotArmV2-EN`, translated at f1cef72) — port code changes there too
