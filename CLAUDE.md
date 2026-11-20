# RobotArmSmooth

ผู้ใช้สื่อสารภาษาไทย · ภาพรวม `README.md` · design `docs/design.md` · โปรเจคพี่: `D:\PIM\RobotControl` (v2)

## กฎความปลอดภัย (แขนจริง)
- **ถามผู้ใช้ก่อนสั่งแขนขยับทุกครั้ง** ให้ผู้ใช้ประคองแขน · ครั้งแรกใช้ความเร็ว ≤ 5 °/s ระยะสั้น
- เช็คไฟ ≥ 6 V และ servo ตอบครบ 4 ตัวก่อน-หลังขยับ · ท่าแขนยื่นหน้าไกล ไฟตก/ไหล่ยกไม่ขึ้น (แหล่งจ่ายกระแสไม่พอ)
- COM11 เปิดได้ทีละโปรแกรม · ปลด torque = แขนหมดแรงทันที
- ผู้ใช้ชอบให้ผู้ช่วยรันคำสั่งทีละขั้นแล้วรายงานผล มากกว่าให้กด GUI เอง

## โค้ด
- ค่าคาลิเบรตอยู่ `src/config.py` ที่เดียว · แขน 4 ข้อ (J5 เอาออกจากโปรแกรมแล้ว 2026-11-20) · DH 4 แถว: a4 = 50 mm, θ₄ = −q₄, φ = θ2+θ3+θ4
- `qt_app.py` (PyQt6 แบบ ws-control) + `viewer3d.py` (PyVista) + `ik_explain.py`/`ik_sketch.py` (แท็บ IK บอกเหตุผลที่ไปไม่ได้ + รูป) + `auto_cycle.py` (แท็บ Auto: ลำดับงาน A → B → … (≤ 8 จุด) วนลูปแบบเครื่องจักร, เวลารอแยกแต่ละจุด (`dwells`), ปุ่ม ✨ ท่าตัวอย่าง A–G (`auto_cycle.DEMO` หยิบซ้าย→วางขวา), หมุนข้อต่อ หรือเส้นตรง `Mover.goto_line_q` (ไปจุดแรกหมุนข้อต่อเสมอ), จุดแสดงในภาพ 3D `Viewer3D.show_points` — state machine, จุดเก็บใน `auto_points.json` ไม่อยู่ใน git, หยุดเองเมื่อผู้ใช้สั่งแขน/แขนหยุดก่อนถึงจุด) + `theme.py` (ธีมเข้มเรียบ ๆ เป็นค่าเริ่มต้น, ธีมสว่าง: env `ROBOTARM_THEME=light` — ปุ่มใช้ property `kind`, ป้ายใช้ `role`, การ์ด = QFrame "card"; แท็บเลื่อนได้ + หน้าต่างย่อตามจอ กันปุ่มทับกันบนจอโน้ตบุ๊ก) → `motion.py` (Mover ไม่แตะฮาร์ดแวร์) → `driver.py` (thread 40 ms) → `link.py` (DirectLink / SimLink)
- Windows Region เป็นไทย → `qt_app.main()` บังคับ QLocale en-US ไม่งั้นช่องตัวเลขเป็นเลขไทย
- test: `cd src && ../.venv/Scripts/python.exe -m pytest -q`
- Windows: รอบ thread 40 ms แกว่งจริง 27–52 ms → driver ส่ง "ท่าล่วงหน้า STREAM_MS (80 ms) + เวลา 80 ms" (`Mover.peek`) ไม่งั้น jog กระตุก (วัดแขนจริง 2026-11-19: ความเร็วศอกแกว่ง ±10.8 → ±5.8 °/s, ตามหลัง 4.4° → 2.5°)
- GitHub: `BananaZz27/RobotArmV2` (private) branch `main` · เอกสารอธิบายสูตร: `docs/explain.md` (กราฟสร้างจาก `docs/make_charts.py`, รูป IK ทีละขั้นจาก `docs/make_ik_derivation.py`)
- **English version:** `BananaZz27/RobotArmV2-EN` (local `D:\PIM\RobotArmV2-EN`, translated at f1cef72) — port code changes there too
