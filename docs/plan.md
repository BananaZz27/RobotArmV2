# RobotArmSmooth Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: superpowers:executing-plans (Native — ทำในแชทเดียว). Steps use `- [ ]`.

**Goal:** GUI ควบคุมแขน 4 ข้อแบบนุ่ม (jog / เป้า / slider) ผ่าน adapter COM11
**Architecture:** `Mover` (motion.py) คำนวณ setpoint ทุก 40 ms → `Driver` (driver.py) ส่งผ่าน `DirectLink` (link.py) → `app.py` tkinter
**Tech:** Python 3.11, numpy, matplotlib, pyserial, pytest · **Spec:** `docs/design.md`

## Global Constraints
- คาลิเบรตอยู่ `src/config.py` ที่เดียว · ไฟ < 6 V ไม่ส่ง · v ≤ 30 °/s · J5 ล็อก 0 · ไม่มีอะไรขยับเองตอนเปิด/ต่อ
- test: `cd src && ../.venv/Scripts/python.exe -m pytest -q`

## Review Focus
1. jog ชน limit → หยุดนุ่มที่ limit ไม่ทะลุ — test `test_jog_stops_at_limit`
2. jog xyz เข้าจุดเอื้อมไม่ถึง → หยุด + เหตุผล ไม่กระโดด — `test_jog_xyz_unreachable_stops`
3. กดเป้าใหม่ระหว่างวิ่ง → เริ่มจาก setpoint+ความเร็วปัจจุบัน ไม่กระโดด — `test_new_goal_mid_move_no_jump`
4. ไฟตก/ออฟไลน์ระหว่างวิ่ง → หยุด — `test_driver_stops_on_low_volt`
5. ทุก step ไม่มีข้อไหนกระโดด > v_max·dt·1.05 — ตรวจในทุก test ของ Mover ด้วย helper `_assert_smooth`

### Task 1: โครง repo + config + link
Files: `src/config.py` (ตัด ESP32/correction เดิม, เพิ่ม `PERIOD_S=0.04`, `STREAM_MS=60`, `V_JOINT=10`, `A_JOINT=30`, `V_CART=20`, `V_MAX_LIMIT=30`, `JUMP_GUARD_DEG=3`), `src/link.py` (`DirectLink`, `SimLink`, + `torque(on)`), tests `test_link.py`
- [ ] test: stream `send_q(q, 60)` ส่ง time 60 · กระโดด > 3° ใน 60 ms → ไม่ส่ง · nan ไม่ส่ง · ต่อ = hold + torque · ไม่ส่ง/อ่าน J5 · SimLink ตามทันที
- [ ] implement → pass → commit

### Task 2: motion.Mover
Interfaces: `Mover(q0, v=10, a=30, v_cart=20)`; `.q`, `.busy`, `.message`; `goto_q(q)->str`, `goto_xyz(x,y,z,phi)->str` (""=ok), `jog_joint(i,d)`, `jog_xyz(axis,d)`, `release()`, `follow(q)`, `stop()`, `reset(q)`, `step(dt)->list`
- [ ] tests: goto_q ถึงเป้าภายใน T+dt, ทุกข้อถึงพร้อมกัน, ความเร็ว ≤ v, เริ่ม/จบความเร็ว ~0 · goto_xyz ทุกจุดห่างเส้นตรง < 1 mm, ถึงเป้า · goto_xyz ปลายทางเอื้อมไม่ถึง → ข้อความ ไม่ขยับ · jog ramp/decel · jog limit · jog xyz unreachable · follow ไล่ทัน ไม่ overshoot · stop · new goal mid move · `_assert_smooth` ทุกตัว
- [ ] implement → pass → commit

### Task 3: driver.Driver
Interfaces: `Driver(link, mover, period=0.04)`; `.start()`, `.close()`, `.correct=True`, `.message`; thread: step → send_q · ไฟ/ออฟไลน์ → stop · ชดเชยหลังจบ goto
- [ ] tests (SimLink + link ปลอมที่ยวบ): ส่งต่อเนื่อง, หยุดเมื่อไฟต่ำ/ออฟไลน์, ชดเชยลด error ≤ 0.5°, ≤ ±5°
- [ ] implement → pass → commit

### Task 4: app.py + bat + README/CLAUDE.md
- [ ] tests (tk ซ่อนหน้าต่าง): ปุ่ม jog กด/ปล่อย → mover.jog/release · เป้า → goto · slider follow · หยุด · ไม่ต่อแขน = จำลอง
- [ ] implement → pass → commit · README (รูป ASCII หน้าต่าง + วิธีใช้), CLAUDE.md (กฎความปลอดภัย)

### Task 5: ทดสอบแขนจริง (ถามผู้ใช้ทุกครั้ง, v = 5 °/s ครั้งแรก)
