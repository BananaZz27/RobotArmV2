# วนลูป A ⇄ B (auto_cycle.py) — state machine ล้วน ๆ + บันทึก/อ่านไฟล์จุด
import pytest
from auto_cycle import AutoCycle, load_dwells, load_points, save_points

A, B = [0.0, 10.0, 20.0, 30.0], [40.0, 0.0, 60.0, -10.0]


def _arrive(c, now, q):
    """จำลองแขนถึงจุด q (ไม่ busy แล้ว) → คืนคำสั่งถัดไป (ถ้ามี)"""
    return c.update(now, False, q)


def test_needs_both_points():
    with pytest.raises(ValueError):
        AutoCycle([A, None]).start(0)


def test_sequence_and_cycle_count():
    c = AutoCycle([A, B], dwell=1.0, cycles=2)
    assert c.start(0) == A                          # ไป A ก่อน (ตั้งต้น — ยังไม่นับรอบ)
    assert c.update(0.5, True, [5, 5, 5, 5]) is None  # ยังวิ่งอยู่
    assert _arrive(c, 1, A) is None and c.phase == "dwell" and c.done == 0
    assert c.update(1.5, False, A) is None           # รอยังไม่ครบ 1 s
    assert c.update(2.0, False, A) == B              # รอครบ → ไป B
    assert _arrive(c, 3, B) is None
    assert c.update(4, False, B) == A
    assert _arrive(c, 5, A) is None and c.done == 1 and c.last_cycle_s == pytest.approx(4)   # A(1) → A(5)
    assert c.update(6, False, A) == B
    _arrive(c, 7, B)
    assert c.update(8, False, B) == A
    assert _arrive(c, 9, A) is None
    assert c.done == 2 and c.phase == "done" and not c.running   # ครบ 2 รอบ → หยุดเองที่ A


def test_unlimited_cycles_keep_going():
    c = AutoCycle([A, B], dwell=0, cycles=0)
    c.start(0)
    t = 0
    for _ in range(20):
        t += 1
        _arrive(c, t, c.points[c.index])
        t += 0.01
        assert c.update(t, False, c.points[c.index]) is not None
    assert c.running and c.done == 9                # ถึง A 10 ครั้ง − ครั้งแรก (ตั้งต้น) = 9 รอบ


def test_stops_when_arm_halts_before_point():
    c = AutoCycle([A, B])
    c.start(0)
    assert _arrive(c, 1, [0.0, 10.0, 25.0, 30.0]) is None     # ไม่ busy แต่ยังห่าง A 5° = ถูกหยุดกลางทาง (ไฟตก/STOP)
    assert not c.running and c.phase == "stopped" and "A" in c.reason


def test_stop_and_status():
    c = AutoCycle([A, B], dwell=2)
    assert "พร้อม" in c.status(0)
    c.start(0)
    assert "ไปจุด A" in c.status(0)
    _arrive(c, 1, A)
    assert "รอที่จุด A" in c.status(1.5)
    c.stop("ทดสอบ")
    assert c.update(10, False, A) is None and "ทดสอบ" in c.status(10)


def test_points_file_roundtrip(tmp_path):
    f = tmp_path / "pts.json"
    assert load_points(f) == [None, None]            # ยังไม่มีไฟล์
    save_points([A, None], f)
    assert load_points(f) == [A, None]
    f.write_text("{bad json", encoding="utf-8")
    assert load_points(f) == [None, None]            # ไฟล์เสีย → เริ่มใหม่ ไม่พัง


def test_three_point_sequence():
    C = [10.0, 30.0, 40.0, 0.0]
    c = AutoCycle([A, B, C], dwell=0, cycles=1)
    assert c.start(0) == A
    _arrive(c, 1, A)
    assert c.update(1.1, False, A) == B
    _arrive(c, 2, B)
    assert c.update(2.1, False, B) == C                               # A → B → C → A
    _arrive(c, 3, C)
    assert c.update(3.1, False, C) == A
    _arrive(c, 4, A)
    assert c.phase == "done" and c.done == 1


def test_points_file_old_format_and_many_points(tmp_path):
    f = tmp_path / "pts.json"
    f.write_text('{"A": [1, 2, 3, 4], "B": null}', encoding="utf-8")   # ไฟล์รูปแบบเดิม (มีแค่ A, B)
    assert load_points(f) == [[1.0, 2.0, 3.0, 4.0], None]
    save_points([A, B, A], f)
    assert load_points(f) == [A, B, A]


def test_approaching_only_until_first_point():
    c = AutoCycle([A, B], dwell=0)
    assert not c.approaching
    c.start(0)
    assert c.approaching                              # วิ่งไป A ครั้งแรก
    _arrive(c, 1, A)
    assert not c.approaching


def test_dwell_per_point():
    c = AutoCycle([A, B], dwell=9, dwells=[0.0, 2.0])
    c.start(0)
    _arrive(c, 1, A)
    assert c.update(1.0, False, A) == B               # A รอ 0 s → ไปต่อทันที
    _arrive(c, 2, B)
    assert c.update(3.9, False, B) is None            # B รอ 2 s ยังไม่ครบ
    assert c.update(4.0, False, B) == A
    assert AutoCycle([A, B], dwell=0.7).dwell_at(1) == 0.7   # ไม่ได้ตั้ง → ค่าเริ่มต้น


def test_dwells_file_roundtrip(tmp_path):
    f = tmp_path / "pts.json"
    assert load_dwells(f) == []
    save_points([A, B], f, [0.5, 2.0])
    assert load_points(f) == [A, B] and load_dwells(f) == [0.5, 2.0]
    save_points([A, B], f)                            # ไฟล์ไม่มีเวลารอ → []
    assert load_dwells(f) == []
