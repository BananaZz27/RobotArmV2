# ทดสอบตัวป้อนท่า Mover — ทุก test ตรวจว่าไม่มีก้าวไหนกระโดดเกินความเร็วที่ตั้ง (_run ตรวจให้)
import math

import config
import numpy as np
import pytest
from kinematics import forward_kinematics
from motion import Mover

DT = config.PERIOD_S


def _run(m, seconds, v_check=None):
    """เดิน Mover ไป seconds วินาที → list ของท่าทุกก้าว · ตรวจความเร็วต่อข้อ ≤ v·1.05"""
    v = v_check or m.v
    out, prev = [], list(m.q)
    for _ in range(int(seconds / DT) + 1):
        q = m.step(DT)
        for a, b in zip(prev, q):
            assert abs(b - a) <= v * DT * 1.05 + 1e-9, f"กระโดด {abs(b - a):.3f}° ใน {DT}s"
        out.append(list(q))
        prev = q
    return out


def _tip(q):
    f = forward_kinematics(q)
    return np.array([f.x, f.y, f.z])


# ---------- ไปมุม ----------

def test_goto_q_reaches_and_all_joints_arrive_together():
    m = Mover([0, 0, 0, 0], v=10)
    assert m.goto_q([10, -5, 20, 0]) == ""
    T = 1.875 * 20 / 10
    traj = _run(m, T + 0.2)
    assert traj[-1] == pytest.approx([10, -5, 20, 0], abs=1e-6) and not m.busy
    # ถึงพร้อมกัน: ที่ครึ่งเวลา ทุกข้อไปได้ครึ่งทางเท่ากัน (สัดส่วนเดียวกัน)
    mid = traj[int(T / 2 / DT)]
    frac = [mid[i] / g for i, g in enumerate([10, -5, 20]) if g]
    assert max(frac) - min(frac) < 1e-6


def test_goto_q_starts_and_ends_slow():
    m = Mover([0, 0, 0, 0], v=10)
    m.goto_q([0, 20, 0, 0])
    traj = _run(m, 4.0)
    steps = np.diff([q[1] for q in traj])
    assert steps[0] < 0.05 and steps[len(steps) // 2 - 5] > 0.3        # ออกตัวช้า กลางทางเร็ว
    end = [s for s in steps if s > 1e-9][-1]
    assert end < 0.05                                                    # ผ่อนก่อนถึง


def test_goto_q_clamped_to_limits():
    m = Mover([0, 0, 0, 0], v=30)
    m.goto_q([0, 500, 0, 0])
    traj = _run(m, 40, v_check=30)
    assert traj[-1][1] == pytest.approx(config.Q_MAX[1])


def test_new_goal_mid_move_no_jump():
    m = Mover([0, 0, 0, 0], v=10)
    m.goto_q([0, 20, 0, 0])
    _run(m, 1.0)
    m.goto_q([0, -10, 0, 0])               # เปลี่ยนใจกลางทาง → ต้องไม่กระโดด (_run ตรวจ)
    assert _run(m, 8)[-1][1] == pytest.approx(-10)


# ---------- เส้นตรง XYZ ----------

def test_goto_xyz_straight_line():
    q0 = [0, 10, 20, 20]
    m = Mover(q0, v=10, v_cart=20)
    p0 = _tip(q0)
    goal = forward_kinematics([5, 15, 25, 15])
    assert m.goto_xyz(goal.x, goal.y, goal.z, goal.phi) == ""
    traj = _run(m, 20)
    p1 = np.array([goal.x, goal.y, goal.z])
    d = (p1 - p0) / np.linalg.norm(p1 - p0)
    for q in traj:
        p = _tip(q) - p0
        assert np.linalg.norm(p - np.dot(p, d) * d) < 1.0            # ห่างเส้นตรง < 1 mm
    assert np.linalg.norm(_tip(traj[-1]) - p1) < 0.1 and not m.busy


def test_goto_xyz_unreachable_refused():
    m = Mover([0, 0, 0, 0])
    msg = m.goto_xyz(500, 0, 0, 0)
    assert "เอื้อมไม่ถึง" in msg and not m.busy
    assert _run(m, 0.5)[-1] == [0, 0, 0, 0]


# ---------- jog ----------

def test_jog_joint_ramps_up_and_down():
    m = Mover([0, 0, 0, 0], v=10, a=30)
    m.jog_joint(1, +1)
    traj = _run(m, 1.0)
    steps = np.diff([q[1] for q in traj])
    assert steps[0] < steps[-1] and steps[-1] == pytest.approx(10 * DT, rel=0.01)
    m.release()
    after = _run(m, 1.0)
    moved = after[-1][1] - traj[-1][1]
    assert 0 < moved <= 10 ** 2 / (2 * 30 * config.JOG_STOP) + 0.5 and not m.busy   # ผ่อน (0.5·a) จนหยุด


def test_jog_stops_at_limit():
    start = config.Q_MAX[1] - 1
    m = Mover([0, start, 0, 0], v=10, a=30)
    m.jog_joint(1, +1)
    traj = _run(m, 2.0)
    assert max(q[1] for q in traj) <= config.Q_MAX[1] + 1e-9


def test_jog_xyz_moves_tip_along_axis():
    q0 = [0, 10, 30, 30]
    m = Mover(q0, v=30, v_cart=20)
    p0 = _tip(q0)
    m.jog_xyz(2, -1)                          # Z ลง
    traj = _run(m, 1.0, v_check=30)
    m.release()
    traj += _run(m, 1.0, v_check=30)
    p = _tip(traj[-1])
    assert p[2] < p0[2] - 5 and abs(p[0] - p0[0]) < 0.5 and abs(p[1] - p0[1]) < 0.5
    assert not m.busy


def test_jog_xyz_unreachable_stops():
    q0 = [0, 0, 0, 5]                      # เกือบเหยียดสุด → ดันขึ้นต่อไม่ได้
    m = Mover(q0, v=30, v_cart=20)
    m.jog_xyz(2, +1)
    _run(m, 3.0, v_check=30)
    assert not m.busy and m.message


# ---------- slider / หยุด ----------

def test_follow_reaches_without_overshoot():
    m = Mover([0, 0, 0, 0], v=10, a=30)
    m.follow([0, 0, 15, 0])
    traj = _run(m, 4.0)
    assert max(q[2] for q in traj) <= 15 + 1e-6 and traj[-1][2] == pytest.approx(15, abs=1e-3)
    assert not m.busy


def test_follow_target_change_keeps_smooth():
    m = Mover([0, 0, 0, 0], v=10, a=30)
    m.follow([0, 0, 15, 0])
    _run(m, 0.8)
    m.follow([0, 0, -5, 0])
    assert _run(m, 6)[-1][2] == pytest.approx(-5, abs=1e-3)


def test_stop_freezes():
    m = Mover([0, 0, 0, 0])
    m.goto_q([0, 20, 0, 0])
    _run(m, 1.0)
    m.stop()
    q = list(m.q)
    assert _run(m, 0.5)[-1] == q and not m.busy


def test_reset_ignores_nan():
    m = Mover([1, 2, 3, 4])
    m.reset([5, math.nan, 7, 8])
    assert m.q == [5, 2, 7, 8]


def test_finished_flag_once():
    m = Mover([0, 0, 0, 0])
    m.goto_q([0, 2, 0, 0])
    _run(m, 2)
    assert m.pop_finished() == pytest.approx([0, 2, 0, 0]) and m.pop_finished() is None


def test_jog_xyz_stops_before_singularity():
    m = Mover([0, 30, 20, 20], v=150, a=600, v_cart=100)
    m.jog_xyz(0, +1)                          # ดันออกไปข้างหน้า → แขนเหยียดตรง (θ3 → 0)
    _run(m, 5.0, v_check=150)
    assert not m.busy and "singularity" in m.message


def test_jog_xyz_can_leave_singularity():
    q0 = [0, 30, 3, 10]                    # เกือบเหยียด (อยู่ในโซนเตือน)
    m = Mover(q0, v=150, a=600, v_cart=50)
    p0 = _tip(q0)
    m.jog_xyz(0, -1)                          # ดึงเข้า = ออกจาก singularity → ต้องไปได้
    _run(m, 1.5, v_check=150)                 # ใกล้ singularity ย่อก้าวให้ข้อไม่เกิน jog v → ออกตัวช้า
    assert _tip(m.q)[0] < p0[0] - 5


def test_jog_arrives_at_limit_slowly():
    """ชน limit ต้องช้า (เดิมชนที่ ~14 °/s) — ก้าวสุดท้ายก่อนถึง limit ≤ 6 °/s (ก้าว 40 ms ทำให้เหลือ ~5.6)"""
    lim = config.Q_MAX[2]
    m = Mover([0, 0, lim - 12, 0])
    m.jog_joint(2, +1)
    prev, last_v = m.q[2], None
    for _ in range(int(3 / DT)):
        q = m.step(DT)[2]
        if q >= lim - 1e-9:
            break
        last_v, prev = (q - prev) / DT, q
    assert last_v is not None and last_v <= 6.0


def _z(q):
    return forward_kinematics(q).z


@pytest.mark.parametrize("q0", [[0, 0, 0, 0], [0, -0.5, 0.7, 0.5], [0, 45, 0, 0], [30, 40, 0, 0]])
def test_jog_minus_z_from_stretched_moves_down(q0):
    """แขนเหยียดตรง (ชี้ฟ้า / เหยียดไปหน้า) แล้วกด Z− ต้องลงได้ — ยอมให้ φ เปลี่ยน, ไม่พุ่งเกิน jog v"""
    m = Mover(q0)
    m.jog_xyz(2, -1)
    traj = _run(m, 2.0, v_check=config.JOG_V)
    assert _z(traj[-1]) < _z(q0) - 5, (m.message, traj[-1])


def test_jog_minus_z_at_horizontal_full_stretch_says_why():
    """เหยียดสุดแนวนอน: ลงตรง ๆ (x เท่าเดิม) ต้องยาวขึ้น = ขอบพื้นที่เอื้อมถึงจริง → หยุดพร้อมบอกเหตุผล"""
    m = Mover([0, 90, 0, 0])
    m.jog_xyz(2, -1)
    _run(m, 1.0, v_check=config.JOG_V)
    assert not m.busy and "เหยียดสุด" in m.message


def test_goto_q_with_slower_speed():
    m = Mover([0, 0, 0, 0], v=150)
    m.goto_q([0, 0, 40, 0], v=20)
    traj = _run(m, 1.875 * 40 / 20 + 0.1, v_check=20)          # ไม่เกิน 20 °/s แม้ Mover ตั้งไว้ 150
    assert traj[-1][2] == pytest.approx(40) and not m.busy


@pytest.mark.parametrize("i", range(4))
@pytest.mark.parametrize("at_max", [True, False])
def test_jog_away_from_limit_moves(i, at_max):
    """ข้อที่ค้างอยู่ที่ limit (เช่นคอก้มสุด 78.2° = ท่า Standby) ต้อง jog ถอยออกได้ — เดิมค้างที่ limit บน"""
    q = [0.0, 0.0, 0.0, 0.0]
    q[i] = config.Q_MAX[i] if at_max else config.Q_MIN[i]
    m = Mover(q)
    m.jog_joint(i, -1 if at_max else +1)
    for _ in range(25):
        m.step(0.04)
    assert abs(m.q[i] - q[i]) > 5
