# ทดสอบ Driver ด้วยแขนปลอม (เรียก tick() เองทีละรอบ ไม่ใช้ thread)
import config
import pytest
from driver import Driver
from link import SimLink
from motion import Mover

DT = config.PERIOD_S


class FakeArm(SimLink):
    """แขนปลอม: ยวบ sag องศาที่ไหล่ (ไปไม่ถึงเป้า) + ปรับไฟ/ออนไลน์ได้ + จดคำสั่ง"""
    simulated = False

    def __init__(self, sag=0.0):
        super().__init__()
        self.sag, self.volt, self.is_online = sag, 7.6, True
        self.sent, self.stops = [], 0

    def send_q(self, q, time_ms):
        self.sent.append((list(q), time_ms))      # เหมือน DirectLink: กันกระโดดเทียบกับท่าที่ส่ง ไม่ใช่มุมที่ยวบ
        self.q_actual = [float(v) if j["used"] else 0.0 for j, v in zip(config.JOINTS, q)]
        self.q_actual[1] -= self.sag
        return True

    def stop(self):
        self.stops += 1

    def online(self):
        return self.is_online


def _drive(d, seconds, t0=0.0):
    t = t0
    for _ in range(int(seconds / DT)):
        t += DT
        d.tick(now=t)
    return t


def test_streams_while_moving_and_stops_sending_when_idle():
    arm = FakeArm()
    d = Driver(arm, Mover([0] * 5, v=10))
    d.correct = False
    d.do(lambda m: m.goto_q([0, 5, 0, 0, 0]))
    _drive(d, 2.0)
    n = len(arm.sent)
    assert n > 10 and all(t == config.STREAM_MS for _q, t in arm.sent)
    assert arm.sent[-1][0][1] == pytest.approx(5)
    _drive(d, 1.0, 2.0)
    assert len(arm.sent) == n                     # นิ่งแล้วไม่ส่งซ้ำ


def test_driver_stops_on_low_volt():
    arm = FakeArm()
    d = Driver(arm, Mover([0] * 5, v=10))
    d.do(lambda m: m.goto_q([0, 20, 0, 0, 0]))
    _drive(d, 0.5)
    arm.volt = 5.5
    _drive(d, 0.2, 0.5)
    assert not d.mover.busy and arm.stops >= 1 and "ไฟ" in d.message


def test_driver_stops_when_offline():
    arm = FakeArm()
    d = Driver(arm, Mover([0] * 5, v=10))
    d.do(lambda m: m.goto_q([0, 20, 0, 0, 0]))
    arm.is_online = False
    _drive(d, 0.2)
    assert not d.mover.busy and arm.sent == [] and "ออฟไลน์" in d.message


def test_refused_send_stops():
    arm = FakeArm()
    arm.send_q = lambda q, t: False
    d = Driver(arm, Mover([0] * 5, v=10))
    d.do(lambda m: m.goto_q([0, 20, 0, 0, 0]))
    _drive(d, 0.2)
    assert not d.mover.busy


def test_correction_removes_sag():
    arm = FakeArm(sag=1.5)
    d = Driver(arm, Mover([0] * 5, v=10))
    d.do(lambda m: m.goto_q([0, 10, 0, 0, 0]))
    _drive(d, 10.0)
    assert abs(10 - arm.q_actual[1]) <= config.CORR_TOL
    assert arm.sent[-1][0][1] - 10 <= config.CORR_MAX
    assert "ชดเชย" in d.message


def test_correction_off():
    arm = FakeArm(sag=1.5)
    d = Driver(arm, Mover([0] * 5, v=10))
    d.correct = False
    d.do(lambda m: m.goto_q([0, 10, 0, 0, 0]))
    _drive(d, 10.0)
    assert arm.q_actual[1] == pytest.approx(8.5)


def test_correction_gives_up_on_big_error():
    arm = FakeArm(sag=8.0)
    d = Driver(arm, Mover([0] * 5, v=10))
    d.do(lambda m: m.goto_q([0, 10, 0, 0, 0]))
    _drive(d, 10.0)
    assert arm.sent[-1][0][1] == pytest.approx(10) and "ไม่ชดเชย" in d.message


def test_new_command_cancels_correction():
    arm = FakeArm(sag=1.5)
    d = Driver(arm, Mover([0] * 5, v=10))
    d.do(lambda m: m.goto_q([0, 10, 0, 0, 0]))
    t = _drive(d, 2.0)
    d.do(lambda m: m.jog_joint(0, +1))
    d.do(lambda m: m.release())
    _drive(d, 5.0, t)
    assert max(q[1] for q, _t in arm.sent) == pytest.approx(10)    # ไม่มีการชดเชยไหล่เกิดขึ้น


def test_emergency_stop_resets_to_actual():
    arm = FakeArm(sag=1.5)
    d = Driver(arm, Mover([0] * 5, v=10))
    d.do(lambda m: m.goto_q([0, 10, 0, 0, 0]))
    _drive(d, 1.0)
    d.emergency_stop()
    assert not d.mover.busy and arm.stops == 1 and d.mover.q[1] == pytest.approx(arm.q_actual[1])


def test_simulated_link_needs_no_volt():
    d = Driver(SimLink(), Mover([0] * 5, v=10))
    d.do(lambda m: m.goto_q([0, 3, 0, 0, 0]))
    _drive(d, 2.0)
    assert d.link.q_actual[1] == pytest.approx(3)


def test_peek_does_not_change_mover():
    m = Mover([0] * 5, v=30, a=90)
    m.jog_joint(2, +1)
    for _ in range(20):
        m.step(DT)
    q, vel = list(m.q), list(m.vel)
    ahead = m.peek(0.08)
    assert ahead[2] > q[2] and m.q == q and m.vel == vel


def test_servo_speed_steady_despite_tick_jitter():
    """รอบแกว่ง 27–52 ms (วัดจริง) → ความเร็วที่ servo ต้องวิ่ง = (เป้าที่ส่ง − ท่าตอนนี้) / STREAM_MS ต้องคงที่"""
    import random
    rnd = random.Random(0)
    arm = FakeArm()
    d = Driver(arm, Mover([0] * 5, v=30, a=90))
    d.correct = False
    d.do(lambda m: m.jog_joint(2, +1))
    t, speeds = 0.0, []
    for k in range(60):
        t += rnd.uniform(0.027, 0.052)
        d.tick(now=t)
        if k > 15:                                   # ช่วงวิ่งความเร็วคงที่ (เร่งเสร็จแล้ว)
            speeds.append((arm.sent[-1][0][2] - d.mover.q[2]) / (config.STREAM_MS / 1000))
    assert min(speeds) == pytest.approx(30, rel=0.02) and max(speeds) == pytest.approx(30, rel=0.02)
