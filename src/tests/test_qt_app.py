# ทดสอบหน้าต่าง Qt แบบ ws-control (ไม่เปิดจอ, ภาพ 3D ปลอม) — เดิน driver ด้วย tick() เอง
import os
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import config  # noqa: E402
import pytest  # noqa: E402
import serial  # noqa: E402
from PyQt6.QtWidgets import QApplication, QWidget  # noqa: E402
from link import SimLink  # noqa: E402
from qt_app import MainWindow  # noqa: E402

DT = config.PERIOD_S
APP = QApplication.instance() or QApplication([])


class FakeViewer(QWidget):
    def __init__(self):
        super().__init__()
        self.target = self.actual = self.marker = None

    def show_target(self, q):
        self.target = list(q)

    def show_actual(self, q):
        self.actual = q

    def show_ik_marker(self, xyz):
        self.marker = xyz


@pytest.fixture
def win():
    w = MainWindow(viewer=FakeViewer())
    w.timer.stop()
    w.driver.close()                       # test เดิน tick() เอง
    yield w
    w.close()


def _run(w, seconds):
    t = time.monotonic()
    for _ in range(int(seconds / DT)):
        t += DT
        w.driver.tick(now=t)


def test_starts_in_simulation(win):
    assert win.link.simulated and "Simulation" in win.conn_label.text()


def test_slider_sets_target_without_sending(win):
    win.rows[1][0].setValue(100)           # slider ×10 → 10°
    assert win.q_target[1] == 10 and win.viewer.target[1] == 10
    assert win.rows[1][1].value() == 10    # ช่องตัวเลขตาม
    _run(win, 1.0)
    assert win.link.q_actual[1] == 0       # ยังไม่ Send


def test_send_moves_smoothly(win):
    win.set_target([0, 10, 0, 0, 0])
    win.send()
    _run(win, 0.2)
    assert 0 < win.link.q_actual[1] < 10
    _run(win, 3.0)
    assert win.link.q_actual[1] == pytest.approx(10)


def test_live_follows(win):
    win.live.setChecked(True)
    win.rows[2][1].setValue(6)
    _run(win, 3.0)
    assert win.link.q_actual[2] == pytest.approx(6)


def test_fk_display(win):
    win.set_target([30, 20, 40, 30, 0])
    assert win.fk_values["X"].text().strip() == "157.6" and win.fk_values["Z"].text().strip() == "274.6"


def test_ik_preview_and_go(win):
    win.ik_solve(send=False)
    assert "★" in win.ik_result.toPlainText() and win.viewer.marker == (110, 10, 350)
    _run(win, 1.0)
    assert win.link.q_actual[2] == 0
    win.ik_solve(send=True)
    _run(win, 6.0)
    assert win.link.q_actual[:4] == pytest.approx(win.q_target[:4], abs=1e-6)


def test_ik_unreachable(win):
    win.ik_x.setValue(400)
    win.ik_z.setValue(0)
    win.ik_solve(send=True)
    assert "ไปจุดนี้ไม่ได้" in win.ik_result.toPlainText() and not win.driver.mover.busy


def test_low_volt_blocks(win):
    arm = SimLink()
    arm.simulated, arm.volt = False, 5.0
    win._use_link(arm)
    win.driver.close()
    win.set_target([0, 10, 0, 0, 0])
    win.send()
    assert not win.driver.mover.busy and "ไฟ servo ต่ำ" in win.log_view.toPlainText()


def test_torque_off_blocks_and_target_follows_actual(win):
    win.set_torque(False)
    win.set_target([0, 10, 0, 0, 0])
    win.send()
    assert not win.driver.mover.busy
    win.link.q_actual = [0, 4, 5, 6, 0]
    win.poll()
    assert win.q_target[:4] == [0, 4, 5, 6]


def test_stop(win):
    win.set_target([0, 20, 0, 0, 0])
    win.send()
    _run(win, 0.5)
    win.stop()
    assert not win.driver.mover.busy and win.q_target[1] == pytest.approx(win.driver.mover.q[1])


def test_connect_failure(win):
    def boom():
        raise serial.SerialException("busy")
    win.connect_fn = boom
    win.toggle_connect()
    assert win.link.simulated and "ต่อไม่ได้" in win.conn_label.text()


def test_connect_success_target_is_actual(win):
    arm = SimLink()
    arm.simulated, arm.volt, arm.q_actual = False, 7.6, [5.0, 6.0, 7.0, 8.0, 0.0]
    win.connect_fn = lambda: arm
    win.toggle_connect()
    win.driver.close()
    assert win.q_target == [5.0, 6.0, 7.0, 8.0, 0.0] and win.connect_btn.text() == "Disconnect"
    win.poll()
    assert "7.6" in win.conn_label.text()


def test_speed(win):
    win.speed.setValue(5)
    assert win.driver.mover.v == 5
