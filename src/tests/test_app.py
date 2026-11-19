# ทดสอบหน้าต่างแบบ v2 (ซ่อนไว้) — แขนจำลอง, เรียก handler ตรง ๆ แล้วเดิน driver ด้วย tick()
import time
import tkinter as tk

import config
import pytest
import serial
from app import DIRECT, SIM, App
from link import SimLink

DT = config.PERIOD_S


@pytest.fixture(scope="module")
def tk_root():
    try:
        root = tk.Tk()
    except tk.TclError:
        pytest.skip("ไม่มีหน้าจอให้เปิด tkinter")
    root.withdraw()
    yield root
    root.destroy()


@pytest.fixture
def app(tk_root):
    win = tk.Toplevel(tk_root)
    win.withdraw()
    a = App(win)
    a.driver.close()                       # หยุด thread — test เดิน tick() เอง
    yield a
    a.close()
    win.destroy()


def _run(app, seconds):
    t = time.monotonic()
    for _ in range(int(seconds / DT)):
        t += DT
        app.driver.tick(now=t)


def test_slider_only_previews(app):
    app.slider_vars[1].set(10.0)
    app.on_slider()
    _run(app, 1.0)
    assert app.q_cmd[1] == 10.0 and app.link.q_actual[1] == 0.0      # ยังไม่กดส่ง = ไม่ขยับ


def test_send_moves_smoothly(app):
    app.set_q([0, 10, 0, 0, 0])
    assert app.on_send()
    _run(app, 0.2)
    assert 0 < app.link.q_actual[1] < 10                                # กำลังไป ไม่กระโดดถึงทันที
    _run(app, 3.0)
    assert app.link.q_actual[1] == pytest.approx(10)


def test_fk_text(app):
    app.set_q([30, 20, 40, 30, 0])
    assert all(t in app.fk_text.get() for t in ("157.6", "91.0", "274.6"))


def test_ik_preview_does_not_send(app):
    assert app.on_ik_preview()
    _run(app, 1.0)
    assert app.link.q_actual == [0.0] * 5


def test_ik_send(app):
    app.on_ik_send()
    _run(app, 6.0)
    assert app.link.q_actual[:4] == pytest.approx(app.q_cmd[:4], abs=1e-6)


def test_ik_unreachable(app):
    app.ik_entries["x"].set("500")
    assert not app.on_ik_preview() and "เอื้อมไม่ถึง" in app.status.get()


def test_ik_bad_input(app):
    app.ik_entries["phi"].set("abc")
    assert not app.on_ik_preview()


def test_over_limit_not_sent(app):
    app.q_cmd = [0, 500, 0, 0, 0]
    assert not app.on_send()


def test_low_volt_blocks_send(app):
    app.link.volt = 5.0
    assert not app.on_send() and "ไฟ" in app.status.get()


def test_unused_joint_slider_locked(app):
    assert str(app.sliders[4].cget("state")) == "disabled"


def test_stop(app):
    app.set_q([0, 20, 0, 0, 0])
    app.on_send()
    _run(app, 0.5)
    app.on_stop()
    assert not app.driver.mover.busy and "หยุด" in app.status.get()


def test_speed(app):
    app.speed.set(5)
    app.on_speed()
    assert app.driver.mover.v == 5


def test_connect_direct_failure_back_to_sim(app):
    def boom():
        raise serial.SerialException("busy")
    app.connect_fn = boom
    app.connect(DIRECT)
    assert app.link.simulated and app.port_var.get() == SIM


def test_connect_direct_sets_sliders_to_actual(app):
    arm = SimLink()
    arm.q_actual = [5.0, 6.0, 7.0, 8.0, 0.0]
    app.connect_fn = lambda: arm
    app.connect(DIRECT)
    app.driver.close()
    assert app.link is arm and app.q_cmd == [5.0, 6.0, 7.0, 8.0, 0.0]


def test_tick(app):
    app.tick()
    assert app.volt_text.get() == SIM
