# ทดสอบหน้าต่าง (ซ่อนไว้) — ใช้แขนจำลอง, เรียก handler ตรง ๆ แล้วเดิน driver ด้วย tick()
import time
import tkinter as tk

import config
import pytest
import serial
from app import App
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
    a = App(win, SimLink())
    a.driver.close()                       # หยุด thread — test เดิน tick() เอง
    yield a
    a.close()
    win.destroy()


def _run(app, seconds):
    t = time.monotonic()
    for _ in range(int(seconds / DT)):
        t += DT
        app.driver.tick(now=t)


def test_starts_in_sim(app):
    assert app.mode_text.get() == "จำลอง" and app.link.simulated


def test_jog_press_release(app):
    app.on_jog(lambda m: m.jog_joint(1, +1))
    _run(app, 1.0)
    assert app.driver.mover.q[1] > 5
    app.on_release()
    _run(app, 1.0)
    assert not app.driver.mover.busy


def test_goto_q_from_entries(app):
    app.q_entries[1].set("8")
    app.on_goto_q()
    _run(app, 3.0)
    assert app.link.q_actual[1] == pytest.approx(8)


def test_goto_q_bad_entry(app):
    app.q_entries[1].set("abc")
    app.on_goto_q()
    assert not app.driver.mover.busy and "ตัวเลข" in app.status.get()


def test_goto_xyz_unreachable_message(app):
    for k, v in zip(("X", "Y", "Z", "φ"), ("500", "0", "0", "0")):
        app.xyz_entries[k].set(v)
    app.on_goto_xyz()
    assert "เอื้อมไม่ถึง" in app.status.get() and not app.driver.mover.busy


def test_fill_then_goto_xyz_is_noop_move(app):
    app.fill_from_current()
    app.on_goto_xyz()
    _run(app, 1.0)
    assert app.link.q_actual[1] == pytest.approx(0, abs=0.1)


def test_slider_follow(app):
    app.slider_vars[2].set(6.0)
    app.on_slider()
    _run(app, 3.0)
    assert app.link.q_actual[2] == pytest.approx(6)


def test_stop(app):
    app.on_goto_q([0, 20, 0, 0, 0])
    _run(app, 0.5)
    app.on_stop()
    assert not app.driver.mover.busy and "หยุด" in app.status.get()


def test_torque_off_blocks_moves(app):
    app.on_torque(False)
    app.on_goto_q([0, 5, 0, 0, 0])
    assert not app.driver.mover.busy and "torque" in app.status.get()
    app.on_torque(True)
    app.on_goto_q([0, 5, 0, 0, 0])
    assert app.driver.mover.busy


def test_speed_applies(app):
    app.v_joint.set(5)
    app.on_speed()
    assert app.driver.mover.v == 5


def test_connect_failure_stays_sim(app):
    def boom():
        raise serial.SerialException("busy")
    app.connect_fn = boom
    app.on_connect()
    assert app.link.simulated and "ต่อแขนไม่ได้" in app.status.get()


def test_tick_updates_readout(app):
    app.tick()
    assert "ปลายแขน" in app.read_text.get()
