# ทดสอบหน้าต่างแบบง่าย (ซ่อนไว้) — แขนจำลอง, เรียก handler ตรง ๆ แล้วเดิน driver ด้วย tick()
import time
import tkinter as tk

import config
import pytest
import serial
from app import App, pick_phi
from kinematics import forward_kinematics

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


def test_starts_in_sim(app):
    assert app.mode_text.get() == "โหมดจำลอง" and app.link.simulated


def test_slider_follow(app):
    app.slider_vars[2].set(6.0)
    app.on_slider()
    _run(app, 3.0)
    assert app.link.q_actual[2] == pytest.approx(6)


def test_goto_xyz(app):
    goal = forward_kinematics([0, 10, 20, 20, 0])
    for k, v in zip("XYZ", (goal.x, goal.y, goal.z)):
        app.xyz_vars[k].set(f"{v:.1f}")
    app.on_goto_xyz()
    _run(app, 10.0)
    tip = forward_kinematics(app.link.q_actual)
    assert abs(tip.x - goal.x) < 0.2 and abs(tip.z - goal.z) < 0.2


def test_goto_xyz_unreachable(app):
    for k, v in zip("XYZ", ("500", "0", "0")):
        app.xyz_vars[k].set(v)
    app.on_goto_xyz()
    assert "ไม่ได้" in app.status.get() and not app.driver.mover.busy


def test_goto_xyz_bad_input(app):
    app.xyz_vars["X"].set("abc")
    app.on_goto_xyz()
    assert "ตัวเลข" in app.status.get()


def test_pick_phi_prefers_current():
    q = [0, 10, 20, 20, 0]
    f = forward_kinematics(q)
    assert abs(pick_phi(q, f.x, f.y, f.z) - f.phi) <= 2.5
    assert pick_phi(q, 500, 0, 0) is None


def test_stop(app):
    app.driver.do(lambda m: m.goto_q([0, 20, 0, 0, 0]))
    _run(app, 0.5)
    app.on_stop()
    assert not app.driver.mover.busy and "หยุด" in app.status.get()


def test_speed_applies(app):
    app.speed.set(5)
    app.on_speed()
    assert app.driver.mover.v == 5 and app.driver.mover.v_cart == 10


def test_connect_failure_stays_sim(app):
    def boom():
        raise serial.SerialException("busy")
    app.connect_fn = boom
    app.on_connect()
    assert app.link.simulated and "ต่อไม่ได้" in app.status.get()


def test_tick_updates(app):
    app.tick()
    assert "ปลายแขน" in app.read_text.get()
