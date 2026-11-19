# ภาพจำลอง 2D สองมุม (matplotlib)
#   มุมบน  = ระนาบ x–y  → เห็นฐานหมุน θ₁ (สไลด์ θ₁ = atan2(yc, xc))
#   มุมข้าง = ระนาบ r–z  → เห็นไหล่/ศอก/คอ (สไลด์ geometric approach: a2, a3, θ₂, θ₃)
# เส้นทึบ = แขนที่สั่ง · เส้นประ = แขนจริงจากมุมที่บอร์ดอ่านได้
import matplotlib
import numpy as np

from kinematics import forward_kinematics

REACH = 450  # mm ขอบภาพ
# ฟอนต์ DejaVu (ค่าเริ่มต้น) ไม่มีอักษรไทย → ใช้ Tahoma ที่มากับ Windows
matplotlib.rcParams["font.family"] = ["Tahoma", "DejaVu Sans"]


def top_points(frames):
    """จุดกำเนิดเฟรม 0…5 มองจากบน: (xs, ys)"""
    return [T[0, 3] for T in frames], [T[1, 3] for T in frames]


def side_points(frames, theta1_deg):
    """จุดกำเนิดเฟรม 0…5 มองจากข้าง: r = x·cos θ₁ + y·sin θ₁ (มีเครื่องหมาย), z"""
    c, s = np.cos(np.radians(theta1_deg)), np.sin(np.radians(theta1_deg))
    return [T[0, 3] * c + T[1, 3] * s for T in frames], [T[2, 3] for T in frames]


class SimView:
    def __init__(self, figure):
        self.figure = figure
        self.ax_top, self.ax_side = figure.subplots(1, 2)
        for ax, title, xlabel, ylabel in (
            (self.ax_top, "มุมบน (x–y)", "x (mm)", "y (mm)"),
            (self.ax_side, "มุมข้าง (r–z)", "r (mm)", "z (mm)"),
        ):
            ax.set_title(title)
            ax.set_xlabel(xlabel)
            ax.set_ylabel(ylabel)
            ax.set_aspect("equal")
            ax.grid(True, alpha=0.3)
        self.ax_top.set_xlim(-REACH, REACH)
        self.ax_top.set_ylim(-REACH, REACH)
        self.ax_side.set_xlim(-REACH, REACH)
        self.ax_side.set_ylim(-50, REACH + 50)
        self.ax_side.axhline(0, color="gray", lw=1)  # พื้นโต๊ะ
        self.cmd_top, = self.ax_top.plot([], [], "o-", color="tab:blue", lw=3, label="สั่ง")
        self.cmd_side, = self.ax_side.plot([], [], "o-", color="tab:blue", lw=3)
        self.act_top, = self.ax_top.plot([], [], "o--", color="tab:orange", lw=2, label="จริง")
        self.act_side, = self.ax_side.plot([], [], "o--", color="tab:orange", lw=2)
        self.ax_top.legend(loc="upper right", fontsize=8)

    def draw(self, q_cmd, q_actual=None):
        self._set(self.cmd_top, self.cmd_side, q_cmd)
        show = q_actual is not None and not any(np.isnan(q_actual))
        self.act_top.set_visible(show)
        self.act_side.set_visible(show)
        if show:
            self._set(self.act_top, self.act_side, q_actual)
        self.figure.canvas.draw_idle()

    @staticmethod
    def _set(line_top, line_side, q):
        frames = forward_kinematics(q).frames
        line_top.set_data(*top_points(frames))
        line_side.set_data(*side_points(frames, q[0]))
