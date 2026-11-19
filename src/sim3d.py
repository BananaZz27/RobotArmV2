# ภาพจำลองแขน 3D (matplotlib) — ลากเมาส์หมุนมุมมองได้ · เส้นทึบ = ท่าที่สั่ง, เส้นประ = มุมจริงจาก servo
import math

import matplotlib
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (ลงทะเบียน projection="3d")

from kinematics import forward_kinematics

matplotlib.rcParams["font.family"] = ["Tahoma", "DejaVu Sans"]   # ฟอนต์ไทย
REACH = 400  # mm


def points(q):
    """จุดกำเนิดเฟรม 0…5 → (xs, ys, zs)"""
    frames = forward_kinematics(q).frames
    return [T[0, 3] for T in frames], [T[1, 3] for T in frames], [T[2, 3] for T in frames]


class Sim3D:
    def __init__(self, figure):
        self.ax = figure.add_subplot(projection="3d")
        ax = self.ax
        ax.set_xlim(-REACH, REACH)
        ax.set_ylim(-REACH, REACH)
        ax.set_zlim(0, REACH + 50)
        ax.set_xlabel("x (mm)")
        ax.set_ylabel("y (mm)")
        ax.set_zlabel("z (mm)")
        ax.set_box_aspect((1, 1, 0.55))
        ax.plot([-REACH, REACH], [0, 0], [0, 0], color="gray", lw=0.5)      # แกนบนพื้น
        ax.plot([0, 0], [-REACH, REACH], [0, 0], color="gray", lw=0.5)
        self.cmd, = ax.plot([], [], [], "o-", color="tab:blue", lw=4, label="สั่ง")
        self.act, = ax.plot([], [], [], "o--", color="tab:orange", lw=2, label="จริง")
        ax.legend(loc="upper left", fontsize=8)
        ax.view_init(elev=20, azim=-60)
        self.figure = figure

    def draw(self, q_cmd, q_actual=None):
        self.cmd.set_data_3d(*points(q_cmd))
        show = q_actual is not None and not any(math.isnan(v) for v in q_actual)
        self.act.set_visible(show)
        if show:
            self.act.set_data_3d(*points(q_actual))
        self.figure.canvas.draw_idle()
