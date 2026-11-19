# ตัวป้อนท่าแบบนุ่ม (design: docs/design.md) — คำนวณท่าถัดไปทุก dt วินาที ไม่แตะฮาร์ดแวร์ (test ได้)
#   goto_q   ไปมุมเป้า: ทุกข้อใช้โปรไฟล์ min-jerk ร่วมกัน → ออกตัวช้า ผ่อนก่อนถึง ถึงพร้อมกัน
#   goto_xyz ปลายแขนวิ่งเส้นตรงไปจุด (IK ทุก ~1 mm)
#   jog_*    กดค้าง = เร่งขึ้นถึงความเร็วที่ตั้ง · release() = ผ่อนจนหยุด
#   follow   ไล่ตามเป้าที่เปลี่ยนได้ตลอด (slider) ด้วยความเร็ว/ความเร่งจำกัด
import math

import numpy as np

import config
from kinematics import forward_kinematics, inverse_kinematics
from singularity import getting_worse

N = len(config.JOINTS)


def min_jerk(tau):
    """s(τ) = 10τ³ − 15τ⁴ + 6τ⁵ : 0→1, ความเร็วและความเร่งเป็น 0 ทั้งต้นและปลาย · ความเร็วสูงสุด = 1.875 ที่ τ = 0.5"""
    tau = min(max(tau, 0.0), 1.0)
    return tau ** 3 * (10 - 15 * tau + 6 * tau ** 2)


def clamp_q(q):
    """บีบทุกข้อใน limit · ข้อที่ไม่ได้ต่อ (J5) = 0"""
    return [min(max(v, config.Q_MIN[i]), config.Q_MAX[i]) if j["used"] else 0.0
            for i, (j, v) in enumerate(zip(config.JOINTS, q))]


class Mover:
    def __init__(self, q0, v=config.V_JOINT, a=config.A_JOINT, v_cart=config.V_CART):
        self.q = clamp_q([float(x) for x in q0])     # ท่าที่กำลังสั่ง (setpoint)
        self.v, self.a, self.v_cart = v, a, v_cart
        self.vel = [0.0] * N                         # ความเร็วข้อต่อตอนนี้ (jog / follow)
        self.mode = None                             # None | "path" | "jog_joint" | "jog_xyz" | "follow"
        self.message = ""
        self._finished = None

    @property
    def busy(self):
        return self.mode is not None

    # ---------- คำสั่ง ----------
    def goto_q(self, q_goal):
        """ไปมุมเป้าแบบ joint (คืน "" = รับคำสั่ง)"""
        goal = clamp_q(q_goal)
        start = list(self.q)
        dist = max(abs(g - s) for g, s in zip(goal, start))
        self._set_path([start, goal], 1.875 * dist / self.v, goal)
        return ""

    def goto_xyz(self, x, y, z, phi):
        """ปลายแขนวิ่งเส้นตรงไป (x, y, z) mm, φ องศา · ปลายทางคำนวณ IK ไม่ได้ = ไม่ขยับ คืนเหตุผล"""
        elbow_up = self.q[2] >= 0
        end = inverse_kinematics(x, y, z, phi, elbow_up, theta1_prev=self.q[0])
        if not end.ok:
            self.message = end.reason
            return end.reason
        f0 = forward_kinematics(self.q)
        p0, p1 = np.array([f0.x, f0.y, f0.z]), np.array([x, y, z])
        length = float(np.linalg.norm(p1 - p0))
        n = max(1, math.ceil(length / config.IK_STEP_MM))
        points, q_prev = [list(self.q)], self.q
        for k in range(1, n + 1):                    # IK ทีละจุดตามเส้นตรง (φ ค่อย ๆ เปลี่ยนเป็นเส้นตรงด้วย)
            s = k / n
            p = p0 + s * (p1 - p0)
            ik = inverse_kinematics(*p, f0.phi + s * (phi - f0.phi), elbow_up, theta1_prev=q_prev[0])
            if not ik.ok:                            # กลางทางไม่ได้ → ไปแบบ joint แทน
                self.goto_q(end.q)
                self.message = "เส้นตรงผ่านจุดที่เอื้อมไม่ถึง → ไปแบบหมุนข้อต่อแทน"
                return ""
            points.append(clamp_q(ik.q))
            q_prev = ik.q
        # เวลา: ปลายแขน ≤ v_cart และทุกข้อ ≤ v (ความเร็วสูงสุดของ min-jerk = 1.875 × ระยะ / T)
        joint_rate = max(max(abs(b - a) for a, b in zip(p, q)) for p, q in zip(points, points[1:])) * n
        t_cart, t_joint = length / self.v_cart, joint_rate / self.v
        if t_joint > 3 * t_cart + 1.0:               # เส้นตรงผ่านใกล้ท่าเหยียดสุด ข้อต้องหมุนเร็วมาก → ช้าเกินใช้
            self.goto_q(end.q)
            self.message = "ใกล้ท่าเหยียดสุด → ไปแบบหมุนข้อต่อแทนเส้นตรง"
            return ""
        self._set_path(points, 1.875 * max(t_cart, t_joint), points[-1])
        return ""

    def jog_joint(self, i, direction):
        self.mode, self._jog = "jog_joint", (i, direction)
        self.message = ""

    def jog_xyz(self, axis, direction):
        """axis 0/1/2 = X/Y/Z · φ คงที่ตามท่าตอนเริ่มกด"""
        f = forward_kinematics(self.q)
        self.mode, self._jog = "jog_xyz", (axis, direction)
        self._tip, self._phi, self._vc = np.array([f.x, f.y, f.z]), f.phi, 0.0
        self.vel = [0.0] * N
        self.message = ""

    def release(self):
        """ปล่อยปุ่ม jog → ผ่อนจนหยุด"""
        if self.mode in ("jog_joint", "jog_xyz"):
            self._jog = (self._jog[0], 0)

    def follow(self, q_goal):
        if self.mode != "follow":
            self.mode = "follow"
            self.message = ""
        self._goal = clamp_q(q_goal)

    def stop(self):
        self.mode, self.vel = None, [0.0] * N

    def reset(self, q):
        """ตั้งท่าที่สั่ง = มุมจริง (ข้อที่อ่านไม่ได้คงค่าเดิม)"""
        self.q = clamp_q([old if q_i is None or math.isnan(q_i) else q_i for old, q_i in zip(self.q, q)])
        self.stop()

    def pop_finished(self):
        """มุมเป้าของ goto ที่เพิ่งจบ (ครั้งเดียว) — ให้ driver ใช้ชดเชย"""
        done, self._finished = self._finished, None
        return done

    # ---------- เดินหนึ่งก้าว ----------
    def step(self, dt):
        if dt <= 0:                                  # เวลาไม่เดิน → ไม่ขยับ (กันหารศูนย์)
            return list(self.q)
        if self.mode == "path":
            self._step_path(dt)
        elif self.mode == "jog_joint":
            self._step_jog_joint(dt)
        elif self.mode == "jog_xyz":
            self._step_jog_xyz(dt)
        elif self.mode == "follow":
            self._step_follow(dt)
        return list(self.q)

    def _set_path(self, points, T, goal):
        self.mode, self.vel, self.message = "path", [0.0] * N, ""
        self._points, self._T, self._t, self._goal = points, max(T, 1e-6), 0.0, goal

    def _step_path(self, dt):
        self._t += dt
        s = min_jerk(self._t / self._T) * (len(self._points) - 1)
        k = min(int(s), len(self._points) - 2)
        a, b = self._points[k], self._points[k + 1]
        self.q = clamp_q([qa + (s - k) * (qb - qa) for qa, qb in zip(a, b)])
        if self._t >= self._T:
            self.q = list(self._goal)
            self._finished = list(self._goal)
            self.mode = None

    def _ramp(self, vel, v_target, dt):
        """เปลี่ยนความเร็วเข้าหาเป้าไม่เกิน a·dt ต่อก้าว"""
        dv = self.a * dt
        return vel + max(-dv, min(dv, v_target - vel))

    def _step_jog_joint(self, dt):
        i, d = self._jog
        lo, hi = config.Q_MIN[i], config.Q_MAX[i]
        room = (hi - self.q[i]) if self.vel[i] >= 0 else (self.q[i] - lo)
        brake = math.sqrt(2 * self.a * max(room, 0.0))   # ความเร็วที่ยังเบรกทันก่อนชน limit
        v_target = d * min(self.v, brake)
        self.vel[i] = self._ramp(self.vel[i], v_target, dt)
        self.q[i] = min(max(self.q[i] + self.vel[i] * dt, lo), hi)
        if self.q[i] in (lo, hi):
            self.vel[i] = 0.0
            if d:
                self.message = f"{config.JOINTS[i]['name']} ถึง limit"
        if d == 0 and self.vel[i] == 0.0:
            self.mode = None

    def _step_jog_xyz(self, dt):
        axis, d = self._jog
        a_cart = self.a / self.v * self.v_cart                 # เร่งถึงความเร็วเต็มในเวลาเท่ากับแบบข้อต่อ
        dv = a_cart * dt
        self._vc += max(-dv, min(dv, d * self.v_cart - self._vc))
        if d == 0 and abs(self._vc) < 1e-9:
            self._vc, self.mode = 0.0, None
            return
        for _ in range(2):                                     # ข้อต้องหมุนเร็วเกิน v → ลดความเร็วปลายแขนแล้วคิดใหม่
            tip = self._tip.copy()
            tip[axis] += self._vc * dt
            ik = inverse_kinematics(*tip, self._phi, self.q[2] >= 0, theta1_prev=self.q[0])
            if not ik.ok:
                # ขอบพื้นที่เอื้อมถึง = แขนเหยียดสุด (θ3 = 0) = singularity — ก้าวเดียวข้ามโซนเตือนได้ตอนเร็ว
                edge = "เอื้อมไม่ถึง" in ik.reason
                self.message = ("หยุด: ใกล้ singularity — แขนเหยียดสุด (ขอบพื้นที่เอื้อมถึง)" if edge
                                else "หยุด: " + ik.reason)
                self.stop()
                return
            q = clamp_q(ik.q)
            ratio = max(abs(b - a) for a, b in zip(self.q, q)) / (self.v * dt)
            if ratio <= 1.0:
                break
            self._vc /= ratio * 1.01
        else:
            q = self.q                                         # ยังเร็วเกิน → ก้าวนี้ไม่ขยับ
        worse = getting_worse(self.q, q)                       # กำลังเข้าใกล้ singularity (ถอยออกได้)
        if worse:
            self.message = "หยุด: ใกล้ singularity — " + worse
            self.stop()
            return
        if q is not self.q:
            self._tip, self.q = tip, q

    def _step_follow(self, dt):
        done = True
        for i in range(N):
            e = self._goal[i] - self.q[i]
            v_target = math.copysign(min(self.v, math.sqrt(2 * self.a * abs(e))), e)
            self.vel[i] = self._ramp(self.vel[i], v_target, dt)
            step = self.vel[i] * dt
            if abs(step) >= abs(e) and (e == 0 or step * e >= 0):   # ก้าวนี้ถึง/เลยเป้า → หยุดที่เป้า
                self.q[i], self.vel[i] = self._goal[i], 0.0
            else:
                self.q[i] += step
                done = False
        self.q = clamp_q(self.q)
        if done:
            self.mode = None
