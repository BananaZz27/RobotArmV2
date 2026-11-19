# พิสูจน์สูตร |det J| = r·a2·a3·|sin θ3| กับ Jacobian เชิงตัวเลข + ตรวจคำเตือน
import random

import config
import pytest
from singularity import analyze, report


def test_formula_matches_numeric_jacobian_random_poses():
    rnd = random.Random(1)
    for _ in range(300):
        q = [rnd.uniform(config.Q_MIN[i], config.Q_MAX[i]) for i in range(4)] + [0.0]
        a = analyze(q)
        assert a["det_numeric"] == pytest.approx(a["det_formula"], rel=1e-4, abs=1.0)


def test_straight_up_is_singular_both_ways():
    a = analyze([0, 0, 0, 0, 0])          # ชี้ฟ้า: r = 0 และ θ3 = 0
    assert a["det_formula"] == pytest.approx(0, abs=1e-6) and a["det_numeric"] < 1.0
    assert len(a["warnings"]) == 2


def test_stretched_forward_elbow_singular():
    a = analyze([0, 45, 0, 0, 0])          # เอนไปหน้า แต่ศอกเหยียดตรง
    assert a["r"] > 100 and a["sigma_min"] < 1.0
    assert any("เหยียด" in w for w in a["warnings"])


def test_normal_pose_safe():
    a = analyze([10, 30, 50, 20, 0])
    assert a["warnings"] == [] and a["det_numeric"] > 1e5
    assert "ไม่ใกล้" in report([10, 30, 50, 20, 0])


def test_tip_over_base_axis():
    a = analyze([0, -30, 120, 0, 0])       # พับกลับให้ปลายแขนเกือบอยู่เหนือฐาน
    assert any("แกนฐาน" in w for w in a["warnings"]) == (a["r"] < 20)
