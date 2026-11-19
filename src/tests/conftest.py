# ให้ test import ไฟล์ใน v2/pc ได้ตรง ๆ (import kinematics, import config)
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
