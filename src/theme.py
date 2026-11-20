# หน้าตาหน้าต่าง: สี ฟอนต์ และ stylesheet ที่ใช้ร่วมกัน (qt_app.py, viewer3d.py)
#   ปุ่มแบ่งตามหน้าที่ด้วย property "kind" · ป้ายแบ่งด้วย property "role" · การ์ด = QFrame ชื่อ "card"
#   2 ธีม: "dark" (ค่าเริ่มต้น — พื้นเข้ม เรียบ ๆ สีฟ้าเฉพาะจุดสำคัญ) / "light" — เลือกด้วย env ROBOTARM_THEME=light
import os

from PyQt6.QtGui import QColor, QFont, QPalette
from PyQt6.QtWidgets import QFrame, QHBoxLayout, QLabel, QPushButton, QVBoxLayout

STYLE = os.environ.get("ROBOTARM_THEME", "dark")
DARK = STYLE != "light"

FONT_UI = "Leelawadee UI"                   # มีทั้งอักษรไทยและละติน (Windows) — Tahoma/Consolas ไม่มีไทยครบ → ตัวเล็กผิดปกติ
FONT_NUM = ["Bahnschrift", FONT_UI]         # ตัวเลขใหญ่ (ค่า FK): ตัวเหลี่ยมอ่านง่ายแบบหน้าปัด
FONT_MONO = ["Cascadia Mono", "Consolas", FONT_UI]

if DARK:
    BG, CARD, LINE, RAISED = "#0f141a", "#161d25", "#243040", "#1d2732"
    TEXT, MUTED = "#e3e9ef", "#8392a3"
    ACCENT, OK, WARN, BAD = "#4cc9f0", "#2ec4b6", "#f4a261", "#ef4444"
    TARGET, ACTUAL = "#ff8c42", "#4cc9f0"   # สีเดียวกับภาพ 3D: ส้ม = Target, ฟ้า = ท่าจริง
    # ภาพ 3D: พื้นหลัง (ล่าง, บน), ตารางพื้น, ฐาน, วงระยะเอื้อม (None = ไม่วาด)
    VIEW = dict(bg="#0f141a", top="#1b2633", grid="#2b3b4b", base="#3a4756", ring="#3d6a80")
else:
    BG, CARD, LINE, RAISED = "#f3f5f8", "#ffffff", "#e2e6ec", "#f0f4fa"
    TEXT, MUTED = "#1f2933", "#6b7685"
    ACCENT, OK, WARN, BAD = "#2563eb", "#2a9d8f", "#c77d14", "#e63946"
    TARGET, ACTUAL = "#ff8c42", "#3a86ff"
    VIEW = dict(bg="#f4f5f7", top="#dfe3ea", grid="#c9ced6", base="#5b6270", ring=None)


def rgba(color, alpha):
    c = QColor(color)
    return f"rgba({c.red()},{c.green()},{c.blue()},{alpha})"


ON_ACCENT = "#06222c" if DARK else "#ffffff"      # ตัวหนังสือบนปุ่มสี ACCENT
QSS = f"""
QMainWindow, QDialog {{ background:{BG}; }}
QWidget {{ color:{TEXT}; }}
QToolTip {{ background:{RAISED}; color:{TEXT}; border:1px solid {LINE}; padding:6px; }}
QFrame#card {{ background:{CARD}; border:1px solid {LINE}; border-radius:8px; }}
QLabel {{ background:transparent; }}
QLabel[role="title"] {{ font-size:11pt; font-weight:600; }}
QLabel[role="app"] {{ font-size:14pt; font-weight:700; }}
QLabel[role="hint"] {{ color:{MUTED}; font-size:9pt; }}
QLabel[role="name"] {{ font-weight:600; }}
QLabel[role="range"] {{ color:{MUTED}; font-size:8pt; }}
QLabel[role="tile-name"] {{ color:{MUTED}; font-size:9pt; }}
QLabel[role="tile-value"] {{ color:{ACCENT if DARK else TEXT}; }}
QLabel[role="actual"] {{ color:{ACTUAL}; background:{rgba(ACTUAL, 25)}; border-radius:5px; padding:3px 8px; }}

QPushButton {{ background:{RAISED}; border:1px solid {LINE}; border-radius:6px; padding:6px 14px; }}
QPushButton:hover {{ border-color:{MUTED}; }}
QPushButton:pressed {{ background:{rgba(ACCENT, 50)}; }}
QPushButton[kind="primary"] {{ background:{ACCENT}; color:{ON_ACCENT}; border:1px solid {ACCENT}; font-weight:600; }}
QPushButton[kind="primary"]:hover {{ background:{QColor(ACCENT).lighter(112).name()}; }}
QPushButton[kind="danger"] {{ background:{BAD}; color:white; border:1px solid {BAD}; font-weight:700; padding:7px 18px; }}
QPushButton[kind="danger"]:hover {{ background:{QColor(BAD).darker(112).name()}; }}
QPushButton:disabled, QPushButton[kind="primary"]:disabled {{ background:{RAISED}; color:{MUTED}; border:1px solid {LINE}; }}
QPushButton[kind="ghost"] {{ background:transparent; border:1px dashed {LINE}; color:{MUTED}; }}
QPushButton[kind="ghost"]:hover {{ color:{TEXT}; border-color:{MUTED}; }}
QPushButton[kind="jog"] {{ font-size:12pt; font-weight:700; min-width:56px; min-height:30px; padding:2px; }}
QPushButton[kind="jog"]:pressed {{ background:{ACCENT}; color:{ON_ACCENT}; border-color:{ACCENT}; }}

QTabWidget::pane {{ border:none; background:transparent; top:-1px; }}
QTabBar::tab {{ background:transparent; color:{MUTED}; padding:7px 14px; margin-right:2px;
               border-bottom:2px solid transparent; font-weight:600; }}
QTabBar::tab:selected {{ color:{ACCENT}; border-bottom:2px solid {ACCENT}; }}
QTabBar::tab:hover:!selected {{ color:{TEXT}; }}
QScrollArea {{ background:transparent; border:none; }}
QScrollArea > QWidget > QWidget {{ background:transparent; }}

QDoubleSpinBox {{ min-height:24px; }}
QSlider::groove:horizontal {{ height:4px; background:{LINE}; border-radius:2px; }}
QSlider::sub-page:horizontal {{ background:{TARGET}; border-radius:2px; }}
QSlider::handle:horizontal {{ background:{CARD}; border:2px solid {TARGET}; width:12px; height:12px;
                             margin:-6px 0; border-radius:8px; }}

QTableWidget {{ background:{CARD}; border:1px solid {LINE}; border-radius:6px; gridline-color:{LINE};
               alternate-background-color:{RAISED}; selection-background-color:{rgba(ACCENT, 60)};
               selection-color:{TEXT}; }}
QHeaderView::section {{ background:{RAISED}; color:{MUTED}; border:none; border-bottom:1px solid {LINE};
                        padding:5px; font-weight:600; }}
QTextEdit {{ background:{CARD}; border:1px solid {LINE}; border-radius:6px; padding:4px; }}
QPlainTextEdit#log {{ background:{"#0b1016" if DARK else "#1f2430"}; color:#c8d3df; border:none; border-radius:6px;
                      padding:6px; }}
QSplitter::handle {{ background:transparent; }}
QSplitter::handle:hover {{ background:{LINE}; }}
QCheckBox {{ spacing:8px; }}
QScrollBar:vertical {{ background:transparent; width:9px; margin:2px; }}
QScrollBar::handle:vertical {{ background:{LINE}; border-radius:4px; min-height:24px; }}
QScrollBar:horizontal {{ background:transparent; height:9px; margin:2px; }}
QScrollBar::handle:horizontal {{ background:{LINE}; border-radius:4px; min-width:24px; }}
QScrollBar::add-line, QScrollBar::sub-line {{ width:0; height:0; }}
"""


def apply(app):
    """ใช้ธีมกับทั้งโปรแกรม (เรียกครั้งเดียวหลังสร้าง QApplication)"""
    app.setStyle("Fusion")                      # หน้าตาเหมือนกันทุกเครื่อง + รับ stylesheet ได้ครบ
    app.setFont(QFont(FONT_UI, 10))
    pal = app.palette()                         # ช่องตัวเลข/checkbox วาดจาก palette (ไม่แต่ง QSS → ลูกศรขึ้นลงไม่หาย)
    for role, color in ((QPalette.ColorRole.Window, BG), (QPalette.ColorRole.WindowText, TEXT),
                        (QPalette.ColorRole.Base, "#0b1016" if DARK else CARD), (QPalette.ColorRole.Text, TEXT),
                        (QPalette.ColorRole.Button, RAISED), (QPalette.ColorRole.ButtonText, TEXT),
                        (QPalette.ColorRole.Highlight, ACCENT), (QPalette.ColorRole.HighlightedText, ON_ACCENT)):
        pal.setColor(role, QColor(color))
    app.setPalette(pal)
    app.setStyleSheet(QSS)


def _font(families, size, bold=False):
    f = QFont()
    f.setFamilies(families)
    f.setPointSize(size)
    f.setBold(bold)
    return f


def mono(size=10, bold=False):
    return _font(FONT_MONO, size, bold)


def label(text="", role=None, **kw):
    lbl = QLabel(text, **kw)
    if role:
        lbl.setProperty("role", role)
    if role == "tile-value":
        lbl.setFont(_font(FONT_NUM, 15, True))
    return lbl


def button(text, kind=None, tip=""):
    b = QPushButton(text)
    if kind:
        b.setProperty("kind", kind)
    if tip:
        b.setToolTip(tip)
    return b


def card(title=None, hint=None):
    """การ์ด → (frame, layout ข้างใน) · title/hint = หัวการ์ด (ไม่ใส่ก็ได้)"""
    frame = QFrame()
    frame.setObjectName("card")
    lay = QVBoxLayout(frame)
    lay.setContentsMargins(12, 10, 12, 10)
    lay.setSpacing(8)
    if title:
        head = QHBoxLayout()
        head.addWidget(label(title, "title"))
        head.addStretch(1)
        if hint:
            h = label(hint, "hint")
            h.setWordWrap(True)                  # หน้าต่างแคบ → ขึ้นบรรทัดใหม่ ไม่ดันการ์ดให้กว้าง
            head.addWidget(h, 1)
        lay.addLayout(head)
    return frame, lay


def pill(color):
    """stylesheet ป้ายสถานะ (ตัวหนังสือสีเดียวกับพื้นจาง ๆ)"""
    c = QColor(color).name()
    return f"color:{c}; background:{rgba(c, 35)}; border-radius:10px; padding:3px 12px; font-weight:600"
