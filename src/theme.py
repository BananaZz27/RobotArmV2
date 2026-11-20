# หน้าตาหน้าต่าง: สี ฟอนต์ และ stylesheet ที่ใช้ร่วมกัน (qt_app.py)
#   การ์ดสีขาวบนพื้นเทาอ่อน · ปุ่มแบ่งตามหน้าที่ด้วย property "kind" · ป้ายแบ่งด้วย property "role"
from PyQt6.QtGui import QColor, QFont, QPalette
from PyQt6.QtWidgets import QFrame, QHBoxLayout, QLabel, QVBoxLayout

FONT_UI = "Leelawadee UI"          # มีทั้งอักษรไทยและละติน (Windows) — Tahoma/Consolas ไม่มีไทยครบ → ตัวเล็กผิดปกติ
FONT_MONO = ["Consolas", FONT_UI]  # ตัวเลขใช้ Consolas, อักษรไทยตกไปใช้ Leelawadee

BG, CARD, LINE = "#f3f5f8", "#ffffff", "#e2e6ec"
TEXT, MUTED = "#1f2933", "#6b7685"
ACCENT, OK, WARN, BAD = "#2563eb", "#2a9d8f", "#f4a261", "#e63946"
TARGET, ACTUAL = "#ff8c42", "#3a86ff"   # สีเดียวกับภาพ 3D: ส้ม = Target, ฟ้า = ท่าจริง

QSS = f"""
QMainWindow, QDialog {{ background:{BG}; }}
QWidget {{ color:{TEXT}; }}
QToolTip {{ background:#1f2933; color:white; border:none; padding:6px; border-radius:4px; }}

QFrame#card {{ background:{CARD}; border:1px solid {LINE}; border-radius:10px; }}
QFrame#header {{ background:{CARD}; border:1px solid {LINE}; border-radius:10px; }}
QLabel {{ background:transparent; }}
QLabel[role="title"] {{ font-size:11pt; font-weight:600; color:{TEXT}; }}
QLabel[role="app"] {{ font-size:14pt; font-weight:700; color:{TEXT}; }}
QLabel[role="hint"] {{ color:{MUTED}; font-size:9pt; }}
QLabel[role="name"] {{ font-weight:600; }}
QLabel[role="range"] {{ color:{MUTED}; font-size:8pt; }}
QLabel[role="tile-name"] {{ color:{MUTED}; font-size:9pt; }}
QLabel[role="tile-value"] {{ font-size:15pt; font-weight:600; color:{TEXT}; }}
QLabel[role="actual"] {{ color:{ACTUAL}; font-weight:600; background:#eaf1ff; border-radius:6px; padding:3px 8px; }}

QPushButton {{ background:{CARD}; border:1px solid #cfd5de; border-radius:7px; padding:6px 14px; }}
QPushButton:hover {{ background:#f0f4fa; border-color:#9fb0c7; }}
QPushButton:pressed {{ background:#e1e8f2; }}
QPushButton[kind="primary"] {{ background:{ACCENT}; color:white; border:1px solid {ACCENT}; font-weight:600; }}
QPushButton[kind="primary"]:hover {{ background:#1d4fd7; }}
QPushButton[kind="danger"] {{ background:{BAD}; color:white; border:1px solid #c92f3c; font-weight:700;
                             font-size:11pt; padding:8px 22px; }}
QPushButton[kind="danger"]:hover {{ background:#d42f3d; }}
QPushButton[kind="ghost"] {{ background:transparent; border:1px dashed #b8c1cd; color:{MUTED}; }}
QPushButton[kind="jog"] {{ font-size:13pt; font-weight:700; min-width:64px; min-height:34px; padding:2px; }}
QPushButton[kind="jog"]:pressed {{ background:{ACCENT}; color:white; border-color:{ACCENT}; }}

QTabWidget::pane {{ border:none; background:transparent; top:-1px; }}
QTabBar::tab {{ background:transparent; color:{MUTED}; padding:8px 16px; margin-right:2px;
               border-bottom:3px solid transparent; font-weight:600; }}
QTabBar::tab:selected {{ color:{ACCENT}; border-bottom:3px solid {ACCENT}; }}
QTabBar::tab:hover:!selected {{ color:{TEXT}; }}

QDoubleSpinBox {{ min-height:26px; }}

QSlider::groove:horizontal {{ height:6px; background:#e4e8ee; border-radius:3px; }}
QSlider::sub-page:horizontal {{ background:{TARGET}; border-radius:3px; }}
QSlider::handle:horizontal {{ background:white; border:2px solid {TARGET}; width:14px; height:14px;
                             margin:-6px 0; border-radius:9px; }}
QSlider::handle:horizontal:hover {{ background:#fff3ea; }}

QTableWidget {{ background:{CARD}; border:1px solid {LINE}; border-radius:6px; gridline-color:#eef1f5;
               alternate-background-color:#f8fafc; selection-background-color:#dbe7ff; selection-color:{TEXT}; }}
QHeaderView::section {{ background:#f1f4f8; color:{MUTED}; border:none; border-bottom:1px solid {LINE};
                        padding:5px; font-weight:600; }}
QTextEdit {{ background:{CARD}; border:1px solid {LINE}; border-radius:6px; padding:4px; }}
QPlainTextEdit#log {{ background:#1f2430; color:#d8dee9; border:none; border-radius:8px; padding:6px;
                      selection-background-color:#3b4252; }}
QSplitter::handle {{ background:transparent; }}
QSplitter::handle:hover {{ background:#d5dce6; }}
QCheckBox {{ spacing:8px; }}
QScrollBar:vertical {{ background:transparent; width:10px; margin:2px; }}
QScrollBar::handle:vertical {{ background:#c5ccd6; border-radius:4px; min-height:24px; }}
QScrollBar::add-line, QScrollBar::sub-line {{ height:0; }}
"""


def apply(app):
    """ใช้ธีมกับทั้งโปรแกรม (เรียกครั้งเดียวหลังสร้าง QApplication)"""
    app.setStyle("Fusion")                      # หน้าตาเหมือนกันทุกเครื่อง + รับ stylesheet ได้ครบ
    app.setFont(QFont(FONT_UI, 10))
    pal = app.palette()
    pal.setColor(QPalette.ColorRole.Window, QColor(BG))
    pal.setColor(QPalette.ColorRole.Base, QColor(CARD))
    app.setPalette(pal)
    app.setStyleSheet(QSS)


def mono(size=10, bold=False):
    f = QFont()
    f.setFamilies(FONT_MONO)
    f.setPointSize(size)
    f.setBold(bold)
    return f


def label(text="", role=None, **kw):
    lbl = QLabel(text, **kw)
    if role:
        lbl.setProperty("role", role)
    return lbl


def button(text, kind=None, tip=""):
    from PyQt6.QtWidgets import QPushButton
    b = QPushButton(text)
    if kind:
        b.setProperty("kind", kind)
    if tip:
        b.setToolTip(tip)
    return b


def card(title=None, hint=None, name="card"):
    """กรอบการ์ดสีขาว → (frame, layout ข้างใน) · title/hint = หัวการ์ด (ไม่ใส่ก็ได้)"""
    frame = QFrame()
    frame.setObjectName(name)
    lay = QVBoxLayout(frame)
    lay.setContentsMargins(14, 12, 14, 12)
    lay.setSpacing(8)
    if title:
        head = QHBoxLayout()
        head.addWidget(label(title, "title"))
        head.addStretch(1)
        if hint:
            head.addWidget(label(hint, "hint"))
        lay.addLayout(head)
    return frame, lay


def pill(color):
    """stylesheet ป้ายสถานะแบบเม็ดยา (ตัวหนังสือสีเดียวกับพื้นจาง ๆ)"""
    c = QColor(color)
    return (f"color:{c.name()}; background:rgba({c.red()},{c.green()},{c.blue()},30); border-radius:11px; "
            "padding:3px 12px; font-weight:600")
