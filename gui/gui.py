import sys
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel,
    QHBoxLayout, QVBoxLayout, QSizePolicy, QFrame
)
from PyQt5.QtGui import (
    QPainter, QColor, QPen, QFont, QLinearGradient
)
from PyQt5.QtCore import Qt, QTimer, QRect


def set_global_font():
    font = QFont()
    font.setFamily("Gmarket Sans TTF")
    font.setBold(True)
    font.setWeight(QFont.Black)
    if font.family() == "":
        font.setFamily("Malgun Gothic")
        font.setBold(True)
        font.setWeight(QFont.Bold)
    font.setPointSize(12)
    QApplication.setFont(font)


class GridBackgroundWidget(QWidget):
    """Y2K 핑크 그리드 배경"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WA_StyledBackground, False)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        W = self.width()
        H = self.height()
        painter.fillRect(self.rect(), QColor(252, 197, 230))
        grid_color = QColor(255, 230, 245)
        painter.setPen(QPen(grid_color, 1))
        grid_size = 40
        for x in range(0, W, grid_size):
            painter.drawLine(x, 0, x, H)
        for y in range(0, H, grid_size):
            painter.drawLine(0, y, W, y)
        super().paintEvent(event)


class PressureBarWidget(QWidget):
    """3채널 압력 막대 그래프 위젯 (P1/P2/P3 → 아기 위치 시각화)"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.p1 = 0
        self.p2 = 0
        self.p3 = 0
        self.setMinimumSize(900, 700)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_pressures(self, p1, p2, p3):
        self.p1 = int(p1) & 0xFF
        self.p2 = int(p2) & 0xFF
        self.p3 = int(p3) & 0xFF
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        W, H = self.width(), self.height()
        margin = 40
        win_x = margin
        win_y = margin
        win_w = W - margin * 2
        win_h = H - margin * 2

        painter.setPen(QPen(QColor(40, 40, 60), 3))
        painter.setBrush(QColor(192, 243, 255))
        painter.drawRoundedRect(win_x, win_y, win_w, win_h, 24, 24)

        title_bar_h = 40
        title_bar_rect = QRect(win_x, win_y, win_w, title_bar_h)
        painter.setBrush(QColor(140, 224, 255))
        painter.drawRoundedRect(title_bar_rect, 24, 24)
        painter.drawRect(win_x, win_y + title_bar_h // 2, win_w, title_bar_h // 2)

        btn_r = 9
        btn_y = win_y + title_bar_h // 2
        btn_x_start = win_x + 20
        colors = [QColor(255, 103, 135), QColor(255, 207, 72), QColor(102, 224, 102)]
        for i, c in enumerate(colors):
            painter.setBrush(c)
            painter.setPen(QPen(QColor(40, 40, 60), 2))
            cx = btn_x_start + i * 26
            painter.drawEllipse(cx - btn_r, btn_y - btn_r, btn_r * 2, btn_r * 2)

        painter.setPen(QPen(QColor(40, 40, 60), 2))
        painter.setFont(QFont("Gmarket Sans TTF", 16, QFont.Bold))
        painter.drawText(
            win_x + 120, win_y, win_w - 200, title_bar_h,
            Qt.AlignVCenter | Qt.AlignLeft,
            "Pressure Monitor"
        )

        gx = win_x + 70
        gy = win_y + title_bar_h + 40
        gw = win_w - 70 - 40
        gh = win_h - (title_bar_h + 40) - 80

        painter.setPen(QPen(QColor(60, 60, 80), 2))
        painter.setBrush(QColor(250, 250, 255))
        painter.drawRoundedRect(gx, gy, gw, gh, 20, 20)

        painter.setPen(QPen(QColor(220, 220, 240), 1))
        for val in [0, 128, 255]:
            y = gy + gh - int((val / 255.0) * gh)
            painter.drawLine(gx + 10, y, gx + gw - 10, y)

        painter.setPen(QPen(QColor(100, 90, 120), 2))
        painter.setFont(QFont("Gmarket Sans TTF", 13, QFont.Bold))
        painter.drawText(win_x + 15, gy + gh + 5, "0")
        painter.drawText(win_x + 15, gy + gh - int(0.5 * gh) + 5, "128")
        painter.drawText(win_x + 15, gy + 15, "255")

        vals = [self.p1, self.p2, self.p3]
        labels = ["P1", "P2", "P3"]
        bar_w = int(gw * 0.18)
        gap = int(gw * 0.10)
        start_x = gx + int((gw - (bar_w * 3 + gap * 2)) / 2)

        for i in range(3):
            v = max(0, min(255, vals[i]))
            bh = int((v / 255.0) * gh)
            bx = start_x + i * (bar_w + gap)
            by = gy + (gh - bh)

            painter.setPen(QPen(QColor(90, 70, 120), 2))
            painter.setBrush(QColor(245, 240, 255))
            painter.drawRoundedRect(bx, gy + 8, bar_w, gh - 16, 18, 18)

            if v > 0:
                painter.setPen(Qt.NoPen)
                grad = QLinearGradient(bx, by, bx, by + bh)
                if v < 85:
                    grad.setColorAt(0.0, QColor(196, 224, 255))
                    grad.setColorAt(1.0, QColor(146, 192, 252))
                elif v < 170:
                    grad.setColorAt(0.0, QColor(254, 222, 242))
                    grad.setColorAt(1.0, QColor(250, 186, 214))
                else:
                    grad.setColorAt(0.0, QColor(255, 210, 210))
                    grad.setColorAt(1.0, QColor(255, 120, 140))
                painter.setBrush(grad)
                painter.drawRoundedRect(bx + 4, by + 4, bar_w - 8, bh - 8, 14, 14)

            painter.setPen(QPen(QColor(60, 40, 80), 2))
            painter.setFont(QFont("Gmarket Sans TTF", 18, QFont.Bold))
            painter.drawText(bx, gy + gh + 50, bar_w, 30, Qt.AlignCenter, labels[i])
            painter.setFont(QFont("Gmarket Sans TTF", 16, QFont.Bold))
            painter.drawText(bx, gy + gh + 80, bar_w, 30, Qt.AlignCenter, str(v))


class BabyMonitorGUI(QMainWindow):
    """
    UART 8-byte packet receiver and visualizer.

    Packet structure (from STM32 master via UART 115200bps):
      [0] P1  - pressure sensor 1 (0~255)
      [1] P2  - pressure sensor 2 (0~255)
      [2] P3  - pressure sensor 3 (0~255)
      [3] HR  - heart rate BPM (0~240)
      [4] HR_ABN  - cardiac arrest flag (0/1)
      [5] FALL    - fall risk flag (0/1)
      [6] STAND   - bad posture flag (0/1)
      [7] CRYING  - crying flag (0/1)
    """
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Baby Monitor")

        self.current_hr = 0
        self.hr_filtered = 0.0
        self.hr_abnormal = False
        self.fall_detected = False
        self.stand_detected = False
        self.crying = False

        self.serial = None
        self.rx_buf = bytearray()

        central = GridBackgroundWidget()
        self.setCentralWidget(central)

        root = QHBoxLayout()
        central.setLayout(root)
        root.setContentsMargins(40, 40, 40, 40)
        root.setSpacing(40)

        self.bar_widget = PressureBarWidget()
        root.addWidget(self.bar_widget, stretch=3)

        right_container = QWidget()
        right_layout = QVBoxLayout(right_container)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(20)
        root.addWidget(right_container, stretch=2)

        # 타이틀 카드
        title_card = self.make_popup_card(bg_color="#ffe6a9")
        t_layout = QVBoxLayout(title_card)
        t_layout.setContentsMargins(20, 15, 20, 15)
        t_layout.setSpacing(5)
        title_label = QLabel("👶✨ Baby Safety Monitor ✨👶")
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setStyleSheet("QLabel { font-size: 30px; font-weight: 900; color: #4a3a6f; }")
        t_layout.addWidget(title_label)

        # HR 카드
        hr_card = self.make_popup_card(bg_color="#c8f3ff")
        hr_layout = QVBoxLayout(hr_card)
        hr_layout.setContentsMargins(25, 20, 25, 20)
        hr_layout.setSpacing(10)
        self.hr_label = QLabel("❤ 0 bpm")
        self.hr_label.setAlignment(Qt.AlignCenter)
        self.hr_label.setStyleSheet("QLabel { font-size: 70px; font-weight: 900; color: #ff4b7d; }")
        hr_layout.addWidget(self.hr_label)
        self.hr_abn_label = QLabel("")
        self.hr_abn_label.setAlignment(Qt.AlignCenter)
        self.hr_abn_label.setStyleSheet("QLabel { font-size: 26px; font-weight: 800; color: #c71334; }")
        hr_layout.addWidget(self.hr_abn_label)

        # 울음 카드
        cry_card = self.make_popup_card(bg_color="#ffd6f0")
        cry_layout = QVBoxLayout(cry_card)
        cry_layout.setContentsMargins(25, 15, 25, 15)
        cry_layout.setSpacing(10)
        self.cry_label = QLabel("")
        self.cry_label.setAlignment(Qt.AlignCenter)
        self.cry_label.setStyleSheet("QLabel { font-size: 34px; font-weight: 800; color: #444; }")
        cry_layout.addWidget(self.cry_label)

        # 경고 카드
        alert_card = self.make_popup_card(bg_color="#fff1c7")
        alert_layout = QVBoxLayout(alert_card)
        alert_layout.setContentsMargins(25, 20, 25, 20)
        alert_layout.setSpacing(10)
        self.alert_label = QLabel("")
        self.alert_label.setAlignment(Qt.AlignCenter)
        self.alert_label.setStyleSheet("QLabel { font-size: 40px; font-weight: 900; color: #ff4b4b; }")
        alert_layout.addWidget(self.alert_label)

        # 시리얼 상태 카드
        status_card = self.make_popup_card(bg_color="#e1d9ff")
        status_layout = QVBoxLayout(status_card)
        status_layout.setContentsMargins(20, 10, 20, 10)
        self.status_label = QLabel("Serial: not connected")
        self.status_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.status_label.setStyleSheet("QLabel { font-size: 18px; font-weight: 600; color: #555; }")
        status_layout.addWidget(self.status_label)

        right_layout.addWidget(title_card, stretch=1)
        right_layout.addWidget(hr_card, stretch=3)
        right_layout.addWidget(cry_card, stretch=2)
        right_layout.addWidget(alert_card, stretch=2)
        right_layout.addWidget(status_card, stretch=1)

        self.serial_timer = QTimer(self)
        self.serial_timer.timeout.connect(self.read_serial)
        self.serial_timer.start(20)   # 50 Hz

        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self.refresh_ui)
        self.ui_timer.start(100)      # 10 Hz

        self.try_open_serial()

    def make_popup_card(self, bg_color="#ffffff"):
        frame = QFrame()
        frame.setStyleSheet(f"""
            QFrame {{
                background-color: {bg_color};
                border-radius: 18px;
                border: 3px solid #2d2344;
            }}
        """)
        return frame

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Escape:
            self.close()
        else:
            super().keyPressEvent(event)

    def try_open_serial(self):
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            try:
                self.serial = serial.Serial(p.device, 115200, timeout=0.0)
                self.status_label.setText(f"Serial: connected {p.device}")
                return
            except Exception:
                pass
        self.status_label.setText("Serial: not connected")

    def read_serial(self):
        if not self.serial:
            return
        try:
            n = self.serial.in_waiting
            if n <= 0:
                return
            self.rx_buf += self.serial.read(n)
            frame_len = 8
            while len(self.rx_buf) >= frame_len:
                frame = bytes(self.rx_buf[:frame_len])
                del self.rx_buf[:frame_len]
                self.parse_packet(frame)
        except Exception as e:
            self.status_label.setText(f"Serial error: {e}")

    def parse_packet(self, packet: bytes):
        if len(packet) != 8:
            return
        p1, p2, p3 = packet[0], packet[1], packet[2]
        hr = packet[3]
        hr_abn = packet[4]
        fall = packet[5]
        stand = packet[6]
        crying = packet[7]

        self.bar_widget.set_pressures(p1, p2, p3)

        alpha = 0.25
        self.hr_filtered = (1 - alpha) * self.hr_filtered + alpha * hr
        self.current_hr = int(self.hr_filtered + 0.5)

        self.hr_abnormal = bool(hr_abn)
        self.fall_detected = bool(fall)
        self.stand_detected = bool(stand)
        self.crying = bool(crying)

    def refresh_ui(self):
        self.hr_label.setText(f"❤ {self.current_hr} bpm")
        self.hr_abn_label.setText("심박수 이상 감지" if self.hr_abnormal else "")
        self.cry_label.setText("애기 웁니다 🍼" if self.crying else "")

        text = ""
        if self.hr_abnormal:
            text = "⚠ 심박수 이상!"
        elif self.fall_detected:
            text = "⚠ 낙상 주의!"
        elif self.stand_detected:
            text = "⚠ 자세 이상!"
        self.alert_label.setText(text)

        if text:
            self.alert_label.setStyleSheet("""
                QLabel {
                    font-size: 40px; font-weight: 900; color: #ffffff;
                    background-color: #ff6b6b;
                    border-radius: 14px; padding: 10px 8px;
                }
            """)
        else:
            self.alert_label.setStyleSheet("""
                QLabel {
                    font-size: 40px; font-weight: 900; color: #ff4b4b;
                    background-color: transparent;
                }
            """)


if __name__ == "__main__":
    set_global_font()
    app = QApplication(sys.argv)
    win = BabyMonitorGUI()
    win.showFullScreen()
    sys.exit(app.exec_())
