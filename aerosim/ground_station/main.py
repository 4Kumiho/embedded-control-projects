"""
main.py — Ground Control Station (GCS) per AeroSim.

Layout:
    ┌──────────────────────────────────────────────┐
    │              AeroSim Ground Station           │
    ├───────────────────┬──────────────────────────┤
    │  Artificial       │  Altitude graph          │
    │  Horizon          │  (tempo reale)           │
    │                   ├──────────────────────────┤
    │                   │  GPS Map                 │
    │                   │  (posizione relativa)    │
    ├───────────────────┴──────────────────────────┤
    │  Status: ARM | Uptime | Link | Errors        │
    └──────────────────────────────────────────────┘

Dipendenze:
    pip install PyQt6 matplotlib numpy
"""

import sys
import math
import collections
from pathlib import Path

import numpy as np
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QHBoxLayout,
    QVBoxLayout, QGroupBox, QLabel, QStatusBar, QPushButton
)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QPainter, QColor, QPen, QBrush, QFont, QPolygonF
from PyQt6.QtCore import QPointF

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from matplotlib.figure import Figure

from parser import TelemetryFileReader, IMUPacket, GPSPacket, StatusPacket

# ------------------------------------------------------------------ #
#  Configurazione                                                    #
# ------------------------------------------------------------------ #

TELEM_FILE   = "../build/telemetry.bin"  # percorso relativo al firmware
UPDATE_MS    = 100                        # periodo aggiornamento GUI [ms]
HISTORY_SIZE = 200                        # numero di campioni nei grafici


# ================================================================== #
#  Widget: Artificial Horizon                                        #
# ================================================================== #

class ArtificialHorizon(QWidget):
    """
    Indicatore di assetto artificiale (attitude indicator).

    Disegna con QPainter:
    - Sfondo bipartito (cielo blu / terra marrone) che ruota con roll
    - Offset verticale proporzionale al pitch
    - Aereo fisso al centro
    - Scala di rollio esterna
    - Testo roll/pitch in gradi
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.roll  = 0.0   # gradi
        self.pitch = 0.0   # gradi
        self.setMinimumSize(260, 260)

    def set_attitude(self, roll_deg: float, pitch_deg: float) -> None:
        self.roll  = roll_deg
        self.pitch = pitch_deg
        self.update()   # richiede ridisegno (schedula paintEvent)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        w, h   = self.width(), self.height()
        cx, cy = w / 2, h / 2
        radius = min(w, h) / 2 - 8

        # -- Clip circolare --
        clip_path = __import__('PyQt6.QtGui', fromlist=['QPainterPath']).QPainterPath()
        clip_path.addEllipse(QPointF(cx, cy), radius, radius)
        painter.setClipPath(clip_path)

        # -- Sfondo: ruota tutto di -roll, traslazione verticale per pitch --
        painter.save()
        painter.translate(cx, cy)
        painter.rotate(-self.roll)

        # Offset pitch: 2 pixel per grado (range utile ±30°)
        pitch_offset = self.pitch * 2.0
        pitch_offset = max(-radius, min(radius, pitch_offset))

        # Cielo (blu)
        painter.setBrush(QBrush(QColor(70, 130, 180)))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawRect(int(-radius), int(-radius * 2 + pitch_offset),
                         int(radius * 2), int(radius * 2))

        # Terra (marrone)
        painter.setBrush(QBrush(QColor(139, 90, 43)))
        painter.drawRect(int(-radius), int(pitch_offset),
                         int(radius * 2), int(radius * 2))

        # Linea orizzonte
        painter.setPen(QPen(QColor(255, 255, 255), 2))
        painter.drawLine(int(-radius), int(pitch_offset),
                         int(radius),  int(pitch_offset))

        # Linee di pitch (ogni 10°)
        painter.setPen(QPen(QColor(255, 255, 255), 1))
        font = QFont("Arial", 7)
        painter.setFont(font)
        for deg in range(-30, 31, 10):
            if deg == 0:
                continue
            y = pitch_offset - deg * 2.0
            line_w = radius * 0.3 if abs(deg) % 20 == 0 else radius * 0.15
            painter.drawLine(int(-line_w), int(y), int(line_w), int(y))
            painter.drawText(int(line_w + 3), int(y + 4), f"{deg}")

        painter.restore()

        # -- Aereo fisso (non ruota con l'attitude) --
        painter.translate(cx, cy)
        painter.setPen(QPen(QColor(255, 200, 0), 3))
        painter.drawLine(-30, 0, -10, 0)   # ala sinistra
        painter.drawLine( 10, 0,  30, 0)   # ala destra
        painter.drawLine(-5,  0,   5, 0)   # fusoliera centrale
        painter.drawLine(0,  -8,   0,  2)  # verticale

        # -- Scala di rollio esterna --
        painter.setPen(QPen(QColor(255, 255, 255), 1))
        for angle in [-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60]:
            a_rad = math.radians(angle - 90)
            x1 = radius * math.cos(a_rad)
            y1 = radius * math.sin(a_rad)
            tick = 8 if angle % 30 == 0 else 4
            x2 = (radius - tick) * math.cos(a_rad)
            y2 = (radius - tick) * math.sin(a_rad)
            painter.drawLine(int(x1), int(y1), int(x2), int(y2))

        # Puntatore roll corrente (triangolo)
        roll_rad = math.radians(-self.roll - 90)
        px = (radius - 2) * math.cos(roll_rad)
        py = (radius - 2) * math.sin(roll_rad)
        painter.setBrush(QBrush(QColor(255, 200, 0)))
        painter.setPen(Qt.PenStyle.NoPen)
        tri = QPolygonF([
            QPointF(px, py),
            QPointF(px + 5 * math.cos(roll_rad + math.pi/2),
                    py + 5 * math.sin(roll_rad + math.pi/2)),
            QPointF(px + 5 * math.cos(roll_rad - math.pi/2),
                    py + 5 * math.sin(roll_rad - math.pi/2)),
        ])
        painter.drawPolygon(tri)

        # -- Testo roll/pitch --
        painter.setPen(QPen(QColor(255, 255, 255)))
        painter.setFont(QFont("Arial", 8, QFont.Weight.Bold))
        painter.drawText(int(-radius + 4), int(radius - 18),
                         f"R: {self.roll:+.1f}°")
        painter.drawText(int(-radius + 4), int(radius - 6),
                         f"P: {self.pitch:+.1f}°")

        # -- Bordo circolare --
        painter.setClipping(False)
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.setPen(QPen(QColor(80, 80, 80), 3))
        painter.drawEllipse(QPointF(cx, cy), radius, radius)


# ================================================================== #
#  Widget: Altitude & Speed Graph (matplotlib)                       #
# ================================================================== #

class TelemetryGraph(FigureCanvasQTAgg):
    """
    Grafico real-time con due subplot:
    - Quota [m] nel tempo
    - Velocità verticale (az − g) nel tempo
    """

    def __init__(self):
        fig = Figure(figsize=(5, 3), facecolor='#1e1e1e')
        super().__init__(fig)

        self._t   = collections.deque(maxlen=HISTORY_SIZE)
        self._alt = collections.deque(maxlen=HISTORY_SIZE)
        self._az  = collections.deque(maxlen=HISTORY_SIZE)

        ax1, ax2 = fig.subplots(2, 1, sharex=True)
        self._ax1, self._ax2 = ax1, ax2
        fig.tight_layout(pad=1.5)

        for ax in (ax1, ax2):
            ax.set_facecolor('#2d2d2d')
            ax.tick_params(colors='white', labelsize=7)
            ax.spines[:].set_color('#555')
            ax.yaxis.label.set_color('white')

        ax1.set_ylabel('Quota [m]', fontsize=8, color='white')
        ax2.set_ylabel('Az [m/s²]', fontsize=8, color='white')

        self._line_alt, = ax1.plot([], [], color='#4fc3f7', lw=1.5)
        self._line_az,  = ax2.plot([], [], color='#a5d6a7', lw=1.5)

    def append(self, t_s: float, alt: float, az: float) -> None:
        self._t.append(t_s)
        self._alt.append(alt)
        self._az.append(az)

    def refresh(self) -> None:
        if not self._t:
            return
        t   = list(self._t)
        alt = list(self._alt)
        az  = list(self._az)

        self._line_alt.set_data(t, alt)
        self._line_az.set_data(t, az)

        self._ax1.relim(); self._ax1.autoscale_view()
        self._ax2.relim(); self._ax2.autoscale_view()
        self.draw()


# ================================================================== #
#  Widget: GPS Map                                                   #
# ================================================================== #

class GPSMap(FigureCanvasQTAgg):
    """
    Mappa 2D della posizione GPS relativa al punto di partenza.
    Mostra la traiettoria del drone in piano (Nord-Est).
    """

    def __init__(self):
        fig = Figure(figsize=(5, 2.5), facecolor='#1e1e1e')
        super().__init__(fig)

        self._ax = fig.add_subplot(111)
        self._ax.set_facecolor('#1a2635')
        self._ax.set_xlabel('Est [m]',  color='white', fontsize=8)
        self._ax.set_ylabel('Nord [m]', color='white', fontsize=8)
        self._ax.set_title('GPS Track', color='white', fontsize=9)
        self._ax.tick_params(colors='white', labelsize=7)
        for s in self._ax.spines.values(): s.set_color('#555')
        fig.tight_layout(pad=1.0)

        self._east  = collections.deque(maxlen=HISTORY_SIZE)
        self._north = collections.deque(maxlen=HISTORY_SIZE)
        self._lat0 = None
        self._lon0 = None

        self._track,  = self._ax.plot([], [], color='#4fc3f7', lw=1)
        self._marker, = self._ax.plot([], [], 'o', color='#ff5722',
                                      markersize=7, zorder=5)
        self._start,  = self._ax.plot([0], [0], '*', color='#ffeb3b',
                                      markersize=10, zorder=6, label='Start')

    def append(self, lat: float, lon: float) -> None:
        # Primo punto = origine della mappa
        if self._lat0 is None:
            self._lat0, self._lon0 = lat, lon

        # Conversione gradi → metri (approssimazione piana)
        # 1° lat ≈ 111320 m,  1° lon ≈ 111320 * cos(lat) m
        north = (lat - self._lat0) * 111_320.0
        east  = (lon - self._lon0) * 111_320.0 * math.cos(math.radians(lat))

        self._north.append(north)
        self._east.append(east)

    def refresh(self) -> None:
        if not self._east:
            return
        e = list(self._east)
        n = list(self._north)

        self._track.set_data(e, n)
        self._marker.set_data([e[-1]], [n[-1]])

        self._ax.relim(); self._ax.autoscale_view()
        margin = max(1.0, max(abs(v) for v in e + n) * 0.15)
        self._ax.set_xlim(min(e) - margin, max(e) + margin)
        self._ax.set_ylim(min(n) - margin, max(n) + margin)
        self.draw()


# ================================================================== #
#  MainWindow                                                        #
# ================================================================== #

class MainWindow(QMainWindow):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("AeroSim — Ground Control Station")
        self.setMinimumSize(900, 600)
        self.setStyleSheet("background-color: #1e1e1e; color: white;")

        self._reader  = TelemetryFileReader(TELEM_FILE)
        self._t_sec   = 0.0
        self._last_imu    = None
        self._last_gps    = None
        self._last_status = None

        self._build_ui()

        # Timer aggiornamento GUI
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._update)
        self._timer.start(UPDATE_MS)

    # ----------------------------------------------------------------
    #  Build UI
    # ----------------------------------------------------------------

    def _build_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)
        root.setSpacing(8)

        # ── Pannello sinistro ────────────────────────────────────────
        left = QVBoxLayout()
        left.setSpacing(8)

        horizon_box = QGroupBox("Assetto")
        horizon_box.setStyleSheet(self._box_style())
        horizon_layout = QVBoxLayout(horizon_box)
        self._horizon = ArtificialHorizon()
        horizon_layout.addWidget(self._horizon)
        left.addWidget(horizon_box)

        status_box = QGroupBox("Stato sistema")
        status_box.setStyleSheet(self._box_style())
        status_layout = QVBoxLayout(status_box)
        self._lbl_arm    = self._make_label("ARM:     —")
        self._lbl_uptime = self._make_label("Uptime:  —")
        self._lbl_errors = self._make_label("Errori:  —")
        self._lbl_link   = self._make_label("Link:    attesa dati...")
        for lbl in (self._lbl_arm, self._lbl_uptime,
                    self._lbl_errors, self._lbl_link):
            status_layout.addWidget(lbl)
        left.addWidget(status_box)

        btn_reset = QPushButton("Reset file")
        btn_reset.clicked.connect(self._reset)
        btn_reset.setStyleSheet(
            "background:#333; color:white; border:1px solid #555;"
            "padding:4px; border-radius:4px;"
        )
        left.addWidget(btn_reset)
        left.addStretch()

        # ── Pannello destro ──────────────────────────────────────────
        right = QVBoxLayout()
        right.setSpacing(8)

        graph_box = QGroupBox("Telemetria real-time")
        graph_box.setStyleSheet(self._box_style())
        graph_layout = QVBoxLayout(graph_box)
        self._graph = TelemetryGraph()
        graph_layout.addWidget(self._graph)
        right.addWidget(graph_box, stretch=2)

        map_box = QGroupBox("GPS Track")
        map_box.setStyleSheet(self._box_style())
        map_layout = QVBoxLayout(map_box)
        self._map = GPSMap()
        map_layout.addWidget(self._map)
        right.addWidget(map_box, stretch=1)

        root.addLayout(left,  stretch=1)
        root.addLayout(right, stretch=2)

        # Status bar
        self._status_bar = QStatusBar()
        self._status_bar.setStyleSheet("color: #aaa; font-size: 11px;")
        self.setStatusBar(self._status_bar)
        self._status_bar.showMessage("In attesa di telemetry.bin...")

    # ----------------------------------------------------------------
    #  Update loop (chiamato ogni UPDATE_MS)
    # ----------------------------------------------------------------

    def _update(self) -> None:
        packets = self._reader.read_new_packets()
        if not packets:
            return

        for pkt in packets:
            if isinstance(pkt, IMUPacket):
                self._last_imu = pkt
                # Roll e pitch stimati dall'accelerometro (attitude statico)
                roll  = math.degrees(math.atan2(pkt.ay, pkt.az))
                pitch = math.degrees(math.atan2(-pkt.ax,
                                    math.sqrt(pkt.ay**2 + pkt.az**2)))
                self._horizon.set_attitude(roll, pitch)
                self._graph.append(self._t_sec, 0.0, pkt.az)
                self._t_sec += UPDATE_MS / 1000.0

            elif isinstance(pkt, GPSPacket):
                self._last_gps = pkt
                # Aggiorna grafico quota con valore GPS (più affidabile)
                if self._t_sec > 0:
                    # Aggiorna l'ultimo punto altitudine con GPS
                    pass
                self._graph._alt.append(pkt.alt) if self._graph._alt else None
                self._map.append(pkt.lat, pkt.lon)

            elif isinstance(pkt, StatusPacket):
                self._last_status = pkt
                arm_str = "ARMATO ✓" if pkt.arm_state else "disarmato"
                self._lbl_arm.setText(f"ARM:     {arm_str}")
                self._lbl_uptime.setText(
                    f"Uptime:  {pkt.uptime_ms / 1000:.1f} s")
                err_str = "nessuno" if pkt.error_flags == 0 \
                          else f"0x{pkt.error_flags:04X}"
                self._lbl_errors.setText(f"Errori:  {err_str}")

        # Aggiorna grafici e mappa
        self._graph.refresh()
        self._map.refresh()

        # Link status bar
        stats = self._reader.stats
        self._lbl_link.setText(
            f"Link:    {stats['received']} pkt  |  "
            f"{stats['bytes']} B  |  err: {stats['invalid']}"
        )
        self._status_bar.showMessage(
            f"Pacchetti ricevuti: {stats['received']}  |  "
            f"Dati: {stats['bytes']} byte  |  "
            f"File: {Path(TELEM_FILE).resolve()}"
        )

    def _reset(self) -> None:
        """Ricomincia lettura dal file dall'inizio."""
        self._reader.reset()
        self._t_sec = 0.0
        self._map._lat0 = None
        self._status_bar.showMessage("Reset effettuato.")

    # ----------------------------------------------------------------
    #  Helpers
    # ----------------------------------------------------------------

    @staticmethod
    def _make_label(text: str) -> QLabel:
        lbl = QLabel(text)
        lbl.setFont(QFont("Courier New", 10))
        lbl.setStyleSheet("color: #ccc; padding: 2px;")
        return lbl

    @staticmethod
    def _box_style() -> str:
        return (
            "QGroupBox { color: #aaa; border: 1px solid #444;"
            "border-radius: 6px; margin-top: 8px; font-size: 11px; }"
            "QGroupBox::title { subcontrol-origin: margin;"
            "subcontrol-position: top left; padding: 0 4px; }"
        )


# ================================================================== #
#  Entry point                                                       #
# ================================================================== #

def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
