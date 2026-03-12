"""
parser.py — Parser del protocollo binario di telemetria AeroSim.

Protocollo (definito in docs/SRS.md sezione 5):

    ┌────────┬────────┬──────────┬──────────────────────┬──────────┐
    │ START  │  TYPE  │  LENGTH  │       PAYLOAD         │ CHECKSUM │
    │  0xAE  │ 1 byte │  2 byte  │    0-255 bytes        │  1 byte  │
    └────────┴────────┴──────────┴──────────────────────┴──────────┘

    Checksum = XOR di tutti i byte del payload.

Tipi di pacchetto:
    0x01 — IMU:    ax, ay, az (float), gx, gy, gz (float)  = 24 byte
    0x02 — GPS:    lat, lon, alt (double), speed (float)    = 28 byte
    0x03 — STATUS: arm(uint8), errors(uint16), uptime(uint32) = 7 byte
"""

import struct
from dataclasses import dataclass, field
from typing import Optional, List, Union
from pathlib import Path

# ------------------------------------------------------------------ #
#  Costanti di protocollo                                              #
# ------------------------------------------------------------------ #

TELEM_START    = 0xAE
TELEM_TYPE_IMU    = 0x01
TELEM_TYPE_GPS    = 0x02
TELEM_TYPE_STATUS = 0x03
TELEM_HEADER_SIZE = 4   # START + TYPE + LENGTH(2)


# ------------------------------------------------------------------ #
#  Data classes — rappresentano i pacchetti decodificati              #
# ------------------------------------------------------------------ #

@dataclass
class IMUPacket:
    """Dati dall'Inertial Measurement Unit."""
    ax: float = 0.0   # accelerazione X [m/s²]
    ay: float = 0.0   # accelerazione Y [m/s²]
    az: float = 0.0   # accelerazione Z [m/s²] — ~9.81 a riposo
    gx: float = 0.0   # velocità angolare X [deg/s]
    gy: float = 0.0   # velocità angolare Y [deg/s]
    gz: float = 0.0   # velocità angolare Z [deg/s]


@dataclass
class GPSPacket:
    """Dati di posizione GPS."""
    lat:   float = 0.0   # latitudine  [gradi decimali]
    lon:   float = 0.0   # longitudine [gradi decimali]
    alt:   float = 0.0   # altitudine  [m]
    speed: float = 0.0   # velocità al suolo [m/s]


@dataclass
class StatusPacket:
    """Stato del sistema firmware."""
    arm_state:   int = 0   # 0=disarmato, 1=armato
    error_flags: int = 0   # bitmask errori
    uptime_ms:   int = 0   # millisecondi dall'avvio


# Tipo union per i pacchetti
Packet = Union[IMUPacket, GPSPacket, StatusPacket]


# ------------------------------------------------------------------ #
#  TelemetryParser — macchina a stati per il parsing dello stream     #
# ------------------------------------------------------------------ #

class TelemetryParser:
    """
    Parser a stream: accumula byte in un buffer interno e restituisce
    pacchetti completi e validi.

    Funziona come una macchina a stati:
        WAIT_START → WAIT_HEADER → WAIT_PAYLOAD → EMIT
    """

    def __init__(self):
        self._buf = bytearray()
        self.packets_received = 0
        self.packets_invalid  = 0

    def feed(self, data: bytes) -> None:
        """Aggiunge nuovi byte al buffer interno."""
        self._buf.extend(data)

    def parse_all(self) -> List[Packet]:
        """
        Estrae tutti i pacchetti completi dal buffer.
        Restituisce lista (può essere vuota).
        """
        packets = []
        while True:
            pkt = self._parse_one()
            if pkt is None:
                break
            packets.append(pkt)
        return packets

    def _parse_one(self) -> Optional[Packet]:
        """
        Tenta di estrarre un singolo pacchetto dal buffer.
        Restituisce None se non ci sono dati sufficienti.
        """
        # 1. Cerca il byte di start (sincronizzazione)
        while self._buf and self._buf[0] != TELEM_START:
            self._buf.pop(0)   # scarta byte non validi

        # 2. Header completo?
        if len(self._buf) < TELEM_HEADER_SIZE:
            return None

        # 3. Leggi lunghezza payload (little-endian, 2 byte)
        payload_len = struct.unpack_from('<H', self._buf, 2)[0]

        # 4. Pacchetto completo? (header + payload + checksum)
        total_len = TELEM_HEADER_SIZE + payload_len + 1
        if len(self._buf) < total_len:
            return None

        # 5. Estrai campi
        type_byte = self._buf[1]
        payload   = bytes(self._buf[TELEM_HEADER_SIZE : TELEM_HEADER_SIZE + payload_len])
        checksum  = self._buf[TELEM_HEADER_SIZE + payload_len]

        # 6. Valida checksum (XOR di tutti i byte del payload)
        computed = 0
        for b in payload:
            computed ^= b

        if computed != checksum:
            # Checksum non valido: salta il byte di start e riprova
            self._buf.pop(0)
            self.packets_invalid += 1
            return None

        # 7. Consuma il pacchetto dal buffer
        self._buf = self._buf[total_len:]
        self.packets_received += 1

        return self._decode(type_byte, payload)

    def _decode(self, type_byte: int, payload: bytes) -> Optional[Packet]:
        """Decodifica il payload in base al tipo di pacchetto."""
        try:
            if type_byte == TELEM_TYPE_IMU and len(payload) == 24:
                # 6 float little-endian (IEEE 754, 4 byte ciascuno)
                vals = struct.unpack('<6f', payload)
                return IMUPacket(*vals)

            elif type_byte == TELEM_TYPE_GPS and len(payload) == 28:
                # 3 double (8 byte) + 1 float (4 byte)
                lat, lon, alt = struct.unpack_from('<3d', payload, 0)
                speed,        = struct.unpack_from('<f',  payload, 24)
                return GPSPacket(lat, lon, float(alt), speed)

            elif type_byte == TELEM_TYPE_STATUS and len(payload) == 7:
                arm_state    = payload[0]
                error_flags, = struct.unpack_from('<H', payload, 1)
                uptime_ms,   = struct.unpack_from('<I', payload, 3)
                return StatusPacket(arm_state, error_flags, uptime_ms)

        except struct.error:
            self.packets_invalid += 1

        return None


# ------------------------------------------------------------------ #
#  TelemetryFileReader — legge il file binario in modalità live        #
# ------------------------------------------------------------------ #

class TelemetryFileReader:
    """
    Legge il file telemetry.bin in modo incrementale (non-blocking).

    Traccia la posizione nel file e legge solo i nuovi byte,
    simulando la lettura da uno stream di rete (socket/UART).
    """

    def __init__(self, filepath: str):
        self._path   = Path(filepath)
        self._pos    = 0       # byte letti finora
        self._parser = TelemetryParser()

    def read_new_packets(self) -> List[Packet]:
        """
        Legge i nuovi byte scritti dal firmware e restituisce
        i pacchetti decodificati.
        """
        if not self._path.exists():
            return []

        with open(self._path, 'rb') as f:
            f.seek(self._pos)
            new_data = f.read()
            self._pos = f.tell()

        if new_data:
            self._parser.feed(new_data)

        return self._parser.parse_all()

    def reset(self) -> None:
        """Ricomincia la lettura dall'inizio del file."""
        self._pos = 0
        self._parser = TelemetryParser()

    @property
    def stats(self) -> dict:
        return {
            'received': self._parser.packets_received,
            'invalid':  self._parser.packets_invalid,
            'bytes':    self._pos,
        }
