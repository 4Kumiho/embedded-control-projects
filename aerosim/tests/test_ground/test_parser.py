"""
test_parser.py — Test unitari per ground_station/parser.py

Esegui con:
    cd tests/test_ground
    pytest test_parser.py -v

Oppure dalla root:
    pytest tests/test_ground/ -v --tb=short
"""

import struct
import sys
import pytest
from pathlib import Path

# Aggiunge il path della ground station al Python path
sys.path.insert(0, str(Path(__file__).parent.parent.parent / "ground_station"))

from parser import (
    TelemetryParser, TelemetryFileReader,
    IMUPacket, GPSPacket, StatusPacket,
    TELEM_START, TELEM_TYPE_IMU, TELEM_TYPE_GPS, TELEM_TYPE_STATUS,
)


# ================================================================== #
#  Funzioni helper per costruire pacchetti validi                     #
# ================================================================== #

def build_packet(type_byte: int, payload: bytes) -> bytes:
    """Costruisce un pacchetto valido secondo il protocollo AeroSim."""
    length = len(payload)
    header = bytes([TELEM_START, type_byte]) + struct.pack('<H', length)
    checksum = 0
    for b in payload:
        checksum ^= b
    return header + payload + bytes([checksum])


def build_imu_packet(ax=0.0, ay=0.0, az=9.81,
                     gx=0.0, gy=0.0, gz=0.0) -> bytes:
    payload = struct.pack('<6f', ax, ay, az, gx, gy, gz)
    return build_packet(TELEM_TYPE_IMU, payload)


def build_gps_packet(lat=45.4654, lon=9.1859,
                     alt=100.0, speed=5.0) -> bytes:
    payload = struct.pack('<3d', lat, lon, alt) + struct.pack('<f', speed)
    return build_packet(TELEM_TYPE_GPS, payload)


def build_status_packet(arm=1, errors=0, uptime=5000) -> bytes:
    payload = bytes([arm]) + struct.pack('<H', errors) + struct.pack('<I', uptime)
    return build_packet(TELEM_TYPE_STATUS, payload)


# ================================================================== #
#  Test: checksum                                                     #
# ================================================================== #

class TestChecksum:

    def test_valid_packet_parsed(self):
        """Un pacchetto con checksum corretto deve essere parsato."""
        parser = TelemetryParser()
        parser.feed(build_imu_packet())
        packets = parser.parse_all()
        assert len(packets) == 1
        assert isinstance(packets[0], IMUPacket)

    def test_corrupted_checksum_rejected(self):
        """Un pacchetto con checksum errato deve essere scartato."""
        data = bytearray(build_imu_packet())
        data[-1] ^= 0xFF   # corrompi l'ultimo byte (checksum)
        parser = TelemetryParser()
        parser.feed(bytes(data))
        packets = parser.parse_all()
        assert len(packets) == 0
        assert parser.packets_invalid >= 1

    def test_corrupted_payload_rejected(self):
        """Corrompere un byte del payload deve invalidare il checksum."""
        data = bytearray(build_gps_packet())
        data[5] ^= 0x01   # corrompi un byte del payload
        parser = TelemetryParser()
        parser.feed(bytes(data))
        assert len(parser.parse_all()) == 0


# ================================================================== #
#  Test: parsing pacchetti IMU                                        #
# ================================================================== #

class TestIMUParsing:

    def test_imu_packet_correct_type(self):
        parser = TelemetryParser()
        parser.feed(build_imu_packet())
        pkts = parser.parse_all()
        assert isinstance(pkts[0], IMUPacket)

    def test_imu_values_preserved(self):
        """I valori float devono essere preservati dopo la serializzazione."""
        parser = TelemetryParser()
        parser.feed(build_imu_packet(ax=1.5, ay=-2.3, az=9.81,
                                     gx=0.1, gy=-0.2, gz=0.05))
        pkt = parser.parse_all()[0]
        assert isinstance(pkt, IMUPacket)
        assert abs(pkt.ax - 1.5)  < 1e-5
        assert abs(pkt.ay - (-2.3)) < 1e-5
        assert abs(pkt.az - 9.81) < 1e-5
        assert abs(pkt.gx - 0.1)  < 1e-5

    def test_imu_gravity_default(self):
        """Az di default deve essere circa 9.81."""
        parser = TelemetryParser()
        parser.feed(build_imu_packet())
        pkt = parser.parse_all()[0]
        assert abs(pkt.az - 9.81) < 1e-4


# ================================================================== #
#  Test: parsing pacchetti GPS                                        #
# ================================================================== #

class TestGPSParsing:

    def test_gps_packet_correct_type(self):
        parser = TelemetryParser()
        parser.feed(build_gps_packet())
        assert isinstance(parser.parse_all()[0], GPSPacket)

    def test_gps_double_precision_preserved(self):
        """Lat/lon sono double — la precisione deve essere mantenuta."""
        lat, lon = 45.465431234567, 9.185912345678
        parser = TelemetryParser()
        parser.feed(build_gps_packet(lat=lat, lon=lon))
        pkt = parser.parse_all()[0]
        assert abs(pkt.lat - lat) < 1e-9   # precisione double
        assert abs(pkt.lon - lon) < 1e-9

    def test_gps_altitude_preserved(self):
        parser = TelemetryParser()
        parser.feed(build_gps_packet(alt=1500.75))
        pkt = parser.parse_all()[0]
        assert abs(pkt.alt - 1500.75) < 1e-6


# ================================================================== #
#  Test: parsing pacchetti STATUS                                     #
# ================================================================== #

class TestStatusParsing:

    def test_status_packet_correct_type(self):
        parser = TelemetryParser()
        parser.feed(build_status_packet())
        assert isinstance(parser.parse_all()[0], StatusPacket)

    def test_status_arm_state(self):
        parser = TelemetryParser()
        parser.feed(build_status_packet(arm=1))
        pkt = parser.parse_all()[0]
        assert pkt.arm_state == 1

    def test_status_error_flags(self):
        parser = TelemetryParser()
        parser.feed(build_status_packet(errors=0xABCD))
        pkt = parser.parse_all()[0]
        assert pkt.error_flags == 0xABCD

    def test_status_uptime_large_value(self):
        """Uptime uint32 deve gestire valori grandi (fino a ~49 giorni)."""
        large_uptime = 4_000_000_000   # quasi al limite uint32
        parser = TelemetryParser()
        parser.feed(build_status_packet(uptime=large_uptime))
        pkt = parser.parse_all()[0]
        assert pkt.uptime_ms == large_uptime


# ================================================================== #
#  Test: gestione stream                                              #
# ================================================================== #

class TestStreamHandling:

    def test_partial_packet_not_emitted(self):
        """Un pacchetto incompleto non deve essere emesso."""
        full = build_imu_packet()
        parser = TelemetryParser()
        parser.feed(full[:10])   # solo i primi 10 byte (su 29)
        assert len(parser.parse_all()) == 0

    def test_partial_then_complete(self):
        """Dati incompleti + completamento = 1 pacchetto."""
        full = build_imu_packet()
        parser = TelemetryParser()
        parser.feed(full[:10])
        assert len(parser.parse_all()) == 0
        parser.feed(full[10:])
        assert len(parser.parse_all()) == 1

    def test_multiple_packets_in_one_feed(self):
        """Due pacchetti concatenati devono produrre 2 risultati."""
        data = build_imu_packet() + build_gps_packet()
        parser = TelemetryParser()
        parser.feed(data)
        packets = parser.parse_all()
        assert len(packets) == 2
        assert isinstance(packets[0], IMUPacket)
        assert isinstance(packets[1], GPSPacket)

    def test_noise_before_start_byte_skipped(self):
        """Byte di rumore prima del pacchetto devono essere ignorati."""
        noise = bytes([0x00, 0xFF, 0x11, 0x22])
        data  = noise + build_imu_packet()
        parser = TelemetryParser()
        parser.feed(data)
        packets = parser.parse_all()
        assert len(packets) == 1

    def test_three_packets_different_types(self):
        data = build_imu_packet() + build_gps_packet() + build_status_packet()
        parser = TelemetryParser()
        parser.feed(data)
        packets = parser.parse_all()
        assert len(packets) == 3
        types = [type(p) for p in packets]
        assert IMUPacket    in types
        assert GPSPacket    in types
        assert StatusPacket in types

    def test_packet_counter_increments(self):
        parser = TelemetryParser()
        parser.feed(build_imu_packet())
        parser.parse_all()
        assert parser.packets_received == 1
        parser.feed(build_gps_packet())
        parser.parse_all()
        assert parser.packets_received == 2


# ================================================================== #
#  Test: TelemetryFileReader                                          #
# ================================================================== #

class TestFileReader:

    def test_nonexistent_file_returns_empty(self, tmp_path):
        reader = TelemetryFileReader(str(tmp_path / "nofile.bin"))
        assert reader.read_new_packets() == []

    def test_reads_packets_from_file(self, tmp_path):
        path = tmp_path / "telem.bin"
        data = build_imu_packet() + build_gps_packet()
        path.write_bytes(data)

        reader = TelemetryFileReader(str(path))
        packets = reader.read_new_packets()
        assert len(packets) == 2

    def test_incremental_reading(self, tmp_path):
        """Simula firmware che scrive dati incrementalmente."""
        path = tmp_path / "telem.bin"
        reader = TelemetryFileReader(str(path))

        # Prima scrittura
        path.write_bytes(build_imu_packet())
        p1 = reader.read_new_packets()
        assert len(p1) == 1

        # Seconda scrittura (append)
        with open(path, 'ab') as f:
            f.write(build_gps_packet())
        p2 = reader.read_new_packets()
        assert len(p2) == 1
        assert isinstance(p2[0], GPSPacket)

    def test_reset_rereads_from_start(self, tmp_path):
        path = tmp_path / "telem.bin"
        path.write_bytes(build_imu_packet())

        reader = TelemetryFileReader(str(path))
        reader.read_new_packets()   # legge il pacchetto
        reader.reset()              # torna all'inizio
        p = reader.read_new_packets()
        assert len(p) == 1          # rilegge lo stesso pacchetto
