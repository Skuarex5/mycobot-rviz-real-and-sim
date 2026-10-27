"""Exercise pymycobot's actual frame reader without opening a serial device."""

import logging
from types import MethodType, SimpleNamespace

import pytest

common = pytest.importorskip('pymycobot.common')


def device(monkeypatch, frame=b''):
    clock = [0.0]

    class Serial:
        timeout = .1

        def __init__(self):
            self.buffer = bytearray(frame)

        def read(self):
            clock[0] += .0001 if self.buffer else self.timeout
            return bytes([self.buffer.pop(0)]) if self.buffer else b''

    result = SimpleNamespace(_serial_port=Serial(), log=logging.getLogger('serial_test'))
    result._read = MethodType(common.read, result)
    monkeypatch.setattr(common.time, 'time', lambda: clock[0])
    return result, clock


def test_missing_sensor_reply_returns_before_state_watchdog_budget(monkeypatch):
    from mycobot_280_hardware.serial_transport import bound_state_read_deadline

    dev, clock = device(monkeypatch)
    bound_state_read_deadline(dev)
    assert dev._read(common.ProtocolCode.GET_ANGLES) == b''
    assert .079 <= clock[0] <= .101


def test_state_reply_parser_still_returns_complete_real_protocol_frame(monkeypatch):
    from mycobot_280_hardware.serial_transport import bound_state_read_deadline

    frame = b'\xfe\xfe\x0e\x20' + b'\x00\x00' * 6 + b'\xfa'
    dev, _ = device(monkeypatch, frame)
    bound_state_read_deadline(dev)
    assert dev._read(common.ProtocolCode.GET_ANGLES) == frame


def test_stop_command_keeps_sdk_reply_budget(monkeypatch):
    from mycobot_280_hardware.serial_transport import bound_state_read_deadline

    dev, clock = device(monkeypatch)
    bound_state_read_deadline(dev)
    assert dev._read(common.ProtocolCode.STOP) == b''
    assert clock[0] >= .5
