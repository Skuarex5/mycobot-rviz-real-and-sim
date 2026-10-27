"""Bound sensor reply waits without changing SDK motion or stop protocols."""

from enum import Enum


class GripperDispatch(Enum):
    SENT = 'sent_without_ack'


def gripper_write_sent(result):
    """A dispatched frame is not an acknowledgement or proof of execution."""
    return result == 1 or result is GripperDispatch.SENT


def bound_state_read_deadline(device):
    from pymycobot.common import ProtocolCode

    sensor_commands = {
        ProtocolCode.GET_ANGLES,
        ProtocolCode.GET_GRIPPER_VALUE,
        ProtocolCode.IS_MOVING,
        ProtocolCode.IS_GRIPPER_MOVING,
    }
    original_read = device._read

    def read(genre, *args, **kwargs):
        if genre in sensor_commands:
            kwargs['timeout'] = min(.08, kwargs.get('timeout') or .08)
        return original_read(genre, *args, **kwargs)

    # The SDK retries a query three times. Its Linux default is 0.5 s per
    # attempt, already longer than the physical-state watchdog. Keep failed
    # sensor queries bounded and let the next polling cycle recover normally.
    device._read = read
    serial_timeout = device._serial_port.timeout
    device._serial_port.timeout = min(.02, serial_timeout or .02)
