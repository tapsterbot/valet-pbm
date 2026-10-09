"""Hardware-free fakes for the GPIO, serial port, and DynamixelSDK boundaries.

The fake packet handler simulates a bus of DYNAMIXEL motors with a small
register map. Motor position is a function of a fake clock, so movement and
contact detection are deterministic and tests run instantly.
"""

from __future__ import annotations

import types

import pytest
from dynamixel_sdk import COMM_RX_TIMEOUT, COMM_SUCCESS

import pbm as pbm_module


def pytest_addoption(parser):
    parser.addoption(
        "--hardware",
        action="store_true",
        help="run end-to-end tests against connected PBM hardware",
    )


def pytest_collection_modifyitems(config, items):
    if config.getoption("--hardware"):
        return

    skip = pytest.mark.skip(reason="needs connected hardware: make test-hw")
    for item in items:
        if "hardware" in item.keywords:
            item.add_marker(skip)


class FakeClock:
    def __init__(self):
        self.now = 0.0
        self.sleeps = []

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.sleeps.append(seconds)
        self.now += seconds


class FakeMotor:
    def __init__(self, clock, model_number=1210, position=2048, speed=2000):
        self.clock = clock
        self.model_number = model_number
        self.speed = speed  # ticks per second
        self.torque = 0
        self.led = 0
        self.effort = 0  # raw 16-bit register value
        self.obstacle = None  # button surface; blocks positions below this
        self._anchor = (0.0, position, position)  # (time, position, goal)

    @property
    def goal(self):
        return self._anchor[2]

    @property
    def position(self):
        t0, p0, goal = self._anchor
        if not self.torque:
            return p0

        step = min(self.speed * (self.clock.now - t0), abs(goal - p0))
        direction = 1 if goal >= p0 else -1
        position = int(p0 + direction * step)

        # A button surface: it resists pressing (decreasing position) past
        # it, but not moving back away from it.
        if self.obstacle is not None and goal < self.obstacle:
            position = max(position, self.obstacle)

        return position

    def set_goal(self, goal):
        self._anchor = (self.clock.now, self.position, goal)

    def set_torque(self, value):
        # Freeze the current position when torque changes.
        position = self.position
        self.torque = value
        self._anchor = (self.clock.now, position, self.goal)

    def read(self, address):
        if address == pbm_module.ADDR_MODEL_NUMBER:
            return self.model_number
        if address == pbm_module.ADDR_TORQUE_ENABLE:
            return self.torque
        if address == pbm_module.ADDR_LED:
            return self.led
        if address == pbm_module.ADDR_GOAL_POSITION:
            return self.goal
        if address == pbm_module.ADDR_PRESENT_EFFORT:
            return self.effort
        if address == pbm_module.ADDR_PRESENT_POSITION:
            return self.position
        raise AssertionError(f"unexpected read address {address}")

    def write(self, address, value):
        if address == pbm_module.ADDR_TORQUE_ENABLE:
            self.set_torque(value)
        elif address == pbm_module.ADDR_LED:
            self.led = value
        elif address == pbm_module.ADDR_GOAL_POSITION:
            self.set_goal(value)
        else:
            raise AssertionError(f"unexpected write address {address}")


class FakePacketHandler:
    """Stand-in for dynamixel_sdk.Protocol2PacketHandler.

    Fault injection:
      drop_acks      number of upcoming write acknowledgements to lose
                     (the write itself is still applied)
      reject_writes  if True, writes are not applied and not acknowledged
      fail_reads     number of upcoming reads that time out
      fail_reads_at  set of addresses whose reads always time out
      raise_on_read  {address: exception} raised when that address is read
      tx_exception   exception raised by the next TX call of any kind
    """

    def __init__(self, clock, events):
        self.clock = clock
        self.events = events
        self.motors = {}
        self.writes = []  # (motor_id, address, value) for every TX attempt
        self.reads = []  # (motor_id, address, size)
        self.drop_acks = 0
        self.reject_writes = False
        self.fail_reads = 0
        self.fail_reads_at = set()
        self.raise_on_read = {}
        self.tx_result = COMM_SUCCESS
        self.tx_exception = None
        self._pending = None

    def add_motor(self, motor_id=1, **kwargs):
        motor = FakeMotor(self.clock, **kwargs)
        self.motors[motor_id] = motor
        return motor

    # -- SDK surface used by PBM -------------------------------------------

    def _maybe_raise(self):
        exc, self.tx_exception = self.tx_exception, None
        if exc is not None:
            raise exc

    def txPacket(self, port, txpacket):
        self.events.append("tx")
        self._maybe_raise()
        motor_id = txpacket[4]
        self._pending = motor_id in self.motors
        return self.tx_result

    def rxPacket(self, port, skip_stuffing):
        self.events.append("rx")
        ok, self._pending = self._pending, None
        return ([], COMM_SUCCESS) if ok else ([], COMM_RX_TIMEOUT)

    def _write(self, motor_id, address, value):
        self.events.append("tx")
        self._maybe_raise()
        self.writes.append((motor_id, address, value))
        if self.tx_result != COMM_SUCCESS:
            return self.tx_result

        motor = self.motors.get(motor_id)
        if motor is None or self.reject_writes:
            self._pending = False
        elif self.drop_acks:
            self.drop_acks -= 1
            motor.write(address, value)
            self._pending = False
        else:
            motor.write(address, value)
            self._pending = True
        return COMM_SUCCESS

    def write1ByteTxOnly(self, port, motor_id, address, value):
        return self._write(motor_id, address, value)

    def write4ByteTxOnly(self, port, motor_id, address, value):
        return self._write(motor_id, address, value)

    def readTx(self, port, motor_id, address, size):
        self.events.append("tx")
        self._maybe_raise()
        self.reads.append((motor_id, address, size))
        if self.tx_result != COMM_SUCCESS:
            return self.tx_result
        if address in self.raise_on_read:
            raise self.raise_on_read.pop(address)
        self._pending = (motor_id, address)
        return COMM_SUCCESS

    def readRx(self, port, motor_id, size):
        self.events.append("rx")
        pending, self._pending = self._pending, None
        motor = self.motors.get(motor_id)

        if (
            motor is None
            or pending is None
            or pending[1] in self.fail_reads_at
            or self.fail_reads
        ):
            if self.fail_reads:
                self.fail_reads -= 1
            return [], COMM_RX_TIMEOUT, 0

        value = motor.read(pending[1]) & ((1 << (size * 8)) - 1)
        data = [(value >> (8 * i)) & 0xFF for i in range(size)]
        return data, COMM_SUCCESS, 0

    def getTxRxResult(self, result):
        return f"result {result}"

    def getRxPacketError(self, error):
        return f"error {error}"

    # -- helpers for assertions ----------------------------------------------

    def writes_to(self, address, motor_id=1):
        return [v for m, a, v in self.writes if m == motor_id and a == address]


class FakeSerial:
    def __init__(self, events):
        self.events = events
        self.flush_exception = None

    def fileno(self):
        return 3

    def flush(self):
        self.events.append("flush")
        exc, self.flush_exception = self.flush_exception, None
        if exc is not None:
            raise exc


class FakePortHandler:
    open_ok = True
    baud_ok = True

    def __init__(self, device, events):
        self.device = device
        self.events = events
        self.ser = FakeSerial(events)
        self.is_open = False
        self.baudrate = None
        self.timeouts = []
        self.close_calls = 0

    def openPort(self):
        self.is_open = self.open_ok
        return self.open_ok

    def setBaudRate(self, baudrate):
        self.baudrate = baudrate
        return self.baud_ok

    def closePort(self):
        self.close_calls += 1
        self.is_open = False

    def setPacketTimeoutMillis(self, ms):
        self.timeouts.append(ms)


class FakeOutputDevice:
    def __init__(self, pin, initial_value=False, *, events):
        self.pin = pin
        self.value = initial_value
        self.events = events
        self.closed = False

    def on(self):
        self.value = True
        self.events.append("dir_tx")

    def off(self):
        self.value = False
        self.events.append("dir_rx")

    def close(self):
        self.closed = True


class Bus:
    """Everything a test needs to drive a PBM against fake hardware."""

    def __init__(self):
        self.events = []
        self.clock = FakeClock()
        self.packet = FakePacketHandler(self.clock, self.events)
        self.ports = []
        self.directions = []
        self.atexit_callbacks = []
        # UART drain fault injection: polls that report TX still busy, and
        # an exception raised by the drain ioctl (e.g. unsupported driver).
        self.tx_busy_polls = 0
        self.tx_idle_exception = None

    def tx_idle(self, fd):
        if self.tx_idle_exception is not None:
            raise self.tx_idle_exception
        if self.tx_busy_polls:
            self.tx_busy_polls -= 1
            self.clock.now += 0.001
            self.events.append("busy")
            return False
        self.events.append("drain")
        return True

    def port_factory(self, device):
        port = FakePortHandler(device, self.events)
        self.ports.append(port)
        return port

    def direction_factory(self, pin, initial_value=False):
        device = FakeOutputDevice(pin, initial_value, events=self.events)
        self.directions.append(device)
        return device

    def add_motor(self, motor_id=1, **kwargs):
        return self.packet.add_motor(motor_id, **kwargs)


@pytest.fixture
def bus(monkeypatch):
    bus = Bus()
    monkeypatch.setattr(pbm_module, "OutputDevice", bus.direction_factory)
    monkeypatch.setattr(pbm_module, "PortHandler", bus.port_factory)
    monkeypatch.setattr(
        pbm_module, "Protocol2PacketHandler", lambda: bus.packet
    )
    monkeypatch.setattr(pbm_module, "time", bus.clock)
    monkeypatch.setattr(pbm_module, "_tx_idle", bus.tx_idle)
    monkeypatch.setattr(
        pbm_module,
        "atexit",
        types.SimpleNamespace(register=bus.atexit_callbacks.append),
    )
    monkeypatch.setattr(FakePortHandler, "open_ok", True)
    monkeypatch.setattr(FakePortHandler, "baud_ok", True)
    return bus


@pytest.fixture
def motor(bus):
    return bus.add_motor(1)


@pytest.fixture
def pbm(bus):
    device = pbm_module.PBM()
    yield device
    device.close()

