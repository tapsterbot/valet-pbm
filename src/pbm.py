"""Valet Push Button Module (PBM)."""

from __future__ import annotations

import atexit
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional

from gpiozero import OutputDevice
from dynamixel_sdk import COMM_SUCCESS, PortHandler, Protocol2PacketHandler


ADDR_MODEL_NUMBER = 0
ADDR_TORQUE_ENABLE = 64
ADDR_LED = 65
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_EFFORT = 126
ADDR_PRESENT_POSITION = 132


@dataclass(frozen=True)
class MotorProfile:
    model_number: int
    name: str
    effort_kind: str
    effort_unit: str
    effort_scale: float = 1.0


MOTOR_PROFILES = {
    1210: MotorProfile(1210, "XC330-T181-T", "current", "mA", 1.0),
    1220: MotorProfile(1220, "XC330-T288-T", "current", "mA", 1.0),
    1060: MotorProfile(1060, "XL430-W250-T", "load", "%", 0.1),
    1080: MotorProfile(1080, "XC430-W240-T", "load", "%", 0.1),
}


@dataclass
class MoveResult:
    position: Optional[int]
    contact: bool = False
    final_effort: Optional[float] = None
    peak_effort: Optional[float] = None
    effort_unit: Optional[str] = None


class PBM:
    def __init__(
        self,
        device="/dev/serial0",
        baudrate=3_000_000,
        direction_pin=18,
        retries=3,
        response_timeout_ms=10,
        retry_delay=0.001,
        poll_interval=0.005,
        contact_samples=2,
        contact_ignore_time=0.04,
        contact_window=0.04,
        contact_min_progress=20,
        contact_min_travel=10,
    ):
        self.device = device
        self.baudrate = baudrate
        self.retries = retries
        self.response_timeout_ms = response_timeout_ms
        self.retry_delay = retry_delay
        self.poll_interval = poll_interval

        self.contact_samples = contact_samples
        self.contact_ignore_time = contact_ignore_time
        self.contact_window = contact_window
        self.contact_min_progress = contact_min_progress
        self.contact_min_travel = contact_min_travel

        self.direction = OutputDevice(direction_pin, initial_value=False)
        self.port = PortHandler(device)
        self.packet = Protocol2PacketHandler()
        self._profiles = {}

        if not self.port.openPort():
            self.direction.close()
            raise RuntimeError(f"could not open {device}")

        if not self.port.setBaudRate(baudrate):
            self.port.closePort()
            self.direction.close()
            raise RuntimeError(f"could not set {device} to {baudrate} baud")

        self._closed = False
        atexit.register(self.close)

    def close(self):
        if self._closed:
            return

        self._closed = True

        try:
            self.direction.off()
        finally:
            try:
                self.port.closePort()
            finally:
                self.direction.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    # ------------------------------------------------------------------
    # Half-duplex transport
    # ------------------------------------------------------------------

    def _tx_begin(self):
        self.direction.on()

    def _tx_end(self):
        # Wait until the PL011 has physically finished transmitting before
        # switching the external half-duplex interface back to RX.
        try:
            self.port.ser.flush()
        finally:
            self.direction.off()

    def _start_rx_timeout(self):
        # Start the receive window only after TX has completed and the
        # external interface has switched to RX.
        #
        # The motors are normally configured for a 508 us Return Delay Time.
        # 10 ms leaves generous Linux scheduling margin. Successful receives
        # return immediately; they do not wait the full timeout.
        self.port.setPacketTimeoutMillis(self.response_timeout_ms)

    def _write1_once(self, motor_id, address, value):
        self._tx_begin()
        try:
            result = self.packet.write1ByteTxOnly(
                self.port, motor_id, address, value
            )
        finally:
            self._tx_end()

        if result != COMM_SUCCESS:
            return False

        self._start_rx_timeout()
        _, result = self.packet.rxPacket(self.port, False)
        return result == COMM_SUCCESS

    def _write4_once(self, motor_id, address, value):
        self._tx_begin()
        try:
            result = self.packet.write4ByteTxOnly(
                self.port, motor_id, address, value
            )
        finally:
            self._tx_end()

        if result != COMM_SUCCESS:
            return False

        self._start_rx_timeout()
        _, result = self.packet.rxPacket(self.port, False)
        return result == COMM_SUCCESS

    def _verify_write(self, motor_id, address, size, expected):
        """
        Verify a write by reading the register back.

        A missing WRITE status packet does not necessarily mean the write
        failed. On Valet's GPIO-controlled half-duplex bus we can occasionally
        miss an acknowledgement even when the motor accepted the command.

        Before retransmitting, read the register back. This prevents a lost
        acknowledgement from being treated as a failed physical command.
        """
        if self.retry_delay:
            time.sleep(self.retry_delay)

        try:
            actual = self._read(motor_id, address, size)
        except RuntimeError:
            return False

        mask = (1 << (size * 8)) - 1
        return actual == (expected & mask)

    def _write1(self, motor_id, address, value):
        for attempt in range(self.retries):
            if self._write1_once(motor_id, address, value):
                return

            # The motor may have accepted the command even if its status
            # packet was missed. Verify before retransmitting.
            if self._verify_write(motor_id, address, 1, value):
                return

            if self.retry_delay and attempt + 1 < self.retries:
                time.sleep(self.retry_delay)

        raise RuntimeError(
            f"DYNAMIXEL {motor_id}: write failed at address {address}"
        )

    def _write4(self, motor_id, address, value):
        for attempt in range(self.retries):
            if self._write4_once(motor_id, address, value):
                return

            # Goal Position is especially important: if the WRITE ACK was
            # lost but the motor accepted the new goal, do not report a
            # failure or blindly resend it. Confirm the register value first.
            if self._verify_write(motor_id, address, 4, value):
                return

            if self.retry_delay and attempt + 1 < self.retries:
                time.sleep(self.retry_delay)

        raise RuntimeError(
            f"DYNAMIXEL {motor_id}: write failed at address {address}"
        )

    def _read(self, motor_id, address, size):
        last_result = None

        for attempt in range(self.retries):
            self._tx_begin()

            # readTx() itself starts the SDK's packet-length timeout before
            # returning. PBM has not switched the external interface to RX
            # yet at that point, so we deliberately reset the timeout below.
            try:
                result = self.packet.readTx(
                    self.port, motor_id, address, size
                )
            finally:
                self._tx_end()

            if result != COMM_SUCCESS:
                last_result = result
                if self.retry_delay and attempt + 1 < self.retries:
                    time.sleep(self.retry_delay)
                continue

            # Critical: restart timeout after TX->RX turnaround.
            self._start_rx_timeout()

            data, result, error = self.packet.readRx(
                self.port, motor_id, size
            )

            if result != COMM_SUCCESS:
                last_result = result
                if self.retry_delay and attempt + 1 < self.retries:
                    time.sleep(self.retry_delay)
                continue

            if error:
                raise RuntimeError(
                    f"DYNAMIXEL {motor_id}: "
                    f"{self.packet.getRxPacketError(error)}"
                )

            value = 0
            for i, byte in enumerate(data):
                value |= byte << (8 * i)

            return value

        detail = (
            self.packet.getTxRxResult(last_result)
            if last_result is not None
            else "communication failed"
        )

        raise RuntimeError(
            f"DYNAMIXEL {motor_id}: "
            f"read failed at address {address}: {detail}"
        )

    @staticmethod
    def _signed16(value):
        return value - 0x10000 if value & 0x8000 else value

    # ------------------------------------------------------------------
    # Identity
    # ------------------------------------------------------------------

    def get_model_number(self, motor_id=1):
        return self._read(motor_id, ADDR_MODEL_NUMBER, 2)

    def _profile(self, motor_id):
        if motor_id in self._profiles:
            return self._profiles[motor_id]

        model_number = self.get_model_number(motor_id)

        if model_number not in MOTOR_PROFILES:
            raise RuntimeError(
                f"DYNAMIXEL {motor_id}: unsupported model number "
                f"{model_number}"
            )

        self._profiles[motor_id] = MOTOR_PROFILES[model_number]
        return self._profiles[motor_id]

    def get_model_name(self, motor_id=1):
        return self._profile(motor_id).name

    # ------------------------------------------------------------------
    # Basic operations
    # ------------------------------------------------------------------

    def ping(self, motor_id=1):
        txpacket = [0] * 10
        txpacket[4] = motor_id
        txpacket[5] = 3
        txpacket[6] = 0
        txpacket[7] = 1

        self._tx_begin()
        try:
            result = self.packet.txPacket(self.port, txpacket)
        finally:
            self._tx_end()

        if result != COMM_SUCCESS:
            return False

        self._start_rx_timeout()
        _, result = self.packet.rxPacket(self.port, False)

        return result == COMM_SUCCESS

    def led_on(self, motor_id=1):
        self._write1(motor_id, ADDR_LED, 1)

    def led_off(self, motor_id=1):
        self._write1(motor_id, ADDR_LED, 0)

    def get_led_status(self, motor_id=1):
        return bool(self._read(motor_id, ADDR_LED, 1))

    def blink(self, motor_id=1, times=5, interval=0.1):
        if times < 0:
            raise ValueError("times must be >= 0")

        for _ in range(times):
            self.led_on(motor_id)
            time.sleep(interval)
            self.led_off(motor_id)
            time.sleep(interval)

    def torque_on(self, motor_id=1):
        self._write1(motor_id, ADDR_TORQUE_ENABLE, 1)

    def torque_off(self, motor_id=1):
        self._write1(motor_id, ADDR_TORQUE_ENABLE, 0)

    def get_torque_status(self, motor_id=1):
        return bool(self._read(motor_id, ADDR_TORQUE_ENABLE, 1))

    def _ensure_torque(self, motor_id):
        if not self.get_torque_status(motor_id):
            self.torque_on(motor_id)

    def get_position(self, motor_id=1):
        return self._read(motor_id, ADDR_PRESENT_POSITION, 4)

    def get_effort(self, motor_id=1):
        profile = self._profile(motor_id)
        raw = self._signed16(
            self._read(motor_id, ADDR_PRESENT_EFFORT, 2)
        )

        return raw * profile.effort_scale, profile.effort_unit

    # ------------------------------------------------------------------
    # Movement / contact
    # ------------------------------------------------------------------

    def stop(self, motor_id):
        position = self.get_position(motor_id)
        self._write4(motor_id, ADDR_GOAL_POSITION, position)
        return position

    def move(
        self,
        motor_id,
        goal_position,
        *,
        stop_on_contact=False,
        wait=False,
        timeout=1.0,
        position_tolerance=10,
    ):
        self._ensure_torque(motor_id)

        profile = self._profile(motor_id)
        start_position = (
            self.get_position(motor_id)
            if stop_on_contact
            else None
        )

        self._write4(
            motor_id,
            ADDR_GOAL_POSITION,
            goal_position,
        )

        if not wait and not stop_on_contact:
            return MoveResult(position=None)

        start = time.monotonic()
        deadline = start + timeout

        contact_count = 0
        last_position = start_position
        last_effort = None
        peak_effort = None
        position_history = deque()

        while time.monotonic() < deadline:
            now = time.monotonic()
            last_position = self.get_position(motor_id)

            if stop_on_contact:
                effort, unit = self.get_effort(motor_id)
                last_effort = abs(effort)

                if peak_effort is None or last_effort > peak_effort:
                    peak_effort = last_effort

            if abs(last_position - goal_position) <= position_tolerance:
                return MoveResult(
                    position=last_position,
                    contact=False,
                    final_effort=last_effort,
                    peak_effort=peak_effort,
                    effort_unit=profile.effort_unit,
                )

            if stop_on_contact:
                position_history.append((now, last_position))

                while (
                    position_history
                    and now - position_history[0][0]
                    > self.contact_window
                ):
                    position_history.popleft()

                elapsed = now - start
                travel = abs(last_position - start_position)
                remaining = abs(goal_position - last_position)

                full_window = (
                    position_history
                    and now - position_history[0][0]
                    >= self.contact_window * 0.8
                )

                progress = None

                if full_window:
                    progress = abs(
                        last_position - position_history[0][1]
                    )

                insufficient_progress = (
                    progress is not None
                    and progress < self.contact_min_progress
                )

                eligible = (
                    elapsed >= self.contact_ignore_time
                    and travel >= self.contact_min_travel
                    and remaining > position_tolerance
                    and full_window
                )

                if eligible and insufficient_progress:
                    contact_count += 1
                else:
                    contact_count = 0

                if contact_count >= self.contact_samples:
                    position = self.stop(motor_id)

                    return MoveResult(
                        position=position,
                        contact=True,
                        final_effort=last_effort,
                        peak_effort=peak_effort,
                        effort_unit=unit,
                    )

            time.sleep(self.poll_interval)

        return MoveResult(
            position=last_position,
            contact=False,
            final_effort=last_effort,
            peak_effort=peak_effort,
            effort_unit=profile.effort_unit,
        )

    def tap(
        self,
        motor_id,
        from_position,
        to_position,
        *,
        stop_on_contact=True,
        hold=0.05,
        timeout=1.0,
    ):
        # Ensure the motor is released before starting.
        self.move(
            motor_id,
            from_position,
            wait=True,
            timeout=timeout,
        )

        press_result = MoveResult(position=None)

        try:
            press_result = self.move(
                motor_id,
                to_position,
                stop_on_contact=stop_on_contact,
                wait=True,
                timeout=timeout,
            )

            time.sleep(hold)

        finally:
            # Release is safety-critical. Always attempt it.
            self.move(
                motor_id,
                from_position,
                wait=True,
                timeout=timeout,
            )

        return press_result
