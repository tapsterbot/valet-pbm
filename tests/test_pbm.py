import inspect

import pytest
from dynamixel_sdk import COMM_TX_FAIL

from pbm import (
    ADDR_GOAL_POSITION,
    ADDR_LED,
    ADDR_PRESENT_EFFORT,
    ADDR_TORQUE_ENABLE,
    PBM,
    MoveResult,
)

from conftest import FakePortHandler


# ----------------------------------------------------------------------
# Construction / close
# ----------------------------------------------------------------------


def test_init_opens_port_and_direction_pin(bus):
    device = PBM(device="/dev/ttyTEST", baudrate=1_000_000, direction_pin=17)

    port = bus.ports[0]
    direction = bus.directions[0]
    assert port.device == "/dev/ttyTEST"
    assert port.is_open
    assert port.baudrate == 1_000_000
    assert direction.pin == 17
    assert direction.value is False
    assert bus.atexit_callbacks == [device.close]


def test_init_defaults(bus):
    PBM()

    assert bus.ports[0].device == "/dev/serial0"
    assert bus.ports[0].baudrate == 3_000_000
    assert bus.directions[0].pin == 18


def test_init_open_failure_releases_gpio(bus, monkeypatch):
    monkeypatch.setattr(FakePortHandler, "open_ok", False)

    with pytest.raises(RuntimeError, match="could not open"):
        PBM()

    assert bus.directions[0].closed
    assert bus.atexit_callbacks == []


def test_init_baud_failure_releases_port_and_gpio(bus, monkeypatch):
    monkeypatch.setattr(FakePortHandler, "baud_ok", False)

    with pytest.raises(RuntimeError, match="baud"):
        PBM()

    assert bus.ports[0].close_calls == 1
    assert bus.directions[0].closed
    assert bus.atexit_callbacks == []


def test_close_releases_resources_once(bus, pbm):
    pbm.close()
    pbm.close()

    assert bus.ports[0].close_calls == 1
    assert bus.directions[0].value is False
    assert bus.directions[0].closed


def test_context_manager_closes(bus):
    with PBM() as device:
        assert isinstance(device, PBM)
        assert bus.ports[0].is_open

    assert bus.ports[0].close_calls == 1
    assert bus.directions[0].closed


def test_context_manager_closes_on_error(bus):
    with pytest.raises(ValueError):
        with PBM():
            raise ValueError("boom")

    assert bus.ports[0].close_calls == 1
    assert bus.directions[0].closed


# ----------------------------------------------------------------------
# Half-duplex transport / ping
# ----------------------------------------------------------------------


def test_ping_success(bus, motor, pbm):
    assert pbm.ping(1) is True


def test_ping_switches_to_rx_before_receive_timeout(bus, motor, pbm):
    bus.events.clear()

    pbm.ping(1)

    # TX completes (flush) before the direction pin returns to RX, and the
    # receive window starts only after that.
    assert bus.events == ["dir_tx", "tx", "flush", "dir_rx", "rx"]
    assert bus.ports[0].timeouts == [10]


def test_ping_missing_motor(bus, motor, pbm):
    assert pbm.ping(2) is False


def test_ping_tx_failure(bus, motor, pbm):
    bus.packet.tx_result = COMM_TX_FAIL
    bus.events.clear()

    assert pbm.ping(1) is False
    assert "rx" not in bus.events
    assert bus.directions[0].value is False


@pytest.mark.parametrize(
    "operation",
    [
        lambda pbm: pbm.ping(1),
        lambda pbm: pbm.led_on(1),
        lambda pbm: pbm.get_position(1),
        lambda pbm: pbm.move(1, 1900),
    ],
    ids=["ping", "write1", "read", "write4"],
)
def test_direction_returns_to_rx_when_tx_raises(bus, motor, pbm, operation):
    motor.set_torque(1)
    bus.packet.tx_exception = OSError("serial write failed")

    with pytest.raises(OSError):
        operation(pbm)

    assert bus.directions[0].value is False
    assert bus.events[-1] == "dir_rx"


def test_direction_returns_to_rx_when_flush_raises(bus, motor, pbm):
    bus.ports[0].ser.flush_exception = OSError("flush failed")

    with pytest.raises(OSError):
        pbm.ping(1)

    assert bus.directions[0].value is False


# ----------------------------------------------------------------------
# LED / torque
# ----------------------------------------------------------------------


def test_led_on_off(bus, motor, pbm):
    assert pbm.get_led_status(1) is False

    pbm.led_on(1)
    assert motor.led == 1
    assert pbm.get_led_status(1) is True

    pbm.led_off(1)
    assert motor.led == 0
    assert pbm.get_led_status(1) is False


def test_blink(bus, motor, pbm):
    pbm.blink(1, times=3, interval=0.25)

    assert bus.packet.writes_to(ADDR_LED) == [1, 0, 1, 0, 1, 0]
    assert bus.clock.sleeps == [0.25] * 6
    assert motor.led == 0


def test_blink_zero_times(bus, motor, pbm):
    pbm.blink(1, times=0)

    assert bus.packet.writes_to(ADDR_LED) == []


def test_blink_negative_times(bus, motor, pbm):
    with pytest.raises(ValueError):
        pbm.blink(1, times=-1)


def test_torque_status(bus, motor, pbm):
    assert pbm.get_torque_status(1) is False

    pbm.torque_on(1)
    assert pbm.get_torque_status(1) is True

    pbm.torque_off(1)
    assert pbm.get_torque_status(1) is False


def test_move_enables_torque(bus, motor, pbm):
    pbm.move(1, 1900)

    assert bus.packet.writes_to(ADDR_TORQUE_ENABLE) == [1]
    assert motor.torque == 1


def test_move_skips_torque_write_when_already_on(bus, motor, pbm):
    pbm.torque_on(1)
    bus.packet.writes.clear()

    pbm.move(1, 1900)

    assert bus.packet.writes_to(ADDR_TORQUE_ENABLE) == []


def test_move_after_torque_off_reenables(bus, motor, pbm):
    pbm.torque_on(1)
    pbm.torque_off(1)

    pbm.move(1, 1900, wait=True)

    assert motor.torque == 1
    assert abs(motor.position - 1900) <= 10


def test_tap_enables_torque(bus, motor, pbm):
    pbm.tap(1, from_position=2048, to_position=1700)

    assert bus.packet.writes_to(ADDR_TORQUE_ENABLE) == [1]


# ----------------------------------------------------------------------
# Reads
# ----------------------------------------------------------------------


@pytest.mark.parametrize(
    "model_number, name",
    [
        (1210, "XC330-T181-T"),
        (1220, "XC330-T288-T"),
        (1060, "XL430-W250-T"),
        (1080, "XC430-W240-T"),
    ],
)
def test_model_identity(bus, pbm, model_number, name):
    bus.add_motor(1, model_number=model_number)

    assert pbm.get_model_number(1) == model_number
    assert pbm.get_model_name(1) == name


def test_model_profile_is_cached(bus, motor, pbm):
    pbm.get_model_name(1)
    pbm.get_model_name(1)

    model_reads = [r for r in bus.packet.reads if r[1] == 0]
    assert len(model_reads) == 1


def test_unsupported_model(bus, pbm):
    bus.add_motor(1, model_number=9999)

    with pytest.raises(RuntimeError, match="unsupported model number 9999"):
        pbm.get_model_name(1)


def test_get_position(bus, pbm):
    bus.add_motor(1, position=3071)

    assert pbm.get_position(1) == 3071


def test_get_effort_current_is_signed_ma(bus, pbm):
    motor = bus.add_motor(1, model_number=1210)
    motor.effort = 0xFF38  # -200

    assert pbm.get_effort(1) == (-200.0, "mA")


def test_get_effort_load_is_scaled_percent(bus, pbm):
    motor = bus.add_motor(1, model_number=1060)
    motor.effort = 0xFFF6  # -10 -> -1.0 %

    assert pbm.get_effort(1) == pytest.approx((-1.0, "%"))


def test_read_retries_then_succeeds(bus, motor, pbm):
    motor.led = 1
    bus.packet.fail_reads = 2

    assert pbm.get_led_status(1) is True
    assert len(bus.packet.reads) == 3


def test_read_failure_raises_after_retries(bus, motor, pbm):
    bus.packet.fail_reads_at = {ADDR_LED}

    with pytest.raises(RuntimeError, match="read failed at address 65"):
        pbm.get_led_status(1)

    assert len(bus.packet.reads) == pbm.retries


def test_read_failure_on_missing_motor(bus, pbm):
    with pytest.raises(RuntimeError, match="DYNAMIXEL 7: read failed"):
        pbm.get_position(7)


# ----------------------------------------------------------------------
# Writes
# ----------------------------------------------------------------------


def test_write_success_sends_once(bus, motor, pbm):
    pbm.led_on(1)

    assert bus.packet.writes_to(ADDR_LED) == [1]
    # No read-back needed when the acknowledgement arrives.
    assert bus.packet.reads == []


def test_missing_write1_ack_verified_by_readback(bus, motor, pbm):
    bus.packet.drop_acks = 1

    pbm.led_on(1)

    # The lost ACK is resolved by read-back, not by retransmitting.
    assert bus.packet.writes_to(ADDR_LED) == [1]
    assert (1, ADDR_LED, 1) in bus.packet.reads
    assert motor.led == 1


def test_missing_write4_ack_verified_by_readback(bus, motor, pbm):
    pbm.torque_on(1)
    bus.packet.drop_acks = 1

    pbm.move(1, 1900)

    assert bus.packet.writes_to(ADDR_GOAL_POSITION) == [1900]
    assert (1, ADDR_GOAL_POSITION, 4) in bus.packet.reads
    assert motor.goal == 1900


def test_write_rejected_raises_after_retries(bus, motor, pbm):
    bus.packet.reject_writes = True

    with pytest.raises(RuntimeError, match="write failed at address 65"):
        pbm.led_on(1)

    assert bus.packet.writes_to(ADDR_LED) == [1] * pbm.retries
    assert motor.led == 0


def test_write_fails_when_ack_and_readback_both_fail(bus, motor, pbm):
    bus.packet.drop_acks = 99
    bus.packet.fail_reads_at = {ADDR_LED}

    with pytest.raises(RuntimeError, match="write failed at address 65"):
        pbm.led_on(1)

    assert bus.packet.writes_to(ADDR_LED) == [1] * pbm.retries


def test_write_tx_failure_raises(bus, motor, pbm):
    bus.packet.tx_result = COMM_TX_FAIL

    with pytest.raises(RuntimeError, match="write failed"):
        pbm.led_on(1)


# ----------------------------------------------------------------------
# MoveResult / move()
# ----------------------------------------------------------------------


def test_move_result_fields():
    result = MoveResult(position=1508)

    assert result.position == 1508
    assert result.contact is False
    assert result.final_effort is None
    assert result.peak_effort is None
    assert result.effort_unit is None


def test_move_defaults():
    params = inspect.signature(PBM.move).parameters

    assert params["stop_on_contact"].default is False
    assert params["wait"].default is False


def test_move_without_wait_returns_immediately(bus, motor, pbm):
    result = pbm.move(1, 1900)

    assert result == MoveResult(position=None)
    assert motor.goal == 1900
    assert bus.clock.now == 0.0


def test_move_wait_reaches_goal(bus, motor, pbm):
    result = pbm.move(1, 1500, wait=True)

    assert abs(result.position - 1500) <= 10
    assert result.contact is False
    assert result.final_effort is None
    assert result.peak_effort is None
    assert result.effort_unit == "mA"


def test_move_wait_timeout_returns_last_position(bus, pbm):
    bus.add_motor(1, speed=100)

    result = pbm.move(1, 1500, wait=True, timeout=0.5)

    assert result.contact is False
    assert 2048 - 60 <= result.position <= 2048 - 40
    assert bus.clock.now == pytest.approx(0.5, abs=0.01)


def test_move_records_effort_with_stop_on_contact(bus, motor, pbm):
    motor.effort = 0xFF38  # -200 mA

    result = pbm.move(1, 1500, stop_on_contact=True)

    assert result.final_effort == 200.0
    assert result.peak_effort == 200.0
    assert result.effort_unit == "mA"


def test_move_effort_unit_for_load_models(bus, pbm):
    bus.add_motor(1, model_number=1080)

    result = pbm.move(1, 1500, stop_on_contact=True)

    assert result.effort_unit == "%"


def test_move_default_does_not_stop_on_contact(bus, motor, pbm):
    motor.obstacle = 1800

    result = pbm.move(1, 1500, wait=True, timeout=0.5)

    assert result.contact is False
    assert result.position == 1800
    # No stop command: the goal is left where the caller put it.
    assert bus.packet.writes_to(ADDR_GOAL_POSITION) == [1500]


def test_move_stop_on_contact_detects_stall(bus, motor, pbm):
    motor.obstacle = 1800
    motor.effort = 400

    result = pbm.move(1, 1500, stop_on_contact=True)

    assert result.contact is True
    assert result.position == 1800
    assert result.peak_effort == 400.0
    assert result.effort_unit == "mA"
    # Contact stops the motor by commanding its present position.
    assert bus.packet.writes_to(ADDR_GOAL_POSITION) == [1500, 1800]
    # Detected promptly, well before the timeout.
    assert bus.clock.now < 0.3


@pytest.mark.parametrize("speed", [1000, 2000, 4000, 8000])
@pytest.mark.parametrize("goal", [1500, 2600])
def test_no_false_contact_during_normal_progress(bus, pbm, speed, goal):
    motor = bus.add_motor(1, speed=speed)
    motor.effort = 403  # high effort alone must not trigger contact

    result = pbm.move(1, goal, stop_on_contact=True)

    assert result.contact is False
    assert abs(result.position - goal) <= 10
    assert bus.packet.writes_to(ADDR_GOAL_POSITION) == [goal]


def test_no_contact_for_short_move_within_tolerance(bus, motor, pbm):
    result = pbm.move(1, 2052, stop_on_contact=True)

    assert result.contact is False
    assert result.position == 2048


# ----------------------------------------------------------------------
# tap()
# ----------------------------------------------------------------------


def test_tap_defaults():
    params = inspect.signature(PBM.tap).parameters

    assert params["stop_on_contact"].default is True
    assert params["hold"].default == 0.05


def test_tap_unobstructed(bus, motor, pbm):
    result = pbm.tap(1, from_position=2048, to_position=1700)

    assert result.contact is False
    assert abs(result.position - 1700) <= 10
    assert bus.packet.writes_to(ADDR_GOAL_POSITION) == [2048, 1700, 2048]
    assert abs(motor.position - 2048) <= 10


def test_tap_stops_on_contact_by_default(bus, motor, pbm):
    motor.obstacle = 1800

    result = pbm.tap(1, from_position=2048, to_position=1500)

    assert result.contact is True
    assert result.position == 1800
    assert bus.packet.writes_to(ADDR_GOAL_POSITION) == [2048, 1500, 1800, 2048]
    assert abs(motor.position - 2048) <= 10


def test_tap_without_stop_on_contact(bus, motor, pbm):
    motor.obstacle = 1800

    result = pbm.tap(
        1, from_position=2048, to_position=1500, stop_on_contact=False
    )

    assert result.contact is False
    assert result.position == 1800
    assert bus.packet.writes_to(ADDR_GOAL_POSITION) == [2048, 1500, 2048]


def test_tap_moves_to_from_position_first(bus, pbm):
    motor = bus.add_motor(1, position=1000)

    pbm.tap(1, from_position=2048, to_position=1700)

    assert bus.packet.writes_to(ADDR_GOAL_POSITION) == [2048, 1700, 2048]
    assert abs(motor.position - 2048) <= 10


def test_tap_holds(bus, motor, pbm):
    pbm.tap(1, from_position=2048, to_position=1700, hold=0.5)

    assert 0.5 in bus.clock.sleeps


@pytest.mark.parametrize("exc", [RuntimeError("bus error"), KeyboardInterrupt()])
def test_tap_returns_to_from_position_on_error(bus, motor, pbm, exc):
    # Fail during the press, which is the first stop_on_contact move.
    bus.packet.raise_on_read = {ADDR_PRESENT_EFFORT: exc}

    with pytest.raises(type(exc)):
        pbm.tap(1, from_position=2048, to_position=1700)

    assert bus.packet.writes_to(ADDR_GOAL_POSITION)[-1] == 2048
    assert abs(motor.position - 2048) <= 10
