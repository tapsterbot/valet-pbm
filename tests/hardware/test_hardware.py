"""End-to-end tests against a connected PBM.

These tests talk to real DYNAMIXEL hardware and MOVE THE MOTOR. They are
skipped unless pytest runs with --hardware (``make test-hw``).

Configure with environment variables:

    PBM_DEVICE          serial device               (default /dev/serial0)
    PBM_MOTOR_ID        motor under test            (default 1)
    PBM_FROM_POSITION   tap start/release position  (default 2048)
    PBM_TO_POSITION     tap press position          (default 1700)

All motion stays between PBM_FROM_POSITION and PBM_TO_POSITION, and every
press uses contact detection, so the motor stops if it meets the button.
"""

import os

import pytest

from pbm import MOTOR_PROFILES, PBM, MoveResult

pytestmark = pytest.mark.hardware

DEVICE = os.environ.get("PBM_DEVICE", "/dev/serial0")
MOTOR_ID = int(os.environ.get("PBM_MOTOR_ID", "1"))
FROM_POSITION = int(os.environ.get("PBM_FROM_POSITION", "2048"))
TO_POSITION = int(os.environ.get("PBM_TO_POSITION", "1700"))

TOLERANCE = 10


@pytest.fixture(scope="module")
def pbm():
    if not os.path.exists(DEVICE):
        pytest.fail(f"{DEVICE} not found; set PBM_DEVICE")

    with PBM(device=DEVICE) as device:
        if not device.ping(MOTOR_ID):
            pytest.fail(
                f"motor {MOTOR_ID} did not answer ping on {DEVICE}; "
                "check power, wiring, and PBM_MOTOR_ID"
            )

        torque_was_on = device.get_torque_status(MOTOR_ID)

        yield device

        # Leave the LED off and torque as we found it. Motion tests return
        # the motor to FROM_POSITION themselves; teardown never moves it.
        try:
            device.led_off(MOTOR_ID)
        finally:
            if torque_was_on:
                device.torque_on(MOTOR_ID)
            else:
                device.torque_off(MOTOR_ID)


def test_ping(pbm):
    assert pbm.ping(MOTOR_ID) is True


def test_model_is_supported(pbm):
    names = {profile.name for profile in MOTOR_PROFILES.values()}

    assert pbm.get_model_name(MOTOR_ID) in names


def test_led(pbm):
    pbm.led_on(MOTOR_ID)
    assert pbm.get_led_status(MOTOR_ID) is True

    pbm.led_off(MOTOR_ID)
    assert pbm.get_led_status(MOTOR_ID) is False

    pbm.blink(MOTOR_ID, times=2, interval=0.05)
    assert pbm.get_led_status(MOTOR_ID) is False


def test_torque(pbm):
    pbm.torque_off(MOTOR_ID)
    assert pbm.get_torque_status(MOTOR_ID) is False

    pbm.torque_on(MOTOR_ID)
    assert pbm.get_torque_status(MOTOR_ID) is True


def test_bus_reliability(pbm):
    # Exercise the half-duplex turnaround: every read must succeed (after
    # PBM's own retries) and return a sane position.
    for _ in range(200):
        assert 0 <= pbm.get_position(MOTOR_ID) <= 4095


def test_effort(pbm):
    effort, unit = pbm.get_effort(MOTOR_ID)

    assert isinstance(effort, float)
    assert unit in {"mA", "%"}


def test_move_enables_torque_and_reaches_goal(pbm):
    pbm.torque_off(MOTOR_ID)

    result = pbm.move(MOTOR_ID, FROM_POSITION, wait=True)

    assert pbm.get_torque_status(MOTOR_ID) is True
    assert result.contact is False
    assert abs(result.position - FROM_POSITION) <= TOLERANCE


def test_tap(pbm):
    result = pbm.tap(
        MOTOR_ID,
        from_position=FROM_POSITION,
        to_position=TO_POSITION,
    )

    print(f"\ntap result: {result}")

    assert isinstance(result, MoveResult)
    assert result.position is not None
    assert result.effort_unit in {"mA", "%"}

    low, high = sorted((FROM_POSITION, TO_POSITION))
    assert low - TOLERANCE <= result.position <= high + TOLERANCE

    # tap() always returns to the release position.
    assert abs(pbm.get_position(MOTOR_ID) - FROM_POSITION) <= TOLERANCE

