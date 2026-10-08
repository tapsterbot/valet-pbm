# Valet PBM

Python library for controlling a **Valet Push Button Module (PBM)** from a Raspberry Pi using ROBOTIS DYNAMIXEL Protocol 2.0.

Supported motors:

- XC330-T181-T
- XC330-T288-T
- XL430-W250-T
- XC430-W240-T

Valet systems normally ship with the Raspberry Pi, PBM software, and DYNAMIXEL motors already configured. If you are using an assembled Valet, start with **Quick start**. See **Install and hardware setup** if you are cloning this repo, installing PBM into another project, or configuring hardware yourself.

## Quick start

```python
from valet_pbm import PBM

pbm = PBM()

pbm.ping(1)
pbm.blink(1, times=3)

pbm.move(1, 1800)

result = pbm.tap(
    1,
    from_position=2048,
    to_position=1500,
)

print(result)
```

Default behavior:

- `tap()` uses `stop_on_contact=True`
- `move()` uses `stop_on_contact=False`
- `move()` and `tap()` automatically enable torque when needed

## API

### Ping

```python
pbm.ping(1)
```

### LED

```python
pbm.led_on(1)
pbm.led_off(1)
pbm.get_led_status(1)

pbm.blink(1)
pbm.blink(1, times=3)
pbm.blink(1, times=10, interval=0.25)
```

### Torque

```python
pbm.torque_on(1)
pbm.torque_off(1)
pbm.get_torque_status(1)
```

You do not need to call `torque_on()` before `move()` or `tap()`. Those commands automatically enable torque if it is off.

`torque_off()` means "release this motor now." A later move or tap will re-enable torque.

### Move

Send a position command and return immediately:

```python
pbm.move(1, 2048)
```

Wait until the motor reaches the requested position:

```python
result = pbm.move(
    1,
    2048,
    wait=True,
)

print(result.position)
```

Stop early if physical contact is detected:

```python
result = pbm.move(
    1,
    1700,
    stop_on_contact=True,
)

print(result.contact)
print(result.position)
```

`move()` defaults to `stop_on_contact=False`.

### Tap

```python
result = pbm.tap(
    motor_id=1,
    from_position=2048,
    to_position=1700,
)
```

A tap:

1. moves to `from_position`
2. moves toward `to_position`
3. stops early if contact is detected
4. holds briefly
5. returns to `from_position`

`tap()` defaults to `stop_on_contact=True`.

Disable contact detection when you explicitly want the full programmed move:

```python
pbm.tap(
    1,
    from_position=2048,
    to_position=1840,
    stop_on_contact=False,
)
```

Change hold time:

```python
pbm.tap(
    1,
    from_position=2048,
    to_position=1700,
    hold=0.5,
)
```

## MoveResult

Movement operations return a `MoveResult`.

Example:

```python
MoveResult(
    position=1508,
    contact=False,
    final_effort=16.0,
    peak_effort=403.0,
    effort_unit="mA",
)
```

Fields:

- `position`: final measured motor position
- `contact`: whether PBM detected physical contact
- `final_effort`: effort reading when the move finished
- `peak_effort`: highest absolute effort observed during the move
- `effort_unit`: `mA` for XC330 motors or `%` for XL430/XC430 motors

For XC330-T181-T and XC330-T288-T, effort is Present Current.

For XL430-W250-T and XC430-W240-T, effort is Present Load. Present Load is an inferred estimate, not a direct force or torque measurement.

## Contact detection

The public API uses a physical abstraction:

```python
stop_on_contact=True
```

PBM does not require callers to choose a current or load threshold.

Contact detection is based primarily on **position progress**:

- PBM ignores the initial acceleration interval
- PBM keeps a short history of actual motor positions
- if the motor is still short of the goal but stops making enough progress, PBM treats that as contact
- the condition must persist for multiple samples before contact is reported

Effort is recorded as telemetry, but a current/load spike by itself does not trigger contact. Normal acceleration can produce a high `peak_effort` even during an unobstructed move.

The detector can be tuned through the `PBM(...)` constructor if needed.

## Multiple motors

Each motor on the bus must have a unique DYNAMIXEL ID.

```python
pbm.blink(1, times=1)
pbm.blink(2, times=2)
pbm.blink(3, times=3)

pbm.tap(1, 2048, 1700)
pbm.tap(2, 2048, 1850)
pbm.tap(3, 2048, 2250)
```

## Context manager

For scripts, PBM can be used as a context manager:

```python
from valet_pbm import PBM

with PBM() as pbm:
    pbm.tap(
        1,
        from_position=2048,
        to_position=1700,
    )
```

This closes the serial port and GPIO resources automatically.

## Install and hardware setup

### Install from this repository

For development:

```bash
git clone https://github.com/tapsterbot/valet-pbm.git
cd valet-pbm

python3 -m venv .venv
source .venv/bin/activate
pip install -e .
```

Once the package metadata is in place, it can also be installed directly from GitHub into another project:

```bash
pip install git+https://github.com/tapsterbot/valet-pbm.git
```

If/when the package is published to PyPI, the intended install will be:

```bash
pip install valet-pbm
```

### Raspberry Pi prerequisites

On Raspberry Pi OS, install the system packages needed by `lgpio`:

```bash
sudo apt update
sudo apt install python3-dev swig liblgpio-dev
```

### Raspberry Pi UART

Valet is tested with a Raspberry Pi 4 using the PL011 UART.

Add to `/boot/firmware/config.txt`:

```text
enable_uart=1
dtoverlay=disable-bt
```

Disable the Bluetooth UART service:

```bash
sudo systemctl disable hciuart
```

Also disable the serial login console.

Reboot, then verify:

```bash
ls -l /dev/serial0
```

For the tested Valet configuration, `/dev/serial0` should resolve to `ttyAMA0`.

### DYNAMIXEL motor settings

Configure each PBM motor with:

| Setting | Value |
| --- | --- |
| Protocol | DYNAMIXEL Protocol 2.0 |
| Baud Rate | **3 Mbps** |
| Return Delay Time | **254** (508 us) |
| Status Return Level | **2** |
| Operating Mode | **3** (Position Control Mode) |
| Bus Watchdog | **0** |

**3 Mbps** is the recommended Valet setting based on testing with the Raspberry Pi 4 PL011 UART and current PBM hardware.

- **Return Delay Time 254** gives the Pi time to switch the half-duplex interface back to receive mode before the motor responds.
- **Status Return Level 2** makes the motor return status packets for all instructions.
- **Operating Mode 3** is the Position Control Mode PBM currently expects.
- **Bus Watchdog 0** leaves the watchdog disabled.

Baud Rate, Return Delay Time, and Operating Mode are EEPROM settings. Torque must be off before changing EEPROM values.

DYNAMIXEL Wizard 2.0 is a convenient way to configure motors.

## Limiting motor force / output

DYNAMIXEL motors can be configured to push less aggressively.

All four supported motors provide `PWM Limit(36)`. Lowering the PWM limit reduces available output.

The XC330-T181-T and XC330-T288-T also support Current-based Position Control Mode, where Goal Current can limit motor current/torque while the motor still moves toward a Goal Position.

PBM currently uses normal Position Control Mode and software contact detection.

## Supported model detection

PBM reads the DYNAMIXEL Model Number automatically.

| Model | Model Number | Effort signal |
| --- | ---: | --- |
| XC330-T181-T | 1210 | Present Current |
| XC330-T288-T | 1220 | Present Current |
| XL430-W250-T | 1060 | Present Load |
| XC430-W240-T | 1080 | Present Load |

## Implementation notes

Valet PBM uses GPIO18 for half-duplex direction control by default.

The transmit sequence calls:

```python
port.ser.flush()
```

before switching the external bus interface back to receive mode.

PBM starts its receive timeout after TX has completed and the half-duplex interface has switched to RX.

The defaults are:

```python
PBM(
    retries=3,
    response_timeout_ms=10,
    retry_delay=0.001,
)
```

Successful transactions return as soon as a complete response arrives; they do not wait the full timeout.

If a write acknowledgement is missed, PBM reads the target register back before retrying. This distinguishes a lost acknowledgement from a command that the motor did not accept.

## Current limitations

- Contact detection still needs final mechanical calibration against the production PBM.
- `move(..., stop_on_contact=True)` uses software polling rather than motor-side force limiting.
- PBM currently expects Position Control Mode.
- The library intentionally targets the four listed motors rather than every DYNAMIXEL X-series actuator.
