"""Perform a normal PBM tap.

Adjust the positions for your PBM before running.
"""

from pbm import PBM

MOTOR_ID = 1
FROM_POSITION = 2048
TO_POSITION = 1700

with PBM() as pbm:
    result = pbm.tap(
        MOTOR_ID,
        from_position=FROM_POSITION,
        to_position=TO_POSITION,
    )

    print(result)
