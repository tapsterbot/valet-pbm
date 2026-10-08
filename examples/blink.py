"""Blink a PBM motor's LED."""

from pbm import PBM

MOTOR_ID = 1

with PBM() as pbm:
    pbm.blink(MOTOR_ID, times=5, interval=0.2)
