"""Ping a PBM motor and print its model name."""

from pbm import PBM

MOTOR_ID = 1

with PBM() as pbm:
    if not pbm.ping(MOTOR_ID):
        raise SystemExit(f"motor {MOTOR_ID} did not respond")

    print(f"motor {MOTOR_ID}: {pbm.get_model_name(MOTOR_ID)}")
