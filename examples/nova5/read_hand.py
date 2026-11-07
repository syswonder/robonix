# SPDX-License-Identifier: MulanPSL-2.0
"""Read the PGI status without commanding motion."""
import argparse
import json

import serial
from pgi_hand.protocol import read_register, read_status


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    args = parser.parse_args()
    with serial.Serial(args.port, baudrate=115200, timeout=.25, exclusive=True) as port:
        state = read_status(port, 1, .25)
        state["configuration"] = {
            "force_setpoint_percent": read_register(port, 1, 0x0101, .25),
            "target_position_raw": read_register(port, 1, 0x0103, .25),
            "speed_setpoint_percent": read_register(port, 1, 0x0104, .25),
        }
    print(json.dumps(state))


if __name__ == "__main__":
    main()
