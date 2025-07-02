#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2025 wheatfox
import RPi.GPIO as gpio
import time
import signal
import sys
import yaml
import os
from typing import Dict, Optional, List


class HCSR04Driver:
    def __init__(self, trig: int, echo: int):
        """
        Initialize the HC-SR04 ultrasonic sensor driver

        Args:
            trig (int): GPIO number for trigger
            echo (int): GPIO number for echo
        """
        self.trig_pin = trig
        self.echo_pin = echo

        gpio.setwarnings(False)
        gpio.setmode(gpio.BCM)
        gpio.setup(self.trig_pin, gpio.OUT)
        gpio.setup(self.echo_pin, gpio.IN)

    def get_distance(self, timeout: float = 0.05) -> Optional[float]:
        """
        Get distance measurement from the sensor with timeout protection.
        Optimized for real-time collision prevention.

        Args:
            timeout (float): Maximum time to wait for echo signal in seconds (default: 0.05s)

        Returns:
            Optional[float]: Distance in cm, or None if measurement failed
        """
        try:
            # Send trigger signal - minimize delays
            gpio.output(self.trig_pin, False)
            time.sleep(0.0001)  # Reduced from 0.1s to 0.0001s
            gpio.output(self.trig_pin, True)
            time.sleep(0.00001)
            gpio.output(self.trig_pin, False)

            # Wait for echo start with timeout
            start_time = time.time()
            while gpio.input(self.echo_pin) == 0:
                if time.time() - start_time > timeout:
                    return None
                # Remove sleep to improve responsiveness
            
            pulse_start = time.time()

            # Wait for echo end with timeout
            while gpio.input(self.echo_pin) == 1:
                if time.time() - start_time > timeout:
                    return None
                # Remove sleep to improve responsiveness
            
            pulse_end = time.time()
            pulse_duration = pulse_end - pulse_start

            # Quick validation
            if pulse_duration >= 0.01746 or pulse_duration <= 0.00001:
                return None

            distance = pulse_duration * 17000

            # Adjust range check for collision prevention
            if distance > 400 or distance < 2:  # Extended range for better detection
                return None

            return round(distance, 1)  # Reduced precision for faster processing

        except Exception as e:
            print(f"Error getting distance: {e}")
            return None


class MultiHCSR04Driver:
    def __init__(self, sensor_configs: Optional[List[Dict[str, int]]] = None):
        if sensor_configs is None:
            # Try to load from YAML file
            current_dir = os.path.dirname(os.path.abspath(__file__))
            config_path = os.path.join(current_dir, "sensors.yaml")
            try:
                with open(config_path, "r") as f:
                    config = yaml.safe_load(f)
                    sensor_configs = config["sensors"]
                    print(
                        f"su ccessfully loaded sensor configurations from {config_path}"
                    )
            except Exception as e:
                print(f"failed to load sensor configurations: {str(e)}")
                raise

        self.sensors = {}
        for idx, config in enumerate(sensor_configs):
            name = config.get("name", f"sensor{idx}")
            self.sensors[name] = HCSR04Driver(config["trig"], config["echo"])

    def get_all_distances(self) -> Dict[str, Optional[float]]:
        return {name: sensor.get_distance() for name, sensor in self.sensors.items()}

    def get_sensor_names(self) -> List[str]:
        return list(self.sensors.keys())

    def cleanup(self):
        gpio.cleanup()

    def __str__(self):
        # print each sensor name and trig and echo pin
        return "\n".join(
            [f"{name}: trig={sensor.trig_pin}, echo={sensor.echo_pin}"
             for name, sensor in self.sensors.items()]
        )


def main():
    def signal_handler(signal, frame):
        print("You pressed Ctrl+C!")
        driver.cleanup()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    # Initialize driver without explicit configs - will load from YAML
    driver = MultiHCSR04Driver()

    print("starting ultrasonic sensor driver")
    print(
        f"initialized {len(driver.get_sensor_names())} sensors: {driver}"
    )

    try:
        while True:
            distances = driver.get_all_distances()
            for sensor_name, distance in distances.items():
                if distance is None:
                    print(f"{sensor_name}: measurement failed")
                else:
                    print(f"{sensor_name}: {distance} cm")
            time.sleep(0.05)  # Reduced from 0.1s to 0.05s for faster updates

    except Exception as e:
        print(f"error: {e}")
    finally:
        driver.cleanup()


if __name__ == "__main__":
    main()
