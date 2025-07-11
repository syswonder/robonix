#!/usr/bin/env python3
import time
import logging
from typing import Optional, Callable
import RPi.GPIO as GPIO
import threading


class BM01Driver:
    def __init__(
        self,
        gpio_pin: int = 20,
        callback: Optional[Callable] = None,
        debounce_ms: int = 10,
    ):
        self.gpio_pin = gpio_pin
        self.callback = callback
        self.debounce_ms = debounce_ms
        self.is_initialized = False
        self.logger = logging.getLogger(__name__)
        self.last_pressure_state = False
        self._callback_lock = threading.Lock()

        try:
            GPIO.remove_event_detect(self.gpio_pin)
        except:
            pass

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.gpio_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        
        self.is_initialized = True
        
        self.last_pressure_state = self.read_digital()
        self.logger.info(f"Initial GPIO {gpio_pin} state: {self.last_pressure_state}")

        if self.callback:
            try:
                GPIO.add_event_detect(
                    self.gpio_pin,
                    GPIO.BOTH,
                    callback=self._pressure_callback,
                    bouncetime=self.debounce_ms,
                )
                self.logger.info(f"GPIO interrupt enabled for pin {gpio_pin}")
            except RuntimeError as e:
                self.logger.warning(
                    f"Failed to add edge detection: {e}. Continuing without interrupt."
                )
                self.callback = None

        self.logger.info(
            f"BM01 driver initialized on GPIO {gpio_pin} with {debounce_ms}ms debounce"
        )

    def _pressure_callback(self, channel):
        if self.callback:
            with self._callback_lock:
                current_state = self.read_digital()
                if current_state and not self.last_pressure_state:
                    self.logger.info(f"Pressure detected on GPIO {self.gpio_pin}")
                    self.callback()
                elif not current_state and self.last_pressure_state:
                    self.logger.info(f"Pressure released on GPIO {self.gpio_pin}")
                
                self.last_pressure_state = current_state

    def read_digital(self) -> bool:
        if not self.is_initialized:
            raise RuntimeError("Driver not initialized")

        return GPIO.input(self.gpio_pin) == GPIO.HIGH

    def read_pressure_state(self) -> Optional[dict]:
        digital_state = self.read_digital()

        if digital_state:
            return {
                "timestamp": time.time(),
                "pressure_detected": True,
                "gpio_pin": self.gpio_pin,
            }
        return None

    def wait_for_pressure(self, timeout: Optional[float] = None) -> bool:
        start_time = time.time()

        while True:
            if self.read_digital():
                return True

            if timeout and (time.time() - start_time) > timeout:
                return False

            time.sleep(0.001)

    def cleanup(self):
        if self.is_initialized:
            GPIO.remove_event_detect(self.gpio_pin)
            GPIO.cleanup(self.gpio_pin)
            self.is_initialized = False
            self.logger.info("BM01 driver cleaned up")


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    def pressure_detected():
        print(f"Pressure detected at {time.time():.6f}")

    driver = BM01Driver(callback=pressure_detected, debounce_ms=5)

    try:
        print("BM01 Driver Test - Only showing pressure detection events")
        print("Press Ctrl+C to exit")

        while True:
            state = driver.read_pressure_state()
            if state:
                print(f"Pressure state: {state}")

            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        driver.cleanup()
