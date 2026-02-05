from __future__ import annotations

import time

import RPi.GPIO as GPIO


MOTORS = {"left": 17, "center": 27, "right": 22}
BUTTONS = {"button1": 23, "button2": 24, "button3": 25}


def main() -> int:
    GPIO.setmode(GPIO.BCM)

    for gpio in MOTORS.values():
        GPIO.setup(gpio, GPIO.OUT, initial=GPIO.LOW)

    for gpio in BUTTONS.values():
        GPIO.setup(gpio, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    try:
        print("Motor test starting...")
        for name, gpio in MOTORS.items():
            print(f"Motor {name}: ON")
            GPIO.output(gpio, GPIO.HIGH)
            time.sleep(0.4)
            GPIO.output(gpio, GPIO.LOW)
            print(f"Motor {name}: OFF")
            time.sleep(0.2)

        print("Button test for 10 seconds (press buttons)...")
        end = time.monotonic() + 10.0
        last_state = {k: GPIO.input(gpio) for k, gpio in BUTTONS.items()}
        while time.monotonic() < end:
            for name, gpio in BUTTONS.items():
                state = GPIO.input(gpio)
                if state != last_state[name]:
                    last_state[name] = state
                    pressed = state == GPIO.LOW
                    print(f"{name}: {'PRESSED' if pressed else 'RELEASED'}")
            time.sleep(0.01)

        print("Done.")
        return 0
    finally:
        for gpio in MOTORS.values():
            try:
                GPIO.output(gpio, GPIO.LOW)
            except Exception:
                pass
        GPIO.cleanup()


if __name__ == "__main__":
    raise SystemExit(main())

