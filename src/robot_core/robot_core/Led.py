#!/usr/bin/env python3

from robot_core.Tools import RobotState
import RPi.GPIO as GPIO


class Led:
    """
    Status LED controller.

    `update()` fonksiyonu 10 Hz çağrılacak şekilde yazıldı.
    Aşağıdaki süreler "tick" cinsindendir.
    """

    IDLE_ON_TICKS = 1
    IDLE_OFF_TICKS = 9

    MANUAL_ON_TICKS = 6
    MANUAL_OFF_TICKS = 14

    MISION_ON_TICKS = 5
    MISION_OFF_TICKS = 5

    MALFUNTION_ON_TICKS = 2
    MALFUNTION_OFF_TICKS = 2

    LED_ON = GPIO.LOW
    LED_OFF = GPIO.HIGH

    def __init__(self, gpio_pin: int) -> None:
        self.gpio_pin = int(gpio_pin)
        self.led_state = False
        self.counter = 0
        self.last_state = None

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.gpio_pin, GPIO.OUT)

        self._write_gpio()

    def update(self, robot_state: RobotState) -> None:
        if robot_state != self.last_state:
            self.last_state = robot_state
            self.counter = 0

        if robot_state == RobotState.IDLE:
            self.update_idle()
        elif robot_state == RobotState.MANUAL:
            self.update_manual()
        elif robot_state == RobotState.MISION:
            self.update_mision()
        elif robot_state == RobotState.MALFUNTION:
            self.update_malfuntion()
        elif robot_state == RobotState.EMERGENCY:
            self.update_other()
        else:
            self.led_state = False

        self._write_gpio()
        self.counter += 1

    def update_idle(self) -> None:
        self._blink(self.IDLE_ON_TICKS, self.IDLE_OFF_TICKS)

    def update_manual(self) -> None:
        self._blink(self.MANUAL_ON_TICKS, self.MANUAL_OFF_TICKS)

    def update_mision(self) -> None:
        self._blink(self.MISION_ON_TICKS, self.MISION_OFF_TICKS)

    def update_malfuntion(self) -> None:
        self._blink(self.MALFUNTION_ON_TICKS, self.MALFUNTION_OFF_TICKS)

    def update_other(self) -> None:
        self.led_state = True

    def shutdown(self) -> None:
        self.led_state = False
        self._write_gpio()
        GPIO.cleanup(self.gpio_pin)

    def _blink(self, on_ticks: int, off_ticks: int) -> None:
        cycle_length = on_ticks + off_ticks
        phase = self.counter % cycle_length
        self.led_state = phase < on_ticks

    def _write_gpio(self) -> None:
        if self.led_state:
            GPIO.output(self.gpio_pin, self.LED_ON)
        else:
            GPIO.output(self.gpio_pin, self.LED_OFF)
