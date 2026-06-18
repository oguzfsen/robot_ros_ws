#!/usr/bin/env python3

import RPi.GPIO as GPIO


class Buzzer:
    """
    Simple buzzer controller.

    `update()` fonksiyonu 10 Hz çağrılacak şekilde yazıldı.
    Aşağıdaki süreler "tick" cinsindendir.

    Davranışlar şöyle:
        arm_on(): saniyede 1 kısa bip
        idle(): sessiz
        malfunction(): saniyede 1 uzun bip
        horn(): sürekli bip
    """

    MODE_IDLE = "idle"
    MODE_ARM_ON = "arm_on"
    MODE_MALFUNCTION = "malfunction"
    MODE_HORN = "horn"

    ARM_ON_TICKS = 1
    ARM_OFF_TICKS = 9

    MALFUNCTION_ON_TICKS = 5
    MALFUNCTION_OFF_TICKS = 5

    BUZZER_ON = GPIO.HIGH  # donanım ters çalışırsa bunların yerini değiştir.
    BUZZER_OFF = GPIO.LOW

    def __init__(self, gpio_pin: int) -> None:
        self.gpio_pin = int(gpio_pin)
        self.buzzer_state = False
        self.counter = 0
        self.mode = self.MODE_IDLE
        self.last_mode = None

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.gpio_pin, GPIO.OUT)

        self._write_gpio()

    def arm_on(self) -> None:
        self.mode = self.MODE_ARM_ON

    def idle(self) -> None:
        self.mode = self.MODE_IDLE

    def malfunction(self) -> None:
        self.mode = self.MODE_MALFUNCTION

    def horn(self) -> None:
        self.mode = self.MODE_HORN

    def update(self) -> None:
        if self.mode != self.last_mode:
            self.last_mode = self.mode
            self.counter = 0

        if self.mode == self.MODE_IDLE:
            self.buzzer_state = False
        elif self.mode == self.MODE_ARM_ON:
            self._blink(self.ARM_ON_TICKS, self.ARM_OFF_TICKS)
        elif self.mode == self.MODE_MALFUNCTION:
            self._blink(self.MALFUNCTION_ON_TICKS, self.MALFUNCTION_OFF_TICKS)
        elif self.mode == self.MODE_HORN:
            self.buzzer_state = True
        else:
            self.buzzer_state = False

        self._write_gpio()
        self.counter += 1

    def shutdown(self) -> None:
        self.buzzer_state = False
        self._write_gpio()
        GPIO.cleanup(self.gpio_pin)

    def _blink(self, on_ticks: int, off_ticks: int) -> None:
        cycle_length = on_ticks + off_ticks
        phase = self.counter % cycle_length
        self.buzzer_state = phase < on_ticks

    def _write_gpio(self) -> None:
        if self.buzzer_state:
            GPIO.output(self.gpio_pin, self.BUZZER_ON)
        else:
            GPIO.output(self.gpio_pin, self.BUZZER_OFF)
