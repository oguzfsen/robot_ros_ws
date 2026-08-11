import math


class Ackermann:
    def __init__(self, wheelbase, wheel_track, wheel_diameter):
        self.wheelbase = wheelbase
        self.wheel_track = wheel_track
        self.wheel_diameter = wheel_diameter

    def get_fixed(self, throttle, avarage_steering_angle):
        throttle = float(throttle)
        steering_angle = float(avarage_steering_angle)

        if throttle == 0.0 or abs(steering_angle) < 0.1:
            fixed_throttle = int(round(throttle))
            return (
                fixed_throttle,
                fixed_throttle,
                fixed_throttle,
                fixed_throttle,
            )

        # Front and rear wheels steer in opposite directions. Therefore the
        # instantaneous centre of rotation is halfway between the axles.
        steering_radians = math.radians(min(abs(steering_angle), 89.9))
        half_wheelbase = self.wheelbase / 2.0
        half_wheel_track = self.wheel_track / 2.0
        turn_radius = half_wheelbase / math.tan(steering_radians)

        inner_wheel_radius = math.hypot(
            half_wheelbase,
            turn_radius - half_wheel_track,
        )
        outer_wheel_radius = math.hypot(
            half_wheelbase,
            turn_radius + half_wheel_track,
        )
        inner_throttle = int(round(throttle * inner_wheel_radius / outer_wheel_radius))
        outer_throttle = int(round(throttle))

        # Positive steering angle means a right turn.
        if steering_angle > 0.0:
            return (
                outer_throttle,
                inner_throttle,
                outer_throttle,
                inner_throttle,
            )

        return (
            inner_throttle,
            outer_throttle,
            inner_throttle,
            outer_throttle,
        )
