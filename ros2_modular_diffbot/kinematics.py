"""Differential-drive kinematics shared by the hardware bridge and tests."""


def twist_to_pwm(
    linear_velocity,
    angular_velocity,
    wheel_base,
    wheel_radius,
    speed_to_pwm_scale,
    max_pwm,
):
    """Convert a robot twist into clamped left and right wheel PWM commands."""
    if wheel_base < 0:
        raise ValueError("wheel_base must be non-negative")
    if wheel_radius <= 0:
        raise ValueError("wheel_radius must be positive")
    if speed_to_pwm_scale < 0:
        raise ValueError("speed_to_pwm_scale must be non-negative")
    if max_pwm < 0:
        raise ValueError("max_pwm must be non-negative")

    right_linear_velocity = linear_velocity + angular_velocity * wheel_base / 2.0
    left_linear_velocity = linear_velocity - angular_velocity * wheel_base / 2.0

    right_angular_velocity = right_linear_velocity / wheel_radius
    left_angular_velocity = left_linear_velocity / wheel_radius

    left_pwm = int(
        max(-max_pwm, min(max_pwm, left_angular_velocity * speed_to_pwm_scale))
    )
    right_pwm = int(
        max(-max_pwm, min(max_pwm, right_angular_velocity * speed_to_pwm_scale))
    )
    return left_pwm, right_pwm
