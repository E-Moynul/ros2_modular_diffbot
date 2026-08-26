"""Tests for differential-drive twist to PWM conversion."""

import unittest

from ros2_modular_diffbot.kinematics import twist_to_pwm


DEFAULTS = {
    "wheel_base": 0.1,
    "wheel_radius": 0.0325,
    "speed_to_pwm_scale": 14.0,
    "max_pwm": 200,
}


def convert(linear_velocity, angular_velocity, **overrides):
    """Call the converter with the bridge's default parameters."""
    parameters = DEFAULTS | overrides
    return twist_to_pwm(linear_velocity, angular_velocity, **parameters)


class KinematicsTest(unittest.TestCase):
    """Exercise nominal motion, saturation, and invalid parameters."""

    def test_straight_motion_drives_both_wheels_equally(self):
        """A zero-yaw command should produce matching wheel commands."""
        self.assertEqual(convert(0.325, 0.0), (140, 140))

    def test_in_place_rotation_drives_wheels_in_opposite_directions(self):
        """Pure yaw should produce equal and opposite wheel commands."""
        self.assertEqual(convert(0.0, 1.0), (-21, 21))

    def test_curved_motion_preserves_left_right_order(self):
        """A positive yaw command should drive the right wheel faster."""
        self.assertEqual(convert(0.1, 1.0), (21, 64))

    def test_pwm_is_clamped_in_both_directions(self):
        """Large forward and reverse commands must respect the PWM limit."""
        cases = [(1.0, (200, 200)), (-1.0, (-200, -200))]
        for linear_velocity, expected in cases:
            with self.subTest(linear_velocity=linear_velocity):
                self.assertEqual(convert(linear_velocity, 0.0), expected)

    def test_invalid_drive_parameters_are_rejected(self):
        """Invalid geometry or scaling parameters should fail clearly."""
        cases = [
            ("wheel_base", -0.1),
            ("wheel_radius", 0.0),
            ("wheel_radius", -0.1),
            ("speed_to_pwm_scale", -1.0),
            ("max_pwm", -1),
        ]
        for parameter, value in cases:
            with self.subTest(parameter=parameter, value=value):
                with self.assertRaises(ValueError):
                    convert(0.1, 0.0, **{parameter: value})


if __name__ == "__main__":
    unittest.main()
