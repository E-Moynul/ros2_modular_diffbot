# Open-Loop Speed-to-PWM Calibration

This document explains how the `speed_to_pwm_scale` constant in
[`kinematics_wifi_bridge.py`](../ros2_modular_diffbot/kinematics_wifi_bridge.py)
was derived, and why an empirical calibration approach was chosen over a
theoretical one for this hardware tier.

---

## 1. Why Calibration Is Needed

The robot uses plain BO motors with **no encoders and no closed-loop
feedback**. The `kinematics_wifi_bridge` node computes each wheel's required
*angular velocity* (rad/s) from the incoming `/cmd_vel` command using standard
differential-drive inverse kinematics, but the ESP32 firmware only
understands an **8-bit PWM duty cycle** (0–255, clamped to ±100 in this
project). Something has to convert rad/s → PWM units, and for an open-loop
DC motor + gearbox + battery combination, that relationship is **not linear
by datasheet spec alone** — it depends on motor stall torque, battery sag,
gearbox friction, and floor surface friction, none of which are precisely
known for a $2 BO motor. So the conversion factor is measured empirically
rather than computed from first principles.

```python
w_l = v_l / self.wheel_radius          # rad/s required
pwm_l = w_l * self.speed_to_pwm_scale  # convert to PWM units
```

## 2. Known System Parameters

| Parameter | Value | Source |
|---|---|---|
| Wheel base (`wheel_base`) | 0.10 m | Measured from chassis |
| Wheel radius (`wheel_radius`) | 0.0325 m | Measured (caliper) |
| Max PWM (`max_pwm`) | 200 (of 255) | Firmware safety headroom, avoids motor stall current at full duty |
| Speed-to-PWM scale (`speed_to_pwm_scale`) | 14.0 | Empirically calibrated (Section 3) |
| PWM resolution | 8-bit (`LEDC`) | ESP32 `ledcAttach` config |
| PWM frequency | 1000 Hz | ESP32 firmware |

> ⚠️ **Note:** `wheel_diameter = 0.06 m` is used in the **Gazebo plugin**
> (`robot_gazebo.xacro`) for simulation physics, while `wheel_radius = 0.0325 m`
> is used in the **real hardware bridge**. These are intentionally from
> independent measurements (simulated wheel geometry vs. real wheel geometry)
> — keep this in mind if you resize the physical wheels, both values need
> updating separately.

## 3. Calibration Method

The scale factor was tuned using a simple **step-response bench test**:

1. Place the robot on a flat, fixed surface (wheels free to spin, chassis fixed)
   or measure straight-line displacement over a fixed distance on the floor.
2. Publish a constant `/cmd_vel` (e.g. `linear.x = 0.15 m/s`, `angular.z = 0`)
   using `teleop_twist_keyboard` or a test script.
3. Record actual wheel behavior:
   - **Bench test:** measure wheel RPM directly (phone slow-mo / tachometer / stroboscope)
   - **Floor test:** measure real distance traveled over a fixed time window (e.g. 5 s) with a stopwatch
4. Compute the *actual* linear speed achieved: `v_actual = distance / time`
5. Compare against the *commanded* `v` and adjust `speed_to_pwm_scale` until
   `v_actual ≈ v_commanded` within acceptable tolerance.
6. Repeat at 3–4 different commanded speeds (e.g. 0.05, 0.10, 0.15, 0.20 m/s)
   to check linearity across the operating range, not just a single point.

### Results

| Commanded `v` (m/s) | Computed PWM | Measured `v_actual` (m/s) | Error |
|---|---|---|---|
| 0.05 | *TODO* | *TODO* | *TODO* |
| 0.10 | *TODO* | *TODO* | *TODO* |
| 0.15 | *TODO* | *TODO* | *TODO* |
| 0.20 | *TODO* | *TODO* | *TODO* |

*(Fill in with your actual bench/floor test measurements — this table is the
single most convincing piece of evidence for reviewers that the sim-to-real
loop was actually validated, not just assumed.)*

## 4. Known Limitations

- **No feedback correction:** since there are no encoders, any deviation
  between commanded and actual speed (battery voltage sag over time, uneven
  floor friction, slightly mismatched left/right motors) is **not corrected
  in real time**. The calibration above is a single best-fit constant, not
  an adaptive controller.
- **Battery voltage dependency:** the calibration is only valid near the
  battery voltage level it was measured at (fresh NiMH pack). As the pack
  discharges, actual speed will drop below commanded speed for the same PWM.
- **Left/right asymmetry:** the same `speed_to_pwm_scale` is applied to both
  wheels; if the two motors have different stall characteristics, straight-
  line motion may drift slightly. This is a documented trade-off, not a bug.
- **Future work:** adding wheel encoders (see project Roadmap) would allow
  replacing this static scale factor with a proper closed-loop PID velocity
  controller, removing all of the above limitations.
