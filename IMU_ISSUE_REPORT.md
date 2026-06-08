# HoloOcean test.bag — IMU issue report

Hi! I tried to use the IMU + DVL from your bag to run odometry (EKF via
`robot_localization`) and feed it into Bruce-SLAM. The DVL works fine, but the
IMU has two problems that make it unusable for navigation. Details below.

## Bag contents (`test.bag`)

| Topic           | Type                        | Rate    |
|-----------------|-----------------------------|---------|
| `/sonar`        | `sensor_msgs/Image` (512×512, mono8, polar) | ~1 Hz |
| `/dvl`          | `geometry_msgs/TwistStamped`| ~500 Hz |
| `/imu`          | `sensor_msgs/Imu`           | ~500 Hz |
| `/ground_truth` | `nav_msgs/Odometry`         | ~500 Hz |

## Problem 1 — IMU `orientation` is always empty

Every `/imu` message has:

```
orientation: x=0.0  y=0.0  z=0.0  w=1.0   (identity quaternion, never updated)
```

So there is no absolute orientation from the IMU at all.

## Problem 2 — IMU `angular_velocity.z` does not match the real motion

I integrated the yaw rate (`angular_velocity.z`) over the whole run and compared
to the ground-truth yaw:

- Ground-truth total yaw change: **+452°**
- Integrated `angular_velocity.z`:  **-1729°**
- Correlation with GT yaw rate: **-0.94** (so it's the right signal, but sign-flipped)
- Ratio integrated/GT ≈ **-1.9**
- Even after fitting the best scale factor, residual yaw error stays at **71° RMS**

→ The AUV spirals; SLAM trajectory is completely wrong (ATE 17–35 m).

## What works

The DVL is fine — body-frame speed magnitude matches GT:
- DVL mean speed: **1.92 m/s**
- GT  mean speed: **1.89 m/s**

## What I need in a new bag

Either of these would unblock the DVL+IMU odometry:

1. **IMU `orientation` filled** with a valid quaternion (like `/ground_truth` has), OR
2. **`angular_velocity` correct**: right axis, units in rad/s, correct sign.

Also please confirm the sonar parameters I assumed: **RangeMax = 40 m**, **Azimuth (FOV) = 120°**.

Thanks!
