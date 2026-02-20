# Human Tracking Drive Feature -- Geometric Rotation Model

## 1. Overview

This feature enables a mobile robot in Gazebo simulation to track and
rotate toward a detected human using geometric angle estimation.

The robot: - Uses YOLOv8 bounding box output. - Computes horizontal
offset of the human relative to the camera center. - Estimates rotation
angle using trigonometry. - Stops if bounding box exceeds vertical frame
limits.

Camera is mounted at the center front of the robot, aligned with robot
heading.

------------------------------------------------------------------------

## 2. Camera Configuration

    frame_width  = 640 px
    frame_height = 480 px

Camera center:

    center_x = 320 px
    center_y = 240 px

Camera optical axis is aligned with robot forward direction.

------------------------------------------------------------------------

## 3. Detection Input

For detected human:

``` python
bbox = {
    x_min: int,
    y_min: int,
    x_max: int,
    y_max: int,
    confidence: float
}
```

Constraints: - Only class = "person" - confidence ≥ threshold (e.g.,
0.5) - If multiple persons → select largest bounding box area

------------------------------------------------------------------------

## 4. Derived Parameters

### 4.1 Bounding Box Center

``` python
bbox_center_x = (x_min + x_max) / 2
bbox_center_y = (y_min + y_max) / 2
```

### 4.2 Horizontal Pixel Offset

``` python
delta_px = bbox_center_x - center_x
```

-   delta_px \> 0 → human to the right
-   delta_px \< 0 → human to the left

------------------------------------------------------------------------

## 5. Convert Pixel Offset to Real-World Lateral Distance (m)

We define: - h = estimated distance from robot to human (meters) - m =
lateral displacement (meters)

### 5.1 Distance h Estimation

Distance may be estimated using bounding box height:

    h = (real_human_height * focal_length_px) / bbox_height

Where: - real_human_height ≈ 1.7 m - focal_length_px = calibrated focal
length in pixels

OR

Distance may be provided by: - Depth camera - LiDAR - Simulation
ground-truth

### 5.2 Convert Pixel Offset to Metric Offset

Using pinhole camera model:

    m = (delta_px * h) / focal_length_px

------------------------------------------------------------------------

## 6. Rotation Angle Calculation

Right triangle definition: - Hypotenuse = h - Opposite side = m - Angle
= θ

Compute:

    θ = arcsin(m / h)

Equivalent (using pinhole relation):

    θ = arcsin(delta_px / focal_length_px)

Result: - θ \> 0 → rotate right - θ \< 0 → rotate left

Units: radians

------------------------------------------------------------------------

## 7. Angular Velocity Control

    angular.z = K_ang * θ

Where:

    K_ang ∈ [0.5, 2.0]

Clamp:

    angular.z ∈ [-1.5, 1.5] rad/s

Deadzone:

If:

    |θ| < 0.02 rad

Then:

    angular.z = 0

------------------------------------------------------------------------

## 8. Linear Motion Control

Robot moves forward only if: - \|θ\| \< angle_threshold - Bounding box
vertical edges within safe range

------------------------------------------------------------------------

## 9. Stop Conditions

### 9.1 Vertical Frame Violation

    vertical_margin = 10 px

If:

    y_min <= vertical_margin
    OR
    y_max >= frame_height - vertical_margin

Then:

    linear.x = 0
    angular.z = 0

### 9.2 No Detection

If no valid human:

    linear.x = 0
    angular.z = 0

------------------------------------------------------------------------

## 10. Forward Speed Logic

If: - \|θ\| \< angle_threshold - No stop condition

Then:

    linear.x = K_lin * (desired_distance - h)

Where:

    desired_distance ≈ 1.5 m
    K_lin ≈ 0.5

Clamp:

    linear.x ∈ [0.0, 0.6]

If:

    h <= desired_distance

Then:

    linear.x = 0

------------------------------------------------------------------------

## 11. Final Velocity Output

Publish to `/cmd_vel`:

    Twist:
        linear.x  = computed_value
        angular.z = computed_value

Publishing frequency: 20 Hz recommended.

------------------------------------------------------------------------

## 12. Control Pipeline

1.  Receive detection
2.  Validate bounding box
3.  Compute bbox center
4.  Compute delta_px
5.  Estimate distance h
6.  Compute m
7.  Compute θ = arcsin(m/h)
8.  Apply angular control
9.  Check stop conditions
10. Apply linear control
11. Publish velocity

------------------------------------------------------------------------

## 13. Safety Constraints

-   Clamp all velocities
-   Ignore extremely small bounding boxes
-   Stop if h estimation unstable
-   Ensure \|m/h\| ≤ 1 before arcsin()

------------------------------------------------------------------------

# Behavioral Summary

  Condition                               Action
  --------------------------------------- --------------
  No detection                            Stop
  Human left                              Rotate left
  Human right                             Rotate right
  Human centered                          Move forward
  Human too close                         Stop
  Bounding box touches frame top/bottom   Stop


Implement this as a ROS2 node that subscribes to detection results and publishes /cmd_vel.