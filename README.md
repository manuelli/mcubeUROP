## Static Transform Publisher
You need to publish the transform from `iiwa_link_ee_gripper_aligned` to `cross_tip`. Please run

```
rosrun tf static_transform_publisher 0 -0.07073 -0.025 0 0 0 1 iiwa_link_ee_gripper_aligned cross_tip 100
```

Next run the script that actually moves the robot

```
./check_webcam_vs_robot.py 
```

Next run the april tag detector

```
./extract_corners_from_apriltag.py
```

Now run the calibration
```
./calib3d2d.py 
```