# NUSLAM
A package implementing Extended Kalman Filter SLAM with landmark detection and unknown data association for the NUTurtle with a 2D lidar.

`ros2 launch nuslam turtlebot_bringup.launch.xml` to launch EKF SLAM using landmark detection and unknown data association on a TurtleBot3.

`ros2 launch nuslam pc_bringup.launch.xml` to bring up corresponding RVIZ nodes to visualize the position of the TurtleBot3.

`ros2 launch nuslam unknown_data_assoc.launch.xml` to launch a simulation with EKF SLAM using landmark detection and unknown data association on lidar data.

`ros2 launch nuslam slam.launch.xml` to launch a simulation with EKF SLAM with fake sensor data for testing EKF SLAM.

`ros2 launch nuslam landmark_detect.launch.xml` to launch a simulation to test landmark detection.

## Demo Video (Real Robot)
EKF SLAM with Landmark Detection and Unknown Data Association

### Results
#### Final Poses
##### Ground Truth
```
x: 0.0 m
y: 0.0 m
θ: 0.0 rad
```

Although this may not be the case, I assume I drove the real robot back to its precise starting location.

##### Odometry Estimate (Blue)
```
x:  0.234 m
y:  0.190 m
θ: -0.502 rad

Positional error: 0.301 m
Rotational error: 0.502 rad
```

Odometry uses the movement of the wheels and kinematics to determine the estimated pose of the robot after driving some distance. However, it has quite some error. This error is entirely induced due to slip of the wheels on the ground as the robot moves quickly. The pose of the robot estimated by odometry is pretty inaccurate by the time the end of the loop is reached.

##### EKF SLAM with Landmark Detection and Unknown Data Association (Green)
```
x:  0.016 m
y:  0.010 m
θ:  0.036 rad

Positional error: 0.019 m
Rotational error: 0.036 rad
```

The EKF SLAM algorithm also uses odometry data for its prediction of the changing state of the robot. However, it also uses lidar data, processed with supervised and unsupervised learning to detect landmarks. Landmark data is then associated with previously detected data using a Mahalanobis distance threshold. This associated data is then used to correct the odometry prediction and arrive at a much closer estimate to the actual pose of the robot.

### Results
#### Final Poses
##### Ground Truth (Red)
```
x:  0.010 m
y:  0.012 m
θ: -0.039 rad
```

This ground truth data comes from simulation motion of the robot. It is subject to input command noise and slipping and can also crash into obstacles. All these together mean this robot will not follow the theoretical perfect path it is commanded to follow.

##### Odometry Estimate (Blue)
```
x: -0.649 m
y: -0.150 m
θ: -0.193 rad

Positional error: 0.679 m
Rotational error: 0.154 rad
```

This large error is primarily due to the robot crashing into an obstacle during its path (but is also due to simulated wheel slippage when not colliding with an obstacle). Odometry has no way to detect this, and so quite a bit of error is introduced.

##### EKF SLAM with Landmark Detection and Unknown Data Association (Green)
```
x:  0.011 m
y:  0.011 m
θ: -0.054 rad

Positional error: 0.001 m
Rotational error: 0.015 rad
```

## Launch File Details
`ros2 launch nuslam turtlebot_bringup.launch.xml --show-args` to show arguments for the launch file that launches EKF SLAM using landmark detection and unknown data association on a TurtleBot3.

```
This launch file currently has no arguments.
```

`ros2 launch nuslam pc_bringup.launch.xml --show-args` to show arguments for the launch file that launches corresponding RVIZ nodes to visualize the position of the TurtleBot3.

```
This launch file currently has no arguments.
```

`ros2 launch nuslam unknown_data_assoc.launch.xml --show-args` to show arguments for the launch file that launches a simulation with EKF SLAM using landmark detection and unknown data association on lidar data.

```
'cmd_src':
        Source for cmd_vel messages. Valid choices are: ['teleop', 'circle', 'none']
        (default: 'teleop')

'robot':
    Simulation or other options for running robot nodes. Valid choices are: ['nusim', 'localhost', 'none']
    (default: 'nusim')

'use_rviz':
    Start RVIZ to visualize robot. Valid choices are: ['true', 'false']
    (default: 'true')
```

`ros2 launch nuslam slam.launch.xml --show-args` to show arguments for launch file that launches the EKF SLAM.

```
'cmd_src':
        Source for cmd_vel messages. Valid choices are: ['teleop', 'circle', 'none']
        (default: 'teleop')

'robot':
    Simulation or other options for running robot nodes. Valid choices are: ['nusim', 'localhost', 'none']
    (default: 'nusim')

'use_rviz':
    Start RVIZ to visualize robot. Valid choices are: ['true', 'false']
    (default: 'true')

'use_fake_sensor':
        Use fake sensor data instead of lidar. Valid choices are: ['true', 'false']
        (default: 'true')
```

`ros2 launch nuslam landmark_detect.launch.xml --show-args` to show arguments for the launch file that launches a simulation to test landmark detection.

```
'cmd_src':
    Source for cmd_vel messages. Valid choices are: ['teleop', 'circle', 'none']
    (default: 'teleop')

'robot':
    Simulation or other options for running robot nodes. Valid choices are: ['nusim', 'localhost', 'none']
    (default: 'nusim')

'use_rviz':
    Start RVIZ to visualize robot. Valid choices are: ['true', 'false']
    (default: 'true')
```

## Parameter Details
* `kalman` - parameters to control the Extended Kalman Filter
    * `process_noise` - parameters to control the Q_bar, process (movement) noise matrix
        * `theta` - Kalman filter process noise for theta coordinate.
        * `x` - Kalman filter process noise for x coordinate.
        * `y` - Kalman filter process noise for y coordinate.
    * `sensor_noise` - Kalman filter sensor noise.
    * `clusters` - parameters to control the supervised learning clustering algorithm.
      * `visualize` - Controls whether clustered points are published as markers.
      * `threshold` - Euclidean distance between points to be considered part of the same cluster.
    * `circles` - parameters to control the unsupervised learning circle regression algorithm.
      * `visualize` - Controls whether fit circles are published as markers.
      * `classification` - parameters for classification of clusters as circles or not circles.
        * `mean_min` - Minimum mean angle for considering a cluster a circle (deg).
        * `mean_max` - Maximum mean angle for considering a cluster a circle (deg).
        * `std_dev_max` - Maximum standard deviation for considering a cluster a circle (deg).
      * `radius_min` - Radius minimum for considering a circle fit as a legitimate circle.
      * `radius_max` - Radius maximum for considering a circle fit as a legitimate circle.
    * `mahalanobis` - parameters to control the Mahalanobis distance data association algorithm.
      * `threshold` - Mahalanobis distance threshold for a new landmark.
