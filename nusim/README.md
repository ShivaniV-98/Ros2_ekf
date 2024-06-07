# NUSim 
Simulation package for the NUTurtle.
* `ros2 launch nusim nusim.launch.xml` to launch the simulation.

## Launch File Details
* `ros2 launch nusim nusim.launch.xml --show-args` to show arguments for launch file that launches the simulation.

```
'use_rviz':
        Start RVIZ to visualize robot. Valid choices are: ['true', 'false']
        (default: 'true')

'world_yaml_path':
    Path to yaml file for world parameters. Set to empty string to omit.
    (default: FindPackageShare(pkg='nusim') + '/config/basic_world.yaml')

'draw_only':
    Draw obstacles only, do not simulate. Valid choices are: ['true', 'false']
    (default: 'false')

'use_jsp':
        Selects whether or not to launch the joint state publisher. Valid choices are: ['true', 'false']
        (default: 'true')
```

## Parameter Details
* `basic_world.yaml` - basic simulation world configuration parameters.
    * `rate` - The rate the simulation runs at (Hz).
    * `x0` - Initial x position of the robot (m).
    * `y0` - Initial y position of the robot (m).
    * `theta0` - Initial rotation of the robot (rad).
    * `obstacles` - parameters for the creation of cylindrical obstacles in the world.
        * `x` - List of x starting positions of obstacles (m). Arbitrary length, but must match length of `y`.
        * `y` - List of y starting positions of obstacles (m). Arbitray length, but must match length of `x`.
        * `r` - Radius of all cylinder obstacles (m). Single value applies to all obstacles.
    * `input_noise` - Standard deviation for noise on input wheel commands (rad/s). Must be nonnegative.
    * `slip_fraction` - Bound of fraction of slip experienced by the wheels during motion (decimal fraction). Must be nonnegative.
    * `basic_sensor_variance` - Standard deviation for noise in obstacle sensing (m). Must be nonnegative.
    * `max_range` - Max range for obstacle sensing (m).
    * `lidar` - parameters for simulated lidar sensor
        * `resolution` - Resolution for lidar scan (m). Set to 0 for perfect resolution.
        * `noise` - Standard deviation for noise in lidar scan (m). Must be nonnegative.
