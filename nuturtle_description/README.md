# NUTurtle  Description
URDF files for NUTurtle Gary
* `ros2 launch nuturtle_description load_one.launch.py` to see the robot in rviz.
* `ros2 launch nuturtle_description load_all.launch.xml` to see four copies of the robot in rviz.

## Launch File Details
* `ros2 launch nuturtle_description load_one.launch.py --show-args` to show arguments for launch file that loads only one robot.

```
Arguments (pass arguments as '<name>:=<value>'):

    'use_rviz':
        Selects whether or not to launch RVIZ. Valid choices are: ['true', 'false']
        (default: 'true')

    'use_jsp':
        Selects whether or not to launch the joint state publisher. Valid choices are: ['true', 'false']
        (default: 'true')

    'color':
        Selects the color of the turtlebot. Valid choices are: ['red', 'green', 'blue', 'purple']
        (default: 'purple')
```

* `ros2 launch nuturtle_description load_all.launch.xml --show-args` to show arguments for launch file that loads all robots.

```
Currently there are no arguments for this launch file.
```
