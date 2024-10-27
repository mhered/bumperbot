# Python code examples

## Section 3: Introduction to ROS2. Simple publisher / subscriber nodes in python (3.21, 3.22, 3.24)

### Setup workspace

* Create workspace

```bash
$ mkdir -p bumperbot_ws/src
```

* initialize

```bash
$ colcon build
```

Generates automatically `build` (intermediate files) `install` (final executable files) and `log` folders

### Create the first package: subscriber in python 

1. create a python package named `bumperbot_py_examples`:

```bash
$ cd src
$ ros2 pkg create --build-type ament_python bumperbot_py_examples

going to create a new package
package name: bumperbot_py_examples
destination directory: /home/mhered/bumperbot_ws/src
package format: 3
version: 0.0.0
description: TODO: Package description
maintainer: ['mhered <manolo.heredia@gmail.com>']
licenses: ['TODO: License declaration']
build type: ament_python
dependencies: []
creating folder ./bumperbot_py_examples
creating ./bumperbot_py_examples/package.xml
creating source folder
creating folder ./bumperbot_py_examples/bumperbot_py_examples
creating ./bumperbot_py_examples/setup.py
creating ./bumperbot_py_examples/setup.cfg
creating folder ./bumperbot_py_examples/resource
creating ./bumperbot_py_examples/resource/bumperbot_py_examples
creating ./bumperbot_py_examples/bumperbot_py_examples/__init__.py
creating folder ./bumperbot_py_examples/test
creating ./bumperbot_py_examples/test/test_copyright.py
creating ./bumperbot_py_examples/test/test_flake8.py
creating ./bumperbot_py_examples/test/test_pep257.py

```

Note difference vs the command for a C++ package:

```bash
$ ros2 pkg create --build-type ament_cmake bumperbot_cpp_examples
```

If we create also the C++ package and we rebuild the workspace two packages are created:

```bash
$ cd .. && colcon build
Starting >>> bumperbot_cpp_examples
Starting >>> bumperbot_py_examples
Finished <<< bumperbot_cpp_examples [0.62s]
Finished <<< bumperbot_py_examples [0.70s]          

Summary: 2 packages finished [0.92s]
```

2. activate the workspace so the packages are recognized by sourcing the workspace (only affects current terminal). He claims it is good practice to do it from a different terminal to the one we are using to build it (??)

```bash
$ source install/setup.bash
$ ros2 pkg list 
ackermann_msgs...
builtin_interfaces
bumperbot_cpp_examples
bumperbot_py_examples
camera_calibration_parsers
...
zstd_vendor
```

Typical structure of a python package

```bash
$ tree ~/bumperbot_ws/src/bumperbot_py_examples/
~/bumperbot_ws/src/bumperbot_py_examples/
├── bumperbot_py_examples
│   └── __init__.py
├── package.xml
├── resource
│   └── bumperbot_py_examples
├── setup.cfg
├── setup.py
└── test
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py

3 directories, 8 files
```

The  [`bumperbot_py_examples`](./src/bumperbot_py_examples/bumperbot_py_examples/)  folder contains the python code.

3. create a python file inside it called `simple_publisher.py`

4. `setup.py` contains the instructions to make an executable. Add an entry point so that when `simple_publisher` node is called, it executes function `main` in `simple_publisher` file inside `bumperbot_py_examples` package :

```json
...   
entry_points={create a simple_subscriber.py file inside
        'console_scripts': [
            'simple_publisher = bumperbot_py_examples.simple_publisher:main',
        ],
```

5. in `package.xml`  declare execution dependencies (the two libraries we included in the script)

```xml
  <exec_depend>rclpy</exec_depend>
  <exec_depend>std-msgs</exec_depend>
```

6. build with colcon

```bash
$ colcon build
Starting >>> bumperbot_cpp_examples
Starting >>> bumperbot_py_examples
Finished <<< bumperbot_cpp_examples [0.20s]                                                             
Finished <<< bumperbot_py_examples [0.64s]          

Summary: 2 packages finished [0.83s]
```

7. in a different terminal source the workspace and run the node

```bash
$ source install/setup.bash
$ ros2 run bumperbot_py_examples simple_publisher 
[INFO] [1719217038.723989913] [simple_publisher]: Publishing at 1 Hz
```

* monitor the topic to check it is broadcasting every 1sec

```bash
$ ros2 topic list
/chatter
/parameter_events
/rosout

$ ros2 topic echo /chatter 
data: 'Hello, ROS2 - counter: 35 '
---
data: 'Hello, ROS2 - counter: 36 '
---
data: 'Hello, ROS2 - counter: 37 '
---
data: 'Hello, ROS2 - counter: 38 '
---
data: 'Hello, ROS2 - counter: 39 '
---

$ ros2 topic info /chatter --verbose
Type: std_msgs/msg/String

Publisher count: 1

Node name: simple_publisher
Node namespace: /
Topic type: std_msgs/msg/String
Endpoint type: PUBLISHER
GID: 01.0f.3a.b1.6d.42.75.04.01.00.00.00.00.00.11.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RMW_QOS_POLICY_RELIABILITY_RELIABLE
  Durability: RMW_QOS_POLICY_DURABILITY_VOLATILE
  Lifespan: 2147483651294967295 nanoseconds
  Deadline: 2147483651294967295 nanoseconds
  Liveliness: RMW_QOS_POLICY_LIVELINESS_AUTOMATIC
  Liveliness lease duration: 2147483651294967295 nanoseconds

Subscription count: 0

$ ros2 topic hz /chatter
average rate: 1.000
	min: 1.000s max: 1.000s std dev: 0.00002s window: 2
average rate: 1.000
	min: 1.000s max: 1.000s std dev: 0.00006s window: 4
average rate: 1.000
	min: 1.000s max: 1.000s std dev: 0.00005s window: 5
average rate: 1.000

```

## Subscriber in python

1. create `simple_subscriber.py` file inside [`bumperbot_py_examples`](./src/bumperbot_py_examples/bumperbot_py_examples/) folder

2. Add an entry point in `setup.py` :

```json
...   
entry_points={create a simple_subscriber.py file inside
        'console_scripts': [
            'simple_publisher = bumperbot_py_examples.simple_publisher:main',
            'simple_subscriber = bumperbot_py_examples.simple_subscriber:main',
        ],
```

3. build with colcon

```bash
$ colcon build
```

4. in a different terminal source the workspace and run the node

```bash
$ source install/setup.bash
$ ros2 run bumperbot_py_examples simple_subscriber 
[INFO] [1719219470.084111799] [simple_subscriber]: I heard: Hello, ROS2 - counter: 716 
[INFO] [1719219470.972924372] [simple_subscriber]: I heard: Hello, ROS2 - counter: 717 
[INFO] [1719219471.972834813] [simple_subscriber]: I heard: Hello, ROS2 - counter: 718 
...s
```

## Section 4: Locomotion. Simple parametric node in python (4.34)

### Node with parameters in python

Certain nodes take parameters , i.e. settings to configure their behaviour

1. create file `simple_parametric.py` inside [`bumperbot_py_examples`](./src/bumperbot_py_examples/bumperbot_py_examples) folder and write the code for parametric node

2. add the entry point in `setup.py`: 

```            'simple_parametric = bumperbot_py_examples.simple_parametric:main',
 entry_points={
        'console_scripts': [
            ...
            bumperbot_py_examples.simple_parametric:main',
        ],
```

3. add a missing dependency in `package.xml`:

```
  <exec_depend>rcl_interfaces</exec_depend>
```

4. as usual, build with colcon, then source in a different terminal.

5. run the node with default param values with:

```bash
$ ros2 run bumperbot_py_examples simple_parametric 
```

* check available parameters, and their values:

```bash
$ ros2 param list
/simple_parametric:
  simple_int_param
  simple_string_param
  use_sim_time
  
$ ros2 param get /simple_parametric simple_int_param 
Integer value is: 0
$ ros2 param get /simple_parametric simple_string_param 
String value is: hello
```

* You can run the node passing custom params:

```bash
$ ros2 run bumperbot_py_examples simple_parametric --ros-args -p simple_int_param:=30 
$ ros2 param get /simple_parametric simple_int_param 
Integer value is: 30
```

* you can also modify parameters at run time:

```bash
$ ros2 param get /simple_parametric simple_string_param
String value is: Hello
$ ros2 param set /simple_parametric simple_string_param "Goodbye"
Set parameter successful
$ ros2 param get /simple_parametric simple_string_param
String value is: Goodbye
```

And with the implementation we made, the callback function yields the following terminal output:

```bash
$ ros2 run bumperbot_py_examples simple_parametric 
[INFO] [1719431653.982840885] [simple_parametric]: Parameter simple_string_param changed to : Goodbye

```

## Section 6: Kinematics. Roto-translation (6.56,6.59)

### Roto-translation in python

Spawn two turtles:

```bash
$ ros2 run turtlesim turtlesim_node
$ ros2 service call /spawn turtlesim/srv/Spawn "x: 4.0
y: 4.0
theta: 0.0
name: 'turtle2'"
```

`turtle1/pose` and `turtle2/pose` topics publish their respective poses.

Create a node `simple_turtlesim_kinematics.py` inside `bumperbot_py_examples` folder, that publishes the translation vector between `turtle2` and `turtle1`

To find the type of messages:

```bash
$ ros2 topic info /turtle1/pose
Type: turtlesim/msg/Pose
Publisher count: 1
Subscription count: 0
```

So we add a dependency

```python
from turtlesim.msg import Pose
```

Write the node:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class SimpleTurtlesimKinematics(Node):
    
    def __init__(self):
        # constructor
        #init node
        super().__init__('simple_turtlesim_kinematics')
        # init two subscribers
        self.turtle1_pose_sub_ = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.turtle1PoseCallback,
            10
        )

        self.turtle2_pose_sub_ = self.create_subscription(
            Pose,
            '/turtle2/pose',
            self.turtle2PoseCallback,
            10
        )
        # 2 variables
        self.last_turtle1_pose_ = Pose()
        self.last_turtle2_pose_ = Pose()

    def turtle1PoseCallback(self, msg):
        # save last turtle1 pose
        self.last_turtle1_pose_ = msg
        
    def turtle2PoseCallback(self, msg):
        # save last turtle2 pose
        self.last_turtle2_pose_ = msg
		
        # calculate and log translation and rotation
        Tx = self.last_turtle2_pose_.x - self.last_turtle1_pose_.x
        Ty = self.last_turtle2_pose_.y - self.last_turtle1_pose_.y

        theta = self.last_turtle2_pose_.theta - self.last_turtle1_pose_.theta
        theta_deg = 180*theta/np.pi
    
        self.get_logger().info(f"""\n
            Translation vector turtle1 -> turtle2 \n
            Tx: {Tx:.2f} \n
            Ty: {Ty:.2f} \n
            Rotation matrix turtle1 -> turtle2 \n
            theta (rad): {theta:.2f} rad \n
            theta (deg): {theta_deg:.2f} deg \n
            [R11 R12] :    [{np.cos(theta):.2f}\t{-np.sin(theta):.2f}] \n
            [R21 R22] :    [{np.sin(theta):.2f}\t{np.cos(theta):.2f}] \n
            """) 

def main():
    rclpy.init()
    simple_turtlesim_kinematics = SimpleTurtlesimKinematics()
    rclpy.spin(simple_turtlesim_kinematics)
    simple_turtlesim_kinematics.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```



install it in `setup.py`:

```python
 entry_points={
        'console_scripts': [
    		...
			'simple_turtlesim_kinematics = bumperbot_py_examples.simple_turtlesim_kinematics:main',
			]
```

and update dependencies in `package.xml`

```xml
  <exec_depend>turtlesim</exec_depend>
```

build, source and run the node:

```bash
$ ros2 run bumperbot_py_examples simple_turtlesim_kinematics 
[INFO] [1727391554.455939178] [simple_turtlesim_kinematics]: 

            Translation vector turtle1 -> turtle2 

            Tx: -4.22 

            Ty: -1.28 

            Rotation matrix turtle1 -> turtle2 

            theta (rad): -2.02 rad 

            theta (deg): -115.51 deg 

            [R11 R12] :    [-0.43	0.90] 

            [R21 R22] :    [-0.90	-0.43] 

...
```

## Section 7: Differential Kinematics. 

### Simple Speed Controller (7.69)

Note the `bumperbot_controller` package was created for cpp nodes so we need to make changes manually:

1)  create a `bumperbot_controller` folder, an `__init__.py` empty file inside, then create a `simple_controller.py` node with:

*  two parameters  `self.wheel_radius_` and `self.wheel_separation` that describe the robot geometry
* a subscriber that listens to joystick commands of type `TwistStamped` in topic `bumperbot_controller/cmd_vel`. The joystick message contains the target robot linear and angular velocities. 
* the subscriber has a callback function `self.velCallback` that computes wheel commands from robot linear and angular velocities using the equations derived in [Section_7_Differential_Kinematics.md](./Section_7_Differential_Kinematics.md)
* a publisher for wheel speed commands of type `Float64MultiArray` in topic `simple_velocity_controller/commands` (see details of the topic and message type used by the ros2 controller in [Section_5_Control.md](./Section_5_Control.md))

2. In `CMakeLists.txt` add dependencies to:
   * `ament_cmake_python` (needed to build and compile python nodes) 
   * dependencies used in the node: `rclpy`, `std_msgs`, `geometry_msgs`
3. Also in `CMakeLists.txt` install the node `simple_controller.py`

```cmake
...
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})
...

install (
PROGRAMS ${PROJECT_NAME}/simple_controller.py
 DESTINATION lib/${PROJECT_NAME}
)

```

4. Add the dependencies also in `package.xml`:

```xml
...
	<buildtool_depend>ament_cmake_python</buildtool_depend>
...
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
...
```

### Launch file (7.71)

Modify the launch file `controller.launch.py` :

* add 3 arguments: `use_python` to toggle between python and cpp nodes, `wheel_radius` and `wheel_separation`
* declare two nodes `simple_controller_py` and `simple_controller_cpp` with conditions `IfCondition` and `UnlessCondition` , passing the parameters`wheel_radius` and `wheel_separation`

1. build with `$ colcon build` 

2. In a second terminal, source and launch gazebo:

```bash
$ source install/setup.bash
$ ros2 launch bumperbot_description gazebo.launch.py
```

3. In another terminal, source and launch the controller:

```bash
$ source install/setup.bash
$ ros2 launch bumperbot_controller controller.launch.py  --show-args
Arguments (pass arguments as '<name>:=<value>'):

    'use_python':
        Whether to use Python or C++ nodes
        (default: 'true')

    'wheel_radius':
        Wheel radius
        (default: '0.033')

    'wheel_separation':
        Wheel separation
        (default: '0.17')
```

4. with the controller running, publish `TwistStamped` messages to `bumperbot_controller/cmd_vel` to get the robot to move:

```bash
$ ros2 topic list
/bumperbot_controller/cmd_vel
/clock
/dynamic_joint_states
/joint_states
/parameter_events
/performance_metrics
/robot_description
/rosout
/simple_velocity_controller/commands
/tf
/tf_static
$ ros2 topic pub /bumperbot_controller/cmd_vel geometry_msgs/msg/TwistStamped "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
twist:
  linear:
    x: 0.1
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.5" 
```

### Joystick teleoperation (7.72)

Create a launch file `joystick_teleop.launch.py` that launches nodes:

* `joy_node` from `joy` package, with parameters defined in `joy_config.yaml`:

```yaml
joystick:
  ros__parameters:
    device_id: 0
    device_name: ""
    deadzone: 0.5
    autorepeat_rate: 0.0
    sticky_buttons: false
    coalesce_interval_ms: 1
```

* and `joy_teleop` from `joy_teleop` package, with parameters defined in `joy_teleop.yaml` :

```yaml
joy_teleop:
  ros__parameters:
    move:
      type: topic
      interface_type: geometry_msgs/msg/TwistStamped
      topic_name: bumperbot_controller/cmd_vel
      deadman_buttons: [5]
      axis_mappings:
        twist-linear-x:
          axis: 1
          scale: 1.0
          offset: 0.0
        twist-angular-z:
          axis: 3
          scale: 1.0
          offset: 0.0
```

Note this configuration works with my gamepad in Mode X:

* fwd/backwards moving the left stick up/down 

* left/right rotation moving the right stick left/right

* deadman button: RB (right index). Note: deadman button does not work properly

