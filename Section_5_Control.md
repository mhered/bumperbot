# ROS2 Control (#5.43)
URDF so far is a static model!

Control is the first and most elementary functionality: actuating joint motors in response to received commands. Will be used by all advanced modules that implement logics.

Example: "rotate a motor with a certain speed" 

Controller is a component that:

1. compares an input variable (goal or desired state) with the current state of the motor (output variable): error = output - input 

2. then takes error variable and produces a command to the system to minimize this error as soon as possible, so that output variable approaches input


The control problem is common to all robots (manipulators, mobile robots, drones...): sending commands to actuators and receiving feedback from sensors on how the actuators commanded have actually moved.

For this problem ROS 2 provides the framework ROS 2 Control, with a modular architecture:

1. Hardware resources are drivers for:

   - actuators - to send commands

   - sensors - from which we can retrieve information about the actuator state 

   - or entire complex systems - composed by several actuators and sensors

   - which could be real or simulated in the physics engine gazebo

2. ROS 2 control library provides two interfaces to interact with hardware resources:

   - Command interface - to write commands to the hardware

   - State interface - to read feedback from the hardware


3. the Resource Manager is a component that manages these two interfaces and can load several hardware resources to make them available. In turn, the Resource Manager interacts with is another level of abstraction or middleware, called the Controller Manager

4. the Controller Manager can load one or multiple controllers simultaneously, which may implement each different control logics, and connects them to the Resource Manager that manages the hardware. There are many controllers already available in the ROS2 controllers library. The controller Manager also provides interfaces to other ROS 2 nodes and the final user (e.g. a CLI and several services and topics)

# Control types (5.44)
ROS2 control library implements 3 interfaces: position, velocity and effort (force or torque) control (corresponds to 3 different logics)
- Position: e.g. the target is to reach a specific angle. The error is the difference between actual and target position. When error is zero, the control system continues working to counter any external disturbances. 
- Velocity: the objective is that the motor reaches a given constant rotating speed
- Effort: e.g. for manipulators, to apply enough pressure to hold the object but not too much so as not to break it
# ROS2 control with gazebo (#5.45)
Adding ROS2 control library support to our robot

Need to add tags and plugins

In  `bumperbot_gazebo.xacro` we need to add at the beginning a `<transmission>` tag for each actuated joint, and at the end a gazebo plugin specifying the topic  `robot_description` and the node publishing it.

```xml
<transmission name="wheel_right_transmission">
	<plugin>transmission_interface/SimpleTransmission</plugin>
	<actuator name="wheel_right_motor" role="actuator_right"/>
	<joint name="wheel_right_joint" role="joint_right">
		<mechanical_reduction>1.0</mechanical_reduction>
	</joint>
</transmission>

<transmission name="wheel_left_transmission">
	<plugin>transmission_interface/SimpleTransmission</plugin>
	<actuator name="wheel_left_motor" role="actuator_left"/>
	<joint name="wheel_left_joint" role="joint_left">
		<mechanical_reduction>1.0</mechanical_reduction>
	</joint>
</transmission>
...
<gazebo>
	<plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
		<robot_param>robot_description</robot_param
 	<robot_param_node>robot_state_publisher</robot_param_node>
		<parameters></parameters>
	</plugin>
</gazebo>
```

Then create a new file `bumperbot_ros2_control.xacro` and include it in `bumperbot.urdf.xacro` with:

`<xacro:include filename="$(find bumperbot_description)/urdf/bumperbot_ros2_control.xacro"/>`

Add the standard xacro boilerplate, define `<ros2_control>` tag for the complete robot, add the gazebo plugin (will later replace for the real robot) and add for each joint:
- a command interface to write velocities, with limits +/-1rad/s
- two state interfaces to read velocity and position feedback from the motor 

```xml
<?xml version="1.0"?>
<robot name="bumperbot" xmlns:xacro="http://www.ros.org/wiki/xacro">
    <ros2_control type="system" name="RobotSystem">
		<hardware>
			<plugin>gazebo_ros2_control/GazeboSystem</plugin>
		</hardware>

        <joint name="wheel_right_joint">
			<command_interface name="velocity">
				<param name="min">-1</param>
				<param name="max">1</param>
			</command_interface>
			<state_interface name="position"/>
			<state_interface name="velocity"/>
		</joint>

		<joint name="wheel_left_joint">
			<command_interface name="velocity">
				<param name="min">-1</param>
				<param name="max">1</param>
			</command_interface>
			<state_interface name="position"/>
			<state_interface name="velocity"/>
		</joint>
</ros2_control>
```

# YAML configuration files (#5.46 - 5.47)

ROS2 control is modular and configurable through node parameters. We already know how to interact with parameters through command line and launch files. An alternative is YAML configuration files, which allow to define configuration parameters of a node easily so they are available at run time and are also friendly for humans so you can easily get an overview of all the parameters in use. 

Structure:

````yaml
node_name:
  ros__parameters:
    param_1: value_1
    ...
    param_n: value_n
````

We create a new package inside `bumperbot_ws/src` with:

```bash
$ cd bumperbot_ws/src
$ ros2 pkg create --build-type ament_cmake bumperbot_controller
$ cd ..
$ colcon build --symlink-install
```

And create a new file `bumperbot_controllers.yaml` by convention inside a `config` folder

To configure the `controller_manager` we declare the controllers we want to load, by giving them a name and specifying the package/node in the `type` parameter.

In the same file we define parameters also for node `simple_velocity_controller` (need to give name of the joints actuated)

```yaml
controller_manager:
    ros__parameters:
        update_rate: 100
        use_sim_time: true
        joint_state_broadcaster:
            type: joint_state_broadcaster/JointStateBroadcaster
        simple_velocity_controller:
            type: velocity_controllers/JointGroupVelocityController

simple_velocity_controller:
    ros__parameters:
        joints:
         - wheel_right_joint
         - wheel_left_joint
```


To install in `CMakeLists.txt` we add:

```cmake
...
install (
DIRECTORY config
DESTINATION share/${PROJECT_NAME}
)
...
```

Back to `bumperbot_gazebo.xacro` we specify the YAML file for the parameters:
```yaml
...
<parameters>
$(find bumperbot_controller)/config/bumperbot_controllers.yaml
</parameters>
...
```

`$ colcon build`, in new window source and launch gazebo:

```bash
$ ros2 launch bumperbot_description gazebo.launch.py
```

It works!

# Launch Controller (#5.48)

We still need to launch the controller manager. 

We create a new launch file `controller.launch.py` inside a new `launch`folder in the `bumperbot_controller` package.

This launch file spawns nodes `joint_state_broadcaster` and `simple_velocity_controller` using node `spawner` from package `controller_manager`

In `CMakeLists.txt` we need to install  `launch` 

In `package.xml` we add dependencies:

```xml
...
	<exec_depend>ros2launch</exec_depend>
	<exec_depend>controller_manager</exec_depend>
...
```

Note: `ros2launch` is needed for `LaunchDescription` and `Node` classes, `controller_manager` for the `spawner`

Compile with `colcon build`

### Interacting with ros2_control CLI

1. Launch robot simulation

```bash
$ source install/setup.bash
$ ros2 launch bumperbot_description gazebo.launch.py 
```

2. Start control system of the robot

```bash
$ source install/setup.bash
$ ros2 launch bumperbot_controller controller.launch.py
```

Controller manager allows interacting from CLI

e.g. to list controllers currently running

```bash
$ ros2 control list_controllers
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active    
simple_velocity_controller[velocity_controllers/JointGroupVelocityController] active   
```

e.g. list hardware interfaces

```bash
$ ros2 control list_hardware_interfaces
command interfaces
	wheel_left_joint/velocity [claimed]
	wheel_right_joint/velocity [claimed]
state interfaces
	 wheel_left_joint/position
	 wheel_left_joint/velocity
	 wheel_right_joint/position
	 wheel_right_joint/velocity
```

List topics:

```bash
$ ros2 topic list
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
```

We can use `/simple_velocity_controller/commands` to publish commands:

```bash
$ ros2 topic pub /simple_velocity_controller/commands std_msgs/msg/Float64MultiArray "layout:
  dim: []
  data_offset: 0
data: [1, -1]" 
publisher: beginning loop
publishing #1: std_msgs.msg.Float64MultiArray(layout=std_msgs.msg.MultiArrayLayout(dim=[], data_offset=0), data=[1.0, -1.0])

publishing #2: std_msgs.msg.Float64MultiArray(layout=std_msgs.msg.MultiArrayLayout(dim=[], data_offset=0), data=[1.0, -1.0])
...
```

![](./assets/bumperbot_is_alive_xs.gif)

Note: using double tab along the way autocompletes to find the list of available topics, then the list of messages this topic accepts, and finally the template for the message selected (remember need to add " !!):

```bash
$ ros2 topic pub <\t\t>
-1                                    --qos-durability
/clock                                --qos-history
/dynamic_joint_states                 --qos-profile
/joint_states                         --qos-reliability
--keep-alive                          -r
-n                                    --rate
--node-name                           /robot_description
--once                                /rosout
-p                                    /simple_velocity_controller/commands
/parameter_events                     -t
/performance_metrics                  /tf
--print                               /tf_static
--qos-depth                           --times

$ ros2 topic pub /simple_velocity_controller/commands <\t\t>
-1                              --qos-history
--keep-alive                    --qos-profile
-n                              --qos-reliability
--node-name                     -r
--once                          --rate
-p                              std_msgs/msg/Float64MultiArray
--print                         -t
--qos-depth                     --times
--qos-durability  
$ ros2 topic pub /simple_velocity_controller/commands std_msgs/msg/Float64MultiArray <\t\t>
-1
--keep-alive
layout:\^J\ \ dim:\ []\^J\ \ data_offset:\ 0\^Jdata:\ []\
-n
--node-name
--once
-p
--print
--qos-depth
--qos-durability
--qos-history
--qos-profile
--qos-reliability
-r
--rate
-t
--times

$ ros2 topic pub /simple_velocity_controller/commands std_msgs/msg/Float64MultiArray "l <\t\t>

$ ros2 topic pub /simple_velocity_controller/commands std_msgs/msg/Float64MultiArray "layout:
  dim: []
  data_offset: 0
data: []" 
```

