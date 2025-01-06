# Section 7 Differential Kinematics

Initial robot pose at time $t_0$ : $x_0$, $y_0$ position of centre of mass and orientation angle of robot $$\theta_0$$
$$
P_0 = \begin{bmatrix} x_0 \\ y_0 \\ \theta_0 \end{bmatrix}
$$
Pose at time $t_1$:
$$
P_1 = \begin{bmatrix} x_1 \\ y_1 \\ \theta_1 \end{bmatrix}
$$
The equations of differential kinematics express robot velocity in world frame as a function of robot geometry and orientation, and wheel rotational speeds:

* $l$ wheel separation 

* $r$ wheel radius 
* $\theta$ angular orientation of robot

* $\omega_R$ rotational speed of right wheel

* $\omega_L$ rotational speed of left wheel

$$
\dot{P} =\frac{\partial P}{\partial t}=\begin{bmatrix} \dot{x} \\ \dot{y} \\ \dot{\theta} \end{bmatrix}= f(l, r, \theta,  \omega_R, \omega_L)
$$

The equations of differential kinematics are a combination of two equations which we can derive separately: 

* linear and angular velocities of the robot in the robot reference frame, as a function of rotational speeds of the robot wheels. These equations allow sending velocity commands, in robot frame, to the robot. 
* robot velocity in the world reference frame from velocity in robot frame (using the rotation matrix)

##  Robot linear velocity V in robot frame

Hypotheses: 

* no slippage in wheel contact points: $ v_{R, L} = r . \omega_{R, L}$

*  CoG midpoint between wheels

$$
V = \frac{r}{2}(\omega_L + \omega_R)
$$

## Robot angular velocity W in robot frame

$$
W = \frac{r}{l}(\omega_R - \omega_L)
$$

## Matrix equations in robot frame

$$
\begin{bmatrix} V \\ W \end{bmatrix} = \frac{r}{2} \begin{bmatrix} 1 & 1 \\ \frac{2}{l} & \frac{-2}{l}\  \end{bmatrix}\begin{bmatrix} \omega_R \\ \omega_L \end{bmatrix}
$$

Inverse equations:


$$
\begin{bmatrix} \omega_R \\ \omega_L \end{bmatrix} = \frac {1}{r} \begin{bmatrix} 1 & \frac{-l}{2} \\ 1 & \frac{l}{2}\  \end{bmatrix} \begin{bmatrix} V \\ W \end{bmatrix}
$$

## Robot velocity in world frame

We apply the rotation matrix $R(\theta)$. Note the only component of linear velocity is V along x axis, there is no component in y axis (perpendicular to wheels):
$$
\dot{P} =\begin{bmatrix} \dot{x} \\ \dot{y} \\ \dot{\theta} \end{bmatrix}= f(V, W, \theta)=\begin{bmatrix} cos(\theta) & -sin(\theta) & 0\\ sin(\theta) & cos(\theta) & 0 \\ 0 & 0 & 1 \end{bmatrix}\begin{bmatrix} V \\ 0 \\ W \end{bmatrix}
$$

## Differential kinematics equation

$$
\dot{P} =\begin{bmatrix} \dot{x} \\ \dot{y} \\ \dot{\theta} \end{bmatrix}=\frac{r}{2}\begin{bmatrix} cos(\theta) & -sin(\theta) & 0\\ sin(\theta) & cos(\theta) & 0 \\ 0 & 0 & 1 \end{bmatrix}\begin{bmatrix} 1 & 1 \\ 0 & 0 \\ \frac{2}{l} & \frac{-2}{l}\  \end{bmatrix}\begin{bmatrix} \omega_R \\ \omega_L \end{bmatrix}
$$

$$
\dot{P} =\begin{bmatrix} \dot{x} \\ \dot{y} \\ \dot{\theta} \end{bmatrix}=\frac{r}{2}\begin{bmatrix} cos(\theta) & cos(\theta) \\ sin(\theta) & sin(\theta)  \\ \frac{2}{l} & \frac{-2}{l} \end{bmatrix}\begin{bmatrix} \omega_R \\ \omega_L \end{bmatrix}
$$

This 3x2 matrix is the Jacobian (??)

## Simple controller

See [Python_code_examples.md](./Python_code_examples.md) for the python code of a simple velocity controller (7.69) and [CPP_code_examples.md](CPP_code_examples.md) for the C++ code (7.70)

## Launch file (7.71)

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

## Joystick teleoperation (7.72)

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

## Using the diff_drive_controller (7.73) 

ros2 control libraries have a standard controller for differential drive robots

1. modify `bumperbot_controllers.yaml` to declare and configure a new node named e.g. `bumperbot_controller` based on `diff_drive_controller/DiffDriveController`

2. modify `controller.launch.py` to add an argument `use_simple_controller`, true by default, which will toggle between using the `simple_controller` we wrote and the standard `diff_drive_controller` node we named `bumperbot_controller`. We use `GroupAction` to launch on condition the group of nodes for `simple_controller`.

colcon build and in other terminals source and launch gazebo, the control system of the robot, and the joystick :

```bash
(T2): $ ros2 launch bumperbot_description gazebo.launch.py 

(T3): $ ros2 launch bumperbot_controller controller.launch.py use_simple_controller:=false

(T4): $ ros2 launch bumperbot_controller joystick_teleop.launch.py
```

it works!