# Section 6 Kinematics

## Definitions

* Direct/Forward Kinematics -> equations to calculate the robot movement from a given movement of joints

* Inverse Kinematics -> equations to determine the movement of joints required for a given robot movement 

## Pose of a mobile robot in 2D

world reference frame x<sub>W</sub>, y<sub>W</sub> (non inertial)

mobile robot frame x<sub>R</sub>, y<sub>R</sub> (inertial)

Need 3 variables: x,y position of centre of mass and orientation angle of robot $$\theta$$
$$
\mathbf{P} = \begin{bmatrix} x \\ y \\ \theta \end{bmatrix}
$$
Can further simplify separately studying the translation (x,y) and rotation $\theta$

## Translation

Translation vector is coordinates of robot CoG in World frame of reference 
$$
T = \begin{bmatrix} T_x \\ T_y \end{bmatrix}
$$
Canget the pose of an object in world coordinates from its pose in the robot frame + the translation vector
$$
\begin{equation}\begin{bmatrix}x_W \\y_W\end{bmatrix}=\begin{bmatrix}T_x \\T_y\end{bmatrix}+\begin{bmatrix}x_R \\y_R\end{bmatrix}\end{equation}
$$

## Rotation matrix

Can get also the pose of an object in world frame from its pose in the robot frame multiplied from the rotation matrix (where $$\theta$$  is the relative angle of robot in world frame).
$$
\begin{equation}
\begin{bmatrix}
x_W \\
y_W
\end{bmatrix}
=
\begin{bmatrix}
\cos \theta & -\sin \theta \\
\sin \theta & \cos \theta
\end{bmatrix}
\begin{bmatrix}
x_R \\
y_R
\end{bmatrix}
\end{equation}
$$


## 2D Transformation or Roto-translation matrix

$$
\begin{equation}
\begin{bmatrix}
x_W \\
y_W \\
1
\end{bmatrix}
=
\begin{bmatrix}
\cos \theta & -\sin \theta & T_x\\
\sin \theta & \cos \theta & T_y \\
0 & 0 & 1
\end{bmatrix}
\begin{bmatrix}
x_R \\
y_R \\
1
\end{bmatrix}
\end{equation}
$$



## intro to turtlesim

Launch turtlesim and teleop nodes:

```bash
$ ros2 run turtlesim turtlesim_node
$ ros2 run turtlesim turtle_teleop_key
```

Explore:

```bash
$ ros2 topic list
/parameter_events
/rosout
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
```

```bash
$ ros2 run rqt_graph rqt_graph
```

![](./assets/rqt_graph_turtlesim.svg)

Can manually publish messages in the topic using `ros2 topic pub` (use \t\t for autocompletion, typing " before last \t for message template)

```bash
$ ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "linear:
  x: 1.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 1.0" 
publisher: beginning loop
publishing #1: geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=1.0, y=0.0, z=0.0), angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=1.0))

publishing #2: geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=1.0, y=0.0, z=0.0), angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=1.0))
...
```

Can listen to a topic e.g. pose:

```bash
$ ros2 topic echo /turtle1/pose
x: 3.576805353164673
y: 5.250796318054199
theta: -1.3278014659881592
linear_velocity: 1.0
angular_velocity: 1.0
---
x: 3.5809032917022705
y: 5.235330104827881
theta: -1.3118014335632324
linear_velocity: 1.0
angular_velocity: 1.0
---
...
```

Add a second turtle:

```bash
$ ros2 service call /spawn turtlesim/srv/Spawn "x: 4.0
y: 4.0
theta: 0.0
name: 'turtle2'"

waiting for service to become available...
requester: making request: turtlesim.srv.Spawn_Request(x=4.0, y=4.0, theta=0.0, name='turtle2')

response:
turtlesim.srv.Spawn_Response(name='turtle2')
```

Check:

```bash
$ ros2 topic list
/parameter_events
/rosout
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
/turtle2/cmd_vel
/turtle2/color_sensor
/turtle2/pose
```

Remap teleop to publish to the second turtle:
```bash
$ ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel
```

Note: See code examples in C++ and python
