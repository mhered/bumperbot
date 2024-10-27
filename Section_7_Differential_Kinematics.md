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

## Simple controller in Python(7.69)

subscribes to `\cmd_vel` commands coming from the joystick V, W, calculates $\omega_R$, $\omega_L$ and publishes them in the topic `simple_velocity_controller/commands`