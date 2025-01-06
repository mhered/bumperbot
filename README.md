# Self Driving and ROS 2 - Learn by Doing

## Course 1 - Odometry and Control

https://www.udemy.com/course/self-driving-and-ros-2-learn-by-doing-odometry-control/

- Differential kinematics (control): translating velocity commands from a joystick into movement

* Odometry: inverse problem, estimating robot movement from encoder signals

* Sensor fusion: improving odometry by fusing data from various sensors to reduce noise (Kalman filters) 

### Notes on Course 1

* [Notes on Section 2: Setup](./notes/Section_2_Setup.md)
* [Notes on Section 3: Introduction to ROS2. Why ROS2?](./notes/Section_3_Intro_to_ROS2_Why_ROS2.md)
* [Notes on Section 4: Locomotion. URDF](./notes/Section_4_Locomotion.md)
* [Notes on Section 5: Control](./notes/Section_5_Control.md)
* [Notes on Section 6: Kinematics](./notes/Section_6_Kinematics.md)
* [Notes on Section 7: Differential Kinematics](./notes/Section_7_Differential_Kinematics.md)
* [Notes on Section 8: TF2 Library](./notes/Section_8_TF2.md)
* [Notes on Section 10: Probability for Robotics](./notes/Section_10_Probability_for_Robotics.md)
* Bonus: [Notes on building the real bumperbot](./assembly/README.md)

### Code examples for Course 1

* [Python code examples](./notes/Python_code_examples.md)
* [C++ code examples](./notes/CPP_code_examples.md)

## Course 2 - Mapping and Localization

https://www.udemy.com/course/self-driving-and-ros-2-learn-by-doing-map-localization/

* 2D laser sensor: 
* Mapping: mapping with a laser scanner assuming we know where the robot is
* Localization: assuming we have a map
* SLAM: simultaneous localization and mapping

## Course 3 - Planning and navigation

* Path planning: comparing different algorithms to plan trajectories to reach a desired location in the map
* Obstacle avoidance: sense the environment constantly and adjust trajectory if there is an unexpected obstable 
* Behaviour trees: more complex logics for flexible navigation techniques

## Course 4 - Vision and Perception

* visual odometry: using camera for localization
* visual SLAM: 
* object recognition and tracking: detection of moving obstacles using a camera

