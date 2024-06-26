# Section 2: Setup

## Configuring the development environment (2.10-2.11)

* install ROS2 following official installation instructions: https://docs.ros.org/en/jazzy/Installation.html
* install Visual Studio Code and extensions: c/c++, python, cmake, cmake tools, xml, ros
* install terminator `sudo apt-get install terminator`
  * `Ctrl + Shift + e` split terminal horizontally
  * `Ctrl +  Shift + o` split vertically
* install following open source ROS2 packages:

```bash
$ # these were already installed
$ sudo apt-get install ros-jazzy-ros2-controllers
$ sudo apt-get install ros-jazzy-xacro
$ sudo apt-get install ros-jazzy-gazebo-ros
$ sudo apt-get install ros-jazzy-gazebo-ros-pkgs
$ sudo apt-get install ros-jazzy-gazebo-ros2-control
$ sudo apt-get install ros-jazzy-ros2-control
$ sudo apt-get install ros-jazzy-ros2-control
$ sudo apt-get install ros-jazzy-joint-state-publisher-gui
$ sudo apt-get install ros-jazzy-turtlesim
$ sudo apt-get install ros-jazzy-joy ros-jazzy-joy-teleop
$ sudo apt-get install python3-pip

$ # these ones I had to install
$ sudo apt-get install ros-jazzy-joy ros-jazzy-joy-teleop
$ sudo apt-get install ros-jazzy-tf-transformations
$ sudo apt-get install ros-jazzy-plotjuggler
$ sudo apt-get install ros-jazzy-plotjuggler-ros

$ pip install transforms3d
```

