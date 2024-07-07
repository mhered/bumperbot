# Section 4: Locomotion

| Type of locomotion | Pros                                                         | Cons                                                         |
| ------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
| Legged             | Agile and adaptable to unstructured terrain. Can jump, run, keep balance. | Complexity of control system                                 |
| Wheeled            | Faster, more efficient and simple                            | Valid only for artificial environments. Inefficient e.g. in rough or soft ground (sand) |
| Aerial             | 3D                                                           | low payload                                                  |

Wheeled architectures

* differential drive: two wheels powered by motors + other support wheels or castors. Allows rotations and trnaslations in spae

* Ackerman drive: typically rear wheels connected to a motor and front wheels can steer, but can also be front drive or 4 wheels drive. Has a minimum rotation radius.

* omnidirectional drive: 3 or 4 mecanum wheels each connected to a motor. Very flexible. 

Pose in 2D defined by `x`, `y` coordinates and orientation `theta`

Friction: wheels deform and this dissipates energy.  Soft ground deforms permanently and dissipates even more. 

ROS has tools to Model , Visualize and Simulate to streamline development and debugging

URDF - Unifier Robot Description Format. Based in XML

<link>  reference frame, can associate:

​	<name>

​	<visual> visualization dimensions or mesh 

​	<collision> <inertial> physical properties

<joint> connection between parent and child links forming a tree structure

## Creating an URDF model

* create a package and build it:

```bash
$ ros2 pkg create --build-type ament_cmake bumperbot_description
going to create a new package
package name: bumperbot_description
destination directory: /home/mhered/bumperbot_ws/src
package format: 3
version: 0.0.0
description: TODO: Package description
maintainer: ['mhered <manolo.heredia@gmail.com>']
licenses: ['TODO: License declaration']
build type: ament_cmake
dependencies: []
creating folder ./bumperbot_description
creating ./bumperbot_description/package.xml
creating source and include folder
creating folder ./bumperbot_description/src
creating folder ./bumperbot_description/include/bumperbot_description
creating ./bumperbot_description/CMakeLists.txt
$ cd ..
$ colcon build
Starting >>> bumperbot_cpp_examples
Starting >>> bumperbot_description
Starting >>> bumperbot_py_examples
Finished <<< bumperbot_cpp_examples [0.28s]              
Finished <<< bumperbot_description [0.57s]  
Finished <<< bumperbot_py_examples [0.81s]          

Summary: 3 packages finished [1.06s]
```

* create a new folders  `urdf` and `meshes` copy STL files inside `meshes` and a  `bumperbot.urdf.xacro` file inside `urdf`:

```bash
$ tree
~/bumperbot_ws/src/bumperbot_description
├── CMakeLists.txt
├── include
│   └── bumperbot_description
├── meshes
│   ├── base_link.STL
│   ├── caster_front_link.STL
│   ├── caster_rear_link.STL
│   ├── imu_link.STL
│   ├── wheel_left_link.STL
│   └── wheel_right_link.STL
├── package.xml
├── src
└── urdf
    └── bumperbot.urdf.xacro
```



```xml
<?xml version="1.0"?>

<robot xmlns:xacro="https://www.ros.org/wiki/xacro" name="bumperbot">

    <!--base_footprint link-->
    <link name="base_footprint"/>

    <!--base_link link-->
    <link name="base_link">
        <visual>
            <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>

            <geometry>
                <mesh filename="package://bumperbot_description/meshes/base_link.STL"/>
            </geometry>
        </visual>
    </link>


    <!--base_joint joint-->
    <joint name="base_joint" type="fixed">
        <parent link="base_footprint"/>
        <child link="base_link"/>
        <origin xyz="0.0 0.0 0.033" rpy="0.0 0.0 0.0"/>
        
    </joint>

</robot>
```

* To install `meshes` and `urdf` folders add following lines to `CMakeLists.txt` :

```cmake
# install folders
install(
  DIRECTORY meshes urdf
  DESTINATION share/${PROJECT_NAME}
)
```

* as usual, build with colcon, then source in a different terminal. 
* install urdf-tutorial library:

```bash 
$ sudo apt-get install ros-jazzy-urdf-tutorial
```

And use this launch file, which is general purpose, to display the URDF model:

```bash
$ ros2 launch urdf_tutorial display.launch.py model:=/home/mhered/bumperbot_ws/src/bumperbot_description/urdf/bumperbot.urdf.xacro
```

![](./assets/bumperbot_1_loop_xs.gif)

* Add the rest of the links: wheels with `continuous` joints and casters with fixed joints, adjusting axes translations and rotations from measurements in CAD. Remember you can evaluate expressions and use some literals, e.g. `${pi/2}` See result in [bumperbot.urdf.xacro](./src/bumperbot_description/urdf/bumperbot.urdf.xacro)
* Instructions to change display colors for meshes: https://answers.gazebosim.org//question/13718/how-to-add-a-colormaterial-to-a-stl-mesh-in-an-urdf-file/

![](./assets/bumperbot_2_loop_s.gif)

## rviz

Tool that allows visualizing robots, obstacles, maps, or simulated sensor readings e.g. from laser scanners or cameras using plugins that produce intuitive graphical displays from messages published in certain ROS2 topics

