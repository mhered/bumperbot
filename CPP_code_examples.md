# C++ code examples

## Section 3: Introduction to ROS2. Simple publisher subscriber nodes in C++ (3.23, 3.25)

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

create a C++ package named `bumperbot_py_examples`:

```bash
$ cd src
$ ros2 pkg create --build-type ament_cmake bumperbot_cpp_examples
```

Note difference with command to create a python package:

```bash
$ ros2 pkg create --build-type ament_python bumperbot_py_examples
```

Now if we rebuild the workspace (assuming python package was created too) two packages are created:

```bash
$ cd .. && colcon build
Starting >>> bumperbot_cpp_examples
Starting >>> bumperbot_py_examples
Finished <<< bumperbot_cpp_examples [0.62s]
Finished <<< bumperbot_py_examples [0.70s]          

Summary: 2 packages finished [0.92s]
```

* activate the workspace so the packages are recognized by sourcing the workspace (only affects current terminal). He claims it is good practice to do it from a different terminal to the one we are using to build it (??)

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

Typical folder structure of CPP package

```bash
$ tree ~/bumperbot_ws/src/bumperbot_cpp_examples/
/home/mhered/bumperbot_ws/src/bumperbot_cpp_examples/
├── CMakeLists.txt
├── include
│   └── bumperbot_cpp_examples
├── package.xml
└── src

```

`include/bumperbot_cpp_examples` contains header files with classes declarations and `src` contains cpp files with implementation

### Simple publisher in C++

1. Create the `simple_publisher.cpp` file in [`./src/bumperbot_cpp_examples/src/`](./src/bumperbot_cpp_examples/src/) folder

2. update `CMakeLists.txt` to add dependencies, declare the executable and install it

```txt
# find dependencies
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# declare our executable
add_executable(simple_publisher src/simple_publisher.cpp)

# declare its dependencies
ament_target_dependencies(simple_publisher rclcpp std_msgs)

#install it
install(TARGETS
  simple_publisher
  DESTINATION lib/${PROJECT_NAME}
)
```

* add the dependencies in `package.xml` (after the buildtool dependencies):

```xml
  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  ...
```

* build with colcon

```bash
$ colcon build
```

* in another terminal source and run the node

```bash
$ source install/setup.bash
$ ros2 run bumperbot_cpp_examples simple_publisher
[INFO] [1719255208.416906703] [simple_publisher]: Publishing at 1 Hz
```

* inspect the topic `/chatter`

```bash
$ ros2 topic list
$ ros2 topic echo /chatter
$ ros2 topic info /chatter --verbose
$ ros2 topic hz /chatter
```

### Simple subscriber in C++

1. Create the `simple_subscriber.cpp` file in [`./src/bumperbot_cpp_examples/src/`](./src/bumperbot_cpp_examples/src/) folder

* Note: I had to do some changes in the code, needed to compile (may be because I am using ROS2 foxy?).

2. update `CMakeLists.txt` to add dependencies, declare the executables and install them

```txt
# find dependencies
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# declare executables and their dependencies 
add_executable(simple_publisher src/simple_publisher.cpp)
ament_target_dependencies(simple_publisher rclcpp std_msgs)

add_executable(simple_subscriber src/simple_subscriber.cpp)
ament_target_dependencies(simple_subscriber rclcpp std_msgs)

#install them
install(TARGETS
  simple_publisher
  simple_subscriber
  DESTINATION lib/${PROJECT_NAME}
)
```

3. nothing new to add in `package.xml`

4. build with colcon

```bash
$ colcon build
```

* in another terminal source and run the node

```bash
$ source install/setup.bash
$ ros2 run bumperbot_cpp_examples simple_subscriber
[INFO] [1719259185.791358012] [simple_subscriber]: Listening in topic /chatter
```

* check the subscriber manually publishing to topic `/chatter`

```bash
$ ros2 topic pub /chatter std_msgs/msg/String "data: 'Hello ROS2 from C++'" 
```

And you start receiving messages:

```bash
$ ros2 run bumperbot_cpp_examples simple_subscriber 
[INFO] [1719259185.791358012] [simple_subscriber]: Listening in topic /chatter
[INFO] [1719259481.250105869] [simple_subscriber]: I heard: 'Hello ROS2 from C++'
[INFO] [1719259482.249915507] [simple_subscriber]: I heard: 'Hello ROS2 from C++'

```

## Section 4: Locomotion. Parameters in C++ (4.35)

### A simple parametric node in C++

1. Create the `simple_parametric.cpp` file in [`./src/bumperbot_cpp_examples/src/`](./src/bumperbot_cpp_examples/src/) folder

2. update `CMakeLists.txt` to add new dependencies (`rcl_interfaces`), declare the executable and install it:

```cmake
# find dependencies
...
find_package(rcl_interfaces REQUIRED)

# declare executables and their dependencies 
...
add_executable(simple_parametric src/simple_parametric.cpp)
ament_target_dependencies(simple_parametric rclcpp rcl_interfaces)
...

#install them
install(TARGETS
  ...
  simple_parametric
  DESTINATION lib/${PROJECT_NAME}
)
```

3. update `package.xml` with the new dependencies:

```xml
...
<depend>rcl_interfaces</depend>
...
```

4. build (from the workspace)

```bash
$ colcon build
```

5.  from another terminal, source and run:

```bash
$ source install/setup.bash
$ ros2 run bumperbot_cpp_examples simple_parametric
[INFO] [1720397151.996866996] [simple_parametric]: Node started with simple_int_param = 7 and simple_string_param = default
```

* you may launch the node with custom values of the parameters:

```bash
$ ros2 run bumperbot_cpp_examples simple_parametric --ros-args -p simple_int_param:=30 -p simple_string_param:="custom"
[INFO] [1720397592.683519755] [simple_parametric]: Node started with simple_int_param = 30 and simple_string_param = custom
```

* and at runtime you may get or set the values of the parameters

```bash
$ ros2 param list
/simple_parametric:
  simple_int_param
  simple_string_param
  use_sim_time
$ ros2 param get /simple_parametric simple_int_param
Integer value is: 30
$ ros2 param set /simple_parametric simple_int_param 48
Set parameter successful
$ ros2 param get /simple_parametric simple_int_param
Integer value is: 48
```

Note the `ros2 param set` command triggers a call to the `paramChangeCallback()` function which in our case checks type and displays a log message in the terminal where the `simple_parametric` node is running:

```bash
[INFO] [1720397763.500604542] [simple_parametric]: New value for simple_int_param: 48
```

Also if I try to set a parameter of an invalid type, it fails gracefully:

```bash
$ ros2 param set /simple_parametric simple_int_param "not_an_integer"
Setting parameter failed: Invalid parameter
```

## Section 6: Kinematics. Roto-translation (6.57,6.60)

### Roto-translation in C++

Spawn two turtles:

```bash
$ ros2 run turtlesim turtlesim_node
$ ros2 service call /spawn turtlesim/srv/Spawn "x: 4.0
y: 4.0
theta: 0.0
name: 'turtle2'"
```

`turtle1/pose` and `turtle2/pose` topics publish their respective poses.

Create:

* a header file `simple_turtlesim_kinematics.hpp ` inside `bumperbot_cpp_examples/include/bumperbot_cpp_examples`, contains declaration of classes and functions and initialization of attributes
* and a node `simple_turtlesim_kinematics.cpp` inside `bumperbot_cpp_examples/src` folder, that contains the definition, i.e. the behavior: publishes the translation vector between `turtle2` and `turtle1`

To find the type of messages:

```bash
$ ros2 topic info /turtle1/pose
Type: turtlesim/msg/Pose
Publisher count: 1
Subscription count: 0
```

So we add a dependency to the header:

```c++
#include <turtlesim/msg/pose.hpp>
```

In `CMakeLists.txt`, add dependencies, install `include` folder to find `.hpp` and add `.cpp` as executable :

```cmake
...
find_package(turtlesim REQUIRED)

include_directories(include)
install(
  DIRECTORY include/
  DESTINATION include
)
...
add_executable(simple_turtlesim_kinematics src/simple_turtlesim_kinematics.cpp)
ament_target_dependencies(simple_turtlesim_kinematics rclcpp turtlesim)

#install them
install(TARGETS
  simple_publisher
  simple_subscriber
  simple_parametric
  simple_turtlesim_kinematics
  DESTINATION lib/${PROJECT_NAME}
  )

```

Add dependency to `package.xml`:

```xml
...
  <depend>turtlesim</depend>
...
```

build source and run

Note: the difficulty was in C++! I had to modify the code to get it to work: used `SharedPtr` in callback arguments and in the variables to store the last pose.

