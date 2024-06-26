# Section 3 C: Introduction to ROS2. Simple publisher subscriber nodes in C++ (3.23, 3.25)

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

## Simple publisher in C++

* Create the `simple_publisher.cpp` file in [`./src/bumperbot_cpp_examples/src/`](./src/bumperbot_cpp_examples/src/) folder
* update `CMakeLists.txt` to add dependencies, declare the executable and install it

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

## Simple subscriber in C++

* Create the `simple_subscriber.cpp` file in [`./src/bumperbot_cpp_examples/src/`](./src/bumperbot_cpp_examples/src/) folder
* Note: I had to do some changes in the code, needed to compile (may be because I am using ROS2 foxy?).
* update `CMakeLists.txt` to add dependencies, declare the executables and install them

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

* nothing new to add in `package.xml`

* build with colcon

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

