# Section 3 B: Introduction to ROS2. Simple publisher subscriber nodes in python (3.21, 3.22, 3.24)

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

create a python package named `bumperbot_py_examples`:

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

Or in C++:

```bash
$ ros2 pkg create --build-type ament_cmake bumperbot_cpp_examples
```

Now if we rebuild the workspace two packages are created:

```bash
$ cd .. && colcon build
Starting >>> bumperbot_cpp_examples
Starting >>> bumperbot_py_examples
Finished <<< bumperbot_cpp_examples [0.62s]
Finished <<< bumperbot_py_examples [0.70s]          

Summary: 2 packages finished [0.92s]
```

Now activate the workspace so the packages are recognized by sourcing the workspace (only affects current terminal). He claims it is good practice to do it from a different terminal to the one we are using to build it (??)

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

* python packages structure

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

* the  [`bumperbot_py_examples`](./src/bumperbot_py_examples/bumperbot_py_examples/)  folder contains the python code, create a python file inside it called `simple_publisher.py`

* `setup.py` contains the instructions to make an executable. Add an entry point so that when `simple_publisher` node is called, it executes function `main` in `simple_publisher` file inside `bumperbot_py_examples` package :

```json
...   
entry_points={create a simple_subscriber.py file inside
        'console_scripts': [
            'simple_publisher = bumperbot_py_examples.simple_publisher:main',
        ],
```

*  in `package.xml`  declare execution dependencies (the two libraries we included in the script)

```xml
  <exec_depend>rclpy</exec_depend>
  <exec_depend>std-msgs</exec_depend>
```

* build with colcon

```bash
$ colcon build
Starting >>> bumperbot_cpp_examples
Starting >>> bumperbot_py_examples
Finished <<< bumperbot_cpp_examples [0.20s]                                                             
Finished <<< bumperbot_py_examples [0.64s]          

Summary: 2 packages finished [0.83s]
```

* in a different terminal source the workspace and run the node

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

### Subscriber in python

* create `simple_subscriber.py` file inside [`bumperbot_py_examples`](./src/bumperbot_py_examples/bumperbot_py_examples/) folder
* Add an entry point in `setup.py` :

```json
...   
entry_points={create a simple_subscriber.py file inside
        'console_scripts': [
            'simple_publisher = bumperbot_py_examples.simple_publisher:main',
            'simple_subscriber = bumperbot_py_examples.simple_subscriber:main',
        ],
```

*  build with colcon

```bash
$ colcon build
```

* in a different terminal source the workspace and run the node

```bash
$ source install/setup.bash
$ ros2 run bumperbot_py_examples simple_subscriber 
[INFO] [1719219470.084111799] [simple_subscriber]: I heard: Hello, ROS2 - counter: 716 
[INFO] [1719219470.972924372] [simple_subscriber]: I heard: Hello, ROS2 - counter: 717 
[INFO] [1719219471.972834813] [simple_subscriber]: I heard: Hello, ROS2 - counter: 718 
...s
```

