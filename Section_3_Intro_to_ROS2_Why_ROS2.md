# Section 3: Why ROS2? (3.14-3.20)

Needed to overcome shortcomings for applications not initially contemplated when ROS was designed in 2007:

* Unreliable networks: ubiquitous wireless networks are less stable than wired

* Fleets of robots: ROS was originally designed for individual robots

* Embedded platforms: upgrade from serial comms to bidirectional (??)

* Secure comms: security becomes essential when moving away from secure labs towards home and industry applications 

* Real time systems: strict timing constraints when complexity increases

The backbone of ROS2 is DDS middleware for message exchange

DDS industrial grade comms protocol that provides node discovery, and messaging capabilities. Stable and robust. Several compnaies offer different implementations, multiple are supported, e.g. cyclone, fast and connext.

ROS middleware or rww

ROS client library or rcl is developed in C/C++. API interfaces available in C++ (rclcpp), python (rclpy) and Java:

ROS is not an OS, but four aspects make it similar:

* hardware abstraction: similar to the way an OS makes it interaction with different CPU or RAMS transparent. OS provides a standard interface and orchestration mechanics regardless of hardware
* low-level device control via drivers: drivers are developed by device manufacturers and expose a standard interface
* communication between parallel processes: in ROS processes are called nodes (applications). 3 protocols:
  * topics: publisher/subscriber nodes connect to topics transmitted over channels
  * services: 2 actors, service server and client. Client requests server functionality (typically short). Server stats processing and meanwhile the client can continue execution while waiting. When server finishes it sends response to client. 
  * actions: 2 actors, and action server (typically long running) and an action client that requests the execution of the action. e.g. navigation action from A to B. Client sends a goal to server. Server starts executing until goal reached, meanwhile client can continue executing. Server sends periodic updates to client, and final result message.  Client may send cancellation message.
* package management: sfw organized in packages to foster modularity, maintainability and reusability. Each package implements one functionality

