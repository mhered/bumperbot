# Section 8 TF2 Library

Where is the robot? Relative to its starting position? Relative to a global reference frame fixed on the map?

Convention:

* Reference frame attached to initial pose: **O**

* Reference frame attached to current pose: **R** 

* Global reference frame attached to map: **M**

What we need are the transformation matrices between **RO** and **OM**. 

ROS provides the TF2 Library to manipulate reference frames. The robot model can be seen as a tree of interconnected reference frames. The `robot_state_publisher` node parses the URDF model of the robot, extracts info from the links to calculate the transformation matrices and publishes them in the topics:

* Static transforms: fixed joints defining the geometry of the robot. This info is published only once in topic `/tf_static`.

* Dynamic transforms: moving joints e.g. rotating wheels. This info is updated and published continuously in `/tf`.

We can compose transformation matrices by multiplying them:

*  a localization app allows expressing the robot pose `base_footprint` relative to fixed reference frame `map_link`, via a transformation matrix $$T^{map}_{base}$$
* the URDF robot model allows finding the `camera_link` relative to `base_footprint`, via a transformation matrix $$T^{base}_{camera}$$
* a vision app allows to detect a person and attaches a reference frame `person_link` to it relative to the camera, via a transformation matrix $$T^{camera}_{person}$$

We can find the person in the map with the transformation matrix $$T^{map}_{person}=T^{map}_{base}\cdot T^{base}_{camera} \cdot T^{camera}_{person}$$

Note: keeping the order is very important when multiplying transformation matrices!

