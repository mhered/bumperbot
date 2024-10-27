#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
import numpy as np

# create a SimpleController class that inherits from Node
class SimpleController(Node): # 
    
    def __init__(self): # constructor
        
        # call constructor of the base class (takes String with name of node)
        super().__init__('simple_controller') 


        # declare and provide defaults for parameters 
        self.declare_parameter('wheel_radius', 0.033)
        self.declare_parameter('wheel_separation', 0.17)

        # read parameter values
        self.wheel_radius_= self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_separation_= self.get_parameter('wheel_separation').get_parameter_value().double_value

        # display parameter values
        self.get_logger().info("Using wheel_radius: %f" % self.wheel_radius_)
        self.get_logger().info("Using wheel_separation: %f" % self.wheel_separation_)

        # create a publisher for wheel speed commands of type Float64MultiArray 
        # in topic 'simple_velocity_controller/commands'

        self.wheel_cmd_pub_ = self.create_publisher(
            Float64MultiArray,                      # message type
            'simple_velocity_controller/commands',  # topic name
            10                                      # size of the publisher queue
            )

        # create a subscriber to joystick commands of type TwistStamped
        # in topic 'bumperbot_controller/cmd_vel'

        self.sub_ = self.create_subscription(
            TwistStamped,                           # message type
            'bumperbot_controller/cmd_vel',         # topic name
            self.velCallback,                       # callback function
            10                                      # size of the subscription queue
            )

        # calculate differential kinematics matrix
        self.diff_kinematics_matrix_ = self.wheel_radius_ /2 * np.array (
                [
                    [1, 1],
                    [2/self.wheel_separation_, -2/self.wheel_separation_]
                ]
            )
        
        self.get_logger().info("Using differential kinematics matrix: \n %s" % self.diff_kinematics_matrix_) 
      
    def velCallback(self, msg): # define callback as a member of the class
        
        # extract robot linear and angular velocities from the joystick message
        V = msg.twist.linear.x
        W = msg.twist.angular.z

        robot_velocities = np.array(
            [
                [V], 
                [W]
            ]
        )

        # calculate wheel rotational speeds from robot speeds
        wheel_speeds = np.matmul(np.linalg.inv(self.diff_kinematics_matrix_), robot_velocities)

        # prepare message with wheel velocity commands to be published 
        omega_right = wheel_speeds[0,0]
        omega_left = wheel_speeds[1,0] 
        wheel_speeds_msg = Float64MultiArray()

        # bumperbot_controllers.yaml declares right_wheel_joint then left_wheel_joint    
        wheel_speeds_msg.data = [omega_right, omega_left]

        # publish wheel commands
        self.wheel_cmd_pub_.publish(wheel_speeds_msg)


def main():

    # 1) Initialization: calling `init` for a particular Context. 
    # Must be done before any ROS nodes can be created.
    rclpy.init()
    

    # 2) Create one or more ROS nodes: by calling `create_node` or instantiating
    # a Node. Nodes may implement common ROS entities such as publishers, 
    # subscriptions, services and actions.
    simple_controller = SimpleController() # create a SimpleController object

    # 3) Process node callbacks: by calling `spin`, `spin_once`, or 
    # `spin_until_future_complete`.
    # After a node is created, one can process work items (e.g. subscription 
    # callbacks) that are waiting to be executed
    rclpy.spin(simple_controller) # keep node running continuously to process callbacks

    # 4) Shutdown
    # When finished with a previously initialized Context (i.e. done using
    # all ROS nodes associated with the context), the shutdown function should 
    # be called. This will invalidate all entities derived from the context.
    simple_controller.destroy_node() # destroy node
    rclpy.shutdown() # shutdown ROS2 interface

if __name__ == '__main__':
    main()