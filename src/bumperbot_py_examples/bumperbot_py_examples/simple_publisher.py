#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# create a publisher class
class SimplePublisher(Node): # inherit from Node
    
    def __init__(self): # constructor
        
        # call constructor of the base class
        super().__init__('simple_publisher') 
        
        # create a publisher for String messages in channel chatter, 
        # max size 10
        self.pub_ = self.create_publisher(String, 'chatter', 10) 
        
        self.counter_ = 0
        self.frequency_ = 1.0
        
        # console output
        self.get_logger().info('Publishing at %d Hz' % self.frequency_)

        # create timer object: executes a callback at regular times interval.
        self.timer = self.create_timer(self.frequency_, self.timerCallback)

    def timerCallback(self): #define callback as a member of the class
        msg = String() # create message
        msg.data = 'Hello, ROS2 - counter: %d ' % self.counter_

        self.pub_.publish(msg) # start publishing
        self.counter_ += 1 # increment counter

def main():

    # 1) Initialization: calling `init` for a particular Context. 
    # Must be done before any ROS nodes can be created.
    rclpy.init()
    

    # 2) Create one or more ROS nodes: by calling `create_node` or instantiating
    # a Node. Nodes may implement common ROS entities such as publishers, 
    # subscriptions, services and actions.
    simple_publisher = SimplePublisher() # create a SimplePublisher object

    # 3) Process node callbacks: by calling `spin`, `spin_once`, or 
    # `spin_until_future_complete`.
    # After a node is created, one can process work items (e.g. subscription 
    # callbacks) that are waiting to be executed
    rclpy.spin(simple_publisher) # process callbacks

    # 4) Shutdown
    # When finished with a previously initialized .Context (i.e. done using
    # all ROS nodes associated with the context), the shutdown function should 
    # be called. This will invalidate all entities derived from the context.
    simple_publisher.destroy_node() # destroy node
    rclpy.shutdown() # shutdown ROS2

if __name__ == '__main__':
    main()