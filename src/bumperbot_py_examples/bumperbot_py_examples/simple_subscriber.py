#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# create a subscriber class
class SimpleSubscriber(Node): # inherit from Node
    
    def __init__(self): # constructor
        
        # call constructor of the base class (takes String with name of node)
        super().__init__('simple_subscriber') 
        
        # create a subscriber for String messages in channel chatter, 
        # max size 10
        self.sub_ = self.create_subscription(
            String, #message type
            'chatter', # topic name
            self.msgCallback, # callback function
            10 # size of the subscription queue
           ) 

    def msgCallback(self, msg): #define callback as a member of the class
        self.get_logger().info('I heard: %s' % msg.data) # log message
        

def main():

    # 1) Initialization: calling `init` for a particular Context. 
    # Must be done before any ROS nodes can be created.
    rclpy.init()
    

    # 2) Create one or more ROS nodes: by calling `create_node` or instantiating
    # a Node. Nodes may implement common ROS entities such as publishers, 
    # subscriptions, services and actions.
    simple_subscriber = SimpleSubscriber() # create a SimpleSubscriber object

    # 3) Process node callbacks: by calling `spin`, `spin_once`, or 
    # `spin_until_future_complete`.
    # After a node is created, one can process work items (e.g. subscription 
    # callbacks) that are waiting to be executed
    rclpy.spin(simple_subscriber) # keep node running continuously to process callbacks

    # 4) Shutdown
    # When finished with a previously initialized .Context (i.e. done using
    # all ROS nodes associated with the context), the shutdown function should 
    # be called. This will invalidate all entities derived from the context.
    simple_subscriber.destroy_node() # destroy node
    rclpy.shutdown() # shutdown ROS2

if __name__ == '__main__':
    main()