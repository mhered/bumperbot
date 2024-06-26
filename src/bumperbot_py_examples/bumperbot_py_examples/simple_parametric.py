#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter
# create a parametric node class
class SimpleParametric(Node): # inherit from Node
    
    def __init__(self): # constructor
        
        # call constructor of the base class
        super().__init__('simple_parametric') 
        
        # declare and provide defaults for parameters 
        self.declare_parameter('simple_int_param', 0)
        self.declare_parameter('simple_string_param', 'hello')

        # add callback for parameter changes
        self.add_on_set_parameters_callback(self.parameterChangeCallback)

    # define callback as a member of the class
    def parameterChangeCallback(self, params): 
        result=SetParametersResult()
        for param in params:
            if param.name == 'simple_int_param' and param.type_ == Parameter.Type.INTEGER:
                self.get_logger().info('Parameter simple_int_param changed to : %d' 
                               % param.value)
                result.successful = True
            
            if param.name == 'simple_string_param' and param.type_ == Parameter.Type.STRING:
                self.get_logger().info('Parameter simple_string_param changed to : %s' 
                               % param.value)
                result.successful = True

        return result


def main():

    # 1) Initialization: calling `init` for a particular Context. 
    # Must be done before any ROS nodes can be created.
    rclpy.init()
    
    # 2) Create one or more ROS nodes: by calling `create_node` or instantiating
    # a Node. Nodes may implement common ROS entities such as publishers, 
    # subscriptions, services and actions.
    simple_parametric = SimpleParametric() # create a SimplePublisher object

    # 3) Process node callbacks: by calling `spin`, `spin_once`, or 
    # `spin_until_future_complete`.
    # After a node is created, one can process work items (e.g. subscription 
    # callbacks) that are waiting to be executed
    rclpy.spin(simple_parametric) # process callbacks

    # 4) Shutdown
    # When finished with a previously initialized .Context (i.e. done using
    # all ROS nodes associated with the context), the shutdown function should 
    # be called. This will invalidate all entities derived from the context.
    simple_parametric.destroy_node() # destroy node
    rclpy.shutdown() # shutdown ROS2

if __name__ == '__main__':
    main()