#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from turtlesim.msg import Pose

class SimpleTurtlesimKinematics(Node):
    
    def __init__(self):
        super().__init__('simple_turtlesim_kinematics')
        
        self.turtle1_pose_sub_ = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.turtle1PoseCallback,
            10
        )

        self.turtle2_pose_sub_ = self.create_subscription(
            Pose,
            '/turtle2/pose',
            self.turtle2PoseCallback,
            10
        )
        
        self.last_turtle1_pose_ = Pose()
        self.last_turtle2_pose_ = Pose()

    def turtle1PoseCallback(self, msg):
        self.last_turtle1_pose_ = msg
        
    def turtle2PoseCallback(self, msg):
        self.last_turtle2_pose_ = msg

        Tx = self.last_turtle2_pose_.x - self.last_turtle1_pose_.x
        Ty = self.last_turtle2_pose_.y - self.last_turtle1_pose_.y

        theta = self.last_turtle2_pose_.theta - self.last_turtle1_pose_.theta
        theta_deg = 180*theta/np.pi
    
        self.get_logger().info(f"""\n
            Translation vector turtle1 -> turtle2 \n
            Tx: {Tx:.2f} \n
            Ty: {Ty:.2f} \n
            Rotation matrix turtle1 -> turtle2 \n
            theta (rad): {theta:.2f} rad \n
            theta (deg): {theta_deg:.2f} deg \n
            [R11 R12] :    [{np.cos(theta):.2f}\t{-np.sin(theta):.2f}] \n
            [R21 R22] :    [{np.sin(theta):.2f}\t{np.cos(theta):.2f}] \n
            """) 

def main():
    rclpy.init()
    simple_turtlesim_kinematics = SimpleTurtlesimKinematics()
    rclpy.spin(simple_turtlesim_kinematics)
    simple_turtlesim_kinematics.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
