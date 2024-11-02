import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class SimpleTfKinematics(Node):
    def __init__(self):
        super().__init__('simple_tf_kinematics')


        # create a static transform broadcaster
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # create a static transform stamped message
        self.static_transtorm_stamped_ = TransformStamped()

        # set time stamp
        self.static_transtorm_stamped_.header.stamp = self.get_clock().now().to_msg()

        # set frame_id and child_frame_id
        self.static_transtorm_stamped_.header.frame_id = 'bumperbot_base'
        self.static_transtorm_stamped_.child_frame_id = 'bumperbot_top'

        # set translation
        self.static_transtorm_stamped_.transform.translation.x = 0.0
        self.static_transtorm_stamped_.transform.translation.y = 0.0
        self.static_transtorm_stamped_.transform.translation.z = 0.3

        # set rotation quaternion
        self.static_transtorm_stamped_.transform.rotation.x = 0.0
        self.static_transtorm_stamped_.transform.rotation.y = 0.0
        self.static_transtorm_stamped_.transform.rotation.z = 0.0
        self.static_transtorm_stamped_.transform.rotation.w = 1.0

        self.static_tf_broadcaster.sendTransform(self.static_transtorm_stamped_)

        self.get_logger().info('Publishing static transform from %s to %s' %
                               (self.static_transtorm_stamped_.header.frame_id,
                                self.static_transtorm_stamped_.child_frame_id)) 

        # create a dynamic transform broadcaster
        self.dynamic_tf_broadcaster = TransformBroadcaster(self)

        # create a dynamic transform stamped message
        self.dynamic_transform_stamped_ = TransformStamped()

        # create a timer to publish the dynamic transform
        self.timer = self.create_timer(0.1, self.timerCallback)

        # set time stamp, frame_id and child_frame_id
        self.dynamic_transform_stamped_.header.stamp = self.get_clock().now().to_msg()
        self.dynamic_transform_stamped_.header.frame_id = 'odom'
        self.dynamic_transform_stamped_.child_frame_id = 'bumperbot_base'

        self.x_increment_ = 0.05
        self.last_x_ = 0.0

    def timerCallback(self):
        # update the time stamp
        self.dynamic_transform_stamped_.header.stamp = self.get_clock().now().to_msg()

        # update the translation
        self.dynamic_transform_stamped_.transform.translation.x = self.last_x_ + self.x_increment_
        self.dynamic_transform_stamped_.transform.translation.y = 0.0
        self.dynamic_transform_stamped_.transform.translation.z = 0.0
        # update the rotation quaternion
        self.dynamic_transform_stamped_.transform.rotation.x = 0.0
        self.dynamic_transform_stamped_.transform.rotation.y = 0.0
        self.dynamic_transform_stamped_.transform.rotation.z = 0.0
        self.dynamic_transform_stamped_.transform.rotation.w = 1.0

        # publish the dynamic transform
        self.dynamic_tf_broadcaster.sendTransform(self.dynamic_transform_stamped_)

        # update the last x
        self.last_x_ = self.dynamic_transform_stamped_.transform.translation.x


def main():
    rclpy.init()
    simple_tf_kinematics = SimpleTfKinematics()
    rclpy.spin(simple_tf_kinematics)
    simple_tf_kinematics.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()