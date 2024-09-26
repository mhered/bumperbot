#include "bumperbot_cpp_examples/simple_turtlesim_kinematics.hpp"
#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>

// constructor, initialize Node
SimpleTurtlesimKinematics::SimpleTurtlesimKinematics(
    const std::string &node_name) : Node(node_name)
{
    // Create a subscriber on the "/turtle1/pose" topic
    turtle1_pose_sub_ = create_subscription<turtlesim::msg::Pose>(
        "/turtle1/pose", // topic name
        10,              // size of buffer queue
        std::bind(&SimpleTurtlesimKinematics::turtle1PoseCallback,
                  this,
                  std::placeholders::_1) // callback function
    );

    RCLCPP_INFO(get_logger(), "Listening in topic /turtle1/pose");

    // Create a subscriber on the "/turtle2/pose" topic
    turtle2_pose_sub_ = create_subscription<turtlesim::msg::Pose>(
        "/turtle2/pose", // topic name
        10,              // size of buffer queue
        std::bind(&SimpleTurtlesimKinematics::turtle2PoseCallback,
                  this, std::placeholders::_1) // callback function
    );

    RCLCPP_INFO(get_logger(), "Listening in topic /turtle2/pose");
}

void SimpleTurtlesimKinematics::turtle1PoseCallback(const turtlesim::msg::Pose::SharedPtr pose)
{
    last_turtle1_pose_ = pose;
}

void SimpleTurtlesimKinematics::turtle2PoseCallback(const turtlesim::msg::Pose::SharedPtr pose)
{
    last_turtle2_pose_ = pose;
    float Tx = last_turtle2_pose_->x - last_turtle1_pose_->x;
    float Ty = last_turtle2_pose_->y - last_turtle1_pose_->y;
    float theta = last_turtle2_pose_->theta - last_turtle1_pose_->theta;
    float theta_deg = theta * 180 / M_PI;

    RCLCPP_INFO_STREAM(get_logger(),
                       "\nTranslation vector turtle1 -> turtle2\n"
                           << "Tx: " << Tx << "\n"
                           << "Ty: " << Ty << "\n"
                           << "Rotation matrix turtle1 -> turtle2\n" 
                           << "Theta(rad): " << theta << " rad\n"
                           << "Theta(deg): " << theta_deg << " deg\n"
                           << "[R11 R12] :    [" << std::cos(theta) << "\t" << -std::sin(theta) << "] \n"
                           << "[R21 R22] :    [" << std::sin(theta) << "\t" << std::cos(theta) << "] \n"
                           );
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv); // initialize ROS2

    auto node = std::make_shared<SimpleTurtlesimKinematics>("simple_kinematics_node"); // create a shared pointer to the SimpleTurtlesimKinematics class

    rclcpp::spin(node); // run the node

    rclcpp::shutdown(); // shutdown ROS2

    return 0;
}