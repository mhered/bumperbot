#ifndef SIMPLE_TURTLESIM_KINEMATICS_HPP
#define SIMPLE_TURTLESIM_KINEMATICS_HPP

#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>

class SimpleTurtlesimKinematics : public rclcpp::Node{
public:
    // constructor
    SimpleTurtlesimKinematics(const std::string &node_name);

private:
    // Callback functions
    void turtle1PoseCallback(const turtlesim::msg::Pose::SharedPtr pose);
    void turtle2PoseCallback(const turtlesim::msg::Pose::SharedPtr pose);

    // Subscribers on the "/turtle1/pose" and "/turtle2/pose" topics
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle1_pose_sub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle2_pose_sub_;

    // Variables to store the last pose of each turtle
    turtlesim::msg::Pose::SharedPtr last_turtle1_pose_;
    turtlesim::msg::Pose::SharedPtr last_turtle2_pose_;
};

#endif