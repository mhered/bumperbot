#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using std::placeholders::_1; // to be able to use _1 as a placeholder

// Define the SimpleSubscriber class as a subclass of rclcpp::Node
class SimpleSubscriber : public rclcpp::Node
{
public:
    // constructor, initialize Node with node name "simple_subscriber"
    SimpleSubscriber() : Node("simple_subscriber")
    {
        // Create a subscriber on the "chatter" topic
        sub_ = this->create_subscription<std_msgs::msg::String>(
            "chatter",                                           // topic name
            10,                                                 // size of buffer queue
            std::bind(&SimpleSubscriber::msgCallback, this, _1) // callback function
        );

        RCLCPP_INFO(get_logger(), "Listening in topic /chatter");
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_; // shared pointer to the publisher with a String interface
    void msgCallback(const std_msgs::msg::String::SharedPtr msg) const     // callback function to be called at 1s intervals
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv); // initialize ROS2

    auto node = std::make_shared<SimpleSubscriber>(); // create a shared pointer to the SimpleSubscriber class

    rclcpp::spin(node); // run the node

    rclcpp::shutdown(); // shutdown ROS2

    return 0;
}