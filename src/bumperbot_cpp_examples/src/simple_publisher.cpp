#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>

using namespace std::chrono_literals; // to be able to use 1s literal

// Define the SimplePublisher class as a subclass of rclcpp::Node
class SimplePublisher : public rclcpp::Node
{
public:
    // constructor, initialize Node with node name "simple_publisher" and counter to 0
    SimplePublisher() : Node("simple_publisher"), counter_(0)
    {
        // Create a publisher on the "chatter" topic
        pub_ = create_publisher<std_msgs::msg::String>(
            "chatter", // topic name
            10         // size of queue
        );

        // create a timer to launch callback at 1s intervals
        timer_ = create_wall_timer(
            1s,                                               // interval to call the callback function (literal: 1 second)
            std::bind(&SimplePublisher::timerCallback, this) // callback function
        );

        RCLCPP_INFO(get_logger(), "Publishing at 1 Hz");
    }

private:
    unsigned int counter_;                                    // counter to keep track of the number of messages published
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_; // shared pointer to the publisher
    rclcpp::TimerBase::SharedPtr timer_;                      // shared pointer to the timer

    void timerCallback() // callback function to be called at 1s intervals
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello ROS2! [" + std::to_string(counter_++) + "]";

        pub_->publish(message);
    }
};


int main (int argc, char* argv[])
{
    rclcpp::init(argc, argv); // initialize ROS2

    auto node = std::make_shared<SimplePublisher>(); // create a shared pointer to the SimplePublisher class
    rclcpp::spin(node); // run the node

    rclcpp::shutdown(); // shutdown ROS2

    return 0;
}