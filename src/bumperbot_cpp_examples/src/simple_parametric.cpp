#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>
#include <memory>

#include <rcl_interfaces/msg/set_parameters_result.hpp>

// Define the SimpleParametric class as a subclass of rclcpp::Node
class SimpleParametric : public rclcpp::Node
{
public:
    // constructor, initialize Node with node name "simple_parametric"
    SimpleParametric() : Node("simple_parametric")
    {
        // declare a parameter named "simple_int_param" of type int with default value 7
        declare_parameter<int>("simple_int_param", 7);

        // declare a parameter named "simple_string_param" of type string with default value "default"
        declare_parameter<std::string>("simple_string_param", "default");

        // add a callback function to be called when parameters are set
        param_callback_handle_ = add_on_set_parameters_callback(std::bind(&SimpleParametric::paramChangeCallback, this, std::placeholders::_1));

        // log a message of which parameters the node has started with 
        RCLCPP_INFO_STREAM(get_logger(), "Node started with simple_int_param = " 
        << get_parameter("simple_int_param").as_int() 
        << " and simple_string_param = " 
        << get_parameter("simple_string_param").as_string());
    }

private:
    // shared pointer to store the output of add_on_set_parameters_callback
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // callback function to be called when parameters are set
    rcl_interfaces::msg::SetParametersResult paramChangeCallback(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;

        // loop through the parameters
        for (const auto &param : parameters)
        {
            if ( 
                param.get_name() == "simple_int_param" && 
                param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                RCLCPP_INFO_STREAM(get_logger(), "New value for simple_int_param: " << param.as_int());
                result.successful = true;
                result.reason = "simple_int_param updated";
            }

            else if (
                param.get_name() == "simple_string_param" &&
                param.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
            {
                RCLCPP_INFO_STREAM(get_logger(), "New value for simple_string_param: " << param.as_string());
                result.successful = true;
                result.reason = "simple_string_param updated";
            }
            else
            {
                result.successful = false;
                result.reason = "Invalid parameter";
            }
        }

        return result;
    }
};

int main(int argc, char *argv[])
{
    // initialize ROS2 interface
    rclcpp::init(argc, argv);

    // create a shared pointer to the SimpleParametric class
    auto node = std::make_shared<SimpleParametric>();
    
    // run the node
    rclcpp::spin(node);

    // shutdown ROS2
    rclcpp::shutdown();

    return 0;
}