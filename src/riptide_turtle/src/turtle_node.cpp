#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "example_interfaces/srv/trigger.hpp"

class TurtleNode : public rclcpp::Node
{
public:
    TurtleNode()
    : Node("turtle_node"), enabled_(false)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        subscription_ = this->create_subscription<std_msgs::msg::Bool>(
            "turtle_enabled", 10, std::bind(&TurtleNode::enabled_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&TurtleNode::publish_twist, this));

        this->declare_parameter<double>("linear_speed", 2.0);
        this->declare_parameter<double>("angular_speed", 1.0);

        reset_speed_service_ = this->create_service<example_interfaces::srv::Trigger>(
            "reset_speed", std::bind(&TurtleNode::reset_speed_callback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Turtle Node started!");
    }

private:
    void enabled_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        enabled_ = msg->data;
        auto state_str = enabled_ ? "Enabled" : "Disabled";
        RCLCPP_INFO(this->get_logger(), "Received state: %s", state_str);
    }

    void publish_twist()
    {
        if (enabled_)
        {
            auto linear_speed = this->get_parameter("linear_speed").as_double();
            auto angular_speed = this->get_parameter("angular_speed").as_double();

            auto twist = geometry_msgs::msg::Twist();
            twist.linear.x = linear_speed;
            twist.angular.z = angular_speed;
            publisher_->publish(twist);

            RCLCPP_INFO(this->get_logger(), "Published twist message with linear_speed=%.2f, angular_speed=%.2f", linear_speed, angular_speed);
        }
    }

    void reset_speed_callback(
        const std::shared_ptr<example_interfaces::srv::Trigger::Request> request,
        std::shared_ptr<example_interfaces::srv::Trigger::Response> response)
    {
        (void)request;
        this->set_parameter(rclcpp::Parameter("linear_speed", 2.0));
        this->set_parameter(rclcpp::Parameter("angular_speed", 1.0));

        RCLCPP_INFO(this->get_logger(), "Linear and Angular speeds reset to default values");
        response->success = true;
        response->message = "Speeds have been reset to default values";
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<example_interfaces::srv::Trigger>::SharedPtr reset_speed_service_;
    bool enabled_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
