#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

class TurtleEnablePublisher : public rclcpp::Node
{
public:
    TurtleEnablePublisher()
    : Node("turtle_kill_switch_publisher"), state_(true)
    {
        publisher_ = this->create_publisher<std_msgs::msg::Bool>("turtle_enabled", 10);
        state_timer_ = this->create_wall_timer(
            std::chrono::seconds(30),
            std::bind(&TurtleEnablePublisher::toggle_state_callback, this));
        publish_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&TurtleEnablePublisher::publish_state_callback, this));

        RCLCPP_INFO(this->get_logger(), "Turtle Enable Publisher started!");
    }

private:
    void toggle_state_callback()
    {
        state_ = !state_;
        auto state_str = state_ ? "Enabled" : "Disabled";
        RCLCPP_INFO(this->get_logger(), "State toggled: %s", state_str);
    }

    void publish_state_callback()
    {
        auto msg = std_msgs::msg::Bool();
        msg.data = state_;
        publisher_->publish(msg);
        auto state_str = state_ ? "Enabled" : "Disabled";
        RCLCPP_INFO(this->get_logger(), "Published: %s", state_str);
    }

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr state_timer_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    bool state_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleEnablePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
