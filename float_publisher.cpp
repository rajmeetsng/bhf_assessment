#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

class FloatPublisher : public rclcpp::Node
{
public:
    FloatPublisher() : Node("float_publisher")
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float32>("float_topic", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(67),  // Publish after every 15 FPS
            std::bind(&FloatPublisher::timerCallback, this));
    }

private:
    void timerCallback()
    {
        auto message = std_msgs::msg::Float32();
        message.data = 1.23f;
        publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FloatPublisher>());
    rclcpp::shutdown();
    return 0;
}

