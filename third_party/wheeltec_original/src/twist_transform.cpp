#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

class TwistTransform : public rclcpp::Node
{
public:
  TwistTransform() : Node("twist_transform")
  {
    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/twist_raw",
      10,
      std::bind(&TwistTransform::twistCmdCallback, this, std::placeholders::_1)
    );
  }

  ~TwistTransform()
  {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;

    if (twist_pub_) {
      twist_pub_->publish(twist);
    }
  }

private:
  void twistCmdCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    geometry_msgs::msg::Twist twist = msg->twist;
    twist_pub_->publish(twist);

    RCLCPP_INFO(this->get_logger(), "Transform Success !");
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TwistTransform>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}