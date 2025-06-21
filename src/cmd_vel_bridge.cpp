#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class CmdVelBridge : public rclcpp::Node
{
public:
  CmdVelBridge()
  : Node("cmd_vel_bridge")
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&CmdVelBridge::twist_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/diff_drive_base_controller/cmd_vel", 10);
  }

private:
  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    geometry_msgs::msg::TwistStamped stamped_msg;
    stamped_msg.header.stamp = this->now();
    stamped_msg.header.frame_id = "base_link";  // oppure altro frame valido
    stamped_msg.twist = *msg;

    publisher_->publish(stamped_msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelBridge>());
  rclcpp::shutdown();
  return 0;
}
