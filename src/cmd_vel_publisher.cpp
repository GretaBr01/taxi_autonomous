#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class CmdVelPublisher : public rclcpp::Node
{
public:
  CmdVelPublisher()
  : Node("cmd_vel_publisher")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }

  void send_velocity(double linear_x, double angular_z)
  {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = linear_x;
    msg.angular.z = angular_z;

    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Sent velocity: linear.x = %.2f, angular.z = %.2f", linear_x, angular_z);
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CmdVelPublisher>();

  // Pubblica una sola volta, poi esce
  rclcpp::sleep_for(std::chrono::milliseconds(500));  // Attendi che i publisher si inizializzino
  node->send_velocity(-1.0, 0.0);

  rclcpp::shutdown();
  return 0;
}
