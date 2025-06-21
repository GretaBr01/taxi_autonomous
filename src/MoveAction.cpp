
#include "MoveAction.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


using namespace std::chrono_literals;

MoveAction::MoveAction(std::string node_name, std::chrono::nanoseconds time_)
: plansys2::ActionExecutorClient(node_name, time_)
{  
  // coordinate dichiarate nei file config_*.yaml
  this->declare_parameter("waypoints", std::vector<std::string>{});
  std::vector<std::string> wp_names_ = this->get_parameter("waypoints").as_string_array();

  for (int i = 0; i < wp_names_.size(); i++) {
    std::string wp_str = wp_names_.at(i);
    
    declare_parameter(wp_str.c_str(), std::vector<double>{});
    std::vector<double> coords = get_parameter(wp_str.c_str()).as_double_array();
    
    //posizione dell'elemento sulla mapppa
    geometry_msgs::msg::PoseStamped wp;
    wp.header.frame_id = "map";
    wp.header.stamp = now();

    wp.pose.position.x = coords.at(0);
    wp.pose.position.y = coords.at(1);
    wp.pose.position.z = 0.0;
    wp.pose.orientation.x = 0.0;
    wp.pose.orientation.y = 0.0;
    wp.pose.orientation.z = 0.0;
    wp.pose.orientation.w = 1.0;
    locations_[wp_str] = wp;

    RCLCPP_INFO(this->get_logger(), "Loaded waypoint [%s]: x=%.2f, y=%.2f", wp_str.c_str(), coords[0], coords[1]);

  }

  using namespace std::placeholders;
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
}

void MoveAction::current_pos_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
  current_pos_ = msg->pose.pose;
}

void MoveAction::send_velocity(double linear_x, double angular_z){
  geometry_msgs::msg::Twist msg;
  msg.linear.x = linear_x;
  msg.angular.z = angular_z;

  cmd_vel_pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "Sent velocity: linear.x = %.2f, angular.z = %.2f", linear_x, angular_z);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MoveAction::on_activate(const rclcpp_lifecycle::State & previous_state)
{

  goal_reached_ = false;
  send_feedback(0.0, "Move starting");

  auto wp_to_navigate = get_arguments()[2];  // The goal is in the 3rd argument of the action
  RCLCPP_INFO(get_logger(), "MOVE ACTION --> Start navigation to [%s]", wp_to_navigate.c_str());

  goal_pos_ = locations_[wp_to_navigate];

  dist_to_move = getDistance(goal_pos_.pose, current_pos_);

  // do_work();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void MoveAction::do_work(){
  RCLCPP_INFO(get_logger(), "Iniziato il lavoro");
  if (goal_reached_) return;

  double dist = getDistance(goal_pos_.pose, current_pos_);
  double angle_to_goal = atan2(
    goal_pos_.pose.position.y - current_pos_.position.y,
    goal_pos_.pose.position.x - current_pos_.position.x);
  double yaw = getYaw(current_pos_);
  double angle_diff = normalizeAngle(angle_to_goal - yaw);

  double linear_x;
  double angular_z;
  geometry_msgs::msg::Twist cmd;

  if (dist < 0.2) {
    // Stop
    linear_x = 0.0;
    angular_z = 0.0;
    send_velocity(linear_x,angular_z);
    finish(true, 1.0, "Goal reached");
    goal_reached_ = true;
    return;
  }

  if (std::abs(angle_diff) > 0.1) {
    // Rotate in place
    angular_z = 0.5 * angle_diff;
  } else {
    // Move forward
    linear_x  = 0.3;
  }

  send_velocity(linear_x,angular_z);
  send_feedback(std::min(1.0, 1.0 - dist / dist_to_move), "Moving...");
}

double MoveAction::getDistance(const geometry_msgs::msg::Pose & pos1, const geometry_msgs::msg::Pose & pos2){
  return std::hypot(pos1.position.x - pos2.position.x, pos1.position.y - pos2.position.y);
}

double MoveAction::getYaw(const geometry_msgs::msg::Pose & pose){
  tf2::Quaternion q(
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z,
    pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

double MoveAction::normalizeAngle(double angle){
  while (angle > M_PI) angle -= 2 * M_PI;
  while (angle < -M_PI) angle += 2 * M_PI;
  return angle;
}


 