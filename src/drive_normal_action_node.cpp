#include <memory>
#include <algorithm>
#include <string>
#include <vector>
#include <iostream>
#include <chrono>

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "nav_msgs/msg/odometry.hpp"




using namespace std::chrono_literals;

class DriveNormalAction : public plansys2::ActionExecutorClient{
public:
  DriveNormalAction()
  : plansys2::ActionExecutorClient("drive_normal", 250ms)
  {
    current_pose_received_ = false;

    // Sottoscrivi la posizione attuale
    // pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    //   "/amcl_pose", 10,
    //   std::bind(&DriveNormalAction::pose_callback, this, std::placeholders::_1));

    pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/diff_drive_base_controller/odom", 10,
      std::bind(&DriveNormalAction::odom_callback, this, std::placeholders::_1));
    
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    this->declare_parameter("waypoints", std::vector<std::string>{});
    std::vector<std::string> wp_names_ = this->get_parameter("waypoints").as_string_array();

    for (const auto& name : wp_names_) {
      this->declare_parameter(name, std::vector<double>{});
      auto coords = this->get_parameter(name).as_double_array();
      if (coords.size() < 2) {
        RCLCPP_WARN(this->get_logger(), "Waypoint '%s' non valido (meno di 2 coordinate)", name.c_str());
        continue;
      }
      locations_[name] = {coords[0], coords[1]};
      RCLCPP_INFO(this->get_logger(), "Caricato waypoint '%s' -> x=%.2f y=%.2f", name.c_str(), coords[0], coords[1]);
    }

    // for (int i = 0; i < wp_names_.size(); i++) {
    //   std::string wp_str = wp_names_.at(i);
      
    //   declare_parameter(wp_str.c_str(), std::vector<double>{});
    //   std::vector<double> coords = get_parameter(wp_str.c_str()).as_double_array();
      
    //   //posizione dell'elemento sulla mapppa
    //   geometry_msgs::msg::PoseStamped wp;
    //   wp.header.frame_id = "map";
    //   wp.header.stamp = now();

    //   wp.pose.position.x = coords.at(0);
    //   wp.pose.position.y = coords.at(1);
    //   wp.pose.position.z = 0.0;
    //   wp.pose.orientation.x = 0.0;
    //   wp.pose.orientation.y = 0.0;
    //   wp.pose.orientation.z = 0.0;
    //   wp.pose.orientation.w = 1.0;
    //   locations_[wp_str] = wp;

    //   RCLCPP_INFO(this->get_logger(), "Loaded waypoint [%s]: x=%.2f, y=%.2f", wp_str.c_str(), coords[0], coords[1]);

    // }
  }

private:
  // void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  //   current_pose_ = msg->pose.pose;
  //   current_pose_received_ = true;
  // }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pose_ = msg->pose.pose;
    current_pose_received_ = true;
  }

  void send_velocity(double linear_x, double angular_z){
    geometry_msgs::msg::Twist msg;
    msg.linear.x = linear_x;
    msg.angular.z = angular_z;

    cmd_vel_pub_->publish(msg);
    // RCLCPP_INFO(this->get_logger(), "Sent velocity: linear.x = %.2f, angular.z = %.2f", linear_x, angular_z);
  }

  void do_work() override{
    if (!parsed_) {
      parse_parameters();
      parsed_ = true;
    }

    if (!current_pose_received_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Aspettando /diff_drive_base_controller/odom...");
      return;
    }

    double dx = target_x_ - current_pose_.position.x;
    double dy = target_y_ - current_pose_.position.y;
    double distance = std::hypot(dx, dy);

    
    double linear = 0.0;
    double angular = 0.0;

    if (distance < 0.2) {
      // Stop
      linear = 0.0;
      angular = 0.0;
      send_velocity(linear,angular);
      parsed_=false;
      finish(true, 1.0, "Goal reached");
      // goal_reached_ = true;
      return;
    }

    double target_angle = std::atan2(dy, dx);
    double yaw = get_yaw(current_pose_);
    double angle_diff = normalize_angle(target_angle - yaw);

    if (std::fabs(angle_diff) > 0.1) {
      angular = 0.5 * angle_diff;
    } else {
      linear = 0.3;
    }

    send_velocity(linear, angular);
    send_feedback(1.0 - (distance / max_expected_distance_), "Navigazione in corso...");
  }

  void parse_parameters(){
    auto arguments = get_arguments();
    if (arguments.size() >= 3) {
      robot_name_ = arguments[0];
      from_ = arguments[1];
      to_ = arguments[2];

      if (locations_.find(to_) == locations_.end()) {
        RCLCPP_ERROR(get_logger(), "Waypoint '%s' non trovato!", to_.c_str());
        finish(false, 0.0, "Waypoint non valido");
        return;
      }

      target_x_ = locations_[to_].first;
      target_y_ = locations_[to_].second;

      parsed_ = true;

      RCLCPP_INFO(get_logger(), "[%s] Navigazione da %s a %s -> x: %.2f, y: %.2f",
                  robot_name_.c_str(), from_.c_str(), to_.c_str(), target_x_, target_y_);

      // RCLCPP_INFO(get_logger(), "Taxi %s moving drive_normal from %s to %s",
      //             robot_name_.c_str(), from_.c_str(), to_.c_str());

      // auto wp_to_navigate = get_arguments()[2];  // The goal is in the 3rd argument of the action
      // RCLCPP_INFO(get_logger(), "MOVE ACTION --> Start navigation to [%s]", wp_to_navigate.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "Insufficient parameters for move action.");
    }
  }

  double get_yaw(const geometry_msgs::msg::Pose & pose) {
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

  double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
  }

  float progress_;
  std::map<std::string, std::pair<double, double>> waypoints_;
  double target_x_, target_y_;
  geometry_msgs::msg::Pose current_pose_;
  bool current_pose_received_ = false;
  bool parsed_ = false;
  std::string robot_name_;
  std::string from_;
  std::string to_;

  double forward_speed_ = 0.25;
  double rotation_speed_ = 0.5;
  double goal_tolerance_ = 0.25;
  double angle_tolerance_ = 0.1;
  double max_expected_distance_ = 8.0;

  // rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  std::map<std::string, std::pair<double, double>> locations_;
};

int main(int argc, char ** argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DriveNormalAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "drive_normal"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  // node->start();

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();
  return 0;
}