#include "MoveAction.hpp"

#include <memory>
#include <algorithm>
#include <string>
#include <vector>
#include <iostream>
#include <chrono>

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class DriveToChargeAction : public plansys2::ActionExecutorClient{
public:
  DriveToChargeAction()
  : plansys2::ActionExecutorClient("drive_to_charge", 250ms)
  {
    progress_ = 0.0;
  }

private:
  void do_work() override{
    if (!parsed_) {
      parse_parameters();
      parsed_ = true;
    }

    if (progress_ < 1.0) {
      progress_ += 0.02;
      send_feedback(progress_, "Taxi is moving");
    } else {
      finish(true, 1.0, "Taxi move drive_to_charge completed");

      progress_ = 0.0;
      std::cout << std::endl;
    }

    std::cout << "\r\e[K" << std::flush;
    std::cout << "Taxi moving drive_to_charge from " << from_ << " to " << to_ << " [" 
              << std::min(100.0, progress_ * 100.0) << "%]  " << std::flush;
  }

  void parse_parameters(){
    auto arguments = get_arguments();
    if (arguments.size() >= 3) {
      robot_name_ = arguments[0];
      from_ = arguments[1];
      to_ = arguments[2];

      RCLCPP_INFO(get_logger(), "Taxi %s moving drive_to_charge from %s to %s",
                  robot_name_.c_str(), from_.c_str(), to_.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "Insufficient parameters for move action.");
    }
  }

  float progress_;
  bool parsed_ = false;
  std::string robot_name_;
  std::string from_;
  std::string to_;
};

int main(int argc, char ** argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveAction>("drive_to_charge", 250ms);

  node->set_parameter(rclcpp::Parameter("action_name", "drive_to_charge"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();
  return 0;
}
