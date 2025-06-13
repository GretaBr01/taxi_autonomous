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

class ChargeAction : public plansys2::ActionExecutorClient{
public:
  ChargeAction()
  : plansys2::ActionExecutorClient("charge", 500ms)
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
      progress_ += 0.05;
      send_feedback(progress_, "Taxi is charging");
    } else {
      finish(true, 1.0, "Charging completed");

      progress_ = 0.0;
      std::cout << std::endl;
    }

    std::cout << "\r\e[K" << std::flush;
    std::cout << "Charging taxi " << robot_name_ << " at " << location_ << " [" 
              << std::min(100.0, progress_ * 100.0) << "%]  " << std::flush;
  }

  void parse_parameters(){
    auto arguments = get_arguments();
    if (arguments.size() >= 2) {
      robot_name_ = arguments[0];
      location_ = arguments[1];

      RCLCPP_INFO(get_logger(), "Taxi %s starts charging at %s",
                  robot_name_.c_str(), location_.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "Insufficient parameters for charge action.");
    }
  }

  float progress_;
  bool parsed_ = false;
  std::string robot_name_;
  std::string location_;
};

int main(int argc, char ** argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ChargeAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "charge"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();
  return 0;
}
