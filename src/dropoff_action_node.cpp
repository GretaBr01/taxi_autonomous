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

class DropoffAction : public plansys2::ActionExecutorClient{
public:
  DropoffAction()
  : plansys2::ActionExecutorClient("dropoff", 100ms)
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
      progress_ += 0.1;
      // send_feedback(progress_, "Dropping off passenger");
    } else {
      finish(true, 1.0, "Dropoff completed");

      progress_ = 0.0;
      std::cout << std::endl;
    }

    // std::cout << "\r\e[K" << std::flush;
    // std::cout << "Taxi " << robot_ << " dropping off " << passenger_ 
    //           << " at " << location_ << " [" 
    //           << std::min(100.0, progress_ * 100.0) << "%]  " << std::flush;
  }

  void parse_parameters(){
    auto args = get_arguments();
    if (args.size() >= 3) {
      robot_ = args[0];
      passenger_ = args[1];
      location_ = args[2];

      RCLCPP_INFO(get_logger(), "Taxi %s dropping off %s at %s", 
                  robot_.c_str(), passenger_.c_str(), location_.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "Insufficient parameters for drop-off action.");
    }
  }

  float progress_;
  bool parsed_ = false;
  std::string robot_;
  std::string passenger_;
  std::string location_;
};

int main(int argc, char ** argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DropoffAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "dropoff"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
