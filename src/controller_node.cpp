#include <memory>

#include "plansys2_pddl_parser/Utils.hpp"

#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class Controller : public rclcpp::Node {
public:
    Controller()
    : rclcpp::Node("controller_node"), state_(NO_PLAN) {}

    void init(){
        domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
        planner_client_ = std::make_shared<plansys2::PlannerClient>();
        problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
        executor_client_ = std::make_shared<plansys2::ExecutorClient>();
        init_knowledge();
    }

    void init_knowledge(){
        problem_expert_->addInstance(plansys2::Instance{"taxi1", "vehicle"});
        problem_expert_->addInstance(plansys2::Instance{"alice", "passenger"});
        problem_expert_->addInstance(plansys2::Instance{"bob", "passenger"});
        problem_expert_->addInstance(plansys2::Instance{"a", "location"});
        problem_expert_->addInstance(plansys2::Instance{"b", "location"});
        problem_expert_->addInstance(plansys2::Instance{"c", "location"});
        problem_expert_->addInstance(plansys2::Instance{"d", "location"});
        problem_expert_->addInstance(plansys2::Instance{"e", "location"});
        problem_expert_->addInstance(plansys2::Instance{"f", "location"});

        problem_expert_->addPredicate(plansys2::Predicate("(at taxi1 a)"));
        problem_expert_->addPredicate(plansys2::Predicate("(at_passenger alice c)"));
        problem_expert_->addPredicate(plansys2::Predicate("(at_passenger bob d)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected a b)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected b c)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected c d)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected d e)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected e f)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected b e)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected b a)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected c b)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected d c)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected e d)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected f e)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected e b)"));

        problem_expert_->addPredicate(plansys2::Predicate("(charging_station a)"));
        problem_expert_->addPredicate(plansys2::Predicate("(charging_station f)"));

        problem_expert_->addPredicate(plansys2::Predicate("(traffic_heavy b c)"));
        problem_expert_->addPredicate(plansys2::Predicate("(traffic_heavy d e)"));

        problem_expert_->setGoal(plansys2::Goal("(and (at_passenger alice f) (at_passenger bob a))"));
    }

    void step(){
        if (state_ == NO_PLAN) {
            // Richiedi nuovo piano
            auto domain = domain_expert_->getDomain();
            auto problem = problem_expert_->getProblem();
            auto plan = planner_client_->getPlan(domain, problem);

            if (plan.has_value()) {
                current_plan_ = plan.value();
                executor_client_->start_plan_execution(current_plan_);
                state_ = EXECUTING;
                RCLCPP_INFO(get_logger(), "Piano trovato, inizio esecuzione");
            } else {
                RCLCPP_WARN(get_logger(), "Nessun piano trovato, ritento...");
            }
        }else if (state_ == EXECUTING) {
            // Controlla se l'azione è ancora in esecuzione
            if (executor_client_->execute_and_check_plan()) {
                // L'azione è ancora in corso
                return;
            }

            // Azione terminata, controlla risultato
            auto result = executor_client_->getResult();
            if (result.has_value()) {
                // Nota: Controlla il valore corretto del risultato (e.g., result.value().success)
                // Esempio ipotetico:
                if (result.value().success) {
                    RCLCPP_INFO(get_logger(), "Piano completato con successo!");
                    state_ = IDLE;
                } else {
                    RCLCPP_WARN(get_logger(), "Esecuzione piano fallita, ritento");
                    state_ = NO_PLAN;
                }
                // Riprova o termina
            }
        }else if (state_ == IDLE){

        }
    }

private:
    typedef enum {NO_PLAN, EXECUTING, IDLE} StateType;
    StateType state_;

    std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
    std::shared_ptr<plansys2::PlannerClient> planner_client_;
    std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
    std::shared_ptr<plansys2::ExecutorClient> executor_client_;
    
    plansys2_msgs::msg::Plan current_plan_;
};

int main(int argc, char ** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Controller>();
    node->init();

    rclcpp::Rate rate(5);
    while (rclcpp::ok()) {
        node->step();
        rate.sleep();
        rclcpp::spin_some(node->get_node_base_interface());
    }

    rclcpp::shutdown();
    return 0;
}
