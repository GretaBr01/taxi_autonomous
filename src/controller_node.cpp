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

//spawn in gz sim world
#include "ros_gz_interfaces/msg/entity_factory.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "yaml-cpp/yaml.h"

#include <fstream>
#include <sstream>

#include <iostream>

#include <vector>
#include <functional>
#include <string>
#include <iostream>

struct MenuOption {
    int id;
    std::string description;
    std::function<void()> action;
};


class Controller : public rclcpp::Node {
public:
    Controller()
    : rclcpp::Node("controller_node"), state_(IDLE) {
        pub_spawn_ = this->create_publisher<ros_gz_interfaces::msg::EntityFactory>("/world/default/create", 10);
    }

    void insertInstance() {
        std::vector<std::string> valid_types = {"passenger"};
        std::string name, type;
        char choice;

        do {
            // Mostra i tipi disponibili
            std::cout << "\n\nScegli tipo istanza tra:\n";
            for (size_t i = 0; i < valid_types.size(); ++i) {
                std::cout << "  " << (i+1) << ") " << valid_types[i] << "\n";
            }
            std::cout << "\nInserisci il numero corrispondente al tipo: ";
            int type_index = 0;
            std::string input_type;
            std::getline(std::cin, input_type);
            try {
                type_index = std::stoi(input_type);
            } catch (...) {
                type_index = 0;
            }
            if (type_index < 1 || type_index > (int)valid_types.size()) {
                std::cout << "Tipo non valido, riprova.\n";
                continue;  // Ricomincia il ciclo senza aggiungere istanza
            }
            type = valid_types[type_index - 1];

            std::cout << "Inserisci nome istanza: ";
            std::getline(std::cin, name);

            problem_expert_->addInstance(plansys2::Instance{name, type});
            RCLCPP_INFO(get_logger(), "\nIstanza aggiunta: %s di tipo %s", name.c_str(), type.c_str());

            std::cout << "\n\nVuoi aggiungere un'altra istanza? (s/n): ";
            std::getline(std::cin, input_type);
            choice = (input_type.empty()) ? 'n' : input_type[0];

        } while (choice == 's' || choice == 'S');
    }


    void insertPredicate() {        
        std::vector<std::string> valid_types = {"at_passenger"};
        std::string name, loc;
        char choice;

        do {
            // Mostra i tipi disponibili
            std::cout << "Scegli tipo predicato tra:\n";
            for (size_t i = 0; i < valid_types.size(); ++i) {
                std::cout << "  " << (i+1) << ") " << valid_types[i] << "\n";
            }
            std::cout << "Inserisci il numero corrispondente al tipo: ";
            int type_index = 0;
            
            std::string input_type;
            std::getline(std::cin, input_type);
            try {
                type_index = std::stoi(input_type);
            } catch (...) {
                type_index = 0;
            }
            if (type_index < 1 || type_index > (int)valid_types.size()) {
                std::cout << "Tipo non valido, riprova.\n";
                continue;  // Ricomincia il ciclo senza aggiungere istanza
            }
            type_index --;

            switch (type_index){
                case 0:{
                    std::cout << "Inserisci passenger: ";
                    std::getline(std::cin, name);

                    std::cout << "Inserisci location: ";
                    std::getline(std::cin, loc);
                    
                    std::stringstream predicate_str;
                    predicate_str << "(at_passenger " << name << " " << loc << ")";
                    problem_expert_->addPredicate(plansys2::Predicate(predicate_str.str()));
                    
                    break;
                }

                default:
                    break;
            }

            RCLCPP_INFO(get_logger(), "predicato di tipo %s aggiunto", valid_types[type_index].c_str());

            std::cout << "Vuoi aggiungere un'altro predicato? (s/n): ";
            std::getline(std::cin, input_type);
            choice = (input_type.empty()) ? 'n' : input_type[0];

        } while (choice == 's' || choice == 'S');
    }

    void insertGoal() {
        auto current_goal = problem_expert_->getGoal();
        const std::string gs = parser::pddl::toString(current_goal);

        if (!gs.empty()) {
            std::cout << gs <<"\n";
            std::cout << "Goal attuale: " << gs  << "\n";
            std::cout << "Vuoi rimuoverlo e impostarne uno nuovo? (s/n): ";
            std::string answer;
            std::getline(std::cin, answer);

            if (answer != "s" && answer != "S") {
                std::cout << "Operazione 'inserimento nuovo goal' annullata.\n";
                return;
            }

            // Rimuove il goal corrente
            problem_expert_->clearGoal();
            RCLCPP_INFO(get_logger(), "Goal precedente rimosso.");
        }

        std::string goal;
        std::cout << "Inserisci goal PDDL [es: (and(at_passenger alice f) (at_passenger bob a)) oppure (and(at tax1 d)) ]: ";
        std::getline(std::cin, goal);

        problem_expert_->setGoal(plansys2::Goal(goal));
        RCLCPP_INFO(get_logger(), "Goal impostato: %s", goal.c_str());
    }

    void viewGoal(){
        auto goal = problem_expert_->getGoal();
        std::cout << "=== Goal corrente ===\n";
        const std::string gs = parser::pddl::toString(goal);
        std::cout << gs <<"\n";
        // // Ricorri i nodi PREDICATE con negate==false per stampare le condizioni del goal
        // for (auto & n : goal_tree.nodes) {
        //     if (n.node_type == plansys2_msgs::msg::Node::PREDICATE) {
        //     std::cout << " - (" << n.name;
        //     for (auto & par : n.parameters) {
        //         std::cout << " " << par.value;
        //     }
        //     std::cout << ")\n";
        //     }
        // }
    }

    void viewInstances(){
        auto instances = problem_expert_->getInstances();
        std::cout << "=== Istanze ===\n";
        for (auto & p : instances) {
            std::cout << " - " << p.name << " : " << p.type << "\n";
        }
    }

    void viewPredicates(){
        auto preds = problem_expert_->getPredicates();

        for (plansys2_msgs::msg::Node predicate : preds) {
            std::cout <<" - " << predicate.name << " ";

            for (size_t i = 0; i < predicate.parameters.size(); i++) {
                std::cout << predicate.parameters[i].name << " ";
            }

            std::cout <<"\n";
        }
    }

    
    void viewPlan(){
        std::cout << "AL momento la funzione non è implementata";
        // auto domain  = domain_expert_->getDomain();
        // auto problem = problem_expert_->getProblem();
        // auto maybe_plan = planner_client_->getPlan(domain, problem);

        // if (!maybe_plan.has_value()) {
        //     std::cout << "Nessun piano trovato.\n";
        //     return;
        // }

        // const auto & plan = maybe_plan.value();

        // size_t n_actions = plan.action.size();
        // std::cout << "=== Piano trovato (" << n_actions << " azioni) ===\n";

        // // 6. Scorri tutti gli array paralleli: action, time, duration
        // for (size_t i = 0; i < n_actions; ++i) {
        //     std::cout
        //     << (i+1) << ") " << plan.action[i]
        //     << "   start: "    << plan.time[i]
        //     << "   dur: "      << plan.duration[i]
        //     << "\n";
        // }

        // for (const auto & item : maybe_plan.items) {
        //     std::cout << item.time << ": " << item.action << "  [" << item.duration << "]\n";
        // }

        // if (!maybe_plan.has_value()) {
        //     std::cout << "Nessun piano trovato\n";
        //     return;
        // }
        // auto & plan = maybe_plan.value();
        // std::cout << "=== Piano (" << plan.action.size() << " azioni) ===\n";
        // for (size_t i = 0; i < plan.action.size(); ++i) {
        //     std::cout
        //     << i+1 << ") " << plan.action[i]
        //     << "  start: " << plan.time[i]
        //     << "  dur: "   << plan.duration[i]
        //     << "\n";
        // }

    }

    // void spawnSquares(std::string name, double x, double y){
    //     std::ostringstream sdf_stream;
    //     sdf_stream << "<sdf version='1.7'>"
    //             << "<model name='" << name << "'>"
    //             << "<static>true</static>"
    //             << "<link name='link'>"
    //             << "<pose>" << x << " " << y << " 0.005 0 0 0</pose>"
    //             << "<visual name='visual'>"
    //             << "<geometry><box><size>1.6 1.6 0.01</size></box></geometry>"
    //             << "<material><ambient>0 1 1 1</ambient><diffuse>0 1 1 1</diffuse></material>"
    //             << "</visual>"
    //             << "</link>"
    //             << "</model>"
    //             << "</sdf>";

    //     ros_gz_interfaces::msg::EntityFactory msg;
    //     msg.sdf = sdf_stream.str();
    //     msg.name = name;
    //     msg.pose.position.x = x;
    //     msg.pose.position.y = y;
    //     msg.pose.position.z = 0.005;

    //     pub_spawn_->publish(msg);
    //     RCLCPP_INFO(get_logger(), "Spawned square at [%f, %f] with name %s", x, y, name.c_str());
    // }


    void init(){
        domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
        planner_client_ = std::make_shared<plansys2::PlannerClient>();
        problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
        executor_client_ = std::make_shared<plansys2::ExecutorClient>();
        // init_knowledge();
    }

    void resetProblem(){
        RCLCPP_INFO(get_logger(), "Resetting the planning problem...");

        // 1. Rimuovi tutti i goal
        problem_expert_->clearGoal();

        problem_expert_->clearKnowledge();

        RCLCPP_INFO(get_logger(), "Planning problem reset complete.");
    }

    void showMenu(std::vector<MenuOption> menu) {
        while (rclcpp::ok() && state_ == IDLE) {
            std::cout << "\n--- Menu ---\n";
            for (const auto &option : menu) {
                std::cout << option.id << ") " << option.description << "\n";
            }
            std::cout << "Scelta: ";
            int choice;
            std::cin >> choice;
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

            auto it = std::find_if(menu.begin(), menu.end(),
                                [choice](const MenuOption &opt) { return opt.id == choice; });

            if (it != menu.end()) {
                if (it->id == 0) break;  // torna indietro
                it->action();
            } else {
                std::cout << "Scelta non valida.\n";
            }
        }
    }


    void showViewMenu() {
        std::vector<MenuOption> view_menu = {
            {1, "Visualizza Goal",        [this]() { viewGoal(); }},
            {2, "Visualizza Istanze",     [this]() { viewInstances(); }},
            {3, "Visualizza Predicati",   [this]() { viewPredicates(); }},
            //{4, "Visualizza Piano",       [this]() { viewPlan(); }},
            {0, "Torna indietro",         []() { /* niente: torna al main */ }}
        };
        showMenu(view_menu);
    }

    void showInsertMenu() {
        std::vector<MenuOption> insert_menu = {
            {1, "Inserisci Goal",         [this]() { insertGoal(); }},
            {2, "Inserisci Istanza",      [this]() { insertInstance(); }},
            {3, "Inserisci Predicato",    [this]() { insertPredicate(); }},
            {0, "Torna indietro",         []() { }}
        };
        showMenu(insert_menu);
    }

    void showResetMenu() {
        std::vector<MenuOption> reset_menu = {
            {1, "Resetta tutto il problema", [this]() { resetProblem(); }},
            {2, "Resetta Goal",              [this]() { problem_expert_->clearGoal(); }},
            {0, "Torna indietro",            []() { }}
        };
        showMenu(reset_menu);
    }

    void init_knowledge(){

        std::string config_path = ament_index_cpp::get_package_share_directory("my_taxi_autonomous") + "/config/problem_init.yaml";

        YAML::Node config = YAML::LoadFile(config_path);
        if (!config["locations"]) {
            RCLCPP_ERROR(get_logger(), "problem_init.yaml non contiene 'locations'");
            // return;
        }else{
            for (const auto& loc : config["locations"]) {
                std::string name = loc.first.as<std::string>();
                double x = loc.second[0].as<double>();
                double y = loc.second[1].as<double>();
                // spawnSquares(name, x, y);
                problem_expert_->addInstance(plansys2::Instance{name, "location"});
                
                if (loc.second.size() >= 3) {
                    std::string third_param = loc.second[2].as<std::string>();
                    if (third_param == "charging_station") {
                        std::string predicate_str = "(charging_station " + name + ")";
                        problem_expert_->addPredicate(plansys2::Predicate(predicate_str));
                    }
                }
            }
        }

        if (!config["vehicle"]) {
            RCLCPP_ERROR(get_logger(), "problem_init.yaml non contiene 'vehicle'");
            // return;
        }else{
            for (const auto& loc : config["vehicle"]) {
                std::string name = loc.first.as<std::string>();
                std::string station = loc.second.as<std::string>();
                problem_expert_->addInstance(plansys2::Instance{name,  "vehicle"});
                std::stringstream predicate_str;
                predicate_str << "(at " << name << " " << station << ")";
                problem_expert_->addPredicate(plansys2::Predicate(predicate_str.str()));
            }
        }

        if (!config["passenger"]) {
            RCLCPP_ERROR(get_logger(), "problem_init.yaml non contiene 'passenger'");
            // return;
        }else{
            for (const auto& loc : config["passenger"]) {
                std::string name = loc.first.as<std::string>();
                std::string station = loc.second.as<std::string>();
                problem_expert_->addInstance(plansys2::Instance{name,  "passenger"});
                std::stringstream predicate_str;
                predicate_str << "(at_passenger " << name << " " << station << ")";
                problem_expert_->addPredicate(plansys2::Predicate(predicate_str.str()));
            }
        }

        if (!config["connected"]) {
            RCLCPP_ERROR(get_logger(), "problem_init.yaml non contiene 'connected'");
            // return;
        }else{
            for (const auto& loc : config["connected"]) {
                std::string from = loc.first.as<std::string>();
                for (const auto& to_node : loc.second) {
                    std::string to = to_node.as<std::string>();
                    std::string predicate_str = "(connected " + from + " " + to + ")";
                    problem_expert_->addPredicate(plansys2::Predicate(predicate_str));
                }
            }
        }

        if (!config["traffic_heavy"]) {
            RCLCPP_ERROR(get_logger(), "problem_init.yaml non contiene 'traffic_heavy'");
            // return;
        }else{
            for (const auto& loc : config["traffic_heavy"]) {
                std::string from = loc.first.as<std::string>();
                for (const auto& to_node : loc.second) {
                    std::string to = to_node.as<std::string>();
                    std::string predicate_str = "(traffic_heavy " + from + " " + to + ")";
                    problem_expert_->addPredicate(plansys2::Predicate(predicate_str));
                }
            }
        }

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

                if (!executor_client_->start_plan_execution(current_plan_)) {
                    RCLCPP_ERROR(get_logger(), "Errore durante l'avvio dell'esecuzione del piano");
                    return;
                }
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

            std::vector<MenuOption> main_menu = {
                {1, "Carica problema esempio", [this]() { init_knowledge(); }},
                {2, "Crea ed esegui piano",    [this]() { state_ = NO_PLAN; }},
                {3, "Visualizza",              [this]() { showViewMenu(); }},
                {4, "Inserisci",               [this]() { showInsertMenu(); }},
                {5, "Resetta",                 [this]() { showResetMenu(); }},
                //{0, "Esci",                    [this]() { rclcpp::shutdown(); }}
            };

            showMenu(main_menu);

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

    rclcpp::Publisher<ros_gz_interfaces::msg::EntityFactory>::SharedPtr pub_spawn_;


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
