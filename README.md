# taxi_autonomous
The project aimed to create a symbolic and temporal planning system for a mobile robot, through the integration of the PlanSys2 framework with the external TFD (Temporal Fast Downward) planner.

The execution of the plan found with Plansys2 is simulated using Gazebo for a visual representation. A two-wheel drive vehicle using a differential driving controller is modeled. To estimate the position of the robot, only odometry is used.

The project is developed using ROS2 Jazzy, PlanSys2 and Gazebo Harmonic.

This project demonstrated how it is possible to integrate temporal AI planning techniques into a simulated mobile robotic system, thanks to the synergy between PlanSys2, TFD and ROS 2. Modeling in PDDL and simulation in Gazebo allowed to test planning and execution, highlighting the scalability and flexibility of the approach.

## Prerequisites
- ROS 2 Jazzy
- Gazebo simulator
- PlanSys2 planning framework
- Temporal Fast Downward (TFD) planner: [TFDPlanSolver](https://github.com/PlanSys2/plansys2_tfd_plan_solver)

## To execute the project:
1. Clone the repository  in your workspace
2. Build the workspace
3. Launch the Simulation:  ```ros2 launch my_taxi_autonomous my_taxi_planner_launch.py```
4. Once the Gazebo simulation is running, start the controller node: ```ros2 run my_taxi_autonomous controller_node```

## Warnings During Execution

You may encounter the following errors or warnings in the terminal when launching the planner or simulation

These messages are typically related to:
- Predicates defined in PDDL that are not declared in the domain file
- Typos or inconsistencies in the `problem.pddl` file
- Passengers or locations that are not yet part of the simulation scene

### These do **not** affect the functionality of the project.
The simulation and planning system will continue to run as long as valid goals and domain/problem definitions are provided.

> You can ignore these messages unless you are debugging or customizing the PDDL logic.



