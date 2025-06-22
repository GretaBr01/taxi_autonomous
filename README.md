# taxi_autonomous
The project aimed to create a symbolic and temporal planning system for a mobile robot, through the integration of the PlanSys2 framework with the external TFD (Temporal Fast Downward) planner.

The execution of the plan found with Plansys2 is simulated using Gazebo for a visual representation. A two-wheel drive vehicle using a differential driving controller is modeled. To estimate the position of the robot, only odometry is used.

The project is developed using ROS2 Jazzy, PlanSys2 and Gazebo Harmonic.

This project demonstrated how it is possible to integrate temporal AI planning techniques into a simulated mobile robotic system, thanks to the synergy between PlanSys2, TFD and ROS 2. Modeling in PDDL and simulation in Gazebo allowed to test planning and execution, highlighting the scalability and flexibility of the approach.