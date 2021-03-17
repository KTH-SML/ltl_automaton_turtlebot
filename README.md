# ltl_automaton_turtlebot
Provides integration of Turtlebot2 with LTL automaton package

## Installation

### Dependencies
- [ltl_automaton_core](https://github.com/KTH-SML/ltl_automaton_core). Core ROS package for LTL.

- [turtlebot ROS package](https://wiki.ros.org/turtlebot). Turtlebot2 base packages.
  - For ROS Kinetic, install using `sudo apt install ros-kinetic-turtlebot* ros-kinetic-kobuki ros-kinetic-kobuki-msgs`
  - For ROS Melodic, Noetic or more recent, build the following packages from source:
    - [https://github.com/turtlebot/turtlebot](https://github.com/turtlebot/turtlebot)
    - [https://wiki.ros.org/turtlebot_msgs](https://wiki.ros.org/turtlebot_msgs)
    - [https://github.com/turtlebot/turtlebot_apps](https://github.com/turtlebot/turtlebot_apps)
    - [https://github.com/yujinrobot/kobuki](https://github.com/yujinrobot/kobuki)
    - [https://github.com/yujinrobot/kobuki_msgs](https://github.com/yujinrobot/kobuki_msgs)

- [turtlebot simulation](https://wiki.ros.org/turtlebot_simulator). Turtlebot2 simulation package, only needed for running gazebo simulation
  - For ROS Kinetic, install using `sudo apt install ros-kinetic-turtlebot-simulator`
  - For ROS Melodic, Noetic or more recent, build the following packages from source:
    - [https://github.com/turtlebot/turtlebot_simulator](https://github.com/turtlebot/turtlebot_simulator)

- [motion_capture_simulator](https://github.com/KTH-SML/motion_capture_simulator.git). Simulate the motion capture system in gazebo, only needed for running gazebo simulation.

- [PyYAML](https://pyyaml.org/). Should be integrated with ROS but it's better to check if version is up-to-date.
	- For Python2 (ROS Kinetic & Melodic):
	`pip install pyyaml`
	- For Python3 (ROS Morenia):
	`pip3 install pyyaml`
  
### Building
To build the package, clone the current repository in your catkin workspace and build it.
```
cd catkin_ws/src
git clone https://github.com/KTH-SML/ltl_automaton_turtlebot.git
```
Build your workspace with either *catkin_make* or *catkin build*
```
cd ...
catkin_make
```

## Usage
The package provides the agent-level code needed for interacting with the turtlebot.

To launch the planner and LTL nexus node, simply run the following command. Please note that a move_base node and low-level nodes need to be running for the turtlebot to receive the velocity commands.

```
roslaunch ltl_automaton_turtlebot ltl_turtlebot.launch
```

This will run the planner using the task specification in `config/turtlebot_ltl_formula.yaml` and transition system in `config/turtlebot_ts.yaml`.

A move base node relying on mocap for localization can be launched using

```
roslaunch ltl_automaton_turtlebot turtlebot_mocap_naivgation.launch
```

### Transition system and actions
The robot transition system needs to be a combination of at least one of the following type: `[2d_pose_region, turtlebot_load, battery_charge]`. The robot can carry the following actions:
- `goto_<region>` (from *2d_pose_region*): Go to the defined regions. The action needs to be part of the transition system textfile with the following attributes
  
  ```Python
  attr:
      region: r6                   # Region name
      pose: [[1,1.2,0], [0,0,0,1]] # Pose of center of region:
                                   # [origin [x,y,0], orientation quaternion [x,y,z,w]]
  ```
  
- `pick` (from *turtlebot_load*): Perform pick up action. The robot doesn't do anything per se but wait for confirmation of placed load. Confirmation is received on `pick_ack` topic.

- `drop` (from *turtlebot_load*): Perform deliver action. The robot doesn't do anything per se but wait for confirmation that assembly has been taken off. Confirmation is received on `drop_ack` topic.

- `charge` (from *battery_charge*): Charge the robot battery. The robot doesn't do anything per se but the guard of the action should ensure the robot is on its charging station. Change of state is triggered by the battery monitor, not the action itself.

## Config files

- **example_ltl_formula.yaml** Example of LTL formula with both hard and soft task.

- **example_ts.yaml** Example of LTL transition system definition.

- **turtlebot_ltl_formula.yaml** Another LTL formula used for a demonstration

- **turtlebot_ts.yaml** Another LTL transition system definition, used for a demonstration

## Launch files

- **ltl_turtlebot.launch** Basic example of the LTL planner implementation with the turtlebot as agent. Run the planner node and turtlebot LTL node with an example TS (Transition System) and example LTL formula.
    - `initial_ts_state_from_agent` If false, get initial TS (Transition System) state from the TS definition text parameter. If true, get initial TS state from agent topic. Default: `true`.
    - `agent_name` Agent name. Default: `turtlebot`.
    - `initial_beta` A higher value will help enforcing the soft task while a lower value can leave the soft task unenforced by the plan. The given value is only the initial one since beta can vary. Default: `10`
    - `gamma` Suffix weighting parameter. A higher value will increase the suffix cost and therefor minimize suffix word length. Default: `10`.


- **ltl_turtlebot_simple_example.launch** Example including HIL (Human-In-the-Loop) features and the move_base node.
    - `initial_ts_state_from_agent` If false, get initial TS (Transition System) state from the TS definition text parameter. If true, get initial TS state from agent topic. Default: `true`.
    - `agent_name` Agent name. Default: `turtlebot`.
    - `initial_beta` A higher value will help enforcing the soft task while a lower value can leave the soft task unenforced by the plan. The given value is only the initial one since beta can vary. Default: `10`
    - `gamma` Suffix weighting parameter. A higher value will increase the suffix cost and therefor minimize suffix word length. Default: `10`.


- **turtlebot_mocap_navigation.launch** Provide a move_base node implementation to use with the mocap ground truth localization.
    - `agent_name` Agent name. Default: `turtlebot`.
    - `run_mocap` Run the mocap from this launch file or not (useful in multi-robot settings when only one mocap node is needed). Default: `false`.
    - `odom_frame_id` Name of the odometry frame. Useful for multi-robot setting. Default: `odom`.
    - `base_frame_id` Name of the base link frame. Useful for multi-robot setting. Default: `base_footprint`.
    - `global_frame_id` Name of the global map frame. Default: `map`.

- **turtlebot_move_base.launch** DEPRECATED. Run a move_base node with some localization features to use with the mocap ground truth localization.
    - `agent_name` Agent name. Default: `turtlebot`.
    - `cmd_vel_topic` Topic to send the velocity command to. Default: `cmd_vel_mux/input/navi`

## Nodes
### ltl_automaton_turtlebot_node.py 
LTL Turtlebot node, execute the action sent by the LTL planner and returns the aggregated TS state from the state monitors. The turtlebot load state monitor is integrated within the LTL Turtlebot node and switching state in automatically done after completed the relevant action. The battery charge monitor is integrated within the LTL Turtlebot node and change states only based on the battery level (not the charge action).

#### Actions
*Action published topics*
- `move_base/goal` ([move_base_msgs/MoveBaseActionGoal](http://docs.ros.org/en/api/move_base_msgs/html/msg/MoveBaseActionGoal.html))
    
    Send pose command to reach region when executing the action `goto_<region>`.
    
#### Subscribed Topics
- `next_move_cmd` ([std_msgs/String](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/String.html))

    Next move from the output word (action sequence) to be carried out by the agent in order to satisfy the plan.
    
- `current_region` ([std_msgs/String](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/String.html))

    Agent region from the transition system state model `2d_pose_region`.
    
- `pick_ack` ([std_msgs/Bool](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html))

    Feedback for the action `pick`. The action is considered completed when an acknowledgement message is received on this topic.

- `drop_ack` ([std_msgs/Bool](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html))

    Feedback for the action `drop`. The action is considered completed when an acknowledgement message is received on this topic.

- `mobile_base/sensors/core` ([kuboki_msgs/SensorState](https://docs.ros.org/en/hydro/api/kobuki_msgs/html/msg/SensorState.html))

   Battery level is monitored from this topic and used to change the battery level state.
    
#### Published Topics
- `ts_state` ([ltl_automaton_msgs/TransitionSystemStateStamped](/ltl_automaton_msgs/msg/TransitionSystemStateStamped.msg))

    Agent TS state topic. The agent TS state is composed of a list of states from the different state models composing the action model. The turtlebot node aggretates the `2d_pose_region` state from a region_2d_pose_monitor with the internal load state.
    
#### Parameters
- `agent_name` (string, default: "agent")

    Agent name.
    
- `transition_system_textfile` (string)

    Action model transition system definition.
