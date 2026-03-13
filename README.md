# nav2_proximity_wait

ROS 2 Humble package containing a custom Nav2 Behavior Tree (BT) plugin for multi-robot swarm coordination. This package provides the `CheckRobotProximity` BT node, which prevents deadlocks and collisions by forcing robots to pause (yield) when another higher-priority robot comes too close during navigation.

> **Note**: This package is part of the work-in-progress CHARS (Collaborative Heterogeneous Autonomous Robot Swarm) architecture, specifically addressing multi-agent navigation conflicts.

## Features

- **Dynamic Priority Right-of-Way**: Automatically extracts numeric priority from robot namespaces (e.g., `agent1` yields to nothing, `agent2` yields to `agent1`). Lower ID implies higher priority, solving mutual deadlock scenarios.
- **Hysteresis-Based Yielding**: Uses separate `safety_radius` (trigger pause) and `clear_radius` (trigger resume) to prevent rapid start/stop stuttering when robots hover around the threshold.
- **Efficient Non-Spinning Pauses**: The plugin returns `BT::NodeStatus::RUNNING` when yielding, giving control back to the BT scheduler gracefully without burning CPU cycles in 100 Hz spin-loops.
- **Reactive BT Integration**: Provides a custom `navigate_w_proximity_wait.xml` that cleanly gates the normal `FollowPath` action behind the proximity check inside a `ReactiveSequence`.
- **Centralized Parameter Management**: Robot frames and radii are read directly from `bt_navigator` ROS parameters (populated dynamically from `fleet_config.yaml`), eliminating hardcoded values in the BT XML.

## Package Contents

```
nav2_proximity_wait/
├── src/
│   └── check_robot_proximity.cpp         # Main BT action node plugin source
├── include/
│   └── nav2_proximity_wait/
│       └── check_robot_proximity.hpp     # Plugin header
├── bt_xml/
│   └── navigate_w_proximity_wait.xml     # Modified Nav2 BT utilizing the proximity node
└── config/
    ├── bt_navigator_proximity.yaml       # Nav2 plugin registration config
    └── fleet_config.yaml                 # Centralized swarm fleet definitions
```

## Dependencies

### ROS 2 Packages
```bash
# Core BT and Navigation
ros-humble-nav2-behavior-tree
ros-humble-behaviortree-cpp-v3
ros-humble-nav2-msgs
ros-humble-nav2-util

# TF2 for proximity checks
ros-humble-tf2
ros-humble-tf2-ros
ros-humble-tf2-geometry-msgs
```

## Architecture Notes

### Priority System and TF Lookups
The `CheckRobotProximity` node determines its "self frame" based on the ROS node namespace (e.g., `agent2/base_link`). It then parses the integer ID (`2`) to establish its priority. 
On every BT tick, it measures the 2D Euclidean distance to all `other_robot_frames` (e.g., `agent1/base_link`, `agent3/base_link`) using `tf2`. If an intersection occurs within the `safety_radius` and the other robot has a lower integer ID, the current robot enters a paused state until the other robot clears the `clear_radius`.

### BT XML Modifications
The custom behavior tree (`navigate_w_proximity_wait.xml`) wraps the `CheckRobotProximity` and `FollowPath` nodes inside a `ReactiveSequence`. 
- If `CheckRobotProximity` returns `SUCCESS`, `FollowPath` is ticked.
- If it returns `RUNNING` (yielding), `FollowPath` is immediately halted, safely stopping the robot on its local path without abandoning the global goal.

## Configuration Files

### bt_navigator_proximity.yaml
Registers the custom `nav2_proximity_wait_bt_node` along with the standard Nav2 BT plugins so `bt_navigator` can dlopen the library at runtime. Also reserves parameter space for `other_robot_frames`, `safety_radius`, and `clear_radius`.

### fleet_config.yaml
Acts as the single source of truth for proximity wait settings for the entire swarm.
- `fleet_namespaces`: List of active agents (e.g., `agent1`, `agent2`).
- `safety_radius`: Pause threshold (e.g., `1.0m`).
- `clear_radius`: Resume threshold (e.g., `1.5m`).

## Building

```bash
cd ~/swarm_ws
colcon build --packages-select nav2_proximity_wait
source install/setup.bash
```

## License

Apache License 2.0

## Maintainer

**Viswa Teja Bottu**  
Email: vss.viswatejabottu@gmail.com
