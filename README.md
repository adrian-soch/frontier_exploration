# frontier_exploration

## Description
A frontier exploration module implementied with ROS 2, C++, and Python. Based on:

```
B. Yamauchi, "A frontier-based approach for autonomous exploration," Proceedings 1997 IEEE International Symposium on Computational Intelligence in Robotics and Automation CIRA'97. 'Towards New Computational Principles for Robotics and Automation', Monterey, CA, USA, 1997, pp. 146-151, doi: 10.1109/CIRA.1997.613851.
```

## Getting Started

To get ready for development or execution:
```
# Clone the repo
git clone https://github.com/adrian-soch/frontier_exploration

# Build
colcon build

## If build fails due to `frontier_interfaces` try:
. install/setup.bash
colcon build
```

To run the exploration node:
```
# Start rosbag or robot simulation or real robot

ros2 launch frontier_exploration exploration.launch.py

# Test the service with:
ros2 service call  /frontier_pose frontier_interfaces/srv/FrontierGoal goal_rank:\ 0\ 
requester: making request: frontier_interfaces.srv.FrontierGoal_Request(goal_rank=0)

```

Output can be visualized in rviz with the frontier map and frontier region markers.

To run the full exploration system:

```
# Start rosbag or robot simulation or real robot
ros2 launch frontier_explorationlite_turtlebot_full_stack.launch.py

# Launch the frontier exploration node
ros2 launch frontier_exploration exploration.launch.py

# Start the nav_to_goal node
ros2 run frontier_exploration nav_to_goal.py
```

### Training Data COllection

1. Launch a simalation or real robot that publishes occupancy grid maps to the `/map`  topic.
2. Then `ros2 run frontier_exploration collection_node --ros-args -p filename:="/workspace/src/text.png"`