# sphero-monash-ros

This repository is a ROS 2 wrapper around Sphero simulator and hardware backends.

Control and state interfaces follow standard ROS conventions:
1. Input command topic: geometry_msgs/Twist on /cmd_vel
2. Output pose topic: geometry_msgs/PoseStamped on /sphero/pose
3. Output odometry topic: nav_msgs/Odometry on /sphero/odom
4. Output map topic: nav_msgs/OccupancyGrid on /sphero/map
5. Output transform: map -> base_link via TF

## Environment Management (Pixi-first)

This project uses Pixi with RoboStack (ROS 2 Humble) for reproducible setup.

Install Pixi, then from the repository root run:

```bash
pixi install
```

Optional editable install for external scripts:

```bash
pixi run develop
```

## Run

Start simulator wrapper node:

```bash
pixi run sim-node
```

Start simulator with RViz:

```bash
pixi run sim-launch
```

Start real robot wrapper node:

```bash
pixi run robot-node
```

Start real robot wrapper with RViz:

```bash
pixi run robot-launch
```

Start keyboard teleop node:

```bash
pixi run teleop-node
```

You can publish commands with:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}, angular: {z: 0.5}}'
```

## ROS Nodes

1. sphero_ros.nodes.sim_wrapper_node
2. sphero_ros.nodes.robot_wrapper_node
3. sphero_ros.nodes.teleop_node

Both nodes subscribe to /cmd_vel and publish /sphero/pose, /sphero/odom, and TF.

## RViz

The default RViz config is at rviz/sphero_pose.rviz and shows:
1. Grid
2. Maze occupancy map (/sphero/map)
3. Sphero pose topic
4. Sphero odometry topic
5. TF frames

## Notes

1. Internal simulator heading convention is converted to ROS yaw automatically.
2. Velocity commands are clipped by the configured vel_limit in the backend.
3. The simulator uses a generated maze by default and publishes it as /sphero/map.
4. Visualiser rendering has been removed and replaced with RViz launch workflows.
5. Occupancy convention is standard ROS: 0=free and 100=occupied. The provided RViz config uses Color Scheme=raw, so black=free and white=occupied.

