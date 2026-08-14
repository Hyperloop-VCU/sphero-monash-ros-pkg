# sphero-monash-ros

This repository is a ROS 2 wrapper around a basic sphero simulator and hardware backends.

Control and state interfaces follow standard ROS conventions:
1. Input command topic: geometry_msgs/Twist on /cmd_vel
2. Output pose topic: geometry_msgs/PoseStamped on /sphero/pose
3. Output odometry topic: nav_msgs/Odometry on /sphero/odom
4. Output map topic: nav_msgs/OccupancyGrid on /sphero/map
5. Output transform: map -> base_link via TF

![RViz simulator motion](docs/images/rviz_sim_motion.png)

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

## ROS Nodes

1. sphero_ros.nodes.sim_wrapper_node
2. sphero_ros.nodes.robot_wrapper_node
3. sphero_ros.nodes.teleop_node

Both nodes subscribe to /cmd_vel and publish /sphero/pose, /sphero/odom, and TF.

The simulator uses a generated maze by default and publishes it as /sphero/map.

