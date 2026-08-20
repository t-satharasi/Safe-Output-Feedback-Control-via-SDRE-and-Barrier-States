# minimal_startup_air

Collection of launch files and helper scripts to bring up the UAV stack for Autonomy Park.
Covers single/multi agent hardware and simulation flight. The launch files run MAVROS,
'px4_telemetry', 'px4_teleop', 'px4_safety_lib', and 'autonomy_park_viz' at once to avoid opening multiple terminals.

## Prerequisites

- ROS2 with 'mavros', 'joy', 'px4_teleop', 'px4_safety_lib', 'fleet_manager', and 'autonomy_park_viz' built in your workspace.
- PX4-Autopilot built at '~/PX4-Autopilot'
- Gazebo Harmonic

## Simulation Scripts

These scripts handle bringing up multiple instances of PX4 in Gazebo.

## Launch Files

The launch files handle spinning up all required packages for running the base UAV stack for single 
and multi-agent flight. For single-agents mavros is included in the launch file. For multiple agents, 
you would run the multi-agent teleop launch file as well as the multi-agent mavros launch file to spin up 
multiple instances of mavros. 

## Typical workflow

```bash
# Open QGroundControl - Gazebo typically requires QGC be running before it will finish initializing PX4

# Terminal 1 — start Gazebo & PX4
./scripts/swarm-launch-gazebo.sh
 
# Terminal 2 — start MAVROS for all three instances
ros2 launch minimal_startup_air mavros_multisim.launch
 
# Terminal 4 — start autonomy stack
ros2 launch minimal_startup_air multiagent_teleop.launch.py
```
