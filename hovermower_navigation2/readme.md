# Hovermower navigation2 package
This apckage contains launch files and configuration to use navigation2 and slam_toolbox together with Hovermower.

# Usage
## Prerequisites
First you need to either start a real Hovermower robot with this command
```
ros2 launch hovermower_bringup teleop.launch.py
```
or you need to start a simulated Hovermower with this command
```
ros2 launch hovermower_simulation hm_simulation.launch.py
```

## navigation
To start navigation2 use this command
```
ros2 launch hovermower_navigation2 navigation_launch.py
```
## SLAM
To start simultaneous locaization and mapping (SLAM) use this command
```
ros2 launch hovermower_navigation2 online_async_launch.py
```