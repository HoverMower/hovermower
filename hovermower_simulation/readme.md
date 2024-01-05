# Hovermower simulation
This repository contains launch files, models and worlds to simulate Hovermower in Gazebo.

# Usage
To just launch the world run one of these launch files
```
ros2 launch hovermower_simulation garden.launch
```
or
```
ros2 launch hovermower_simulation garden_world.launch
```

To load the world, spawing the robot and teleop it around, use this command
```
ros2 launch hovermower_simulation hm_simulation.launch.py
```


# Worlds
## garden_world.world
A world consisting of a house, lawn, a terrace, greenhouse, kids playground and so on. It consits out of just one model created in Blender

## garden.world
Similar to 1 beside that all objects as been placed in the world as individual objects.

## lawn.world
This is a garden world taken from [Automatic Adsison](https://automaticaddison.com/useful-world-files-for-gazebo-and-ros-2-simulations/)

# Models
All models used are either taken from Automatic Addison or from 3D Warehouse. I placed a link in each model for reference