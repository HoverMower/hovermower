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

# Prerequisites and installation instructions
## Hector plugins
This simulation uses hector_gazebo_plugins provided by [Technische Universität Darmstadt](https://github.com/tu-darmstadt-ros-pkg/hector_gazebo)
I has some trouble to get humble-devel branch running. After clone and build, gazebo keeps crashing, even if I don't use the plugin in my world. I tried to figure
out the root cause and found at least the package, which causes the issue. It gets caused by
```
hector_gazebo_worlds
```

Before you build, delete this package from the cloned repo in your src folder. Now gazebo starts without any crash. I don't know exactly why this happens but I  raised a issue at [github.com](I don't know exactly why this happens.)