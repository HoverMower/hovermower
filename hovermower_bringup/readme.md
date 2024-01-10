# Hovermower bringup
This package contains different launch files and configuration to start a Hovermower robot.

# Usage
To start a real Hovermower use this command
```
ros2 launch hovermower_bringup teleop.launch.py
```

# Prerequisites
## ublox driver
You need to install ublox driver from here [KumarRobotics ublox](https://github.com/KumarRobotics/ublox). Beware to use the ros2 branch.
This package also requires nmea-msgs package, which you may also need to install manually for succesful build.

Please don't install the package by 
```
sudo apt install ros-$ROS_DISTRO-ublox*
```
this may install the wrong version. I tried it with this first but I noted that this will not subscribe to /rtcm topic, which is needed to use corretion data of your base station or public ntrip server.

## ntrip client
To send RTCM correction data to ublox driver (to /rtcm topic), you need a ntrip client, which connects to your ntrip serve r(i.e. your own base station). To achieve this, I use this ntrip client, which sends data as ros topic [ntrip client](https://github.com/LORD-MicroStrain/ntrip_client/).

This solution has been provided by [olvdhrm](https://github.com/olvdhrm/RTK_GPS_NTRIP). But he had to add a message translation as both packages uses different message types. The ntrip-client repository provides a branch named feature/ros2_prefer_rtcm_msgs, which seems to fix this. At time of this writing (Janary 2024), it was necessary to use the feature branch.

