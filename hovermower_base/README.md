# hovermower_base
hovermower_base package provides a Joystick interface as well as a safety controller.

## Joystick interface
The purpose of the node is to translate Joystick commands into service requests for Hovermower. There are three main tasks:
- enable / disable Hoverboard PCB by pressing "options" key
- toggle Mow motor py pressing "share" key
- toggle e_stop, an emergency stop by pressing the "Playstation" key

## safety controller
The safety controller is used to handle bump events. If the robot collids with an obstacle, it overrides cmd_vel topics, back up a bit and rotate away from obstacle.
