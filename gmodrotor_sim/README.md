# Modrotor-ROS for Simulation

## Installation
1. Requirements
  - MORSE beta verison (install by sources)
  - Ros packages:
  - multi_mav_manager
  - quadrotor_control

2. Configure
Change the number of robots in the following files
~~~~
modrotor_sim/morse/params.py change N
modrotor_sim/launch/morse_multimav.launch
multi_mav_manager/config/crazyflie/multi_mav_manager.yaml
~~~~

## Execution

Run in different terminals
~~~~
$ roscore
~~~~
Run the simulator
~~~~
$ roscd modrotor_sim/morse
$ morse run modrotor_ros.py
~~~~

Run the attitude controller
~~~~
roslaunch modrotor_sim morse_multimav.launch
~~~~
Take off
~~~~
roscd multi_mav_manager/scripts/ && source aliases.sh
motors on
takeoff
~~~~
Example of sending waypoints for three robots:
~~~~
rosservice call /multi_mav_services/goFormRawPos "goals: [{x: 0.0, z: 1.0},{x: 1.0, z: 1.0}, {x: 2.0, z: 1.0}]"
~~~~
