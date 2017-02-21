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
$ roslaunch gmodrotor_control test_waypoints.launch
~~~~
Run the simulator
~~~~
$ roscd gmodrotor_sim/morse
$ morse run gmodrotor_ros.py
~~~~


Test
