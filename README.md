# MSRR-System
High-Level System and Mediator for Modular Self-Reconfiguration Robot (MSRR)

*by Cheng Jun-Yan*

**Prerequisite:** opencv2, apriltags3, ros, parrot-olympe, parrot-sphinx(for simulation)

## I. How to use:

1.Run BootServer.py (args: 1 for listen ros. none for server., also need run the roscore)

2.Run DroneScript.py (args: 1 for init, none for keep streaming.)

3.Run MSRRScript.py (args: 1 for reset stage, none for continue.)

## II. How to Install Apriltags:

1.Download apriltags from https://github.com/AprilRobotics/apriltag (Or directly use apriltags.zip in /Coordinator)

2.Rename the folder 'apriltags', put it in Coordinator/

3.Enter the folder, then build it:

cmake .

make

## III. How to Activate drone env:

source ~/code/parrot-groundsdk/./products/olympe/linux/env/shell

## IV. How to launch drone simulation:

sudo systemctl start firmwared

sphinx /opt/parrot-sphinx/usr/share/sphinx/drones/anafi4k.drone::stolen_interface=

## V. How to modify

1.Write the MSRRScript.py to plan a high level task

2.Write the DroneScript.py to define the behavior of Drone. Or use Coordinator/drone.py to directly control it.

3.Modify configs.py and also parameters for planner, coordinator and actuator.

## Sample

![image](https://github.com/chengjunyan1/MSRR-System/raw/master/sample.png)
