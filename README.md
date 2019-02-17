# FRC 2019
Control code for robot of team Galaxia 5987 in memory of David Zohar, _Genesis_, playing in the 2019 _FIRST_ Robotics Competition season, Destination: Deep Space. _Genesis_ will compete at _FIRST_ Israel District Events [#1](https://www.thebluealliance.com/event/2019isde1) and [#4](https://www.thebluealliance.com/event/2019isde4) and in [_FIRST_ Championship (Detroit)](https://www.thebluealliance.com/event/2019cmpmi).

This repository holds the control code of _Genesis_' subsystems written in Java. The vision recognition code is stored [here](https://github.com/Repulser/FRCVision).

## Main Features
### Subsystems
- **Drivetrain**: Six-whell WCD with a Pure Pursuit alogrithm to follow paths.
- **Elevator**: Two-staged Elevator controlled by a motion profile using the Talon SRX's "Motion Magic" control mode. The Elevator has preset heights: Rocket Levels 1/2/3 (for Hatch Panels and for Cargo), Cargo Ship, Loading Station and ground.
- **Hatch Panels**: Pneumatic system holding the Hatch Panels.
- **Cargo Gripper**: Wristed Gripper holding the Cargo. The Gripper has preset angles controlled by a motion profile to collect Cargo from the ground and score it in the Cargo Ship or the Rocket.
- **Climbing**: Four-legged mechanism raising _Genesis_ to Habitat Level 2 or 3. Uses rotation matrices to stay leveled and stable. 

### Features
WIP: Auto adjustment to the Cargo Ship, Rocket and Loading Station. _Genesis_ uses two Intel RealSense D435 depth cameras to calculate distances and angles from the vision targets. _Genesis_ uses edge detection to find Cargo and a neural network to find Hatch Panels.
If the Operator chooses to score in the Cargo Ship, _Genesis_ chooses by itself which Bay in the Cargo Ship to go to. 
