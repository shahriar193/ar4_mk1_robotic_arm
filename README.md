# AR4-MK1-Robotic-Arm


# Arduino Code
The arduino file is flashed on the robot controller teensy 4.1. It receives and sends serial commands. Using that API is created. 

# Python API
Using the API the robot can be controlled. At first the robot will go to the limit switches and initialize itself. Then 2 joints can be controlled giving the joint angle value. Using forward Kinematics the end-effector position can be found.

A specific position in 3D can be given with respect to the robot base. Robot will go to the closest position to that point using inverse kinematics.
