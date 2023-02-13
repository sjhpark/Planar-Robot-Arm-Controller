# Planar-Robot-Arm-Controller
3R Planar Manipulator (Robot Arm) Controller

---

## About the Controller
Created a controller for the movement of a 3-jointed planar arm end-effector using Python.

Implemented mouse interaction using the tkinter Python module so that users can click anywhere in the grid in the controller window to move the 3R planar maniuplator.

Divided each movement of the manipulator into 21 separate stages and optimized the joint angles at each stage by having the minimum max angle changle between adjacent stages. Considered both elbow-up and elbow-down cases, and accounted for 256 end-effector orientations for each stage.

## Dependencies
- Numpy
- Tkinter

## Calculation
- Inverse Kinematics for calculating each joint angle, given the desired (x,y) position of the end effector.
- Forward Kinematics for moving the each arm segment and the end effector to the corresponding position when given joint angles.

## Demonstration
![demo](https://user-images.githubusercontent.com/83327791/218372244-b3af1dfc-2d22-4e2e-a0d2-04e6f7f7649b.gif)
