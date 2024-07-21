# Overview
The assignment involves solving various tasks related to the modelling and control of a 7 Degree of Freedom (DOF) manipulator. The tasks require the use of MATLAB to implement solutions, simulate the robot motion, and achieve specific goals. The goal frame moves with the linear velocity with respect to the base frame  [-0.01, 0.02, 0]'. 

# Tasks and Solutions
## Define Model Matrices
  a. Implemented the **BuildTree()** function to define all the model matrices.
  b. Defined the transformation and rotation matrices, as well as vectors defining the frames of the manipulator.
## Direct Geometry Calculation 
  a. Created the **DirectGeometry()** function to calculate the transformation matrix of link j with respect to link j-1 when joint j rotates or translates.
  b Developed **GetDirectGeometry()** function to return all model matrices for a given joint configuration q*. 

## Transformation Matrices with Respect to Base 
  a. Implemented **GetTransformationWrtBase()** function to compute the transformation matrices between each link and the base.
  
  b. Tested the function by calculating the transformation of the end effector with respect to the base (eTb).

## End-Effector Jacobian Matrix 
  a. Computed the end-effector Jacobian matrix for the given joint configuration q∗ using the **GetJacobian()** function.

## Robot Tool Frame Control
  a. Implemented control logic to move the robot tool frame to a specified goal frame.
  b. Considered initial configuration (q0), goal position and goal frame roll-pitch-yaw parameters. 

## Desired Joint Velocities
  a. Calculated the desired joint velocities for the manipulator.

## Robot Motion Simulation
  a. Simulated the robot motion using the **KinematicSimulation()** function, taking into account the joint limits.


# Files

a. main.m: Main script to compile and run the assignment.
b. /include: Folder containing the required functions:
      BuildTree.m: Defines the model matrices.
      DirectGeometry.m: Calculates transformation matrices.
      GetDirectGeometry.m: Returns model matrices for a given joint configuration.
      GetTransformationWrtBase.m: Computes transformation matrices with respect to the base.
      GetJacobian.m: Computes the Jacobian matrix.
      KinematicSimulation.m: Simulates the robot motion.
