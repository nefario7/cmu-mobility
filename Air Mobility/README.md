## General
16-665 Air Mobility Assignment
Last Updated: 10 October 2019
Shubham Garg (ssgarg) and Hannah Lyness (hlyness)

## Structure
* main.m: central function that calls all other functions. Takes in question number. Make sure TAs can run your code by entering main(2), for example.
   * lookup_waypoints.m: write this file to store and retreive waypoints for each question
   * trajectory_planner.m: write this function to turn waypoints into a time-discritized trajectory
   * attitude_planner.m: write this function to plan attitude
   * position_controller.m: write this function to find desired acceleration
   * attitude_controller.m: write this funciton to find desired moment
   * motor_model.m: write this funciton to turn desired attributes into motor RPMS
   * dynamics.m: write this function to get the change in state given desired state
   * plot_quadrotor_errors.m: file given to you to plot desired, actual, and error values

## Definitions
* state: quadrotor pose 
   * Size: 16x1
   * 1:3: position
   * 4:6: linear velocity
   * 7:9: orientation
   * 10:12: angular velocity
   * 13:16: motor speeds
* trajectory_matrix: trajectory discritized by time step
   * Size: 15 x n where n is the number of time steps
   * 1:3: position
   * 4:6: linear velocity
   * 7:9: orientation
   * 10:12: angular velocity
   * 13:15: linear accelerations
* actual_state_matrix: actual state discritized by time step
   * Size: 15 x n where n is the number of time steps
   * 1:3: position
   * 4:6: linear velocity
   * 7:9: orientation
   * 10:12: angular velocity
   * 13:15: linear accelerations
* actual_desired_state_matrix: desired state discritized by time step (not just the trajectory, this is what the controllers plan for)
   * Size: 15 x n where n is the number of time steps
   * 1:3: position
   * 4:6: linear velocity
   * 7:9: orientation
   * 10:12: angular velocity
   * 13:15: linear accelerations
* current_state: current struct, used to make feeding current state into control functions easier
   * Size: struct with 5 elements: pos, vel, rot, omega, rpm
* desired_state: desired struct, used to make feeding current state into control functions easier
   * Size: struct with 5 elements: pos, vel, rot, omega, acc
