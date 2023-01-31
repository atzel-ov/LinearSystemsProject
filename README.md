# Linear Systems - SYS 411

![lyapunov-35-test20120303-02-1200x800 (2)](https://user-images.githubusercontent.com/83914255/215894627-18f49387-3279-44ca-a66c-550eb7b4b182.jpg)

# Longitudinal dynamics of an aircraft 

This project contains 4 MATLAB scripts for analyzing a 4th order non-linear system, linearized about its equilibrium point.

PART A :
Finding eigenvalues, simulating the time response and determining stability, observability and controllability.

PART B:
Designing a Full-State-Feedback controller using pole placement and the Linear Quadratic Regulator method for optimization.
Designing a Luenberger state observer with pole placement.
Combining the above to design a full state feedback controller based on observer.

->linear_system_project.m : 
Contains the main script for getting the simulations.

->ssmodel1.m : 
Contains the time response of the state of the system for the different inputs we use and for the full-state-feedback controller.

->ssmodel2.m : 
Contains the dynamics of the estimation error.

->ssmodel3.m : 
Contains both the dynamics of the longitudinal motion and the dynamics of the state estimator.
