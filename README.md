# Linear Systems - SYS 411

![lyapunov-35-test20120303-02-1200x800 (2)](https://user-images.githubusercontent.com/83914255/215894627-18f49387-3279-44ca-a66c-550eb7b4b182.jpg)

## Aircraft Dynamics Control 

>   This project contains 4 MATLAB scripts for analyzing the longitudinal motion of an aircraft. The aircraft can be described by a 4th order nonlinear system, linearized around its equilibrium point.

PART A:
>   Finding the eigenvalues of the system, simulating the time response and determining stability, observability and controllability.

PART B:
>   Designing a Full-State-Feedback controller using pole placement and the Linear Quadratic Regulator method for optimal control.
Designing a Luenberger state observer with pole placement.
Combining the above to design a full state feedback controller based on observer.

>   <a href = "https://github.com/atzel-ov/LinearSystemsProject/blob/main/linear_systems_project.m">linear_system_project.m</a> : 
Contains the main script for getting the simulations.

>   <a href = "https://github.com/atzel-ov/LinearSystemsProject/blob/main/ssmodel1.m">ssmodel1.m</a> : 
Contains the dynamics of the state of the system, open and closed loop.

>   <a href = "https://github.com/atzel-ov/LinearSystemsProject/blob/main/ssmodel2.m">ssmodel2.m</a> : 
Contains the dynamics of the estimation error.

>   <a href = "https://github.com/atzel-ov/LinearSystemsProject/blob/main/ssmodel3.m">ssmodel3.m</a> : 
Contains both the dynamics of the longitudinal motion and the dynamics of the state estimator.
