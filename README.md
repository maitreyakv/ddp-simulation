# Differential Dynamic Programming Simulation Using MATLAB

This code is a MATLAB implementation of the **Differential Dynamic Programming** (DDP) algorithm, implemented in **MATLAB**, and utilized to control a simulated inverted pendulum and a simulated cart-pole system. This project is for the AE4803-ROB class, *Robotic Systems and Autonomy*, taught by Prof. Evangelos Theodorou at the Georgia Institute of Technology during the spring 2020 semester. 

The DDP algorithm is a powerful method for optimal control, that can be used to control many different types of systems from a variety of application domains. It is for this reason that the approach to the implementation in this code was to create a DDP routine that can be used for any system without the need to change any of the actual algorithm. The two things that chage between problems that could utilize DDP are:

1. Dynamics of the system
2. Cost functions used in the optimization

The approach to ensuring that this code can effectively used for many different problems efficiently is abstracting these two elements and create abstract classes for both the dynamics and the cost functions. These classes are defined in [`Dynamics.m`](src/Dynamics.m) and [`Cost.m`](src/Cost.m) as `Dynamics` and `Cost`, respectively. This way, anyone who wants to apply the DDP algorithm to their problem of interest need only write:

1. A class that inherits `Dynamics` and implements its abstract methods (e.g. `CartPoleDynamics` in [`CartPoleDynamics.m`](src/CartPoleDynamics.m))
2. A class that inherits `Cost` and implements its abstract methods (e.g. `QuadraticCost` in [`QuadraticCost.m`](src/QuadraticCost.m))

The code that performs the actual DDP procedure to generate an optimal control sequence is implemented as a routine in [`ddp.m`](src/ddp.m), and does not need to be changed at all to use DDP for a specific problem.

<p align="center"><img src="cart-pole.gif" width="400" ></p>
