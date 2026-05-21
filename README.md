# Precision Positioning Using Adaptive Control

## Overview

PLEASE GIVE ADVICE/FEEDBACK!

This project simulates a precision positioning system using 3 actuators. The goal is to move a circular object to a desired position and orientation using electrical input signals. The system uses adaptive control to compensate for actuator uncertainty.

## System Description
The control loop works as follows:
1. Compute error between current position and target
2. Compute electrical input signal (u) for each actuator
3. Actuators convert input into force
4. Forces move the object
5. Adaptive control updates system gain (alpha)

## Model
The system follows:
u → actuator → force → motion

Where:
- u = electrical input signal
- actuator maps input to force (unknown efficiency)
- force is mapped to motion via a Jacobian matrix

## Features
- Input-based control (electrical signal → force → motion)
- Adaptive gain control (alpha)
- Stochastic actuator behavior (Gaussian variation)
- 3-degree-of-freedom system (x, y, θ)
- Real-time visualization

## Current Limitations
- No full friction or nonlinear dynamics yet
- Simplified actuator model (linear mapping)
- No real hardware integration

## Future Work
- Add nonlinear friction and actuator dynamics
- Learn actuator behavior from data (system identification)
- Explore alternative actuation mechanisms
- Implement machine learning-based control

## Goal
Move object to:
(x, y, θ) = (0, 0, 0) but theta does not matter
