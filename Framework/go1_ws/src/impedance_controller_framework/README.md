# Giszter Framework Design (as of 1/12/26)

## Impedance Controllers
- Each controller implements a commmon interface
    - Holds the number of joints (12 for the Unitree Go1)
    - Defines/exposes computeTorques function
    - Identification for each controller (string name or int ID)
- Controller can be:
    - Joint space controllers (using joint positions/velocities)
    - Cartesian task space controllers (using Cartesian poses/velocities)
    - Composite controllers that combine other controllers and balance the computed torques they output

## Impedance Controller Input Struct
- Holds optional joint and task space data
- Controllers use whichever part of the data is relevant

## Impedance Controller Factory (factory pattern)
- Central place to control instantiation of controllers (by name or type)

## Impedance Controller Manager
- Manages all impedance controllers
- Holds the factory and Input Handler
- Receives ROS2 inputs
- Ouputs produced torques
