# Giszter Framework Design'

Author: Cameron Romero
Date: 1/12/26

## Impedance Controllers
- Each controller implements a commmon interface
    - Holds the number of joints (12 for the Unitree Go1)
    - Defines/exposes computeTorques function
    - Identification for each controller (string name or int ID)
    - Controllers do not manage stacking or priority
- Controller can be:
    - Joint space controllers (using joint positions/velocities)
    - Cartesian task space controllers (using Cartesian poses/velocities)

## Impedance Controller Input Struct
- Holds optional joint and task space data
- Controllers use only whichever part of the data is relevant to them
- Shared input object passed to controllers from the manager or stack

## Impedance Controller Factory (factory pattern)
- Central place to control instantiation of controllers (by name or type)
- Owns controller registration
- Creates controllers by name (or type)
- Enables runtime selection or swapping of controllers
- Controllers are created as independent instances (not singletons)
    - Could change in the future to save memory?

## Impedance Task Struct
- Represents the output of a single impedance objective
- Produced by one controller and consumed by the stack
- Encapsulates control effort and task structure
- Typical contents:
    - Joint torque vector
    - Possibly task metadata?

## Impedance Stack Struct
- Represents the ordered collection of impedance tasks
- Responsible for task composition, not aggregation
- Defines how multiple impedance tasks are combined:
    - Additive
    - Hierarchal/priority
    - Nullspace projections
- Produces a single joint torque command

## Impedance Controller Manager
- Manages all created impedance controllers
- Select active stack (e.g. using multiple controllers or single controller)
- Receives ROS2 inputs (through nodes) to the stack
- Invokes active stack to compute final torque outputs
- Ouputs produced torques to ROS2 topics
