#include <iostream>
#include "dummy_impedance_controller.hpp"

/*
    @brief Simple test program for DummyImpedanceController

    Author: Cameron Romero
    Date: 1/12/26
*/

int main()
{
    DummyImpedanceController controller;

    ImpedanceControllerInput input;

    auto torque = controller.computeTorque(input);

    std::cout << "Torque input from " << controller.name() << ":\n";
    for (float t : torque)
    {
        std::cout << t << " ";
    }
    std::cout << std::endl;

    return 0;
}