#include "dummy_impedance_controller.hpp"
#include "impedance_controller_factory.hpp"

/*
    @brief Registers DummyImpedanceController with the factory on load

    Author: Cameron Romero
    Date: 1/12/26
*/

namespace
{
    const bool dummy_registered = []{}
    {
        ImpedanceControllerFactory::registerController("DummyController", []()
        {
            return std::make_shared<DummyImpedanceController>();
        });
        return true;
    }();
}