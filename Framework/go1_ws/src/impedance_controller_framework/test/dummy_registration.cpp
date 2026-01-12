/*
    @brief Registers DummyImpedanceController with the factory on load
*/

#include "dummy_impedance_controller.hpp"
#include "impedance_controller_factory.hpp"

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