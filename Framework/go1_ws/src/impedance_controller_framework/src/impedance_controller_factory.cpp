#include "impedance_controller_factory.hpp"

/*
    @brief Implementation of the ImpedanceControllerFactory class

    Author: Cameron Romero
    Date: 1/12/26
*/

std::unordered_map<std::string, ImpedanceControllerFactory::CreatorFunc>& ImpedanceControllerFactory::getRegistry()
{
    static std::unordered_map<std::string, CreatorFunc> registry;
    return registry;
}

void ImpedanceControllerFactory::registerController(const std::string& name, CreatorFunc creator)
{
    getRegistry()[name] = creator;
}

std::shared_ptr<ImpedanceController> ImpedanceControllerFactory::create(const std::string& name)
{
    auto& registry = getRegistry();
    auto it = registry.find(name);
    if (it != registry.end())
    {
        return (it->second)();
    }
    return nullptr;
}