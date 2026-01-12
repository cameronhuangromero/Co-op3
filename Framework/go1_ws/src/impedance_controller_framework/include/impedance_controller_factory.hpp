#ifndef IMPEDANCE_CONTROLLER_FACTORY
#define IMPEDANCE_CONTROLLER_FACTORY

#include <memory>
#include <string>
#include <unordered_map>
#include <functional>
#include "impedance_controller.hpp"

/*
    @brief Factory class for creating impedance controllers by name

    Provides a registry-based factory pattern to dynamically create impedance controllers
    Allows for easy addition of new controller types without modifying factory code
*/

class ImpedanceControllerFactory
{
public:
    using CreatorFunc = std::function<std::shared_ptr<ImpedanceController>()>;

    static void registerController(const std::string& name, CreatorFunc creator);

    static std::shared_ptr<ImpedanceController> create(const std::string& name);

private:
    static std::unordered_map<std::string, CreatorFunc>& getRegistry();
};

#endif