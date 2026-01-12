#include <gtest/gtest.h>
#include "dummy_impedance_controller.hpp"
#include "impedance_controller_factory.hpp"
#include "impedance_controller_manager.hpp"

/*
    @brief Unit tests for ImpedanceControllerFactory and ImpedanceControllerManager

    Author: Cameron Romero
    Date: 1/12/26
*/

namespace
{
    const bool dummy_registered = []()
    {
        ImpedanceControllerFactory::registerController("DummyController", []()
    {
        return std::make_shared<DummyImpedanceController>();
    });
    return true;
    }();
}

// Tests for ImpedanceControllerFactory
class FactoryTest : public ::testing::Test {};

TEST_F(FactoryTest, CreateDummyController)
{
    auto controller = ImpedanceControllerFactory::create("DummyController");
    ASSERT_NE(controller, nullptr);
    EXPECT_EQ(controller->name(), "DummyController");
}

TEST_F(FactoryTest, CreateNonexistantController)
{
    auto controller = ImpedanceControllerFactory::create("Nonexistant");
    EXPECT_EQ(controller, nullptr);
}

// Tests for ImpedanceControllerManager
class ManagerTest : public ::testing::Test
{  
public:
    ImpedanceControllerManager manager;
};

TEST_F(ManagerTest, SetActiveControllerSuccess)
{
    bool result = manager.setActiveController("DummyController");
    EXPECT_TRUE(result);
    EXPECT_EQ(manager.getActiveControllerName(), "DummyController");
}

TEST_F(ManagerTest, SetActiveControllerFailure)
{
    bool result = manager.setActiveController("Nonexistant");
    EXPECT_FALSE(result);
    EXPECT_EQ(manager.getActiveControllerName(), "");
}

TEST_F(ManagerTest, ComputeTorqueNoActiveControllerReturnsZero)
{
    ImpedanceControllerInput input;
    auto torque = manager.computeTorque(input);
    for (float t : torque)
    {
        EXPECT_FLOAT_EQ(t, 0.0f);
    }
}

TEST_F(ManagerTest, ComputeTorqueWithActiveController)
{
    manager.setActiveController("DummyController");
    ImpedanceControllerInput input;
    auto torque = manager.computeTorque(input);
    for (float t : torque)
    {
        EXPECT_NE(t, 0.0f);
    }
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}