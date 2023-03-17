#pragma once

#include "subsystems/TankDrive.cpp"
#include <frc/XboxController.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>


class RobotContainer
{
    private:
    TankDrive driveTrain;
    frc2::CommandXboxController m_driver{0}; //defines driver controller type and port
    frc2::CommandXboxController m_operator{1}; //Defines operator controller and port


    public:
    RobotContainer()
    {
        driveTrain.initialize();
        //configure bindings to commands
        ConfigureBindings();
    }

    void ConfigureBindings()
    {
        // Set up default drive command
        driveTrain.SetDefaultCommand(frc2::cmd::Run(
        [this] {
            driveTrain.tankDrive(-m_driver.GetLeftY(),
                                -m_driver.GetRightX());
        },
        {&driveTrain}));
    }

    frc2::CommandPtr GetAutonomousCommand(){}

};