#pragma once
#include "../subsystems/TankDrive.cpp"
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <utility>

class DriveCommand : public frc2::CommandHelper<frc2::CommandBase, DriveCommand>
{
    private:
    TankDrive* m_drive;
    std::function<double()> m_forward;
    std::function<double()> m_rotation;

    public:
    explicit DriveCommand(TankDrive* subsystem, std::function<double()> forward, std::function<double()> rotation) : m_drive{subsystem}, m_forward{std::move(forward)}, m_rotation{std::move(rotation)}
    {
        AddRequirements({subsystem});
    }

    void Execute() override
    {
        m_drive->tankDrive(m_forward(), m_rotation());
    }
};