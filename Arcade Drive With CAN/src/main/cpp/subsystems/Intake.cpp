#pragma once
#include "rev/CANSparkMax.h"

class Intake
{
    private:
    rev::CANSparkMax m_wrist{11, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_intakeRollers{12, rev::CANSparkMax::MotorType::kBrushless};

    // Constants
    const double wristVelocity = .4;
    const double intakeRollerVelocity = .4;

    public:

    Intake(){}

    void initialize()
    {
        m_wrist.RestoreFactoryDefaults();
        m_intakeRollers.RestoreFactoryDefaults();
    }

    void setWristMotor(double wristVelocity)
    {
        m_wrist.Set(wristVelocity);
    }

    void setIntakeRollersMotor(double intakeRollerVelocity)
    {
        m_intakeRollers.Set(intakeRollerVelocity);
    }

};