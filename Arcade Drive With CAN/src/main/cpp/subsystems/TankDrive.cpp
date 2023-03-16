#include <frc/drive/DifferentialDrive.h>
#include "rev/CANSparkMax.h"
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

class TankDrive : public frc2::SubsystemBase
{
    // Constants
    const int currentLimit = 50;

    // make the motors with their CAN sparks and ports
    static const int leftLeadDeviceID = 6, leftFollowDeviceID = 5, leftFollowDeviceID2= 4, rightLeadDeviceID = 9, rightFollowDeviceID = 8, rightFollowDeviceID2 = 7;
    rev::CANSparkMax m_leftLeadMotor{leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_rightLeadMotor{rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_leftFollowMotor{leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_leftFollowMotor2{leftFollowDeviceID2, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_rightFollowMotor{rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_rightFollowMotor2{rightFollowDeviceID2, rev::CANSparkMax::MotorType::kBrushless};

    // make the differential drive with the two lead motors
    frc::DifferentialDrive m_robotDrive{m_leftLeadMotor, m_rightLeadMotor};

    public:
    
    // Default constructor
    TankDrive(){}

    void initialize()
    {
        // reset all the motors 
        m_leftLeadMotor.RestoreFactoryDefaults();
        m_rightLeadMotor.RestoreFactoryDefaults();

        m_leftFollowMotor.RestoreFactoryDefaults();
        m_rightFollowMotor.RestoreFactoryDefaults();
        m_leftFollowMotor2.RestoreFactoryDefaults();
        m_rightFollowMotor2.RestoreFactoryDefaults();

        // set the left side as inverted so the robot goes straight
        m_leftLeadMotor.SetInverted(1);
        m_leftFollowMotor2.SetInverted(1);
        m_leftFollowMotor.SetInverted(1);

        // have two motors on each side follow the lead motor for their sides
        m_leftFollowMotor.Follow(m_leftLeadMotor);
        m_rightFollowMotor.Follow(m_rightLeadMotor);
        m_leftFollowMotor2.Follow(m_leftLeadMotor);
        m_rightFollowMotor2.Follow(m_rightLeadMotor);

        // set to 50 amps
        m_leftLeadMotor.SetSmartCurrentLimit(currentLimit);
        m_leftFollowMotor.SetSmartCurrentLimit(currentLimit);
        m_leftFollowMotor2.SetSmartCurrentLimit(currentLimit);

        m_rightLeadMotor.SetSmartCurrentLimit(currentLimit);
        m_rightFollowMotor.SetSmartCurrentLimit(currentLimit);
        m_rightFollowMotor2.SetSmartCurrentLimit(currentLimit);
    }
    

    // maybe limit to a max speed that way we can try to conserve power?
    void tankDrive(double leftJoystickY, double rightJoystickX)
    {
        // drive the robot using arcade drive and the Xbox joystick values
        // left joystick is forward/backward and right joystick is right/left turning
        m_robotDrive.ArcadeDrive(-leftJoystickY, -rightJoystickX);
    }
};