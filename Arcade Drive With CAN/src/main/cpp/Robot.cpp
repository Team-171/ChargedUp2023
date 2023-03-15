/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/XboxController.h>
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include "rev/CANSparkMax.h"
#include "frc/Solenoid.h"
#include "frc/Compressor.h"
#include "subsystems/ArcadeDrive.cpp"
#include <frc/smartdashboard/SmartDashboard.h>
#pragma once
#include <frc/DoubleSolenoid.h>
#include <frc/PneumaticHub.h>
#include <frc/TimedRobot.h>
#define PH_CAN_ID 1


class Robot : public frc::TimedRobot {
  /**
   * SPARK MAX controllers are intialized over CAN by constructing a CANSparkMax object
   * 
   * The CAN ID, which can be configured using the SPARK MAX Client, is passed as the
   * first parameter
   * 
   * The motor type is passed as the second parameter. Motor type can either be:
   *  rev::CANSparkMax::MotorType::kBrushless
   *  rev::CANSparkMax::MotorType::kBrushed
   * 
   * The example below initializes four brushless motors with CAN IDs 1, 2, 3 and 4. Change
   * these parameters to match your setup
   */
  ArcadeDrive driveTrain;
  // one motor for wrist one motor for rollers
  rev::CANSparkMax m_intake1{2, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_intake2{10, rev::CANSparkMax::MotorType::kBrushless};
// frc::Compressor m_ph{1, frc::PneumaticsModuleType::REVPH};
  frc::DoubleSolenoid m_solenoidDouble{1, frc::PneumaticsModuleType::REVPH, 0,1};
  


  /**
   * In RobotInit() below, we will configure m_leftFollowMotor and m_rightFollowMotor to follow 
   * m_leftLeadMotor and m_rightLeadMotor, respectively. 
   * 
   * Because of this, we only need to pass the lead motors to m_robotDrive. Whatever commands are 
   * sent to them will automatically be copied by the follower motors
   */

  frc::XboxController m_driver{0};
  


private:
  frc::PneumaticHub m_ph{PH_CAN_ID};

 public:
  void RobotInit() {
      // Add buttons to enable/disable the compressor
  frc::SmartDashboard::SetDefaultBoolean("Enable Compressor Analog", false);
  frc::SmartDashboard::SetDefaultBoolean("Disable Compressor", false);

    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    driveTrain.initialize();
    m_intake1.RestoreFactoryDefaults();
    m_intake2.RestoreFactoryDefaults();
    //m_intake2.SetInverted(0);
    //m_intake1.SetInverted(0);
    m_solenoidDouble.Set(frc::DoubleSolenoid::Value::kReverse);
    /**
     * In CAN mode, one SPARK MAX can be configured to follow another. This is done by calling
     * the Follow() method on the SPARK MAX you want to configure as a follower, and by passing
     * as a parameter the SPARK MAX you want to configure as a leader.
     * 
     * This is shown in the example below, where one motor on each side of our drive train is
     * configured to follow a lead motor.
     */
    //m_intake2.Follow(m_intake1);
  }

  void TeleopPeriodic() {

    // Get values from Shuffleboard
    double minPressure =75;
    double maxPressure =110;

    /**
     * Enable the compressor with analog pressure sensor control.
     *
     * This uses hysteresis between a minimum and maximum pressure value,
     * the compressor will run when the sensor reads below the minimum pressure
     * value, and the compressor will shut off once it reaches the maximum.
     *
     *
     */
    m_ph.EnableCompressorAnalog(units::pounds_per_square_inch_t{minPressure},
                                units::pounds_per_square_inch_t{maxPressure});
  


   
    // Drive with arcade style
    
    driveTrain.arcadeDrive(-m_driver.GetLeftY(), -m_driver.GetRightX());

    if (m_driver.GetAButtonPressed()) {
      m_solenoidDouble.Toggle();
    }

    if (m_driver.GetRightBumper())
    {
      // which one is wrist and which is motor??
      m_intake1.Set(.4);
      m_intake2.Set(-0.4);
    } 
    else if (m_driver.GetLeftBumper())
    {
    m_intake1.Set(-.4);
    m_intake2.Set(.4);
    }
    else 
    {
    m_intake1.Set(0);
      m_intake2.Set(0);
    }

  }
};
 

  




#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
