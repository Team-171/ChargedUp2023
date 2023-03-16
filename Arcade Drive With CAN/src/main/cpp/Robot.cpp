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
#include "subsystems/TankDrive.cpp"
#include "subsystems/Intake.cpp"
#include <frc/smartdashboard/SmartDashboard.h>
#pragma once
#include <frc/DoubleSolenoid.h>
#include <frc/PneumaticHub.h>
#include <frc/TimedRobot.h>
#define PH_CAN_ID 1
//#include <pathplanner/lib/PathPlannerTrajectory.h>
//#include <pathplanner/lib/PathPlanner.h>
//#include <pathplanner/lib/PathPoint.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/PrintCommand.h>
//#include <pathplanner/lib/auto/BaseAutoBuilder.h>
#include <PID.h>

//using namespace pathplanner;

const double wristVelocity = .4;
const double intakeRollerVelocity = .4;

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

  TankDrive driveTrain;
  Intake intake;
// frc::Compressor m_ph{1, frc::PneumaticsModuleType::REVPH};
  
  rev::CANSparkMax m_arm0{2, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_arm1{10, rev::CANSparkMax::MotorType::kBrushless};  

  frc::DoubleSolenoid m_solenoidDouble{1, frc::PneumaticsModuleType::REVPH, 0,1};
  
  PID pidController{0.5, 1.0, 1.0};

  /**
   * In RobotInit() below, we will configure m_leftFollowMotor and m_rightFollowMotor to follow 
   * m_leftLeadMotor and m_rightLeadMotor, respectively. 
   * 
   * Because of this, we only need to pass the lead motors to m_robotDrive. Whatever commands are 
   * sent to them will automatically be copied by the follower motors
   */

 frc::XboxController m_driver{0}; //defines driver controller type and port
 frc::XboxController m_operator{1}; //Defines operator controller and port



private:
  frc::PneumaticHub m_ph{PH_CAN_ID};  //Defines can address of pneumatic hub

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
    intake.initialize();
    m_arm0.RestoreFactoryDefaults();
    m_arm1.RestoreFactoryDefaults();


    m_solenoidDouble.Set(frc::DoubleSolenoid::Value::kReverse); //Default shifters to low gear
    /**
     * In CAN mode, one SPARK MAX can be configured to follow another. This is done by calling
     * the Follow() method on the SPARK MAX you want to configure as a follower, and by passing
     * as a parameter the SPARK MAX you want to configure as a leader.
     * 
     * This is shown in the example below, where one motor on each side of our drive train is
     * configured to follow a lead motor.
     */

    pidController.setSatLimit(1.0);
    pidController.setMax(1);
    pidController.setFeedForward(0,0);
  }

  void AutonomousPeriodic(){
    
  }

  void TeleopPeriodic() {

    // Get values from Shuffleboard
    double minPressure = 50;
    double maxPressure = 110;

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
                                units::pounds_per_square_inch_t{maxPressure}); // enable and disable compressor when between these values
  


   
    // Drive with arcade style

	  driveTrain.tankDrive(-m_driver.GetLeftY(), -m_driver.GetRightX());
    if (m_driver.GetAButtonPressed()) { //ball shifter button
      m_solenoidDouble.Toggle();
    }

    // Intake config for wrist
    if (m_operator.GetRightBumper()) 
    {
      intake.setWristMotor(wristVelocity);
    } 
    else if (m_operator.GetLeftBumper())
    {
      intake.setWristMotor(-wristVelocity);
    }
    else
    {
      intake.setWristMotor(0);
    }

    // Intake config
    // Y picks up cones, B picks up cubes (change?)
    if (m_operator.GetYButton())
      intake.setIntakeRollersMotor(intakeRollerVelocity);
    else if (m_operator.GetBButton())
      intake.setIntakeRollersMotor(intakeRollerVelocity);
    else
      intake.setIntakeRollersMotor(0);
    
    // arm movement
    // is right joystick up and down
    m_arm0.Set(m_operator.GetRightY()*0.5);
    m_arm1.Set(-m_operator.GetRightY()*0.5);
  }
};
 

   




#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
