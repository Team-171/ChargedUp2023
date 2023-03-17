// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.RollerConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class IntakeRollersSubsystem extends SubsystemBase {

  CANSparkMax rollersMotor;

  /** Creates a new ExampleSubsystem. */
  public IntakeRollersSubsystem() {
    rollersMotor = new CANSparkMax(RollerConstants.rollerMotorID, MotorType.kBrushless);

    rollersMotor.restoreFactoryDefaults();
  }

  public void moveRoller(double speed){
      rollersMotor.set(speed);
  }

  public void stopIntakeRollers(){
    rollersMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
