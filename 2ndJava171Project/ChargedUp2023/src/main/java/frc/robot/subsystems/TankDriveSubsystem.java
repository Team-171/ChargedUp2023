// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class TankDriveSubsystem extends SubsystemBase {

  CANSparkMax leftLeadMotor;
  CANSparkMax leftFollowMotor;
  CANSparkMax leftFollowMotor2;
  CANSparkMax rightLeadMotor;
  CANSparkMax rightFollowMotor;
  CANSparkMax rightFollowMotor2;

  DifferentialDrive roboDrive;

  /** Creates a new ExampleSubsystem. */
  public TankDriveSubsystem() {
    CANSparkMax leftLeadMotor = new CANSparkMax(DriveConstants.leftLeadDeviceID, MotorType.kBrushless);
    CANSparkMax leftFollowMotor = new CANSparkMax(DriveConstants.leftFollowDeviceID, MotorType.kBrushless);
    CANSparkMax leftFollowMotor2 = new CANSparkMax(DriveConstants.leftFollowDeviceID2, MotorType.kBrushless);
    CANSparkMax rightLeadMotor = new CANSparkMax(DriveConstants.rightLeadDeviceID, MotorType.kBrushless);
    CANSparkMax rightFollowMotor = new CANSparkMax(DriveConstants.rightFollowDeviceID, MotorType.kBrushless);
    CANSparkMax rightFollowMotor2 = new CANSparkMax(DriveConstants.rightFollowDeviceID2, MotorType.kBrushless);

    leftLeadMotor.restoreFactoryDefaults();
    leftFollowMotor.restoreFactoryDefaults();
    leftFollowMotor2.restoreFactoryDefaults();
    rightLeadMotor.restoreFactoryDefaults();
    rightFollowMotor.restoreFactoryDefaults();
    rightFollowMotor2.restoreFactoryDefaults();

    leftLeadMotor.setInverted(true);
    leftFollowMotor.setInverted(true);
    leftFollowMotor2.setInverted(true);

    leftLeadMotor.setSmartCurrentLimit(DriveConstants.driveMotorsCurrentLimit);
    leftFollowMotor.setSmartCurrentLimit(DriveConstants.driveMotorsCurrentLimit);
    leftFollowMotor2.setSmartCurrentLimit(DriveConstants.driveMotorsCurrentLimit);
    rightLeadMotor.setSmartCurrentLimit(DriveConstants.driveMotorsCurrentLimit);
    rightFollowMotor.setSmartCurrentLimit(DriveConstants.driveMotorsCurrentLimit);
    rightFollowMotor2.setSmartCurrentLimit(DriveConstants.driveMotorsCurrentLimit);

    leftFollowMotor.follow(leftLeadMotor);
    leftFollowMotor2.follow(leftLeadMotor);
    rightFollowMotor.follow(rightLeadMotor);
    rightFollowMotor2.follow(rightLeadMotor);

    roboDrive = new DifferentialDrive(leftLeadMotor, rightLeadMotor);

    SmartDashboard.putNumber("Left Drive Speed: ", leftLeadMotor.get());
    SmartDashboard.putNumber("Right Drive Speed: ", rightLeadMotor.get());
  }

  public void arcadeDrive(double forward, double rotation){
    roboDrive.arcadeDrive(forward, rotation);
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
