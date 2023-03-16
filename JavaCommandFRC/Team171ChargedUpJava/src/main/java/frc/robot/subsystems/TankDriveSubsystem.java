// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class TankDriveSubsystem extends SubsystemBase {

  CANSparkMax leftLeadMotor;
  CANSparkMax rightLeadMotor;
  CANSparkMax leftFollowMotor;
  CANSparkMax leftFollowMotor2;
  CANSparkMax rightFollowMotor;
  CANSparkMax rightFollowMotor2;
  DifferentialDrive robotDrive;

  /** Creates a new TankDriveSystem. */
  public TankDriveSubsystem() {
    leftLeadMotor = new CANSparkMax (OperatorConstants.leftLeadDeviceID, MotorType.kBrushless);
    rightLeadMotor = new CANSparkMax(OperatorConstants.rightLeadDeviceID, MotorType.kBrushless);
    leftFollowMotor2 = new CANSparkMax(OperatorConstants.leftFollowDeviceID2, MotorType.kBrushless);
    rightFollowMotor = new CANSparkMax(OperatorConstants.rightFollowDeviceID, MotorType.kBrushless);
    rightFollowMotor2 = new CANSparkMax(OperatorConstants.rightFollowDeviceID2, MotorType.kBrushless);
    leftFollowMotor = new CANSparkMax(OperatorConstants.leftFollowDeviceID, MotorType.kBrushless);

    leftLeadMotor.restoreFactoryDefaults();
    leftFollowMotor.restoreFactoryDefaults();
    leftFollowMotor2.restoreFactoryDefaults();
    rightLeadMotor.restoreFactoryDefaults();
    rightFollowMotor.restoreFactoryDefaults();
    rightFollowMotor2.restoreFactoryDefaults();

    leftLeadMotor.setInverted(true);
    leftFollowMotor.setInverted(true);
    leftFollowMotor2.setInverted(true);

    leftLeadMotor.setSmartCurrentLimit(OperatorConstants.driverMotorsCurrentLimit);
    leftFollowMotor.setSmartCurrentLimit(OperatorConstants.driverMotorsCurrentLimit);
    leftFollowMotor2.setSmartCurrentLimit(OperatorConstants.driverMotorsCurrentLimit);
    rightLeadMotor.setSmartCurrentLimit(OperatorConstants.driverMotorsCurrentLimit);
    rightFollowMotor.setSmartCurrentLimit(OperatorConstants.driverMotorsCurrentLimit);
    rightFollowMotor2.setSmartCurrentLimit(OperatorConstants.driverMotorsCurrentLimit);

    leftFollowMotor.follow(leftLeadMotor);
    leftFollowMotor2.follow(leftLeadMotor);
    rightFollowMotor.follow(rightLeadMotor);
    rightFollowMotor2.follow(rightLeadMotor);

    robotDrive = new DifferentialDrive(leftLeadMotor, rightLeadMotor);
  }

  public void arcadeDrive(double forward, double rotation){
    robotDrive.arcadeDrive(forward, rotation);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
