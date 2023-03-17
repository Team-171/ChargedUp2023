// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

import java.util.concurrent.CancellationException;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// probably can use a duty cycle encoder. Then it just gets the distance
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;


public class ArmSubsystem extends SubsystemBase {

  CANSparkMax armMotor;
  CANSparkMax armMotor2;

  // ****IF YOU NEED TO MAKE 0 BE AT A BETTER SPOT, USE reset() ONCE TO DO SO****
  DutyCycleEncoder absoluteEncoder;

  /** Creates a new ExampleSubsystem. */
  public ArmSubsystem() {
    armMotor = new CANSparkMax(ArmConstants.armMotorID, MotorType.kBrushless);
    armMotor2 = new CANSparkMax(ArmConstants.armMotorID2, MotorType.kBrushless);

    armMotor.restoreFactoryDefaults();
    armMotor2.restoreFactoryDefaults();

    armMotor.setInverted(true);

    // ****NEED TO PUT IN THE CORRECT PORT NUMBER IN THE CONSTANTS FILE****
    absoluteEncoder = new DutyCycleEncoder(EncoderConstants.encoderPort);
    absoluteEncoder.setDistancePerRotation(EncoderConstants.distancePerRotation);
  }

  public void moveArm(double speed){
      // ****NEED TO CALIBRATE THE MIN AND MAX DISTANCE****
      if (absoluteEncoder.getDistance() > EncoderConstants.minDistance && absoluteEncoder.getDistance() < EncoderConstants.maxDistance){
        armMotor.set(speed);
        armMotor2.set(speed);
      }
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
