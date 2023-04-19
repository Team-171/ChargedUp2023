// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;


public class IntakeRollersSubsystem extends SubsystemBase {
  // Initialization of variables
  CANSparkMax rollersMotor;
  PIDController pid;
  double setDistance;
  double holdPosition;

  /** 
   * Creates a new IntakeRollersSubsystem.
   * Controls speed of intake rollers 
  */
  public IntakeRollersSubsystem() {
    // Creates a roller for intake
    rollersMotor = new CANSparkMax(RollerConstants.rollerMotorID, MotorType.kBrushless);

    // Resets to default, always do before changing config
    rollersMotor.restoreFactoryDefaults();

    // Gets the built in encoder and resets it
    rollersMotor.getEncoder().setPosition(0);

    // Sets up pid so the piece doesn't fall out
    pid = new PIDController(RollerConstants.rollerPIDkp, RollerConstants.rollerPIDki, RollerConstants.rollerPIDkd);
  }

  /**
   * Moves the roller based on an axis, in this case, triggers
   * @param speed double Input from triggers / axis
   */
  public void moveRoller(double speed){
    // Deadzone
    if(Math.abs(speed) < RollerConstants.rollerDeadZone) {
      speed = 0;
    }

    // Set speed and reset encoder
    // If not moving, try to get back to what it was when it stopped moving
    if(speed != 0){
      rollersMotor.set(speed);
      rollersMotor.getEncoder().setPosition(0);
    }else{
      rollersMotor.set(MathUtil.clamp(pid.calculate(rollersMotor.getEncoder().getPosition(), 0), -RollerConstants.rollerSpeed, RollerConstants.rollerSpeed));
    }
  }

  /**
   * Resets the encoder in the motor
   */
  public void reset(){
    rollersMotor.getEncoder().setPosition(0);
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
