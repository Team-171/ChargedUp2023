// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class ArmSubsystem extends SubsystemBase {
  // Initialization of variables
  CANSparkMax armMotor;
  CANSparkMax armMotor2;

  public static DutyCycleEncoder armEncoder;

  PIDController pid;
  double setpoint;
  double setPower;
  double holdPosition;

  /** 
   * Creates a new ArmSubsystem. 
   * Used to control the arm
  */
  public ArmSubsystem() {
    // Creates two motors to control the arm
    armMotor = new CANSparkMax(ArmConstants.armMotorID, MotorType.kBrushless);
    armMotor2 = new CANSparkMax(ArmConstants.armMotorID2, MotorType.kBrushless);

    // Resets to default, always do before changing config
    armMotor.restoreFactoryDefaults();
    armMotor2.restoreFactoryDefaults();

    // Sets one motor inverted so they work together
    armMotor.setInverted(true);

    // Creates an absolute encoder
    armEncoder = new DutyCycleEncoder(ArmConstants.armEncoderChannel);

    // Sets up pid for arm
    pid = new PIDController(ArmConstants.armPIDkp, ArmConstants.armPIDki, ArmConstants.armPIDkd);

    // Sets the power for the motors to 0 to begin
    setPower = 0;

    // What the arm should be at
    holdPosition = armEncoder.getAbsolutePosition();
  }

  /**
   * Moves the arm based on input from stick axis
   * @param speed Input from controller
   */
  public void moveArmStick(double speed){

    // Controller deadzone
    if(Math.abs(speed) < ArmConstants.armDeadZone) {
          speed = 0;
    }

    // Speed multiplier
    speed = speed * 0.1;

    // Calculates the next position of the arm, as long as it is in hard stops
    setpoint = MathUtil.clamp(speed + holdPosition, ArmConstants.armLowHardStop, ArmConstants.armHighHardStop);

    // Calculates the power to send to the arm based on position and where it is currently at
    // The armSpeed is to make sure the arm doesn't move too fast
    setPower = -MathUtil.clamp(pid.calculate(armEncoder.getAbsolutePosition(), setpoint), -ArmConstants.armSpeed, ArmConstants.armSpeed);
    
    // Sets the hold position to the current position if the arm is moving
    if(speed != 0)
      holdPosition = armEncoder.getAbsolutePosition();
    
    // Sets the power of the motors
    armMotor.set(setPower);
    armMotor2.set(setPower);
  }

  /**
   * Moves the arm to a set preset
   * @param position double Destination of the arm
   * @return boolean If the arm is in the correct position
   */
  public boolean moveArmButton(double position){
    // Sets the destination to wherever the preset is
    setpoint = position;
    holdPosition = position;

    // Calculates the power to send to the end based on position and where it is currently at
    // The armSpeed is to make sure the arm doesn't move too fast
    setPower = -MathUtil.clamp(pid.calculate(armEncoder.getAbsolutePosition(), setpoint), -ArmConstants.armSpeed, ArmConstants.armSpeed);

    // Sets the power of the motors
    armMotor.set(setPower);
    armMotor2.set(setPower);

    // Returns true when the arm is close enough to the destination, mainly used for autonomous
    if(armEncoder.getAbsolutePosition() > holdPosition - AutoConstants.armTolerance && armEncoder.getAbsolutePosition() < holdPosition + AutoConstants.armTolerance){
      return true;
    }

    return false;
  }

  /**
   * Gets where the arm should be
   * @return double Where the arm should be
   */
  public double getHoldPosition(){
    return holdPosition;
  }

  /**
   * Gets where the arm is
   * @return double Where the arm is
   */
  public double getArmEncoder(){
    return armEncoder.getDistance();
  }

  @Override
  public void periodic() {
    // SmartDashboard posting
    // Put SmartDashboard in periodic so the output is always working even when the robot is disabled
    
    SmartDashboard.putNumber("Arm PID Output: ", setPower);
    SmartDashboard.putNumber("Both Arm Motor Speed: ", armMotor.get());
    SmartDashboard.putNumber("Arm Hold Position: ", holdPosition);
    SmartDashboard.putNumber("Arm Encoder", armEncoder.getAbsolutePosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
