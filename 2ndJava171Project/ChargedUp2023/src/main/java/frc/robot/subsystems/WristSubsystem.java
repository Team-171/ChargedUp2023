// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;



public class WristSubsystem extends SubsystemBase {
  // Initializes variables
  public static CANSparkMax wristMotor;

  public static DutyCycleEncoder wristEncoder;

  PIDController pid;
  double setpoint;
  double setPower;
  double holdPosition;
  double offsetPosition;

  /** 
   * Creates a new WristSubsystem.
   * Used to move the wrist
   */
  public WristSubsystem() {
    // Creates the motor for the wrist
    wristMotor = new CANSparkMax(WristConstants.wristMotorID, MotorType.kBrushless);

    // Resets to default, always do before changing config
    wristMotor.restoreFactoryDefaults();

    // Creates an absolute encoder 
    wristEncoder = new DutyCycleEncoder(WristConstants.wristEncoderChannel);

    // Sets up pid for the wrist
    pid = new PIDController(WristConstants.wristPIDkp, WristConstants.wristPIDki, WristConstants.wristPIDkd);

    // Sets the power for the motors to 0 to begin
    setPower = 0;

    // Resets the encoder
    wristEncoder.reset(); 

    // everytime use getDistance() make sure to add the offset
    // Distance from persistant point (0.5)
    offsetPosition = wristEncoder.getAbsolutePosition() - WristConstants.absoluteRoughMiddle;

    // What the wrist should be at
    holdPosition = wristEncoder.getDistance() + offsetPosition;
  }

  /**
   * Resets the encoder to 0
   */
  public void resetRelativeDistance(){
    wristEncoder.reset();
  }

  /**
   * Moves the wrist with input from controller
   * @param speed double Input from controller
   */
  public void moveWristStick(double speed){
    // Controller deadzone
    if(Math.abs(speed) < WristConstants.wristDeadZone) {
      speed = 0;
    }

    // Speed multiplier
    speed = speed * 0.25;

    // Calculates the next position of the wrist, as long as it is in hard stops
    setpoint = MathUtil.clamp(speed + holdPosition, WristConstants.wristLowHardStop, WristConstants.wristHighHardStop);

    // Calculates the power to send to the wrist based on position and where it is currently at
    // The wristSpeed is to make sure the wrist doesn't move too fast
    setPower = MathUtil.clamp(pid.calculate(wristEncoder.getDistance() + offsetPosition, setpoint), -WristConstants.wristSpeed, WristConstants.wristSpeed);
    
    // Sets the hold position to the current position if the arm is moving
    if(speed != 0)
      holdPosition = wristEncoder.getDistance() + offsetPosition;
    
    // Sets the power of the motor
    wristMotor.set(setPower);
  }

  /**
   * Moves the wrist to a set preset
   * @param position double Destination of the wrist
   * @return boolean If the wrist is in the correct position
   */
  public boolean moveWristButton(double position){
    // Sets the destination to wherver the preset is
    setpoint = position;
    holdPosition = position;

    // Calculates the power to send to the wrist based on position and where it is currently at
    // The wristSpeed is to make sure the wrist doesn't move too fast
    setPower = MathUtil.clamp(pid.calculate(wristEncoder.getDistance() + offsetPosition, setpoint), -WristConstants.wristSpeed, WristConstants.wristSpeed);

    // Sets the power of the motor
    wristMotor.set(setPower);

    // Returns true when the wrist is close enough to the destination
    if(wristEncoder.getDistance() + offsetPosition > holdPosition - AutoConstants.wristTolerance && wristEncoder.getDistance() + offsetPosition < holdPosition + AutoConstants.wristTolerance){
      return true;
    }

    return false;
  }

  /**
   * Gets where the wrist should be
   * @return double Where the arm should be
   */
  public double getHoldPosition(){
    return holdPosition;
  }

  /**
   * Gets where the wrist is
   * @return double Where the arm is
   */
  public double getWristEncoder(){
    return wristEncoder.getDistance() + offsetPosition;
  }

  @Override
  public void periodic() {
    // SmartDashboard posting
    // Put SmartDashboard in periodic so the output is always working even when the robot is disabled

    SmartDashboard.putNumber("Wrist PID Output: ", setPower);
    SmartDashboard.putNumber("Wrist Speed: ", wristMotor.get());
    SmartDashboard.putNumber("Wrist Hold Distance: ", holdPosition);
    SmartDashboard.putNumber("Wrist absolute position: ", wristEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Offset with getDistance: ", wristEncoder.getDistance() + offsetPosition);
    SmartDashboard.putNumber("Get distance ", wristEncoder.getDistance());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
