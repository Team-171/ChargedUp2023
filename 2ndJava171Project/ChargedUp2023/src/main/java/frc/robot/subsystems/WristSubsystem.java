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

  public static CANSparkMax wristMotor;

  public static DutyCycleEncoder wristEncoder;

  PIDController pid;
  double setpoint;
  double setPower;
  double holdPosition;
  double offsetPosition;

  /** Creates a new ExampleSubsystem. */
  public WristSubsystem() {
    wristMotor = new CANSparkMax(WristConstants.wristMotorID, MotorType.kBrushless);

    wristMotor.restoreFactoryDefaults();

    wristEncoder = new DutyCycleEncoder(WristConstants.wristEncoderChannel);

    pid = new PIDController(WristConstants.wristPIDkp, WristConstants.wristPIDki, WristConstants.wristPIDkd);

    setPower = 0;

    wristEncoder.reset(); 

    // everytime use getDistance() make sure to add the offset
    offsetPosition = wristEncoder.getAbsolutePosition() - WristConstants.absoluteRoughMiddle;

    holdPosition = wristEncoder.getDistance() + offsetPosition;
  }

  public void resetRelativeDistance(){
    wristEncoder.reset();
  }

  public boolean moveWristStick(double speed){
    
    if(Math.abs(speed) < WristConstants.wristDeadZone) {
      speed = 0;
    }

    speed = speed * 0.25;
    setpoint = MathUtil.clamp(speed + holdPosition, WristConstants.wristLowHardStop, WristConstants.wristHighHardStop);


    setPower = MathUtil.clamp(pid.calculate(wristEncoder.getDistance() + offsetPosition, setpoint), -WristConstants.wristSpeed, WristConstants.wristSpeed);
    if(speed != 0)
      holdPosition = wristEncoder.getDistance() + offsetPosition;
    
    wristMotor.set(setPower);

    if(wristEncoder.getDistance() + offsetPosition > holdPosition - AutoConstants.wristTolerance && wristEncoder.getDistance() + offsetPosition < holdPosition + AutoConstants.wristTolerance){
      return true;
    }

    return false;
  }

  public boolean moveWristButton(double position){
    setpoint = position;
    holdPosition = position;

    setPower = MathUtil.clamp(pid.calculate(wristEncoder.getDistance() + offsetPosition, setpoint), -WristConstants.wristSpeed, WristConstants.wristSpeed);

    wristMotor.set(setPower);

    if(wristEncoder.getDistance() + offsetPosition > holdPosition - AutoConstants.wristTolerance && wristEncoder.getDistance() + offsetPosition < holdPosition + AutoConstants.wristTolerance){
      return true;
    }

    return false;
  }

  public double getHoldPosition(){
    return holdPosition;
  }

  public double getWristEncoder(){
    return wristEncoder.getDistance() + offsetPosition;
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Wrist PID Output: ", setPower);
    SmartDashboard.putNumber("Wrist Speed: ", wristMotor.get());
    SmartDashboard.putNumber("Wrist Hold Distance: ", holdPosition);
    SmartDashboard.putNumber("Wrist absolute position: ", wristEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Offset with getDistance: ", wristEncoder.getDistance() + offsetPosition);
    SmartDashboard.putNumber("Get distance ", wristEncoder.getDistance());
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
