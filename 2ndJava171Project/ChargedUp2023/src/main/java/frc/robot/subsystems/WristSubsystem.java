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

  /** Creates a new ExampleSubsystem. */
  public WristSubsystem() {
    wristMotor = new CANSparkMax(WristConstants.wristMotorID, MotorType.kBrushless);

    wristMotor.restoreFactoryDefaults();

    wristEncoder = new DutyCycleEncoder(WristConstants.wristEncoderChannel);

    pid = new PIDController(WristConstants.wristPIDkp, WristConstants.wristPIDki, WristConstants.wristPIDkd);

    setPower = 0;
    holdPosition = wristEncoder.getDistance();
  }

  public boolean moveWristStick(double speed){
    
    if(Math.abs(speed) < WristConstants.wristDeadZone) {
      speed = 0;
    }

    speed = speed * 0.25;
    setpoint = speed + holdPosition;

    setPower = MathUtil.clamp(pid.calculate(wristEncoder.getDistance(), setpoint), -WristConstants.wristSpeed, WristConstants.wristSpeed);
    if(speed != 0)
      holdPosition = wristEncoder.getDistance();
    
    if(wristEncoder.getDistance() > WristConstants.wristLowHardStop && wristEncoder.getDistance() < WristConstants.wristHighHardStop){
      wristMotor.set(setPower);
    }else{
      setPower = MathUtil.clamp(pid.calculate(wristEncoder.getDistance(), WristConstants.wristRoughMiddle), -WristConstants.wristReturnSpeed, WristConstants.wristReturnSpeed);
      wristMotor.set(setPower);
    }

    if(wristEncoder.getDistance() > holdPosition - AutoConstants.wristTolerance && wristEncoder.getDistance() < holdPosition + AutoConstants.wristTolerance){
      return true;
    }

    return false;
  }

  public boolean moveWristButton(double position){
    setpoint = position;
    holdPosition = position;

    setPower = MathUtil.clamp(pid.calculate(wristEncoder.getDistance(), setpoint), -WristConstants.wristSpeed, WristConstants.wristSpeed);

    wristMotor.set(setPower);

    if(wristEncoder.getDistance() > holdPosition - AutoConstants.wristTolerance && wristEncoder.getDistance() < holdPosition + AutoConstants.wristTolerance){
      return true;
    }

    return false;
  }

  public double getHoldPosition(){
    return holdPosition;
  }

  public double getWristEncoder(){
    return wristEncoder.getDistance();
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Wrist PID Output: ", setPower);
    SmartDashboard.putNumber("Wrist Speed: ", wristMotor.get());
    SmartDashboard.putNumber("Wrist Hold Distance: ", holdPosition);
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
