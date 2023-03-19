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
  double setDistance;
  double currentDistance;

  /** Creates a new ExampleSubsystem. */
  public WristSubsystem() {
    wristMotor = new CANSparkMax(WristConstants.wristMotorID, MotorType.kBrushless);

    wristMotor.restoreFactoryDefaults();

    wristEncoder = new DutyCycleEncoder(WristConstants.wristEncoderChannel);

    pid = new PIDController(WristConstants.wristPIDkp, WristConstants.wristPIDki, WristConstants.wristPIDkd);

    setDistance = 0;
    currentDistance = wristEncoder.getDistance();
  }

  public void moveWrist(double speed, boolean aButton, boolean bButton, boolean xButton, boolean yButton, boolean reset){
    if(Math.abs(speed) < WristConstants.wristDeadZone) {
      speed = 0;
    }

    speed = speed * 0.25;
    setpoint = speed + currentDistance;

    if(aButton){
      setpoint = WristConstants.aButton;
      currentDistance = wristEncoder.getDistance();
    }else if(bButton){
      setpoint = WristConstants.bButton;
      currentDistance = wristEncoder.getDistance();
    }else if(xButton){
      setpoint = WristConstants.xButton;
      currentDistance = wristEncoder.getDistance();
    }else if(yButton){
      setpoint = WristConstants.yButton;
      currentDistance = wristEncoder.getDistance();
    }else if(reset){
      setpoint = WristConstants.reset;
      currentDistance = wristEncoder.getDistance();
    }  

    setDistance = MathUtil.clamp(pid.calculate(wristEncoder.getDistance(), setpoint), -WristConstants.wristSpeed, WristConstants.wristSpeed);
    if(speed != 0){currentDistance = wristEncoder.getDistance();}

    if(wristEncoder.getDistance() > WristConstants.wristLowHardStop && wristEncoder.getDistance() < WristConstants.wristHighHardStop){
      wristMotor.set(setDistance);
    }else{
      setDistance = MathUtil.clamp(pid.calculate(wristEncoder.getDistance(), WristConstants.wristRoughMiddle), -WristConstants.wristReturnSpeed, WristConstants.wristReturnSpeed);
      wristMotor.set(setDistance);
    }

    SmartDashboard.putNumber("Wrist PID Output: ", setDistance);
    SmartDashboard.putNumber("Wrist Speed: ", wristMotor.get());
    SmartDashboard.putNumber("Wrist Hold Distance: ", currentDistance);
  }

  public double getWristEncoder(){
    return wristEncoder.getDistance();
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
