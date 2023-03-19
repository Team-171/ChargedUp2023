// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;



public class WristSubsystem extends SubsystemBase {

  public static CANSparkMax wristMotor;

  public static DutyCycleEncoder wristEncoder;

  PIDController pid;
  double setpoint;
  double aButton;
  double bButton;
  double xButton;
  double yButton;
  double reset;

  double setDistance;
  double currentDistance;

  /** Creates a new ExampleSubsystem. */
  public WristSubsystem() {
    wristMotor = new CANSparkMax(WristConstants.wristMotorID, MotorType.kBrushless);

    wristMotor.restoreFactoryDefaults();

    wristEncoder = new DutyCycleEncoder(1);

    pid = new PIDController(2, 0.05, 0.1);

    aButton = 0.656;
    bButton = -0.9;
    xButton = 0.464;
    yButton = -0.44;
    reset = 0.471;

    setDistance = 0;
    currentDistance = wristEncoder.getDistance();
  }

  public void moveWrist(double speed, boolean aButton, boolean bButton, boolean xButton, boolean yButton, boolean reset){
    if(Math.abs(speed) < .1) {
      speed = 0;
    }

    speed = speed * 0.25;
    setpoint = speed + currentDistance;

    if(aButton){
      setpoint = this.aButton;
      currentDistance = wristEncoder.getDistance();
    }else if(bButton){
      setpoint = this.bButton;
      currentDistance = wristEncoder.getDistance();
    }else if(xButton){
      setpoint = this.xButton;
      currentDistance = wristEncoder.getDistance();
    }else if(yButton){
      setpoint = this.yButton;
      currentDistance = wristEncoder.getDistance();
    }else if(reset){
      setpoint = this.reset;
      currentDistance = wristEncoder.getDistance();
    }  

    setDistance = MathUtil.clamp(pid.calculate(wristEncoder.getDistance(), setpoint), -0.75, 0.75);
    if(speed != 0){currentDistance = wristEncoder.getDistance();}

    // if(wristEncoder.getDistance() > -1.5 && wristEncoder.getDistance() < 1.45 ){
    //   wristMotor.set(setDistance);
    // }else{
    //   setDistance = MathUtil.clamp(pid.calculate(wristEncoder.getDistance(), 0.2), -0.5, 0.5);
    //   wristMotor.set(setDistance);
    // }

    if(wristEncoder.getDistance() > -1.5 && wristEncoder.getDistance() < 1.45 ){
      wristMotor.set(setDistance);
    }else{
      setDistance = MathUtil.clamp(pid.calculate(wristEncoder.getDistance(), 0.2), -0.25, 0.25);
      wristMotor.set(setDistance);
    }

    SmartDashboard.putNumber("Wrist PID Going To: ", setDistance);
    SmartDashboard.putNumber("Wrist Speed: ", wristMotor.get());
    SmartDashboard.putNumber("Wrist Setpoint: ", setpoint);
    SmartDashboard.putNumber("Wrist Current Distance: ", currentDistance);
  }

  public double getWristEncoder(){
    return wristEncoder.getDistance();
  }

  // public void moveWristForward(){
  //     // if potentiameter
  //     wristMotor.set(WristConstants.wristSpeed);
  // }

  // public void moveWristBackward(){
  //   // if potentiameter
  //   wristMotor.set(-WristConstants.wristSpeed);
  // }

  // public void stopWrist(){
  //   wristMotor.set(0);
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
