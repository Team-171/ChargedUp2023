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
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.math.MathUtil;


public class WristSubsystem extends SubsystemBase {

  public static CANSparkMax wristMotor;

  PIDController pid;
  double setpoint;
  double aButton;
  double bButton;
  double xButton;
  double yButton;

  double setDistance;

  /** Creates a new ExampleSubsystem. */
  public WristSubsystem() {
    wristMotor = new CANSparkMax(WristConstants.wristMotorID, MotorType.kBrushless);

    wristMotor.restoreFactoryDefaults();

    pid = new PIDController(2, 0, 0);

    aButton = 0.1;
    bButton = 0.2;
    xButton = 0.25;
    yButton = 0.30;

    setDistance = 0;
  }

  public void moveWrist(double speed, boolean aButton, boolean bButton, boolean xButton, boolean yButton){
    if(Math.abs(speed) < .1) {
      speed = 0;
    }

    speed = speed * 0.25;
    setpoint = speed + ArmSubsystem.wristEncoder.getDistance();

    if(aButton){
      setpoint = this.aButton;
    }else if(bButton){
      setpoint = this.bButton;
    }else if(xButton){
      setpoint = this.xButton;
    }else if(yButton){
      setpoint = this.yButton;
    }  

    setDistance = -MathUtil.clamp(pid.calculate(ArmSubsystem.wristEncoder.getDistance(), setpoint), -0.5, 0.5);

    wristMotor.set(setDistance);

    SmartDashboard.putNumber("Wrist PID Going To: ", -MathUtil.clamp(pid.calculate(ArmSubsystem.wristEncoder.getDistance(), setpoint), -0.15, 0.15));
    SmartDashboard.putNumber("Arm Speed: ", wristMotor.get());
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
