// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.aruco.EstimateParameters;
import org.opencv.core.Mat;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

  CANSparkMax armMotor;
  CANSparkMax armMotor2;

  public static DutyCycleEncoder armEncoder;
  public static DutyCycleEncoder wristEncoder;

  PIDController pid;
  double setpoint;
  double aButton;
  double bButton;
  double xButton;
  double yButton;

  double setDistance;

  /** Creates a new ExampleSubsystem. */
  public ArmSubsystem() {
    armMotor = new CANSparkMax(ArmConstants.armMotorID, MotorType.kBrushless);
    armMotor2 = new CANSparkMax(ArmConstants.armMotorID2, MotorType.kBrushless);

    armMotor.restoreFactoryDefaults();
    armMotor2.restoreFactoryDefaults();

    armMotor.setInverted(true);

    armEncoder = new DutyCycleEncoder(0);
    wristEncoder = new DutyCycleEncoder(1);

    pid = new PIDController(5, 0, 0);

    aButton = 0.1;
    bButton = 0.2;
    xButton = 0.25;
    yButton = 0.30;

    setDistance = 0;
  }

  public void moveArm(double speed, boolean aButton, boolean bButton, boolean xButton, boolean yButton){

      if(Math.abs(speed) < .1) {
            speed = 0;
      }

      speed = speed * 0.25;
      setpoint = speed + armEncoder.getDistance();

      if(aButton){
        setpoint = this.aButton;
      }else if(bButton){
        setpoint = this.bButton;
      }else if(xButton){
        setpoint = this.xButton;
      }else if(yButton){
        setpoint = this.yButton;
      }  

      setDistance = -MathUtil.clamp(pid.calculate(armEncoder.getDistance(), setpoint), -0.5, 0.5);
      
      if(armEncoder.getDistance() > 0.03 && armEncoder.getDistance() < 0.4 ){
        armMotor.set(setDistance);
        armMotor2.set(setDistance);
      }else{
        setDistance = -MathUtil.clamp(pid.calculate(armEncoder.getDistance(), 0.25), -0.5, 0.5);
        armMotor.set(setDistance);
        armMotor2.set(setDistance);
      }

      SmartDashboard.putNumber("PID Going To: ", -MathUtil.clamp(pid.calculate(armEncoder.getDistance(), setpoint), -0.5, 0.5));
      SmartDashboard.putNumber("Arm Speed: ", armMotor.get());
      SmartDashboard.putNumber("Arm Speed 2:", armMotor2.get());
  }

  public double getArmEncoder(){
    return armEncoder.getDistance();
  }

  public double getWristEncoder(){
    return wristEncoder.getDistance();
  }

  public void setArmPreset() {

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
