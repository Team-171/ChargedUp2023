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
import frc.robot.Constants.*;

public class ArmSubsystem extends SubsystemBase {

  CANSparkMax armMotor;
  CANSparkMax armMotor2;

  public static DutyCycleEncoder armEncoder;

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
  public ArmSubsystem() {
    armMotor = new CANSparkMax(ArmConstants.armMotorID, MotorType.kBrushless);
    armMotor2 = new CANSparkMax(ArmConstants.armMotorID2, MotorType.kBrushless);

    armMotor.restoreFactoryDefaults();
    armMotor2.restoreFactoryDefaults();

    armMotor.setInverted(true);

    armEncoder = new DutyCycleEncoder(0);

    pid = new PIDController(3.5, 0.2, 0.5);

    aButton = 0.05;
    bButton = 0.4;
    xButton = 0.057;
    yButton = 0.38;
    reset = 0.2;

    setDistance = 0;
    currentDistance = armEncoder.getAbsolutePosition();
  }

  public void moveArm(double speed, boolean aButton, boolean bButton, boolean xButton, boolean yButton, boolean reset){

      if(Math.abs(speed) < .1) {
            speed = 0;
      }

      speed = speed * 0.1;
      setpoint = speed + currentDistance;

      if(aButton){
        setpoint = this.aButton;
        currentDistance = armEncoder.getAbsolutePosition();
        aButton = false;
      }else if(bButton){
        setpoint = this.bButton;
        currentDistance = armEncoder.getAbsolutePosition();
        bButton = false;
      }else if(xButton){
        setpoint = this.xButton;
        currentDistance = armEncoder.getAbsolutePosition();
        xButton = false;
      }else if(yButton){
        setpoint = this.yButton;
        currentDistance = armEncoder.getAbsolutePosition();
        yButton = false;
      }else if(reset){
        setpoint = this.reset;
        currentDistance = armEncoder.getAbsolutePosition();
      }

      setDistance = -MathUtil.clamp(pid.calculate(armEncoder.getAbsolutePosition(), setpoint), -1, 1);
      if(speed != 0){currentDistance = armEncoder.getAbsolutePosition();}
      
      if(armEncoder.getAbsolutePosition() > 0.03 && armEncoder.getAbsolutePosition() < 0.45 ){
        armMotor.set(setDistance);
        armMotor2.set(setDistance);
      }else{
        setDistance = -MathUtil.clamp(pid.calculate(armEncoder.getAbsolutePosition(), 0.16), -0.25, 0.25);
        armMotor.set(setDistance);
        armMotor2.set(setDistance);
      }

      SmartDashboard.putNumber("PID Going To: ", setDistance);
      SmartDashboard.putNumber("Arm Speed: ", armMotor.get());
      SmartDashboard.putNumber("Arm Speed 2: ", armMotor2.get());
      SmartDashboard.putNumber("Setpoint: ", setpoint);
      SmartDashboard.putNumber("Current Distance: ", currentDistance);
  }

  public double getArmEncoder(){
    return armEncoder.getDistance();
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
