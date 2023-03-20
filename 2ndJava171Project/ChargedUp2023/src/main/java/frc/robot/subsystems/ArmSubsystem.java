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

  CANSparkMax armMotor;
  CANSparkMax armMotor2;

  public static DutyCycleEncoder armEncoder;

  PIDController pid;
  double setpoint;
  double setDistance;
  double currentDistance;

  /** Creates a new ExampleSubsystem. */
  public ArmSubsystem() {
    armMotor = new CANSparkMax(ArmConstants.armMotorID, MotorType.kBrushless);
    armMotor2 = new CANSparkMax(ArmConstants.armMotorID2, MotorType.kBrushless);

    armMotor.restoreFactoryDefaults();
    armMotor2.restoreFactoryDefaults();

    armMotor.setInverted(true);

    armEncoder = new DutyCycleEncoder(ArmConstants.armEncoderChannel);

    pid = new PIDController(ArmConstants.armPIDkp, ArmConstants.armPIDki, ArmConstants.armPIDkd);

    setDistance = 0;
    currentDistance = armEncoder.getAbsolutePosition();
  }

  public void moveArm(double speed, boolean aButton, boolean bButton, boolean xButton, boolean yButton, boolean reset){

      if(Math.abs(speed) < ArmConstants.armDeadZone) {
            speed = 0;
      }

      speed = speed * 0.1;
      setpoint = speed + currentDistance;

      if(aButton){
        setpoint = ArmConstants.aButton;
        currentDistance = armEncoder.getAbsolutePosition();
        aButton = false;
      }else if(bButton){
        setpoint = ArmConstants.bButton;
        currentDistance = armEncoder.getAbsolutePosition();
        bButton = false;
      }else if(xButton){
        setpoint = ArmConstants.xButton;
        currentDistance = armEncoder.getAbsolutePosition();
        xButton = false;
      }else if(yButton){
        setpoint = ArmConstants.yButton;
        currentDistance = armEncoder.getAbsolutePosition();
        yButton = false;
      }else if(reset){
        setpoint = ArmConstants.reset;
        currentDistance = armEncoder.getAbsolutePosition();
      }

      setDistance = -MathUtil.clamp(pid.calculate(armEncoder.getAbsolutePosition(), setpoint), -ArmConstants.armSpeed, ArmConstants.armSpeed);
      if(speed != 0){currentDistance = armEncoder.getAbsolutePosition();}
      
      if(armEncoder.getAbsolutePosition() > ArmConstants.armLowHardStop && armEncoder.getAbsolutePosition() < ArmConstants.armHighHardStop){
        armMotor.set(setDistance);
        armMotor2.set(setDistance);
      }else{
        setDistance = -MathUtil.clamp(pid.calculate(armEncoder.getAbsolutePosition(), ArmConstants.armRoughMiddle), -ArmConstants.armReturnSpeed, ArmConstants.armReturnSpeed);
        armMotor.set(setDistance);
        armMotor2.set(setDistance);
      }

      SmartDashboard.putNumber("Arm PID Output: ", setDistance);
      SmartDashboard.putNumber("Both Arm Motor Speed: ", armMotor.get());
      SmartDashboard.putNumber("Arm Hold Position: ", currentDistance);
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
