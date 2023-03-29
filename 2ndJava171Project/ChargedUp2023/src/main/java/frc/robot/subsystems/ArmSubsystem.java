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
  double setPower;
  double holdPosition;

  /** Creates a new ExampleSubsystem. */
  public ArmSubsystem() {
    armMotor = new CANSparkMax(ArmConstants.armMotorID, MotorType.kBrushless);
    armMotor2 = new CANSparkMax(ArmConstants.armMotorID2, MotorType.kBrushless);

    armMotor.restoreFactoryDefaults();
    armMotor2.restoreFactoryDefaults();

    armMotor.setInverted(true);

    armEncoder = new DutyCycleEncoder(ArmConstants.armEncoderChannel);

    pid = new PIDController(ArmConstants.armPIDkp, ArmConstants.armPIDki, ArmConstants.armPIDkd);

    setPower = 0;
    holdPosition = armEncoder.getAbsolutePosition();
  }

  public boolean moveArmStick(double speed){

      if(Math.abs(speed) < ArmConstants.armDeadZone) {
            speed = 0;
      }

      speed = speed * 0.1;
      setpoint = MathUtil.clamp(speed + holdPosition, ArmConstants.armLowHardStop, ArmConstants.armHighHardStop);

      setPower = -MathUtil.clamp(pid.calculate(armEncoder.getAbsolutePosition(), setpoint), -ArmConstants.armSpeed, ArmConstants.armSpeed);
      if(speed != 0)
        holdPosition = armEncoder.getAbsolutePosition();
      
      armMotor.set(setPower);
      armMotor2.set(setPower);

      // might not need this, but keep for now
      if(armEncoder.getAbsolutePosition() > holdPosition - AutoConstants.armTolerance && armEncoder.getAbsolutePosition() < holdPosition + AutoConstants.armTolerance){
        return true;
      }

      return false;
  }

  public boolean moveArmButton(double position){
    setpoint = position;
    holdPosition = position;

    setPower = -MathUtil.clamp(pid.calculate(armEncoder.getAbsolutePosition(), setpoint), -ArmConstants.armSpeed, ArmConstants.armSpeed);

    armMotor.set(setPower);
    armMotor2.set(setPower);

    if(armEncoder.getAbsolutePosition() > holdPosition - AutoConstants.armTolerance && armEncoder.getAbsolutePosition() < holdPosition + AutoConstants.armTolerance){
      return true;
    }

    return false;
  }

  public double getHoldPosition(){
    return holdPosition;
  }

  public double getArmEncoder(){
    return armEncoder.getDistance();
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Arm PID Output: ", setPower);
    SmartDashboard.putNumber("Both Arm Motor Speed: ", armMotor.get());
    SmartDashboard.putNumber("Arm Hold Position: ", holdPosition);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
