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

  public boolean moveArm(double speed, boolean aButton, boolean bButton, boolean xButton, boolean yButton, boolean reset, boolean safe){

      if(Math.abs(speed) < ArmConstants.armDeadZone) {
            speed = 0;
      }

      speed = speed * 0.1;
      setpoint = speed + holdPosition;

      if(aButton){
        setpoint = ArmConstants.conePickupEncoderPosition;
        holdPosition = ArmConstants.conePickupEncoderPosition;
        aButton = false;
      }else if(bButton){
        setpoint = ArmConstants.thirdLevelEncoderPosition;
        holdPosition = ArmConstants.thirdLevelEncoderPosition;
        bButton = false;
      }else if(xButton){
        setpoint = ArmConstants.cubePickupEncoderPostion;
        holdPosition = ArmConstants.cubePickupEncoderPostion;
        xButton = false;
      }else if(yButton){
        setpoint = ArmConstants.secondLevelEncoderPosition;
        holdPosition = ArmConstants.secondLevelEncoderPosition;
        yButton = false;
      }else if(reset){
        setpoint = ArmConstants.reset;
        holdPosition = ArmConstants.reset;
        reset = false;
      }else if(safe){
        setpoint = ArmConstants.safe;
        holdPosition = ArmConstants.safe;
        safe = false;
      }

      setPower = -MathUtil.clamp(pid.calculate(armEncoder.getAbsolutePosition(), setpoint), -ArmConstants.armSpeed, ArmConstants.armSpeed);
      if(speed != 0){holdPosition = armEncoder.getAbsolutePosition();}
      
      if(armEncoder.getAbsolutePosition() > ArmConstants.armLowHardStop && armEncoder.getAbsolutePosition() < ArmConstants.armHighHardStop){
        armMotor.set(setPower);
        armMotor2.set(setPower);
      }else{
        setPower = -MathUtil.clamp(pid.calculate(armEncoder.getAbsolutePosition(), ArmConstants.armRoughMiddle), -ArmConstants.armReturnSpeed, ArmConstants.armReturnSpeed);
        armMotor.set(setPower);
        armMotor2.set(setPower);
      }

      SmartDashboard.putNumber("Arm PID Output: ", setPower);
      SmartDashboard.putNumber("Both Arm Motor Speed: ", armMotor.get());
      SmartDashboard.putNumber("Arm Hold Position: ", holdPosition);

      if(armEncoder.getAbsolutePosition() > holdPosition - AutoConstants.armTolerance && armEncoder.getAbsolutePosition() < holdPosition + AutoConstants.armTolerance){
        return true;
      }

      return false;
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
