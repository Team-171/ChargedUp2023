// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.RollerConstants;
import edu.wpi.first.math.controller.PIDController;

import javax.management.relation.RoleResult;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;


public class IntakeRollersSubsystem extends SubsystemBase {

  CANSparkMax rollersMotor;
  PIDController pid;
  double setDistance;
  double currentDistance;
  double setpoint;

  /** Creates a new ExampleSubsystem. */
  public IntakeRollersSubsystem() {
    rollersMotor = new CANSparkMax(RollerConstants.rollerMotorID, MotorType.kBrushless);

    rollersMotor.restoreFactoryDefaults();
    rollersMotor.getEncoder().setPosition(0);

    pid = new PIDController(0.025, 0, 0);

    currentDistance = 0;
  }

  public void moveRoller(double speed){
    if(Math.abs(speed) < .1) {
      speed = 0;
    }

    // setpoint = currentDistance + speed;
    // setDistance = MathUtil.clamp(pid.calculate(rollersMotor.getEncoder().getPosition(), setpoint), -1, 1);

    // if(speed != 0){
    //   currentDistance = rollersMotor.getEncoder().getPosition();
    // }

    if(speed != 0){
      rollersMotor.set(speed);
      rollersMotor.getEncoder().setPosition(0);
    }else{
      rollersMotor.set(MathUtil.clamp(pid.calculate(rollersMotor.getEncoder().getPosition(), 0), -1, 1));
    }
  }

  public void stopIntakeRollers(){
    rollersMotor.set(0);
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
