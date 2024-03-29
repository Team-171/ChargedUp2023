// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

/** A command that uses the arm subsystem. */
public class ArmCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem armSubsystem;

  // speed to run the arm at
  DoubleSupplier speed;

  /**
   * Creates a new ArmCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmCommand(ArmSubsystem subsystem, DoubleSupplier speed) {
    armSubsystem = subsystem;
    this.speed = speed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // move the arm based on manual input
    armSubsystem.moveArmStick(speed.getAsDouble());

    SmartDashboard.putNumber("Arm Encoder Absolute Position: ", armSubsystem.getArmEncoder());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
