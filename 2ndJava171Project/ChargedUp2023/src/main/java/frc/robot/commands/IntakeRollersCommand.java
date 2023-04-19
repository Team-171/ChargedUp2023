// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeRollersSubsystem;

/** A command to run the intake */
public class IntakeRollersCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeRollersSubsystem intakeRollersSubsystem;
  // speed to run the intake at
  private DoubleSupplier speed;
  /**
   * Creates a new IntakeRollersCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntakeRollersCommand(IntakeRollersSubsystem subsystem, DoubleSupplier speed) {
    intakeRollersSubsystem = subsystem;
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
    // run the intake at the speed
    intakeRollersSubsystem.moveRoller(speed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
