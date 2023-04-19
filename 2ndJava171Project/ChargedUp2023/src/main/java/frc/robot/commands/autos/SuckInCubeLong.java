// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.IntakeRollersSubsystem;

/** A command that sucks in a cube. */
public class SuckInCubeLong extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeRollersSubsystem rollersSubsystem;


  /**
   * Creates a new SuckInCubeLong command.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SuckInCubeLong(IntakeRollersSubsystem subsystem) {
    rollersSubsystem = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // move the roller to suck in a cube
    rollersSubsystem.moveRoller(AutoConstants.intakeSpeedCubePickup);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // set the intake to 0
    rollersSubsystem.moveRoller(0);
    // reset the intake encoder
    rollersSubsystem.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
