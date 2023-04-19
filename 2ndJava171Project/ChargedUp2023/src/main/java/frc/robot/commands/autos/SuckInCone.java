// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeRollersSubsystem;
import edu.wpi.first.wpilibj.Timer;

/** A command that sucks in the cone for a few seconds. */
public class SuckInCone extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeRollersSubsystem rollersSubsystem;

  // time that the command starts
  private double startTime;

  /**
   * Creates a new SuckInCone command.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SuckInCone(IntakeRollersSubsystem subsystem) {
    rollersSubsystem = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // get the time when the command started
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // suck in the cone with the intake
    rollersSubsystem.moveRoller(-1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // set the intake speed to 0
    rollersSubsystem.moveRoller(0);
    // reset the encoder
    rollersSubsystem.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if the amount of time the code has run for .25 seconds
    if(Timer.getFPGATimestamp() - startTime > 0.25)
        return true;
    return false;
  }
}
