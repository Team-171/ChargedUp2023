// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TankDriveSubsystem;


/** An example command that uses an example subsystem. */
public class TurnAuto extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final TankDriveSubsystem driveSubsystem;
  private double degrees;
  private boolean finished;
  private boolean slow;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TurnAuto(TankDriveSubsystem subsystem, double degrees, boolean slow) {
    driveSubsystem = subsystem;
    this.degrees = degrees;
    this.slow = slow;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    finished = driveSubsystem.driveForward(degrees, slow);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.arcadeDrive(0, 0, false);
    driveSubsystem.resetYaw();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (finished)
      return true;
    return false;
  }
}
